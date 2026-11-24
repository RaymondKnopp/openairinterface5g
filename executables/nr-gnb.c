/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief Top-level threads for gNodeB
 */

#define _GNU_SOURCE
#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all

#include <stdatomic.h>
#include <fcntl.h> // for SEEK_SET
#include <pthread.h> // for pthread_join
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "common/utils/LOG/log.h"
#include "common/utils/system.h"
#include "PHY/NR_ESTIMATION/nr_ul_estimation.h"
#include "openair1/PHY/NR_TRANSPORT/nr_dlsch.h"
#include "openair1/PHY/NR_TRANSPORT/nr_ulsch.h"
#include "NR_PHY_INTERFACE/NR_IF_Module.h"
#include "PHY/INIT/nr_phy_init.h"
#include "PHY/MODULATION/nr_modulation.h"
#include "PHY/NR_TRANSPORT/nr_transport_proto.h"
#include "PHY/defs_RU.h"
#include "PHY/defs_gNB.h"
#include "executables/nr-softmodem.h"
#include "radio/COMMON/gnb_fhi72_if.h"
#include "PHY/defs_nr_common.h"
#include "PHY/impl_defs_nr.h"
#include "PHY/nr_phy_common/inc/nr_phy_meas.h"
#include "SCHED_NR/phy_frame_config_nr.h"
#include "SCHED_NR/sched_nr.h"
#include "assertions.h"
#include "common/ran_context.h"
#include "common/utils/LOG/log.h"
#include "executables/softmodem-common.h"
#include "nfapi/oai_integration/vendor_ext.h"
#include "nfapi_nr_interface_scf.h"
#include "notified_fifo.h"
#include "thread-pool.h"
#include "common/utils/rt_deferred_log.h"
#include "time_meas.h"
#include "utils.h"
#include "common/utils/ds/spsc_q.h"
#include "executables/rt_slot_hist.h"

#define TICK_TO_US(ts) (ts.trials==0?0:ts.diff/ts.trials)
#define L1STATSSTRLEN 16384
static void rx_func(processingData_L1_t *param);

/* split 7.2: precoding and BF, C-plane and DL U-plane through the fhi_72 library */
/* own_meas: false when the TX pipeline runs this on L1_tx_send, where phy_proc_tx was
 * already stopped at the hand-off (it is not safe to stop it from a second thread). */
static void tx_split72(PHY_VARS_gNB *gNB, int slot_type, int frame_tx, int slot_tx, c16_t **txdataF, bool own_meas)
{
  const bool dl_slot = slot_type & NR_DOWNLINK_SLOT;
  fhi72_transport_t *t = gNB->fhi72_transport;
  if (dl_slot) {
    t->prec_bf(gNB, txdataF, frame_tx, slot_tx);
    if (own_meas)
      STOP_MEAS_FULL_SLOT(&gNB->phy_proc_tx, slot_type, NR_DOWNLINK_SLOT);
  }
  // beam API - fill TX and/or RX beam IDs
  if (gNB->common_vars.analog_bf)
    t->cp(&t->s7cfg, gNB->common_vars.beam_id, frame_tx, slot_tx);
  if (dl_slot) // xran sends DL U-plane only in DL/mixed slots
    t->dl_up(&t->s7cfg, frame_tx, slot_tx);
}

/* other splits: precoding, BF and IDFT, control and TX through the RU */
static void tx_split8(PHY_VARS_gNB *gNB, int slot_type, int frame_tx, int slot_tx, uint64_t timestamp_tx)
{
  const bool dl_slot = slot_type & NR_DOWNLINK_SLOT;
  RU_t *ru = gNB->RU_list[0];
  if (dl_slot) {
    ru->feptx_ofdm(ru, frame_tx, slot_tx);
    STOP_MEAS_FULL_SLOT(&gNB->phy_proc_tx, slot_type, NR_DOWNLINK_SLOT);
  }
  // beam API - fill TX and/or RX beam IDs
  if (gNB->common_vars.analog_bf)
    ru->fh_south_ctrl(ru, frame_tx, slot_tx, timestamp_tx);
  // DL is also sent in UL slots with continuous TX and rfsim
  if (dl_slot || get_softmodem_params()->continuous_tx || IS_SOFTMODEM_RFSIM)
    ru->fh_south_out(ru, frame_tx, slot_tx, timestamp_tx);
}

static void tx_overrun_alarm(PHY_VARS_gNB *gNB, const nfapi_nr_dl_tti_request_t *DL_req, int frame_tx, int slot_tx)
{
  /* L1 TX overrun alarm.  A TX that misses its deadline still hands xran a buffer on
   * time, so the O-RU reports nothing wrong (RX_LATE ~0, RX_CORRUPT 0) -- only the
   * CONTENTS are stale, and the UE silently fails to decode the DCI.  The aggregate
   * time_stats cannot show this: it reports a mean plus a max that start_meas() zeroes
   * every 16384 trials, so an outlier is visible but its cause is not.  Report the
   * configuration that produced the overrun instead. */
  if (gNB->tx_overrun_us > 0 && cpu_meas_enabled) {
    const double us = (double)gNB->phy_proc_tx.p_time / 1000.0;
    if (us > gNB->tx_overrun_us) {
      gNB->tx_overrun_count++;
      /* Rate limit: the first 10, then one in 100.  An overrun storm must not add
       * logging to a thread that is already behind. */
      if (gNB->tx_overrun_count <= 10 || (gNB->tx_overrun_count % 100) == 0) {
        gNB->tx_overrun_logged++;
        const nfapi_nr_dl_tti_request_body_t *b = &DL_req->dl_tti_request_body;
        int n_pdsch = 0;
        for (int i = 0; i < b->nPDUs; i++)
          if (b->dl_tti_pdu_list[i].PDUType == NFAPI_NR_DL_TTI_PDSCH_PDU_TYPE)
            n_pdsch++;
        /* Deferred, not logged: this runs on the TX thread at the moment it is already
           past its deadline, and OAI's log path ends in a blocking write(2) on the
           calling thread. Arming the alarm must not be able to make the overrun worse.
           The stats thread drains the ring. One line per PDU rather than one packed
           line, because a deferred line is RT_LOG_LINE bytes; each carries frame.slot
           so the record stays readable if another producer interleaves. */
        RT_LOG_DEFER("%4d.%2d L1 TX overrun: %.1f us > %d us (%llu total, %d PDSCH PDU%s)",
                     frame_tx,
                     slot_tx,
                     us,
                     gNB->tx_overrun_us,
                     (unsigned long long)gNB->tx_overrun_count,
                     n_pdsch,
                     n_pdsch == 1 ? "" : "s");
        for (int i = 0; i < b->nPDUs; i++) {
          if (b->dl_tti_pdu_list[i].PDUType != NFAPI_NR_DL_TTI_PDSCH_PDU_TYPE)
            continue;
          const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *p = &b->dl_tti_pdu_list[i].pdsch_pdu.pdsch_pdu_rel15;
          RT_LOG_DEFER("%4d.%2d L1 TX overrun:   rnti %04x mcs %d rb %d+%d layers %d symb %d+%d tbs %u",
                       frame_tx,
                       slot_tx,
                       p->rnti,
                       p->mcsIndex[0],
                       p->rbStart,
                       p->rbSize,
                       p->nrOfLayers,
                       p->StartSymbolIndex,
                       p->NrOfSymbols,
                       p->TBSize[0]);
        }
      }
    }
  }
}

/* RX job dispatch.
 *
 * The RX job for a slot used to be pushed from tx_func, right after the MAC slot
 * indication. That puts every RX dispatch behind the TX queue: L1_tx_thread gets a
 * job every slot, and once the DL slots average more than a slot's time (607 us at
 * 400 Mbps DL against a 500 us cadence) it falls behind, every RX job starts late,
 * the rxdataF ring is lapped, and the lost PUCCH ACK/NACKs drive DL MCS to 0. The
 * delay was invisible because pool_wait started its clock at the (already late) push.
 *
 * What the RX job actually needs from tx_func is the UL config for its slot, which
 * nr_save_ul_tti_req() stores when tx_func runs for slot_tx == slot_rx, i.e. sl_ahead
 * slots BEFORE the slot is even received. So with split 7.2, split7_thread pushes the
 * job as soon as the slot is unpacked, and rx_func only waits if that config is not
 * stored yet (rx_wait_ul_cfg), which takes a TX backlog of more than sl_ahead slots.
 * Split 8 keeps pushing from tx_func. */
static _Atomic int g_ul_cfg_abs = -1; /* absolute slot of the newest stored UL_tti_req */

/* L1 TX pipeline, split 7.2 only, OAI_L1_TX_PIPE=1 (core OAI_L1_TX_PIPE_CORE, default 0).
 * tx_func is MAC indication + grid generation (~330 us at 300 Mbps DL) and then
 * tx_split72, precoding/copy + BFP compression + send (~185 us). Serially that was 607
 * us per DL slot at 400 Mbps against a 500 us cadence. With the pipeline, tx_func hands
 * tx_split72 to L1_tx_send and goes on to generate the next slot, so a slot costs
 * max(generate, send) instead of their sum. The grid (txdataF) alternates between two
 * sets so the next slot never writes the one being sent; a set is taken back only once
 * L1_tx_send has finished with it (g_txset_free). L1_tx_send is one thread, so slots
 * still leave in order. Off: tx_func is exactly as before. */
static int g_txp_on = -1;
static void tx_pipe_init(PHY_VARS_gNB *gNB);
static int tx_pipe_take_set(PHY_VARS_gNB *gNB, int frame, int slot);
static void tx_pipe_handoff(PHY_VARS_gNB *gNB, int slot_type, int frame, int slot, int set, uint64_t t_start);

void nr_push_rx_job(PHY_VARS_gNB *gNB, int frame_rx, int slot_rx, uint64_t timestamp_tx)
{
  notifiedFIFO_elt_t *res = newNotifiedFIFO_elt(sizeof(processingData_L1_t), 0, &gNB->resp_L1, NULL);
  processingData_L1_t *syncMsg = NotifiedFifoData(res);
  syncMsg->gNB = gNB;
  syncMsg->frame_rx = frame_rx;
  syncMsg->slot_rx = slot_rx;
  syncMsg->timestamp_tx = timestamp_tx;
  syncMsg->t_dispatch = (uint64_t)clock_gettime_oai();
  res->key = slot_rx;
  pushNotifiedFIFO(&gNB->resp_L1, res);
}

static void tx_func(processingData_L1tx_t *info)
{
  int frame_tx = info->frame;
  int slot_tx = info->slot;
  int frame_rx = info->frame_rx;
  int slot_rx = info->slot_rx;
  LOG_D(NR_PHY, "%d.%d running tx_func\n", frame_tx, slot_tx);
  PHY_VARS_gNB *gNB = info->gNB;
  const uint64_t t_tx0 = (uint64_t)clock_gettime_oai();
  if (g_txp_on < 0)
    tx_pipe_init(gNB);
  const int tx_set = g_txp_on ? tx_pipe_take_set(gNB, frame_tx, slot_tx) : 0;
  NR_IF_Module_t *ifi = gNB->if_inst;
  nfapi_nr_config_request_scf_t *cfg = &gNB->gNB_config;

  T(T_GNB_PHY_DL_TICK, T_INT(gNB->Mod_id), T_INT(frame_tx), T_INT(slot_tx));

  if (slot_rx == 0) {
    reset_active_stats(gNB, frame_rx);
    reset_active_ulsch(gNB, frame_rx);
  }

  clear_slot_beamid(gNB, slot_tx);

  nfapi_nr_slot_indication_scf_t ind = {.header = {.phy_id = 0}, .sfn = frame_tx, .slot = slot_tx};
  start_meas(&gNB->slot_indication_stats);
  // this variable is very big (multiple MB), so we put it into static storage
  // to not overflow the stack while still having it in local (function) scope
  // also, tx_func() is only executed by one thread, serially
  static NR_Sched_Rsp_t sched_response;
  ifi->NR_slot_indication(&ind, &sched_response);
  stop_meas(&gNB->slot_indication_stats);

  info->gNB = gNB;

  // At this point, MAC scheduler just ran, including scheduling
  // PRACH/PUCCH/PUSCH, so trigger RX chain processing
  nr_save_ul_tti_req(gNB, &sched_response.UL_tti_req);
  atomic_store(&g_ul_cfg_abs, frame_tx * gNB->frame_parms.slots_per_frame + slot_tx);
  if (!gNB->fhi72_transport) { /* split 7.2 pushes from split7_thread: see nr_push_rx_job */
    LOG_D(NR_PHY, "Trigger RX for %d.%d\n", frame_rx, slot_rx);
    nr_push_rx_job(gNB, frame_rx, slot_rx, info->timestamp_tx);
  }

  int tx_slot_type = nr_slot_select(cfg, frame_tx, slot_tx);
  if (tx_slot_type & NR_DOWNLINK_SLOT) { // also covers NR_MIXED_SLOT
    START_MEAS_FULL_SLOT(&gNB->phy_proc_tx, tx_slot_type, NR_DOWNLINK_SLOT);
    START_MEAS_FULL_SLOT(&gNB->gnb_tx_procedures_stats, tx_slot_type, NR_DOWNLINK_SLOT);
    phy_procedures_gNB_TX(gNB, &sched_response.DL_req, &sched_response.TX_req, &sched_response.UL_dci_req, frame_tx, slot_tx);
    STOP_MEAS_FULL_SLOT(&gNB->gnb_tx_procedures_stats, tx_slot_type, NR_DOWNLINK_SLOT);
  }

  if (gNB->fhi72_transport) {
    if (g_txp_on) {
      if (tx_slot_type & NR_DOWNLINK_SLOT)
        STOP_MEAS_FULL_SLOT(&gNB->phy_proc_tx, tx_slot_type, NR_DOWNLINK_SLOT); /* generate only */
      tx_pipe_handoff(gNB, tx_slot_type, frame_tx, slot_tx, tx_set, t_tx0);
    } else {
      tx_split72(gNB, tx_slot_type, frame_tx, slot_tx, gNB->common_vars.txdataF, true);
    }
  } else
    tx_split8(gNB, tx_slot_type, frame_tx, slot_tx, info->timestamp_tx);
  if (tx_slot_type & NR_DOWNLINK_SLOT)
    tx_overrun_alarm(gNB, &sched_response.DL_req, frame_tx, slot_tx);
}

void *L1_rx_thread(void *arg) 
{
  PHY_VARS_gNB *gNB = (PHY_VARS_gNB*)arg;

  while (oai_exit == 0) {
     notifiedFIFO_elt_t *res = pullNotifiedFIFO(&gNB->resp_L1);
     if (res == NULL)
       break;
     processingData_L1_t *info = (processingData_L1_t *)NotifiedFifoData(res);
     int slot_type = nr_slot_select(&gNB->gNB_config, info->frame_rx, info->slot_rx);
     START_MEAS_FULL_SLOT(&gNB->l1_rx_proc, slot_type, NR_UPLINK_SLOT);
     rx_func(info);
     STOP_MEAS_FULL_SLOT(&gNB->l1_rx_proc, slot_type, NR_UPLINK_SLOT);
     delNotifiedFIFO_elt(res);
  }
  return NULL;
}
// Added for URLLC, requires MAC scheduling to be split from UL indication
void *L1_tx_thread(void *arg) {
  PHY_VARS_gNB *gNB = (PHY_VARS_gNB*)arg;

  while (oai_exit == 0) {
     notifiedFIFO_elt_t *res = pullNotifiedFIFO(&gNB->L1_tx_out);
     if (res == NULL) // stopping condition, happens only when queue is freed
       break;
     processingData_L1tx_t *info = (processingData_L1tx_t *)NotifiedFifoData(res);
     int slot_type = nr_slot_select(&gNB->gNB_config, info->frame, info->slot);
     START_MEAS_FULL_SLOT(&gNB->l1_tx_proc, slot_type, NR_DOWNLINK_SLOT);
     tx_func(info);
     STOP_MEAS_FULL_SLOT(&gNB->l1_tx_proc, slot_type, NR_DOWNLINK_SLOT);
     delNotifiedFIFO_elt(res);
  }
  return NULL;
}

/* ---- deferred PUSCH back half (OAI_L1_RX_DEFER) -----------------------------
 *
 * Runs the LDPC decode and its rx_data/CRC indications one slot behind, on its
 * own thread, so slot N's decode overlaps slot N+1's front end. Without this the
 * single L1_rx_thread pays the SUM of ARM work and DSP wait; with it, the max.
 * Measured at single layer, 273 PRB: ARM ~2,360 us, DSP decode ~2,180 us, so
 * 4,540 serial against a 2,500 us budget for one U slot in DDDSU -- it does not
 * fit serially and does fit overlapped. That is one ARM core for RX instead of
 * two, and the second is the one we want to give back to TX.
 *
 * ONE thread, deliberately: handle_nr_ulsch pops feedback_ul_harq from the head
 * and matches the incoming harq_pid, so rx_data/CRC must arrive in slot order. A
 * pool would not guarantee that. (RACH, UCI and SRS already go from the front
 * half and are unaffected.)
 *
 * Depth is ONE outstanding slot, enforced by g_defer_slots. That is what makes
 * the two-buffer schemes safe: consecutive RX jobs alternate buffers
 * (nr_l1_rx_buf), so they never share an LLR buffer (nr_pusch_llr_buf) or a UL_INFO. Raising the
 * depth needs more of both, and RU_RX_SLOT_DEPTH = 4 caps it at 3 regardless.
 *
 * UL_INFO lives here rather than on rx_func's stack because it has to outlive the
 * call. It is used in place, never copied -- NR_UL_IND_t holds pointers into
 * itself (crc_list -> crc_pdu_list and so on), so a struct copy would leave them
 * aimed at the original.
 *
 * Known cosmetic aliasing: the back half also reads pusch->ulsch_power[] and
 * ulsch_noise_power[], which the next slot's front half overwrites, so reported
 * power can be one slot stale. Those feed statistics only, so they are not
 * double-buffered.
 *
 * PIPELINED (OAI_L1_RX_PIPE, default on with the deferral): the back half is two
 * stages on two threads. L1_rx_irx runs the inner-RX batches the front half left
 * pending (nr_pusch_irx_finish, the Adreno), then hands the slot on to L1_rx_back,
 * which decodes and indicates. Slot N+1's GPU run then overlaps slot N's decode, so
 * a PUSCH slot costs max(GPU, decode) instead of their sum: at MCS 18, 273 PRB,
 * ~410 us GPU + ~470-550 us decode serial reached 2.5 ms whenever failing blocks
 * ran to max iterations, and the depth-1 credit backed that up into the front half.
 * Two stages need TWO outstanding slots, in two buffer sets. The sets used to be
 * picked by SLOT parity, but consecutive JOBS need not differ in parity: every slot
 * with UL symbols is a job, so in DDDSU the special slot is one too (3, 4, 8, 9, 13,
 * ...), and 9 and 13 share parity 1. Slot 13's front half then cleared g_defer_ul[1]
 * while slot 9 was still indicating, and slot 9's CRC was lost ("HARQ feedback is in
 * the past", ~500 per run on the board). Making same-parity slots serialise stopped
 * that but cost ~1,000 RX ring overwrites a minute at 50 Mbps UL: 13 waited on 9's
 * decode, which queues behind 8's, and every later job waited on 13. So the sets now
 * alternate by JOB (nr_l1_rx_buf): a set's next user is two jobs later, and the depth
 * credit already guarantees that job is done, because both stages complete jobs in
 * order. g_defer_buf (one credit per set) only asserts that: its wait should always
 * be zero, and sem_wait shows it if not. Both FIFOs are single-consumer and
 * FIFO, so rx_data/CRC still leave in slot order. Once stage 1 has run a slot's
 * batch, the nr_pusch_irx_finish calls in nr_pusch_decode_and_indicate are no-ops
 * (ctx is NULL). */
static NR_UL_IND_t    g_defer_ul[2];
static notifiedFIFO_t g_defer_fifo;
static pthread_t      g_defer_thread;
static sem_t          g_defer_slots;  /* depth: 2 pipelined, 1 otherwise */
static sem_t          g_defer_buf[2]; /* one per buffer set (nr_l1_rx_buf): g_defer_ul[], LLR, irx_pend, power */
static int            g_defer_up;
static notifiedFIFO_t g_irx_fifo;
static pthread_t      g_irx_thread;
static int            g_pipe_up;

typedef struct {
  PHY_VARS_gNB *gNB;
  nr_pusch_defer_t job;
  uint64_t t_push;          /* clock_gettime_oai() ns at hand-off, for queue latency */
} defer_msg_t;

/* Per-slot timing for the deferral.  Averages over seconds cannot see a stall
 * that happens once a second -- 10 ms in a 5 s window is 0.2% and vanishes --
 * so keep THRESHOLD COUNTERS and the worst slot's identity instead of a mean.
 * Updated on RT threads, so: no locks, no LOG_*, no floating point in the hot
 * path.  Drained by the 1 Hz stats thread via defer_hist_dump(). */
typedef struct {
  uint64_t n;
  uint64_t sum_ns;
  uint64_t max_ns;
  uint32_t max_frame, max_slot;
  uint64_t over[4];         /* > 0.5 / 1 / 5 / 10 ms */
} defer_hist_t;

static defer_hist_t g_h_semwait;  /* front half blocked handing the slot over */
static defer_hist_t g_h_qlat;     /* push -> pop                              */
static defer_hist_t g_h_work;     /* pop  -> NR_UL_indication returned        */
static defer_hist_t g_h_disp;     /* RX job queued -> a pool worker starts it */
static defer_hist_t g_h_irx;      /* pipelined stage 1: pop -> inner-RX batches done */
static defer_hist_t g_h_cfgwait;  /* split 7.2: RX job waiting for tx_func to store its UL config */
static uint64_t g_cfgwait_timeouts;
static defer_hist_t g_h_txset;  /* tx_func waiting for a free grid set (send stage behind) */
static defer_hist_t g_h_txq;    /* hand-off -> L1_tx_send starts the slot */
static defer_hist_t g_h_txsend; /* copy + C-plane + BFP compression + send */
static defer_hist_t g_h_txe2e;  /* tx_func start -> slot sent */


/* High-water marks on the two queues that are demonstrably FULL when the DU dies:
 * prach_l1rx_queue (xran_fh_rx_prach_read_slot DevAsserts on it) and pucch_queue
 * ("PUCCH list is full: dropping PUCCH UE ..."). Sampled at the top of rx_func,
 * i.e. ~2000/s, which is far finer than the 1 Hz stats file. */
static size_t g_prach_hwm, g_pucch_hwm, g_prach_cap, g_pucch_cap;

static inline void defer_hist_add(defer_hist_t *h, uint64_t ns, int frame, int slot)
{
  h->n++;
  h->sum_ns += ns;
  if (ns > h->max_ns) { h->max_ns = ns; h->max_frame = frame; h->max_slot = slot; }
  static const uint64_t th[4] = {500000ull, 1000000ull, 5000000ull, 10000000ull};
  for (int i = 0; i < 4; i++)
    if (ns > th[i]) h->over[i]++;
}

static size_t defer_hist_one(char *o, size_t n, const char *tag, const defer_hist_t *h)
{
  if (h->n == 0)
    return snprintf(o, n, "  %-10s : no samples\n", tag);
  return snprintf(o, n,
                  "  %-10s : n=%-8llu avg=%7.1f us  max=%9.1f us @%u.%u"
                  "  >0.5ms=%llu >1ms=%llu >5ms=%llu >10ms=%llu\n",
                  tag, (unsigned long long)h->n,
                  (double)h->sum_ns / h->n / 1000.0,
                  (double)h->max_ns / 1000.0, h->max_frame, h->max_slot,
                  (unsigned long long)h->over[0], (unsigned long long)h->over[1],
                  (unsigned long long)h->over[2], (unsigned long long)h->over[3]);
}

static size_t defer_hist_dump(char *o, size_t n)
{
  size_t k = snprintf(o, n, "L1 RX deferral (%s), per-slot:\n", g_pipe_up ? "pipelined, depth 2" : "depth 1");
  k += defer_hist_one(o + k, n - k, "sem_wait", &g_h_semwait);
  if (g_pipe_up)
    k += defer_hist_one(o + k, n - k, "irx_stage", &g_h_irx);
  k += defer_hist_one(o + k, n - k, "queue_lat", &g_h_qlat);
  k += defer_hist_one(o + k, n - k, "back_half", &g_h_work);
  k += defer_hist_one(o + k, n - k, "pool_wait", &g_h_disp);
  k += defer_hist_one(o + k, n - k, "ul_cfg_wait", &g_h_cfgwait);
  if (g_txp_on > 0) {
    k += snprintf(o + k, n - k, "L1 TX pipeline, per-slot:\n");
    k += defer_hist_one(o + k, n - k, "tx_set_wait", &g_h_txset);
    k += defer_hist_one(o + k, n - k, "tx_send_q", &g_h_txq);
    k += defer_hist_one(o + k, n - k, "tx_send", &g_h_txsend);
    k += defer_hist_one(o + k, n - k, "tx_e2e", &g_h_txe2e);
  }
  if (g_cfgwait_timeouts)
    k += snprintf(o + k, n - k, "  ul_cfg_wait timed out %llu times\n", (unsigned long long)g_cfgwait_timeouts);
  k += snprintf(o + k, n - k,
                "  queue_hwm  : prach %zu/%zu   pucch %zu/%zu  (full => the DU dies)\n",
                g_prach_hwm, g_prach_cap, g_pucch_hwm, g_pucch_cap);
  return k;
}

static void *defer_back_half_thread(void *arg)
{
  (void)arg;
  while (oai_exit == 0) {
    notifiedFIFO_elt_t *res = pullNotifiedFIFO(&g_defer_fifo);
    if (res == NULL)
      break;
    defer_msg_t *m = (defer_msg_t *)NotifiedFifoData(res);
    const uint64_t t_pop = (uint64_t)clock_gettime_oai();
    defer_hist_add(&g_h_qlat, t_pop - m->t_push, m->job.frame, m->job.slot);
    NR_UL_IND_t *ul = &g_defer_ul[nr_l1_rx_buf(m->job.slot)];
    nr_pusch_decode_and_indicate(m->gNB, m->job.frame, m->job.slot, m->job.ulsch_idx,
                                 m->job.num_pusch, ul);
    m->gNB->if_inst->NR_UL_indication(ul);
    const uint64_t t_done = (uint64_t)clock_gettime_oai();
    defer_hist_add(&g_h_work, t_done - t_pop, m->job.frame, m->job.slot);
    /* Name the slot for any back half that blew the ring's reuse window, so it can
     * be matched against the "RX slot overwrite" lines, which print frame.slot too.
     * RT_LOG_DEFER claims a slot lock-free; LOG_* here would block this thread. */
    if (t_done - t_pop > 2000000ull)
      RT_LOG_DEFER("L1_rx_back LATE %u.%u: %.0f us (qlat %.0f us)",
                   m->job.frame, m->job.slot,
                   (double)(t_done - t_pop) / 1000.0,
                   (double)(t_pop - m->t_push) / 1000.0);
    const int buf = nr_l1_rx_buf(m->job.slot);
    delNotifiedFIFO_elt(res);
    sem_post(&g_defer_buf[buf]);
    sem_post(&g_defer_slots);
  }
  return NULL;
}

/* Pipelined stage 1: run the slot's pending inner-RX batches, then pass the slot on
 * to the decode stage. The element is re-pushed as is; pull detached it. */
static void *defer_irx_stage_thread(void *arg)
{
  (void)arg;
  while (oai_exit == 0) {
    notifiedFIFO_elt_t *res = pullNotifiedFIFO(&g_irx_fifo);
    if (res == NULL)
      break;
    defer_msg_t *m = (defer_msg_t *)NotifiedFifoData(res);
    const uint64_t t0 = (uint64_t)clock_gettime_oai();
    for (int i = 0; i < m->job.num_pusch; i++)
      nr_pusch_irx_finish(m->gNB, &m->gNB->pusch_vars[m->job.ulsch_idx[i]], m->job.frame, m->job.slot, true);
    const uint64_t t1 = (uint64_t)clock_gettime_oai();
    defer_hist_add(&g_h_irx, t1 - t0, m->job.frame, m->job.slot);
    m->t_push = t1; /* queue_lat of the decode stage = wait between the stages */
    pushNotifiedFIFO(&g_defer_fifo, res);
  }
  return NULL;
}

static int l1_rx_pipe_enabled(void)
{
  const char *e = getenv("OAI_L1_RX_PIPE");
  return !(e && e[0] == '0');
}

static void defer_back_half_start(void)
{
  if (g_defer_up)
    return;
  initNotifiedFIFO(&g_defer_fifo);
  const int pipe = l1_rx_pipe_enabled();
  /* one credit == one outstanding slot; the pipeline holds one per stage */
  sem_init(&g_defer_slots, 0, pipe ? 2 : 1);
  sem_init(&g_defer_buf[0], 0, 1);
  sem_init(&g_defer_buf[1], 0, 1);
  if (pthread_create(&g_defer_thread, NULL, defer_back_half_thread, NULL) != 0) {
    LOG_E(NR_PHY, "L1 RX defer: pthread_create failed, staying synchronous\n");
    return;
  }
  pthread_setname_np(g_defer_thread, "L1_rx_back");
  if (pipe) {
    initNotifiedFIFO(&g_irx_fifo);
    if (pthread_create(&g_irx_thread, NULL, defer_irx_stage_thread, NULL) == 0) {
      pthread_setname_np(g_irx_thread, "L1_rx_irx");
      /* Inherits L1_rx_thread's RT priority and core; both stages mostly sleep (GPU
       * completion, CDSP sleep-then-poll), so sharing is the default. OAI_L1_RX_IRX_CORE
       * moves the GPU stage, for experiments. */
      const char *c = getenv("OAI_L1_RX_IRX_CORE");
      if (c && c[0]) {
        cpu_set_t cs;
        CPU_ZERO(&cs);
        CPU_SET(atoi(c), &cs);
        if (pthread_setaffinity_np(g_irx_thread, sizeof(cs), &cs) != 0)
          LOG_E(NR_PHY, "L1 RX pipeline: cannot pin L1_rx_irx to core %s\n", c);
      }
      g_pipe_up = 1;
    } else {
      /* Fall back to one stage. The second credit has to go, or two slots would share
       * the decode thread with nothing to keep their parity buffers apart. */
      LOG_E(NR_PHY, "L1 RX pipeline: pthread_create failed, one back-half stage (depth 1)\n");
      sem_wait(&g_defer_slots);
    }
  }
  g_defer_up = 1;
  LOG_I(NR_PHY, "L1 RX back half DEFERRED onto its own thread%s\n", g_pipe_up ? "s (pipelined: inner RX, then decode; depth 2)" : " (depth 1)");
}

/* ---- L1 TX pipeline (see g_txp_on) ---- */
typedef struct {
  PHY_VARS_gNB *gNB;
  int slot_type, frame, slot, set;
  uint64_t t_start, t_push; /* tx_func start, hand-off */
} tx_send_msg_t;
static notifiedFIFO_t g_tx_send_fifo;
static sem_t g_txset_free[2];
static pthread_t g_tx_send_thread;

static void *tx_send_thread(void *arg)
{
  (void)arg;
  /* No oai_exit check: tx_func may be parked on g_txset_free, so this must keep
   * draining until the process goes. */
  for (;;) {
    notifiedFIFO_elt_t *res = pullNotifiedFIFO(&g_tx_send_fifo);
    if (res == NULL)
      break;
    tx_send_msg_t *m = (tx_send_msg_t *)NotifiedFifoData(res);
    const uint64_t t0 = (uint64_t)clock_gettime_oai();
    defer_hist_add(&g_h_txq, t0 - m->t_push, m->frame, m->slot);
    tx_split72(m->gNB, m->slot_type, m->frame, m->slot, m->gNB->common_vars.txdataF_set[m->set], false);
    const uint64_t t1 = (uint64_t)clock_gettime_oai();
    defer_hist_add(&g_h_txsend, t1 - t0, m->frame, m->slot);
    defer_hist_add(&g_h_txe2e, t1 - m->t_start, m->frame, m->slot);
    const int set = m->set;
    delNotifiedFIFO_elt(res);
    sem_post(&g_txset_free[set]);
  }
  return NULL;
}

static void tx_pipe_init(PHY_VARS_gNB *gNB)
{
  const char *e = getenv("OAI_L1_TX_PIPE");
  g_txp_on = (e && e[0] && e[0] != '0' && gNB->fhi72_transport) ? 1 : 0;
  if (!g_txp_on)
    return;
  initNotifiedFIFO(&g_tx_send_fifo);
  sem_init(&g_txset_free[0], 0, 1);
  sem_init(&g_txset_free[1], 0, 1);
  const char *c = getenv("OAI_L1_TX_PIPE_CORE");
  const int core = (c && c[0]) ? atoi(c) : 0;
  threadCreate(&g_tx_send_thread, tx_send_thread, NULL, "L1_tx_send", core, OAI_PRIORITY_RT_MAX);
  LOG_I(NR_PHY, "L1 TX pipeline ON: generate on L1_tx_thread, copy+compress+send on L1_tx_send (core %d)\n", core);
}

static int tx_pipe_take_set(PHY_VARS_gNB *gNB, int frame, int slot)
{
  static unsigned seq;
  const int set = seq++ & 1;
  const uint64_t t0 = (uint64_t)clock_gettime_oai();
  sem_wait(&g_txset_free[set]);
  defer_hist_add(&g_h_txset, (uint64_t)clock_gettime_oai() - t0, frame, slot);
  gNB->common_vars.txdataF = gNB->common_vars.txdataF_set[set]; /* everything that generates reads this */
  return set;
}

static void tx_pipe_handoff(PHY_VARS_gNB *gNB, int slot_type, int frame, int slot, int set, uint64_t t_start)
{
  notifiedFIFO_elt_t *e = newNotifiedFIFO_elt(sizeof(tx_send_msg_t), 0, &g_tx_send_fifo, NULL);
  tx_send_msg_t *m = (tx_send_msg_t *)NotifiedFifoData(e);
  *m = (tx_send_msg_t){.gNB = gNB, .slot_type = slot_type, .frame = frame, .slot = slot, .set = set,
                       .t_start = t_start, .t_push = (uint64_t)clock_gettime_oai()};
  pushNotifiedFIFO(&g_tx_send_fifo, e);
}

/* Split 7.2 only (see nr_push_rx_job): wait until tx_func has stored the UL config for
 * (frame, slot). Normally already true by sl_ahead slots. Bounded: past the bound the
 * slot is processed with whatever is stored (its PUSCH/PUCCH then read as DTX), since
 * holding the ring longer would only destroy the following slots too. */
static void rx_wait_ul_cfg(PHY_VARS_gNB *gNB, int frame, int slot)
{
  const int spf = gNB->frame_parms.slots_per_frame;
  const int mod = 1024 * spf;
  const int want = frame * spf + slot;
  const uint64_t t0 = (uint64_t)clock_gettime_oai();
  for (;;) {
    const int have = atomic_load(&g_ul_cfg_abs);
    if (have >= 0 && (((have - want) % mod) + mod) % mod < mod / 2)
      break;
    if ((uint64_t)clock_gettime_oai() - t0 > 2000000ull) {
      g_cfgwait_timeouts++;
      break;
    }
    usleep(20);
  }
  defer_hist_add(&g_h_cfgwait, (uint64_t)clock_gettime_oai() - t0, frame, slot);
}

static void rx_func(processingData_L1_t *info)
{
  PHY_VARS_gNB *gNB = info->gNB;
  int frame_rx = info->frame_rx;
  int slot_rx = info->slot_rx;
  nfapi_nr_config_request_scf_t *cfg = &gNB->gNB_config;

  if (info->t_dispatch)
    defer_hist_add(&g_h_disp, (uint64_t)clock_gettime_oai() - info->t_dispatch, frame_rx, slot_rx);
  if (gNB->fhi72_transport)
    rx_wait_ul_cfg(gNB, frame_rx, slot_rx);
  {
    const size_t pc = spsc_q_count(&gNB->prach_l1rx_queue);
    const size_t uc = spsc_q_count(&gNB->pucch_queue);
    if (pc > g_prach_hwm) g_prach_hwm = pc;
    if (uc > g_pucch_hwm) g_pucch_hwm = uc;
    g_prach_cap = spsc_q_capacity(&gNB->prach_l1rx_queue);
    g_pucch_cap = spsc_q_capacity(&gNB->pucch_queue);
  }

  T(T_GNB_PHY_UL_TICK, T_INT(gNB->Mod_id), T_INT(frame_rx), T_INT(slot_rx));

  // RX processing
  int rx_slot_type = nr_slot_select(cfg, frame_rx, slot_rx);
  if (rx_slot_type == NR_UPLINK_SLOT || rx_slot_type == NR_MIXED_SLOT) {
    LOG_D(NR_PHY, "%d.%d Starting RX processing\n", frame_rx, slot_rx);

    // UE-specific RX processing for subframe n
    const int defer = nr_l1_rx_defer_enabled();
    if (defer)
      defer_back_half_start();
    nr_l1_rx_buf_assign(slot_rx); /* before anything below looks the buffer up */
    /* Deferred, UL_INFO must outlive this call, so it lives in this job's buffer set.
     * Wait for a depth credit first, then for the set's own credit (which should never
     * wait: see above): the back half has then finished with the set's UL_INFO and LLRs. */
    NR_UL_IND_t UL_INFO_local;
    NR_UL_IND_t *ULI;
    if (defer && g_defer_up) {
      /* This is the stall that matters: while parked here the front half is not
       * releasing rxdataF ring slots, so the fronthaul writer laps it. */
      const uint64_t t_w0 = (uint64_t)clock_gettime_oai();
      sem_wait(&g_defer_slots);
      sem_wait(&g_defer_buf[nr_l1_rx_buf(slot_rx)]); /* always after g_defer_slots: one order, no deadlock */
      defer_hist_add(&g_h_semwait, (uint64_t)clock_gettime_oai() - t_w0, frame_rx, slot_rx);
      ULI = &g_defer_ul[nr_l1_rx_buf(slot_rx)];
    } else {
      ULI = &UL_INFO_local;
    }
    /* Upstream dropped this clear ("each indication fills only its used PDUs").
     * Kept here for now because the parity ring REUSES a buffer every second slot,
     * so an indication that leaves a stale count behind would be read as this
     * slot's. Revisit once the per-indication counts are confirmed exhaustive. */
    memset(ULI, 0, sizeof(*ULI));
    ULI->frame = frame_rx;
    ULI->slot = slot_rx;
    ULI->module_id = gNB->Mod_id;
    ULI->phy_id = 0;
#define UL_INFO (*ULI)
    // Do PRACH RU processing
    UL_INFO.rach_ind.header.transmit_timestamp = 0;
    UL_INFO.rach_ind.pdu_list = UL_INFO.prach_pdu_indication_list;
    UL_INFO.rach_ind.number_of_pdus = 0;
    // even if processing is late, we might collect all PRACH
    // the last PRACH's frame/slot is when all UE's appear to have accessed
    prach_item_t p;
    while (spsc_q_get(&gNB->prach_l1rx_queue, &p, sizeof(p)))
      L1_nr_prach_procedures(gNB, &p, &UL_INFO.rach_ind);

    // apply the rx signal rotation here
    if (gNB->phase_comp) {
      const NR_DL_FRAME_PARMS *fp = &gNB->frame_parms;
      const uint soffset = (slot_rx % RU_RX_SLOT_DEPTH) * fp->symbols_per_slot * fp->ofdm_symbol_size;
      for (int aa = 0; aa < fp->nb_antennas_rx; aa++) {
        for (int sym = 0; sym < fp->symbols_per_slot; sym++) {
          c16_t *this_symbol = gNB->common_vars.rxdataF[aa] + soffset + sym * fp->ofdm_symbol_size;
          apply_nr_rotation_symbol_fftshifted_RX(fp->symbols_per_slot,
                                                 fp->slots_per_subframe,
                                                 fp->timeshift_symbol_rotation,
                                                 this_symbol,
                                                 fp->symbol_rotation[1],
                                                 fp->N_RB_UL,
                                                 slot_rx,
                                                 sym);
        }
      }
    }
    nr_pusch_defer_t job = {0};
    phy_procedures_gNB_uespec_RX_defer(gNB, frame_rx, slot_rx, &UL_INFO,
                                       (defer && g_defer_up) ? &job : NULL);

    if (defer && g_defer_up) {
      /* Hand the back half over and return, so this thread can start slot N+1
       * while the decode for slot N runs. The credit is released by the back-half
       * thread once it has delivered rx_data/CRC. */
      notifiedFIFO_elt_t *e = newNotifiedFIFO_elt(sizeof(defer_msg_t), 0, &g_defer_fifo, NULL);
      defer_msg_t *m = (defer_msg_t *)NotifiedFifoData(e);
      m->gNB = gNB;
      m->job = job;
      m->t_push = (uint64_t)clock_gettime_oai();
      pushNotifiedFIFO(g_pipe_up ? &g_irx_fifo : &g_defer_fifo, e);
    } else {
      // Call the scheduler
      START_MEAS_FULL_SLOT(&gNB->ul_indication_stats, rx_slot_type, NR_UPLINK_SLOT);
      gNB->if_inst->NR_UL_indication(&UL_INFO);
      STOP_MEAS_FULL_SLOT(&gNB->ul_indication_stats, rx_slot_type, NR_UPLINK_SLOT);
    }

    /* Signal that this slot's rxdataF is free.  RFSIM needs it because wait_free_rx_tti()
     * blocks on it; on real radios nothing blocks, but ru_thread still drains this queue to
     * notice when it is about to overwrite a slot L1 has not finished with -- which
     * destroys the samples and shows up as an unexplained L1 DTX, not as any kind of
     * fronthaul error.  ru_thread MUST keep draining this or the queue grows without
     * bound. */
    {
      notifiedFIFO_elt_t *res = newNotifiedFIFO_elt(sizeof(processingData_L1_t), 0, &gNB->L1_rx_out, NULL);
      processingData_L1_t *syncMsg = NotifiedFifoData(res);
      syncMsg->gNB = gNB;
      syncMsg->frame_rx = frame_rx;
      syncMsg->slot_rx = slot_rx;
      /* Stamped HERE, on the L1 thread, because this is the instant L1 stops needing
       * this slot's rxdataF.  ru_thread only drains this queue on the next UL slot, so
       * reading the clock at drain time would add up to one UL-slot period of slack and
       * hide exactly the effect we are chasing. */
      syncMsg->t_release = (uint64_t)clock_gettime_oai();
      res->key = slot_rx;
      LOG_D(NR_PHY, "Signaling completion for %d.%d (mod_slot %d) on L1_rx_out\n", frame_rx, slot_rx, slot_rx % RU_RX_SLOT_DEPTH);
      pushNotifiedFIFO(&gNB->L1_rx_out, res);
    }
#undef UL_INFO
  }

}

static void nrL1_stats_init_sorted_list(PHY_VARS_gNB *gNB, RU_t *ru, unsigned int size)
{
  init_sorted_list_meas(&gNB->l1_tx_proc, size);
  init_sorted_list_meas(&gNB->l1_rx_proc, size);
  init_sorted_list_meas(&gNB->phy_proc_tx, size);
  init_sorted_list_meas(&gNB->gnb_tx_procedures_stats, size);
  init_sorted_list_meas(&gNB->dlsch_encoding_stats, size);
  init_sorted_list_meas(&gNB->dlsch_ldpc_encode_stats, size);
  init_sorted_list_meas(&gNB->dlsch_scrambling_stats, size);
  init_sorted_list_meas(&gNB->dlsch_modulation_stats, size);
  init_sorted_list_meas(&gNB->dlsch_pdsch_generation_stats, size);
  init_sorted_list_meas(&gNB->phy_proc_rx, size);
  init_sorted_list_meas(&gNB->pucch01_proc_rx, size);
  init_sorted_list_meas(&gNB->pucch23_proc_rx, size);
  init_sorted_list_meas(&gNB->ulsch_decoding_stats, size);
  init_sorted_list_meas(&gNB->ts_ldpc_decode, size);
  init_sorted_list_meas(&gNB->ul_indication_stats, size);
  init_sorted_list_meas(&gNB->slot_indication_stats, size);
  init_sorted_list_meas(&gNB->rx_pusch_stats, size);
  init_sorted_list_meas(&gNB->rx_pusch_irx_deferred_stats, size);
  init_sorted_list_meas(&gNB->rx_prach, size);
  if (gNB->fhi72_transport) {
    init_sorted_list_meas(&gNB->fhi72_transport->s7cfg.prec_bf, size);
    init_sorted_list_meas(&gNB->fhi72_transport->s7cfg.dl_cp, size);
    init_sorted_list_meas(&gNB->fhi72_transport->s7cfg.dl_up, size);
    init_sorted_list_meas(&gNB->fhi72_transport->s7cfg.ul_cp, size);
    init_sorted_list_meas(&gNB->fhi72_transport->s7cfg.ul_up, size);
  } else {
    init_sorted_list_meas(&ru->ofdm_demod_stats, size);
    init_sorted_list_meas(&ru->precoding_stats, size);
    init_sorted_list_meas(&ru->txdataF_copy_stats, size);
    init_sorted_list_meas(&ru->ofdm_mod_stats, size);
    init_sorted_list_meas(&ru->ofdm_total_stats, size);
    init_sorted_list_meas(&ru->tx_fhaul, size);
    init_sorted_list_meas(&ru->rx_fhaul, size);
  }
}

static void nrL1_stats_free_sorted_list(PHY_VARS_gNB *gNB, RU_t *ru)
{
  free_sorted_list_meas(&gNB->l1_tx_proc);
  free_sorted_list_meas(&gNB->l1_rx_proc);
  free_sorted_list_meas(&gNB->phy_proc_tx);
  free_sorted_list_meas(&gNB->gnb_tx_procedures_stats);
  free_sorted_list_meas(&gNB->dlsch_encoding_stats);
  free_sorted_list_meas(&gNB->dlsch_ldpc_encode_stats);
  free_sorted_list_meas(&gNB->dlsch_scrambling_stats);
  free_sorted_list_meas(&gNB->dlsch_modulation_stats);
  free_sorted_list_meas(&gNB->dlsch_pdsch_generation_stats);
  free_sorted_list_meas(&gNB->phy_proc_rx);
  free_sorted_list_meas(&gNB->pucch01_proc_rx);
  free_sorted_list_meas(&gNB->pucch23_proc_rx);
  free_sorted_list_meas(&gNB->ulsch_decoding_stats);
  free_sorted_list_meas(&gNB->ts_ldpc_decode);
  free_sorted_list_meas(&gNB->ul_indication_stats);
  free_sorted_list_meas(&gNB->slot_indication_stats);
  free_sorted_list_meas(&gNB->rx_pusch_stats);
  free_sorted_list_meas(&gNB->rx_pusch_irx_deferred_stats);
  free_sorted_list_meas(&gNB->rx_prach);
  if (gNB->fhi72_transport) {
    free_sorted_list_meas(&gNB->fhi72_transport->s7cfg.prec_bf);
    free_sorted_list_meas(&gNB->fhi72_transport->s7cfg.dl_cp);
    free_sorted_list_meas(&gNB->fhi72_transport->s7cfg.dl_up);
    free_sorted_list_meas(&gNB->fhi72_transport->s7cfg.ul_cp);
    free_sorted_list_meas(&gNB->fhi72_transport->s7cfg.ul_up);
  } else {
    free_sorted_list_meas(&ru->ofdm_demod_stats);
    free_sorted_list_meas(&ru->precoding_stats);
    free_sorted_list_meas(&ru->txdataF_copy_stats);
    free_sorted_list_meas(&ru->ofdm_mod_stats);
    free_sorted_list_meas(&ru->ofdm_total_stats);
    free_sorted_list_meas(&ru->txdataF_copy_stats);
    free_sorted_list_meas(&ru->tx_fhaul);
    free_sorted_list_meas(&ru->rx_fhaul);
  }
}

static void nrL1_stats_reset(PHY_VARS_gNB *gNB, RU_t *ru)
{
  reset_meas(&gNB->l1_tx_proc);
  reset_meas(&gNB->l1_rx_proc);
  reset_meas(&gNB->phy_proc_tx);
  reset_meas(&gNB->gnb_tx_procedures_stats);
  reset_meas(&gNB->dlsch_encoding_stats);
  reset_meas(&gNB->dlsch_ldpc_encode_stats);
  reset_meas(&gNB->dlsch_scrambling_stats);
  reset_meas(&gNB->dlsch_modulation_stats);
  reset_meas(&gNB->dlsch_resource_mapping_stats);
  reset_meas(&gNB->dlsch_pdsch_generation_stats);
  reset_meas(&gNB->phy_proc_rx);
  reset_meas(&gNB->pucch01_proc_rx);
  reset_meas(&gNB->pucch23_proc_rx);
  reset_meas(&gNB->ulsch_decoding_stats);
  reset_meas(&gNB->ts_ldpc_decode);
  reset_meas(&gNB->ul_indication_stats);
  reset_meas(&gNB->slot_indication_stats);
  reset_meas(&gNB->rx_pusch_stats);
  reset_meas(&gNB->rx_pusch_irx_deferred_stats);
  reset_meas(&gNB->rx_prach);
  if (gNB->fhi72_transport) {
    reset_meas(&gNB->fhi72_transport->s7cfg.prec_bf);
    reset_meas(&gNB->fhi72_transport->s7cfg.dl_cp);
    reset_meas(&gNB->fhi72_transport->s7cfg.dl_up);
    reset_meas(&gNB->fhi72_transport->s7cfg.ul_cp);
    reset_meas(&gNB->fhi72_transport->s7cfg.ul_up);
  } else {
    reset_meas(&ru->precoding_stats);
    reset_meas(&ru->ofdm_demod_stats);
    reset_meas(&ru->txdataF_copy_stats);
    reset_meas(&ru->ofdm_mod_stats);
    reset_meas(&ru->ofdm_total_stats);
    reset_meas(&ru->txdataF_copy_stats);
    reset_meas(&ru->tx_fhaul);
    reset_meas(&ru->rx_fhaul);
  }
}

static size_t dump_L1_meas_stats(PHY_VARS_gNB *gNB, RU_t *ru, char *output, size_t outputlen) {
  const char *begin = output;
  const char *end = output + outputlen;
  output += print_meas_log_header(NULL, NULL, output, end - output, cpu_meas_enabled);
  output += print_meas_log(&gNB->l1_tx_proc, "L1 Tx job", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->l1_rx_proc, "L1 Rx job", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->phy_proc_tx, "L1 Tx processing", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->gnb_tx_procedures_stats,
                           "L1 gNB TX procedures",
                           NULL,
                           NULL,
                           output,
                           end - output);

  output += print_meas_log(&gNB->dlsch_encoding_stats, "DLSCH encoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_ldpc_encode_stats, "LDPC encoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_layer_clear_stats, "DLSCH tx_layers clear", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_scrambling_stats, "DLSCH scrambling", NULL, NULL, output, end-output);
  output += print_meas_log(&gNB->dlsch_modulation_stats, "DLSCH modulation", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_pdsch_generation_stats, "PDSCH generation", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->phy_proc_rx, "L1 Rx processing", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pucch01_proc_rx, "PUCCH01 reception", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pucch23_proc_rx, "PUCCH23 reception", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ulsch_decoding_stats, "ULSCH decoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ts_ldpc_decode, "UL segments decoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ul_indication_stats, "UL Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->slot_indication_stats, "Slot Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_pusch_stats, "PUSCH inner-receiver", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_pusch_irx_deferred_stats, "PUSCH inner-RX offload (back half)", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_prach, "PRACH RX", NULL, NULL, output, end - output);

  if (gNB->fhi72_transport) {
    output += print_meas_log(&gNB->fhi72_transport->s7cfg.prec_bf,"prec_bf",NULL,NULL, output, end - output);
    output += print_meas_log(&gNB->fhi72_transport->s7cfg.dl_cp,"dl_cp",NULL,NULL, output, end - output);
    output += print_meas_log(&gNB->fhi72_transport->s7cfg.dl_up,"dl_up",NULL,NULL, output, end - output);
    output += print_meas_log(&gNB->fhi72_transport->s7cfg.ul_cp,"ul_cp",NULL,NULL, output, end - output);
    output += print_meas_log(&gNB->fhi72_transport->s7cfg.ul_up,"ul_up",NULL,NULL, output, end - output);
  } else {
    bool full_slot = ru->half_slot_parallelization == 0;
    output += print_meas_log(&ru->ofdm_demod_stats, "feprx", NULL, NULL, output, end - output);
    output += print_meas_log(&ru->precoding_stats,
                             full_slot ? "feptx_prec (per port)" : "feptx_prec (per port, half_slot)",
                             NULL,
                             NULL,
                             output,
                             end - output);
    output += print_meas_log(&ru->txdataF_copy_stats,"txdataF_copy",NULL,NULL, output, end - output);
    output += print_meas_log(&ru->ofdm_mod_stats,
                             full_slot ? "feptx_ofdm (per port)" : "feptx_ofdm (per port, half_slot)",
                             NULL,
                             NULL,
                             output,
                             end - output);
    output += print_meas_log(&ru->ofdm_total_stats,"feptx_total",NULL,NULL, output, end - output);
    output += print_meas_log(&ru->txdataF_copy_stats, "txdataF_copy", NULL, NULL, output, end - output);

    output += print_meas_log(&ru->tx_fhaul,"tx_fhaul",NULL,NULL, output, end - output);
    output += print_meas_log(&ru->rx_fhaul,"rx_fhaul",NULL,NULL, output, end - output);
  }

  if (cpu_meas_enabled == TIME_STATS_ADVANCED_MODE) {
    nrL1_stats_reset(gNB, ru);
  }
  return output - begin;
}

#define SORTED_LIST_SIZE 2048
void *nrL1_stats_thread(void *param) {
  PHY_VARS_gNB     *gNB      = (PHY_VARS_gNB *)param;
  RU_t *ru = RC.ru[0];
  char output[L1STATSSTRLEN];
  memset(output,0,L1STATSSTRLEN);
  FILE *fd=fopen("nrL1_stats.log","w");
  if (!fd) {
    LOG_W(NR_PHY, "Cannot open nrL1_stats.log: %d, %s\n", errno, strerror(errno));
    return NULL;
  }

  if (cpu_meas_enabled == TIME_STATS_ADVANCED_MODE) {
    nrL1_stats_init_sorted_list(gNB, ru, SORTED_LIST_SIZE);
  }

  nrL1_stats_reset(gNB, ru);

  while (!oai_exit) {
    sleep(1);
    if (ftruncate(fileno(fd), 0) != 0 || fseek(fd, 0, SEEK_SET) != 0) {
      LOG_E(NR_MAC, "error while writing nrL1_stats.log: %d, %s\n", errno, strerror(errno));
      break;
    }
    dump_nr_I0_stats(fd,gNB);
    dump_pdsch_stats(fd,gNB);
    dump_pusch_stats(fd,gNB);
    dump_L1_meas_stats(gNB, ru, output, L1STATSSTRLEN);
    fprintf(fd,"%s\n",output);
    {
      char dbuf[2048];
      defer_hist_dump(dbuf, sizeof dbuf);
      fprintf(fd, "%s", dbuf);
      ru_hist_dump(dbuf, sizeof dbuf);
      fprintf(fd, "%s", dbuf);
    }
    fflush(fd);
    /* Anything ru_thread deferred: it drives fronthaul timing, and OAI's log path ends in a
       blocking write(2) on the CALLING thread, so it must never write. Drained here, on the
       1 Hz stats thread, which is unpinned and non-RT. */
    rt_log_drain();
  }

  if (cpu_meas_enabled == TIME_STATS_ADVANCED_MODE) {
    nrL1_stats_free_sorted_list(gNB, ru);
  }

  fclose(fd);
  return(NULL);
}

void init_gNB_Tpool(int inst)
{
  AssertFatal(NFAPI_MODE == NFAPI_MODE_PNF || NFAPI_MODE == NFAPI_MONOLITHIC,
              "illegal NFAPI_MODE %d (%s): it cannot have an L1\n",
              NFAPI_MODE,
              nfapi_get_strmode());

  PHY_VARS_gNB *gNB;
  gNB = RC.gNB[inst];
  gNB_L1_proc_t *proc = &gNB->proc;
  // ULSCH decoding threadpool
  initTpool(get_softmodem_params()->threadPoolConfig, &gNB->threadPool, cpumeas(CPUMEAS_GETSTATE));

  // L1 RX result FIFO
  initNotifiedFIFO(&gNB->resp_L1);
  // L1 TX result FIFO
  initNotifiedFIFO(&gNB->L1_tx_out);
  initNotifiedFIFO(&gNB->L1_rx_out);

  // create the RX thread responsible for RX processing start event (resp_L1 msg queue), then launch rx_func()
  threadCreate(&gNB->L1_rx_thread, L1_rx_thread, (void *)gNB, "L1_rx_thread", gNB->L1_rx_thread_core, OAI_PRIORITY_RT_MAX);
  // create the TX thread responsible for TX processing start event (L1_tx_out msg queue), then launch tx_func()
  threadCreate(&gNB->L1_tx_thread, L1_tx_thread, (void *)gNB, "L1_tx_thread", gNB->L1_tx_thread_core, OAI_PRIORITY_RT_MAX);

  if (!IS_SOFTMODEM_NOSTATS)
    threadCreate(&proc->L1_stats_thread, nrL1_stats_thread, (void *)gNB, "L1_stats", -1, OAI_PRIORITY_RT_LOW);
}

void term_gNB_Tpool(int inst) {
  PHY_VARS_gNB *gNB = RC.gNB[inst];
  abortNotifiedFIFO(&gNB->resp_L1);
  pthread_join(gNB->L1_rx_thread, NULL);
  abortNotifiedFIFO(&gNB->L1_tx_out);
  pthread_join(gNB->L1_tx_thread, NULL);
  // after the L1 RX thread (its producer), before split7_thread, which may block in wait_free_rx_tti()
  abortNotifiedFIFO(&gNB->L1_rx_out);

  // the stats thread reads fhi72_transport->s7cfg
  pthread_join(gNB->proc.L1_stats_thread, NULL);

  if (gNB->fhi72_transport) {
    pthread_join(gNB->fhi72_transport->pthread, NULL);
    free(gNB->fhi72_transport);
    LOG_I(NR_PHY, "O-RAN 7.2 fronthaul thread removed\n");
  }

  abortTpool(&gNB->threadPool);
}

/// eNB kept in function name for nffapi calls, TO FIX
void init_eNB_afterRU(void)
{
  for (int inst = 0; inst < RC.nb_nr_L1_inst; inst++) {
    PHY_VARS_gNB *gNB = RC.gNB[inst];
    phy_init_nr_gNB(gNB);

    // map antennas and PRACH signals to gNB RX
    if (0) AssertFatal(gNB->num_RU>0,"Number of RU attached to gNB %d is zero\n",gNB->Mod_id);

    LOG_D(NR_PHY, "Mapping RX ports from %d RUs to gNB %d\n", gNB->num_RU, gNB->Mod_id);
    int aa = 0;
    for (int ru_id = 0; ru_id < gNB->num_RU; ru_id++) {
      AssertFatal(gNB->RU_list[ru_id]->common.rxdataF != NULL, "RU %d : common.rxdataF is NULL\n", gNB->RU_list[ru_id]->idx);
      for (int i = 0; i < gNB->RU_list[ru_id]->nb_rx; aa++, i++) {
        LOG_I(PHY, "Attaching RU %d antenna %d to gNB antenna %d\n", gNB->RU_list[ru_id]->idx, i, aa);
        gNB->common_vars.rxdataF[aa] = (c16_t *)gNB->RU_list[ru_id]->common.rxdataF[i];
      }
    }
    /* TODO: review this code, there is something wrong.
     * In monolithic mode, we come here with nb_antennas_rx == 0
     * (not tested in other modes).
     */
    //init_precoding_weights(RC.gNB[inst]);
    init_gNB_Tpool(inst);
  }
}

/**
 * @brief Initialize gNB struct in RAN context
 */
void init_gNB()
{
  LOG_I(NR_PHY, "Initializing gNB RAN context: RC.nb_nr_L1_inst = %d \n", RC.nb_nr_L1_inst);
  if (RC.gNB == NULL) {
    RC.gNB = (PHY_VARS_gNB **)calloc_or_fail(RC.nb_nr_L1_inst, sizeof(PHY_VARS_gNB *));
    LOG_D(NR_PHY, "gNB L1 structure RC.gNB allocated @ %p\n", RC.gNB);
  }

  for (int inst = 0; inst < RC.nb_nr_L1_inst; inst++) {
    // Allocate L1 instance
    if (RC.gNB[inst] == NULL) {
      RC.gNB[inst] = (PHY_VARS_gNB *)calloc_or_fail(1, sizeof(PHY_VARS_gNB));
      LOG_D(NR_PHY, "[nr-gnb.c] gNB structure RC.gNB[%d] allocated @ %p\n", inst, RC.gNB[inst]);
    }
    PHY_VARS_gNB *gNB = RC.gNB[inst];
    LOG_D(NR_PHY, "Initializing gNB %d\n", inst);

    // Init module ID
    gNB->Mod_id = inst;

    // Register MAC interface module
    AssertFatal((gNB->if_inst = NR_IF_Module_init(inst)) != NULL, "Cannot register interface");

    LOG_I(NR_PHY, "Registered with MAC interface module (%p)\n", gNB->if_inst);
    gNB->if_inst->NR_PHY_config_req = nr_phy_config_request;

    gNB->prach_energy_counter = 0;
    gNB->chest_time = get_softmodem_params()->chest_time;
    gNB->chest_freq = get_softmodem_params()->chest_freq;
  }
}

void stop_gNB(int nb_inst) {
  for (int inst=0; inst<nb_inst; inst++) {
    LOG_I(PHY,"Killing gNB %d processing threads\n",inst);
    term_gNB_Tpool(inst);
  }
}
