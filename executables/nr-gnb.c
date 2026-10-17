/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief Top-level threads for gNodeB
 */

#define _GNU_SOURCE
#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all

#include <errno.h>
#include <fcntl.h> // for SEEK_SET
#include <pthread.h> // for pthread_join
#include <semaphore.h>
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

#define TICK_TO_US(ts) (ts.trials==0?0:ts.diff/ts.trials)
#define L1STATSSTRLEN 16384
static void rx_func(processingData_L1_t *param);

/* Alarm ring producer/consumer.  See l1_alarm_ring_t in defs_gNB.h for why the RT threads
   must not format or write anything themselves. */
static inline l1_alarm_rec_t *l1_alarm_claim(l1_alarm_ring_t *r)
{
  return &r->rec[atomic_load_explicit(&r->seq, memory_order_relaxed) & (L1_ALARM_RING - 1)];
}

static inline void l1_alarm_publish(l1_alarm_ring_t *r)
{
  atomic_store_explicit(&r->seq, atomic_load_explicit(&r->seq, memory_order_relaxed) + 1, memory_order_release);
}

/* Drained from the stats thread, which is unpinned and non-RT: the write(2) inside LOG_W
   is harmless there. */
/* Print the ULSCH CRC failures recorded since the last call; see ul_fail_ring_t */
static void ul_fail_drain(ul_fail_ring_t *r)
{
  const uint64_t head = atomic_load_explicit(&r->claim, memory_order_acquire);
  if (head - r->drained > UL_FAIL_RING) {
    LOG_W(NR_PHY, "UL CRC fail: %llu record(s) lost, ring holds %u\n", (unsigned long long)(head - r->drained - UL_FAIL_RING), UL_FAIL_RING);
    r->drained = head - UL_FAIL_RING;
  }
  while (r->drained < head) {
    const ul_fail_rec_t *e = &r->rec[r->drained % UL_FAIL_RING];
    if (atomic_load_explicit(&e->tag, memory_order_acquire) != r->drained + 1)
      break; // still being written; next call
    const double dt_ms = r->last_t_ns ? (double)(e->t_ns - r->last_t_ns) / 1e6 : 0;
    r->last_t_ns = e->t_ns;
    char pucch_s[40];
    if (e->co_pucch_fmt == 0xff)
      snprintf(pucch_s, sizeof(pucch_s), "none");
    else
      snprintf(pucch_s, sizeof(pucch_s), "F%u sym %u+%u ack %u sr %u", e->co_pucch_fmt, e->co_pucch_sym, e->co_pucch_nsym,
               e->co_pucch_harq, e->co_pucch_sr);
    LOG_W(NR_PHY,
          "%4u.%2u UL CRC fail #%llu: rnti %04x pid %u round %u rv %u mcs %u Qm %u L %u rb %u+%u TBS %u C %u tbslbrm %u SNR %.1f dB delay %d%s DMRS drift %.1f deg / %.1f dB over %u symb, symb %u+%u dmrs 0x%04x (rx comb 0x%04x) type %u cdm %u tp %u scid %u, PUCCH %s, +%.1f ms\n",
          e->frame, e->slot, (unsigned long long)r->drained + 1, e->rnti, e->pid, e->round, e->rv, e->mcs, e->qm, e->layers,
          e->rb_start, e->rb_size, e->tbs, e->C, e->tbslbrm, e->snr_x10 / 10.0, e->delay, e->delay_valid ? "" : " (invalid)", e->drift_x10 / 10.0, e->pwr_ratio_x10 / 10.0, e->dsym, e->start_sym, e->nsym, e->dmrs_pos, e->comb_mask, e->dmrs_type, e->cdm, e->tp, e->scid, pucch_s, dt_ms);
    {
      /* per-antenna, per-symbol received power of this PUSCH, dB relative to the antenna mean */
      char sp[4 * 14 * 6 + 32], *q = sp;
      for (int aa = 0; aa < 4; aa++) {
        q += sprintf(q, " a%d:", aa);
        for (int sy = e->start_sym; sy < e->start_sym + e->nsym && sy < 14; sy++)
          q += sprintf(q, " %+.0f", e->sym_pwr_x2[aa][sy] / 2.0);
      }
      LOG_W(NR_PHY, "    fail #%llu symbol power (dB):%s\n", (unsigned long long)r->drained + 1, sp);
    }
    r->drained++;
  }
}

static void l1_alarm_drain(l1_alarm_ring_t *r, bool is_tx, int thresh_us)
{
  const uint64_t head = atomic_load_explicit(&r->seq, memory_order_acquire);
  if (head - r->drained > L1_ALARM_RING) {
    const uint64_t lost = head - r->drained - L1_ALARM_RING;
    r->drained = head - L1_ALARM_RING;
    LOG_W(NR_PHY, "L1 %s overrun: %llu record(s) lost, ring holds %u\n", is_tx ? "TX" : "RX",
          (unsigned long long)lost, L1_ALARM_RING);
  }
  while (r->drained < head) {
    const l1_alarm_rec_t *e = &r->rec[r->drained & (L1_ALARM_RING - 1)];
    if (is_tx) {
      char cfg[256];
      int off = 0;
      for (int i = 0; i < e->n_pdsch && off < (int)sizeof(cfg) - 64; i++)
        off += snprintf(cfg + off, sizeof(cfg) - off, " [rnti %04x mcs %u rb %u+%u layers %u symb %u+%u tbs %u]",
                        e->pdsch[i].rnti, e->pdsch[i].mcs, e->pdsch[i].rb_start, e->pdsch[i].rb_size,
                        e->pdsch[i].layers, e->pdsch[i].symb_start, e->pdsch[i].nr_symb, e->pdsch[i].tbs);
      LOG_W(NR_PHY,
            "%4u.%2u L1 TX overrun: %.1f us > %d us (gen %.1f, ru %.1f = prec %.1f + fhaul %.1f)"
            " (%llu total, %u PDSCH PDU%s)%s\n",
            e->frame, e->slot, e->a, thresh_us, e->b, e->c, e->d, e->e,
            (unsigned long long)e->count, e->n_pdsch, e->n_pdsch == 1 ? "" : "s",
            e->n_pdsch ? cfg : " none");
    } else {
      LOG_W(NR_PHY,
            "%4u.%2u L1 RX overrun: %.1f us > %d us (queued %.1f = txq %.1f + sched %.1f + rxq %.1f, proc %.1f), ring idx %u, %llu total\n",
            e->frame, e->slot, e->a, thresh_us, e->b, e->d, e->e, e->b - e->d - e->e, e->c, e->slot % RU_RX_SLOT_DEPTH,
            (unsigned long long)e->count);
    }
    r->drained++;
  }
}

/* Launch vehicle for phy_procedures_gNB_TX_encode() on the encoding pool.
 *
 * Note this task BLOCKS: nrLDPC_coding_encoder() pushes its own ceil(C/8) sub-tasks and
 * joins them, and join_task_ans() is a plain sem_wait that does not execute tasks while
 * waiting.  So the pool must have at least one thread more than the sub-task count, or
 * this wrapper occupies the only worker and the sub-tasks never run.  init_gNB_proc()
 * refuses to enable the pipeline on a pool that cannot satisfy that. */
typedef struct {
  PHY_VARS_gNB *gNB;
  const nfapi_nr_dl_tti_request_t *DL_req;
  const nfapi_nr_tx_data_request_t *TX_req;
  int frame;
  int slot;
  nr_dlsch_encoded_t *enc;
  task_ans_t *ans;
} tx_encode_arg_t;

static void tx_encode_task(void *arg)
{
  tx_encode_arg_t *a = (tx_encode_arg_t *)arg;
  start_meas(&a->gNB->tx_encode_task_stats);
  phy_procedures_gNB_TX_encode(a->gNB, a->DL_req, a->TX_req, a->frame, a->slot, a->enc);
  stop_meas(&a->gNB->tx_encode_task_stats);
  completed_task_ans(a->ans);
}

/* The TX slot whose grid has not been filled yet because its encoding was still in
 * flight.  Only tx_gen() touches it, and tx_gen() is serial. */
typedef struct {
  int frame;
  int slot;
  openair0_timestamp_t timestamp_tx;
  int ring_idx;
  bool valid;
} tx_pending_t;

/* MAC scheduler responses, handed from tx_func() (scheduler + RX trigger) to tx_gen()
 * (encode, grid fill, RU path).
 *
 * Each one is very big (multiple MB: TX_req carries the PDSCH payloads inline), so they
 * live in static storage.  Serially, two suffice: with L1_tx_pipeline the grid of slot
 * N-1 is filled from its DL_req while the MAC has already written slot N's.  With
 * L1_tx_split the scheduler runs ahead of tx_gen() by however many slots tx_gen() is
 * behind, so it needs a ring, and a count of free entries to stop it overwriting one
 * tx_gen() still reads.  Indexed by a running sequence number, not by slot: ru_thread
 * may skip a slot when it is late, and slot parity would then collide. */
#define TX_SCHED_RING 8
static NR_Sched_Rsp_t tx_sched_ring[TX_SCHED_RING];
static sem_t tx_sched_free;
static uint32_t tx_sched_seq; // tx_func() only

/* One slot handed from tx_func() to tx_gen() */
typedef struct {
  PHY_VARS_gNB *gNB;
  int frame;
  int slot;
  openair0_timestamp_t timestamp_tx;
  int ring_idx;
  uint64_t push_tsc; // clock_gettime_oai() when queued, for the tx_gen_qwait stat
} tx_gen_job_t;

/* accumulate one RX dispatch lag component, windowed like rx_dispatch_lag */
static void lag_acc(time_stats_t *ts, uint64_t t0, uint64_t t1)
{
  const uint64_t d = t1 > t0 ? t1 - t0 : 0;
  ts->trials++;
  ts->diff += d;
  if ((ts->trials & 16383) < 10)
    ts->max = 0;
  if (d > ts->max)
    ts->max = d;
}

static void run_tx_gen(const tx_gen_job_t *job);

/* One generated slot handed from tx_gen() to L1_tx_ru_thread (L1_tx_ru_split).  It
 * carries the PDSCH summary for the overrun alarm because the scheduler response it came
 * from is returned to the ring before the RU stage runs. */
typedef struct {
  PHY_VARS_gNB *gNB;
  int frame;
  int slot;
  int slot_type;
  openair0_timestamp_t timestamp_tx;
  c16_t **grid; // the txdataF this slot was generated into
  uint64_t t_gen_start;
  uint64_t t_gen_end;
  int n_pdsch;
  l1_alarm_pdsch_t pdsch[L1_ALARM_MAX_PDSCH];
} tx_ru_job_t;

/* The RU path for one generated slot: split 7.2 precoding, split 8 precoding and IDFT, then
 * the fronthaul.  Inline at the end of tx_gen(), or on L1_tx_ru_thread with L1_tx_ru_split. */
static void tx_ru_path(PHY_VARS_gNB *gNB, int frame, int slot, int slot_type, openair0_timestamp_t timestamp_tx)
{
  RU_t *ru = gNB->RU_list[0];
  LOG_D(PHY, "gNB: %d.%d : RU TX (pipelined)\n", frame, slot);
  START_MEAS_FULL_SLOT(&gNB->ru_tx_func_stats, slot_type, NR_DOWNLINK_SLOT);
  // split 7.2 precoding
  if (ru->feptx_prec)
    ru->feptx_prec(ru, frame, slot);
  // split 8 precoding and IDFT
  if (ru->feptx_ofdm)
    ru->feptx_ofdm(ru, frame, slot);
  if (ru->fh_south_out)
    ru->fh_south_out(ru, frame, slot, timestamp_tx);
  STOP_MEAS_FULL_SLOT(&gNB->ru_tx_func_stats, slot_type, NR_DOWNLINK_SLOT);
}

/* PDSCH summary of a slot for the overrun alarm */
static int tx_pdsch_summary(const nfapi_nr_dl_tti_request_body_t *b, l1_alarm_pdsch_t *pdsch)
{
  int n = 0;
  for (int i = 0; i < b->nPDUs && n < L1_ALARM_MAX_PDSCH; i++) {
    if (b->dl_tti_pdu_list[i].PDUType != NFAPI_NR_DL_TTI_PDSCH_PDU_TYPE)
      continue;
    const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *p = &b->dl_tti_pdu_list[i].pdsch_pdu.pdsch_pdu_rel15;
    pdsch[n++] = (l1_alarm_pdsch_t){.rnti = p->rnti,
                                    .rb_start = p->rbStart,
                                    .rb_size = p->rbSize,
                                    .tbs = p->TBSize[0],
                                    .mcs = p->mcsIndex[0],
                                    .layers = p->nrOfLayers,
                                    .symb_start = p->StartSymbolIndex,
                                    .nr_symb = p->NrOfSymbols};
  }
  return n;
}

/* Per-slot TX stats and the L1 TX overrun alarm for one generated slot.  tot_us is the
 * whole TX of that slot: phy_proc_tx inline, generation start to RU end with
 * L1_tx_ru_split (so it includes the wait for L1_tx_ru). */
static void tx_slot_account(PHY_VARS_gNB *gNB,
                            int frame,
                            int slot,
                            double tot_us,
                            uint64_t t_gen_start,
                            uint64_t t_gen_end,
                            uint64_t t_ru_start,
                            uint64_t t_ru_end,
                            int n_pdsch,
                            const l1_alarm_pdsch_t *pdsch)
{
  const double ghz = 1000.0 * get_cpu_freq_GHz();
  const double gen_us = (double)(t_gen_end - t_gen_start) / ghz;
  const double ru_us = (double)(t_ru_end - t_ru_start) / ghz;
  AssertFatal(slot < NR_MAX_SLOTS_PER_FRAME, "slot %d out of range\n", slot);
  typeof(gNB->tx_slot_stats[0]) *st = &gNB->tx_slot_stats[slot];
  st->n++;
  st->tot_us += (uint64_t)tot_us;
  st->gen_us += (uint64_t)gen_us;
  st->ru_us += (uint64_t)ru_us;
  if ((uint32_t)tot_us > st->max_us)
    st->max_us = (uint32_t)tot_us;
  if ((uint32_t)gen_us > st->max_gen_us)
    st->max_gen_us = (uint32_t)gen_us;

  if (gNB->tx_overrun_us <= 0 || tot_us <= gNB->tx_overrun_us)
    return;
  gNB->tx_overrun_count++;
  /* Rate limit: the first 10, then one in 101.  An overrun storm must not add logging
   * to a thread that is already behind.
   *
   * The modulus is prime on purpose.  With one-in-100 and a TDD pattern giving 16 TX
   * slots per frame, gcd(100,16)=4, so the sampled slot phase-locks: a run at a 100 us
   * threshold logged 16600 lines that were ALL slots 3/8/13/18 and never once visited
   * the other twelve.  Anything read off these lines is then a property of the
   * aliasing, not of the DU.  A prime cannot share a factor with the frame structure.
   * For per-slot statistics use tx_slot_stats[] above, which counts every slot. */
  if (gNB->tx_overrun_count > 10 && (gNB->tx_overrun_count % 101) != 0)
    return;
  gNB->tx_overrun_logged++;
  /* Record only.  Formatting and the write(2) happen on the stats thread: doing
     them here put an unbounded blocking syscall on L1_tx_thread. */
  const RU_t *ru = gNB->RU_list[0];
  l1_alarm_rec_t *e = l1_alarm_claim(&gNB->tx_alarms);
  e->frame = frame;
  e->slot = slot;
  e->count = gNB->tx_overrun_count;
  e->a = tot_us;
  e->b = gen_us;
  e->c = ru_us;
  e->d = (double)ru->precoding_stats.p_time / ghz;
  e->e = (double)ru->tx_fhaul.p_time / ghz;
  e->n_pdsch = n_pdsch;
  memcpy(e->pdsch, pdsch, n_pdsch * sizeof(*pdsch));
  l1_alarm_publish(&gNB->tx_alarms);
}

/* Stage A of L1 TX: MAC scheduler, then the RX trigger, then hand the slot to tx_gen().
 *
 * With L1_tx_split, tx_gen() runs on its own thread (L1_tx_gen_thread), so a slot whose
 * grid fill + RU path overruns delays only the TX slots behind it.  Before the split it
 * also delayed the scheduler and RX trigger of every following slot, because both ran
 * after the previous slot's tx_gen() on this one thread: at 4T4R full band the PUCCH and
 * PUSCH of the UL slots then started 1-3 ms late although the RX thread was half idle,
 * and the UL collapsed.  The TX work fits the TDD period on average (DDDSU gives 625 us
 * per TX slot); it only must not hold up RX while it catches up. */
static void tx_func(processingData_L1tx_t *info)
{
  const uint64_t t_txstart = (uint64_t)clock_gettime_oai();
  int frame_tx = info->frame;
  int slot_tx = info->slot;
  int frame_rx = info->frame_rx;
  int slot_rx = info->slot_rx;
  LOG_D(NR_PHY, "%d.%d running tx_func\n", frame_tx, slot_tx);
  PHY_VARS_gNB *gNB = info->gNB;
  NR_IF_Module_t *ifi = gNB->if_inst;

  T(T_GNB_PHY_DL_TICK, T_INT(gNB->Mod_id), T_INT(frame_tx), T_INT(slot_tx));

  if (slot_rx == 0) {
    reset_active_stats(gNB, frame_rx);
    reset_active_ulsch(gNB, frame_rx);
  }

  clear_slot_beamid(gNB, slot_tx);
  gNB->last_tx_abs_slot = frame_tx * gNB->frame_parms.slots_per_frame + slot_tx;

  /* Wait for a free scheduler response.  Never blocks serially (tx_gen() holds at most
   * two); split, it blocks only if tx_gen() has fallen TX_SCHED_RING - 2 slots behind,
   * by which time those slots have missed the air anyway. */
  while (sem_wait(&tx_sched_free) != 0)
    AssertFatal(errno == EINTR, "sem_wait: %s\n", strerror(errno));
  const int ring_idx = tx_sched_seq++ % TX_SCHED_RING;
  NR_Sched_Rsp_t *sched = &tx_sched_ring[ring_idx];

  nfapi_nr_slot_indication_scf_t ind = {.header = {.phy_id = 0}, .sfn = frame_tx, .slot = slot_tx};
  start_meas(&gNB->slot_indication_stats);
  ifi->NR_slot_indication(&ind, sched);
  stop_meas(&gNB->slot_indication_stats);

  // At this point, MAC scheduler just ran, including scheduling
  // PRACH/PUCCH/PUSCH, so trigger RX chain processing
  //
  // The RX job must stay on this thread: dispatching it from ru_thread lets
  // NR_UL_indication(slot_rx) race NR_slot_indication(slot_tx) for sched_lock, which
  // measurably destabilises the UE.  Queuing here keeps their order fixed.
  nr_save_ul_tti_req(gNB, &sched->UL_tti_req);
  LOG_D(NR_PHY, "Trigger RX for %d.%d\n", frame_rx, slot_rx);
  notifiedFIFO_elt_t *res = newNotifiedFIFO_elt(sizeof(processingData_L1_t), 0, &gNB->resp_L1, NULL);
  processingData_L1_t *syncMsg = NotifiedFifoData(res);
  syncMsg->gNB = gNB;
  syncMsg->frame_rx = frame_rx;
  syncMsg->slot_rx = slot_rx;
  syncMsg->timestamp_tx = info->timestamp_tx;
  syncMsg->dispatch_tsc = info->dispatch_tsc; // stamped by ru_thread after feprx()
  syncMsg->txstart_tsc = t_txstart;
  res->key = slot_rx;
  syncMsg->push_tsc = (uint64_t)clock_gettime_oai();
  pushNotifiedFIFO(&gNB->resp_L1, res);

  const tx_gen_job_t job = {.gNB = gNB,
                            .frame = frame_tx,
                            .slot = slot_tx,
                            .timestamp_tx = info->timestamp_tx,
                            .ring_idx = ring_idx,
                            .push_tsc = (uint64_t)clock_gettime_oai()};
  if (gNB->tx_split) {
    notifiedFIFO_elt_t *g = newNotifiedFIFO_elt(sizeof(tx_gen_job_t), slot_tx, &gNB->L1_tx_gen, NULL);
    *(tx_gen_job_t *)NotifiedFifoData(g) = job;
    pushNotifiedFIFO(&gNB->L1_tx_gen, g);
  } else {
    run_tx_gen(&job);
  }
}

/* Stage B of L1 TX: start this slot's encoding, fill a grid, run the RU path.  Serial
 * (one caller at a time): inline in tx_func(), or on L1_tx_gen_thread with L1_tx_split. */
static void tx_gen(const tx_gen_job_t *job)
{
  PHY_VARS_gNB *gNB = job->gNB;
  const int frame_tx = job->frame;
  const int slot_tx = job->slot;
  nfapi_nr_config_request_scf_t *cfg = &gNB->gNB_config;
  /* the encode task takes &sched->DL_req etc.; tx_gen() only reads the ring entry */
  NR_Sched_Rsp_t *sched = &tx_sched_ring[job->ring_idx];

  RU_t *ru = gNB->RU_list[0];
  int tx_slot_type = nr_slot_select(cfg, frame_tx, slot_tx);
  // TODO check for analog_bf_vendor_ext set to 1 is a workaround while no beam API for beam selection is implemented
  const bool tx_this_slot = tx_slot_type == NR_DOWNLINK_SLOT || tx_slot_type == NR_MIXED_SLOT
                            || get_softmodem_params()->continuous_tx || IS_SOFTMODEM_RFSIM
                            || cfg->analog_beamforming_ve.analog_bf_vendor_ext.value;

  /* Which slot's grid is filled and handed to the RU in this call.  Serially it is this
   * one; pipelined it is the previous TX slot, whose encoding was launched last call and
   * completed before that call returned. */
  static tx_pending_t pend;
  bool do_generate = tx_this_slot;
  int gen_frame = frame_tx;
  int gen_slot = slot_tx;
  openair0_timestamp_t gen_ts = job->timestamp_tx;
  int gen_ring_idx = job->ring_idx;
  if (gNB->tx_pipeline) {
    do_generate = pend.valid;
    gen_frame = pend.frame;
    gen_slot = pend.slot;
    gen_ts = pend.timestamp_tx;
    gen_ring_idx = pend.ring_idx;
  }
  const NR_Sched_Rsp_t *gen_sched = &tx_sched_ring[gen_ring_idx];

  if (do_generate || (gNB->tx_pipeline && tx_this_slot)) {
    start_meas(&gNB->phy_proc_tx);
    /* Split the measured region so the overrun alarm can say WHICH half was slow: PDCCH/PDSCH
     * generation, or the RU path (precoding + fronthaul BFP compression).  One aggregate
     * number cannot distinguish them, and they have entirely different causes. */
    /* clock_gettime_oai(), not rdtsc_oai(): every duration below is divided by
     * get_cpu_freq_GHz(), which is calibrated against clock_gettime_oai().  On aarch64
     * rdtsc_oai() reads cntvct_el0, the generic timer -- 25 MHz on the LX2160A -- so raw
     * rdtsc deltas came out 40x too small there (and off by the TSC frequency on x86). */
    const uint64_t t_gen_start = (uint64_t)clock_gettime_oai();

    /* Start this slot's encoding, then spend the wait for it filling the previous slot's
     * grid.  The two stages are gated separately on purpose: tx_func() runs on every slot,
     * so a UL slot must still push out the DL slot left sitting in the pipeline. */
    task_ans_t enc_ans;
    tx_encode_arg_t enc_arg;
    bool encoding = false;
    if (gNB->tx_pipeline && tx_this_slot) {
      init_task_ans(&enc_ans, 1);
      enc_arg = (tx_encode_arg_t){.gNB = gNB,
                                  .DL_req = &sched->DL_req,
                                  .TX_req = &sched->TX_req,
                                  .frame = frame_tx,
                                  .slot = slot_tx,
                                  .enc = &gNB->dlsch_encoded[slot_tx & 1],
                                  .ans = &enc_ans};
      task_t t = {.func = tx_encode_task, .args = &enc_arg};
      pushTpool(gNB->threadPoolEnc, t);
      encoding = true;
    }

    uint64_t t_gen_end = t_gen_start;
    uint64_t t_ru_end = t_gen_start;
    if (do_generate) {
      const int gen_slot_type = nr_slot_select(cfg, gen_frame, gen_slot);
      START_MEAS_FULL_SLOT(&gNB->gnb_tx_procedures_stats, gen_slot_type, NR_DOWNLINK_SLOT);
      /* L1_tx_ru_split: fill a grid L1_tx_ru is not sending.  Blocks only when the RU stage
       * still holds both, i.e. it is two slots behind. */
      c16_t **grid = NULL;
      if (gNB->tx_ru_split) {
        const uint64_t t_wait = cpu_meas_enabled ? (uint64_t)clock_gettime_oai() : 0;
        while (sem_wait(&gNB->tx_grid_free) != 0)
          AssertFatal(errno == EINTR, "sem_wait: %s\n", strerror(errno));
        if (cpu_meas_enabled)
          lag_acc(&gNB->tx_grid_wait, t_wait, (uint64_t)clock_gettime_oai());
        grid = gNB->tx_grid[gNB->tx_grid_seq++ & 1];
        gNB->common_vars.txdataF = grid;
      }
      if (gNB->tx_pipeline)
        phy_procedures_gNB_TX_generate(gNB,
                                       &gen_sched->DL_req,
                                       &gen_sched->UL_dci_req,
                                       gen_frame,
                                       gen_slot,
                                       &gNB->dlsch_encoded[gen_slot & 1]);
      else
        phy_procedures_gNB_TX(gNB, &sched->DL_req, &sched->TX_req, &sched->UL_dci_req, frame_tx, slot_tx);
      t_gen_end = (uint64_t)clock_gettime_oai();
      STOP_MEAS_FULL_SLOT(&gNB->gnb_tx_procedures_stats, gen_slot_type, NR_DOWNLINK_SLOT);

      /* develop's 5ee8de9fd0 removed ru_tx_func() and processingData_RU_t, inlining the
         three stages at the call site.  Same three stages here (tx_ru_path()), on the
         pipelined slot rather than the current one. */
      if (gNB->tx_ru_split) {
        notifiedFIFO_elt_t *r = newNotifiedFIFO_elt(sizeof(tx_ru_job_t), gen_slot, &gNB->L1_tx_ru, NULL);
        tx_ru_job_t *rj = (tx_ru_job_t *)NotifiedFifoData(r);
        *rj = (tx_ru_job_t){.gNB = gNB,
                            .frame = gen_frame,
                            .slot = gen_slot,
                            .slot_type = gen_slot_type,
                            .timestamp_tx = gen_ts,
                            .grid = grid,
                            .t_gen_start = t_gen_start,
                            .t_gen_end = t_gen_end};
        if (cpu_meas_enabled && gNB->tx_overrun_us > 0)
          rj->n_pdsch = tx_pdsch_summary(&gen_sched->DL_req.dl_tti_request_body, rj->pdsch);
        pushNotifiedFIFO(&gNB->L1_tx_ru, r);
      } else {
        tx_ru_path(gNB, gen_frame, gen_slot, gen_slot_type, gen_ts);
        t_ru_end = (uint64_t)clock_gettime_oai();
      }
    }

    if (encoding)
      join_task_ans(&enc_ans);
    stop_meas(&gNB->phy_proc_tx);

    /* L1 TX overrun alarm.  A TX that misses its deadline still hands xran a buffer on
     * time, so the O-RU reports nothing wrong (RX_LATE ~0, RX_CORRUPT 0) -- only the
     * CONTENTS are stale, and the UE silently fails to decode the DCI.  The aggregate
     * time_stats cannot show this: it reports a mean plus a max that start_meas() zeroes
     * every 16384 trials, so an outlier is visible but its cause is not.  Report the
     * configuration that produced the overrun instead. */
    if (do_generate && cpu_meas_enabled && !gNB->tx_ru_split) {
      const double tot_us = (double)gNB->phy_proc_tx.p_time / (1000.0 * get_cpu_freq_GHz());
      l1_alarm_pdsch_t pdsch[L1_ALARM_MAX_PDSCH];
      const int n_pdsch = gNB->tx_overrun_us > 0 ? tx_pdsch_summary(&gen_sched->DL_req.dl_tti_request_body, pdsch) : 0;
      tx_slot_account(gNB, gen_frame, gen_slot, tot_us, t_gen_start, t_gen_end, t_gen_end, t_ru_end, n_pdsch, pdsch);
    }
  }

  /* Return scheduler responses tx_gen() is done with.  Pipelined, the one generated above
   * is finished, and this slot's is kept for next call's generate if it has a grid to
   * fill; serially, this slot's is finished. */
  if (gNB->tx_pipeline) {
    if (do_generate)
      sem_post(&tx_sched_free);
    if (!tx_this_slot)
      sem_post(&tx_sched_free);
    pend.valid = tx_this_slot;
    pend.frame = frame_tx;
    pend.slot = slot_tx;
    pend.timestamp_tx = job->timestamp_tx;
    pend.ring_idx = job->ring_idx;
  } else {
    sem_post(&tx_sched_free);
  }

  // beam API - fill TX and/or RX beam IDs
  if (gNB->common_vars.analog_bf) {
    if (ru->fh_south_ctrl)
      ru->fh_south_ctrl(ru, frame_tx, slot_tx, job->timestamp_tx);
  }
  /* No fh_south_out() here: develop calls it at this point for slot_tx, but this tx_func
     already ran it above for the slot it generated (gen_slot), and with L1_tx_pipeline
     slot_tx is not generated yet.  Calling it here too repeated the fronthaul stage on
     every DL slot (~120-220 us at 4T4R) and shipped a not-yet-generated grid. */
}

/* tx_gen() with its stats: "L1 Tx job" times the TX work itself wherever it runs, and
 * tx_gen_qwait how long the slot waited for it after the scheduler (0 when inline). */
static void run_tx_gen(const tx_gen_job_t *job)
{
  PHY_VARS_gNB *gNB = job->gNB;
  const int slot_type = nr_slot_select(&gNB->gNB_config, job->frame, job->slot);
  if (cpu_meas_enabled)
    lag_acc(&gNB->tx_gen_qwait, job->push_tsc, (uint64_t)clock_gettime_oai());
  START_MEAS_FULL_SLOT(&gNB->l1_tx_proc, slot_type, NR_DOWNLINK_SLOT);
  tx_gen(job);
  STOP_MEAS_FULL_SLOT(&gNB->l1_tx_proc, slot_type, NR_DOWNLINK_SLOT);
}

/* L1_tx_split: stage B of L1 TX on its own thread, fed by tx_func() through L1_tx_gen */
static void *L1_tx_gen_thread(void *arg)
{
  PHY_VARS_gNB *gNB = (PHY_VARS_gNB *)arg;
  while (oai_exit == 0) {
    notifiedFIFO_elt_t *res = pullNotifiedFIFO(&gNB->L1_tx_gen);
    if (res == NULL)
      break;
    run_tx_gen((const tx_gen_job_t *)NotifiedFifoData(res));
    delNotifiedFIFO_elt(res);
  }
  return NULL;
}

/* L1_tx_ru_split: stage C of L1 TX, the RU path of each generated grid, fed by tx_gen()
 * through L1_tx_ru.  The fronthaul compression of slot N then overlaps the grid fill of
 * slot N+1 instead of following it on L1_tx_gen. */
static void *L1_tx_ru_thread(void *arg)
{
  PHY_VARS_gNB *gNB = (PHY_VARS_gNB *)arg;
  while (oai_exit == 0) {
    notifiedFIFO_elt_t *res = pullNotifiedFIFO(&gNB->L1_tx_ru);
    if (res == NULL)
      break;
    const tx_ru_job_t *job = (const tx_ru_job_t *)NotifiedFifoData(res);
    RU_t *ru = gNB->RU_list[0];
    /* the precoding and OFDM stages read common_vars.txdataF, which tx_gen() has already
     * moved on to the other grid; only the 7.2 path takes its grid from txdataF_tx */
    AssertFatal(ru->feptx_prec == NULL && ru->feptx_ofdm == NULL, "L1_tx_ru_split needs a 7.2 RU (no RU precoding or OFDM)\n");
    const uint64_t t_ru_start = (uint64_t)clock_gettime_oai();
    if (cpu_meas_enabled)
      lag_acc(&gNB->tx_ru_qwait, job->t_gen_end, t_ru_start);
    ru->txdataF_tx = job->grid;
    tx_ru_path(gNB, job->frame, job->slot, job->slot_type, job->timestamp_tx);
    const uint64_t t_ru_end = (uint64_t)clock_gettime_oai();
    sem_post(&gNB->tx_grid_free);
    if (cpu_meas_enabled) {
      const double tot_us = (double)(t_ru_end - job->t_gen_start) / (1000.0 * get_cpu_freq_GHz());
      tx_slot_account(gNB, job->frame, job->slot, tot_us, job->t_gen_start, job->t_gen_end, t_ru_start, t_ru_end, job->n_pdsch, job->pdsch);
    }
    delNotifiedFIFO_elt(res);
  }
  return NULL;
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
     const int slot_rx = info->slot_rx;
     if (cpu_meas_enabled && info->dispatch_tsc) {
       /* how long this job sat between ru_thread queuing it and us starting it */
       gNB->rx_dispatch_lag.trials++;
       const uint64_t lag = (uint64_t)clock_gettime_oai() - info->dispatch_tsc;
       gNB->rx_dispatch_lag.diff += lag;
       /* windowed like start_meas() does, so the max describes recent behaviour */
       if ((gNB->rx_dispatch_lag.trials & 16383) < 10)
         gNB->rx_dispatch_lag.max = 0;
       if (lag > gNB->rx_dispatch_lag.max)
         gNB->rx_dispatch_lag.max = lag;
       const double lag_us = (double)lag / (1000.0 * get_cpu_freq_GHz());
       if (lag_us > 200.0) gNB->rx_lag_over[0]++;
       if (lag_us > 500.0) gNB->rx_lag_over[1]++;
       if (lag_us > 1000.0) gNB->rx_lag_over[2]++;
       const uint64_t now = info->dispatch_tsc + lag;
       lag_acc(&gNB->rx_lag_txq, info->dispatch_tsc, info->txstart_tsc);
       lag_acc(&gNB->rx_lag_sched, info->txstart_tsc, info->push_tsc);
       lag_acc(&gNB->rx_lag_rxq, info->push_tsc, now);
       if (info->txstart_tsc > info->dispatch_tsc
           && (double)(info->txstart_tsc - info->dispatch_tsc) / (1000.0 * get_cpu_freq_GHz()) > 500.0)
         gNB->rx_lag_txq_over500++;
     }
     const uint64_t t_rx_start = (uint64_t)clock_gettime_oai();
     START_MEAS_FULL_SLOT(&gNB->l1_rx_proc, slot_type, NR_UPLINK_SLOT);
     rx_func(info);
     STOP_MEAS_FULL_SLOT(&gNB->l1_rx_proc, slot_type, NR_UPLINK_SLOT);
     const uint64_t t_rx_end = (uint64_t)clock_gettime_oai();
     /* RX overrun alarm: the ring budget applies to the whole feprx -> here path, so judge
      * that, and report which half consumed it.  proc_ comes from our own timestamps, not
      * l1_rx_proc.p_time, because STOP_MEAS_FULL_SLOT measures only full UL slots while the
      * ring budget applies to every slot.  Rate limited with a prime modulus for the same
      * reason as the TX alarm: a one-in-100 limiter phase-locks to a TDD pattern whose UL
      * slot count shares a factor with 100 and would then only ever report a fixed subset. */
     if (gNB->rx_overrun_us > 0 && cpu_meas_enabled && info->dispatch_tsc) {
       const double ghz_ = 1000.0 * get_cpu_freq_GHz();
       const double proc_ = (double)(t_rx_end - t_rx_start) / ghz_;
       const double e2e_ = (double)(t_rx_end - info->dispatch_tsc) / ghz_;
       if (e2e_ > gNB->rx_overrun_us) {
         gNB->rx_overrun_count++;
         if (gNB->rx_overrun_count <= 10 || (gNB->rx_overrun_count % 101) == 0) {
           gNB->rx_overrun_logged++;
           /* Record only; the stats thread prints. See l1_alarm_ring_t. */
           l1_alarm_rec_t *e = l1_alarm_claim(&gNB->rx_alarms);
           e->frame = info->frame_rx;
           e->slot = slot_rx;
           e->count = gNB->rx_overrun_count;
           e->a = e2e_;
           e->b = e2e_ - proc_;
           e->c = proc_;
           e->d = info->txstart_tsc > info->dispatch_tsc ? (double)(info->txstart_tsc - info->dispatch_tsc) / ghz_ : 0;
           e->e = info->push_tsc > info->txstart_tsc ? (double)(info->push_tsc - info->txstart_tsc) / ghz_ : 0;
           e->n_pdsch = 0;
           l1_alarm_publish(&gNB->rx_alarms);
         }
       }
     }
     if (cpu_meas_enabled) {
       /* Own timestamps rather than l1_rx_proc.p_time: STOP_MEAS_FULL_SLOT measures only
          full UL slots, and the point of this table is to compare those against the mixed
          slot, whose cost differs by several times. */
       AssertFatal(slot_rx < NR_MAX_SLOTS_PER_FRAME, "slot_rx %d out of range\n", slot_rx);
       const double us = (double)((uint64_t)clock_gettime_oai() - t_rx_start) / (1000.0 * get_cpu_freq_GHz());
       typeof(gNB->rx_slot_stats[0]) *st = &gNB->rx_slot_stats[slot_rx];
       st->n++;
       st->tot_us += (uint64_t)us;
       if ((uint32_t)us > st->max_us)
         st->max_us = (uint32_t)us;
     }
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
     const int call_slot = info->slot;
     const uint64_t t_call = (uint64_t)clock_gettime_oai();
     tx_func(info);
     if (cpu_meas_enabled) {
       const uint64_t t_end = (uint64_t)clock_gettime_oai();
       const uint64_t old_max = gNB->tx_call_all.max;
       lag_acc(&gNB->tx_call_all, t_call, t_end);
       if (gNB->tx_call_all.max != old_max)
         gNB->tx_call_max_slot = call_slot;
       const double us = (double)(t_end - t_call) / (1000.0 * get_cpu_freq_GHz());
       if (us > 500.0) gNB->tx_call_over[0]++;
       if (us > 1000.0) gNB->tx_call_over[1]++;
     }
     delNotifiedFIFO_elt(res);
  }
  return NULL;
}

/* PRACH detection off the L1 RX thread.
 *
 * At 4T4R on the A72, PRACH detection costs ~340 us per occasion and ran serially on
 * L1_rx_thread together with PUCCH, PUSCH and the noise estimate.  PRACH has no slot
 * deadline of its own -- the RAR window is several slots -- but those 340 us delay the
 * PUCCH/PUSCH work queued behind it, and the dedicated RX pool was idle.  So run it on
 * the pool and let the task deliver its own RACH indication.
 *
 * Safe to run concurrently with the RX thread:
 *  - the job owns its sample buffer (calloc'd per job in nr_schedule_rx_prach() and
 *    freed by L1_nr_prach_procedures() via free_nr_prach_entry());
 *  - nr_initiate_ra_proc(), the MAC entry point, takes nr_mac->sched_lock itself;
 *  - every handler in NR_UL_indication is guarded by its own count, so an indication
 *    carrying only RACH does nothing else.
 * The only other shared state is the prach_I0 EMA and prach_energy_counter, which at
 * worst see a different update order. */
typedef struct {
  PHY_VARS_gNB *gNB;
  prach_item_t item;
  NR_UL_IND_t ul;
} prach_task_t;

static void prach_task(void *arg)
{
  prach_task_t *t = (prach_task_t *)arg;
  t->ul.rach_ind.pdu_list = t->ul.prach_pdu_indication_list;
  t->ul.rach_ind.number_of_pdus = 0;
  L1_nr_prach_procedures(t->gNB, &t->item, &t->ul.rach_ind);
  if (t->ul.rach_ind.number_of_pdus > 0)
    t->gNB->if_inst->NR_UL_indication(&t->ul);
  free(t);
}

static void rx_func(processingData_L1_t *info)
{
  PHY_VARS_gNB *gNB = info->gNB;
  int frame_rx = info->frame_rx;
  int slot_rx = info->slot_rx;
  nfapi_nr_config_request_scf_t *cfg = &gNB->gNB_config;

  T(T_GNB_PHY_UL_TICK, T_INT(gNB->Mod_id), T_INT(frame_rx), T_INT(slot_rx));

  // RX processing
  int rx_slot_type = nr_slot_select(cfg, frame_rx, slot_rx);
  if (rx_slot_type == NR_UPLINK_SLOT || rx_slot_type == NR_MIXED_SLOT) {
    LOG_D(NR_PHY, "%d.%d Starting RX processing\n", frame_rx, slot_rx);

    // UE-specific RX processing for subframe n
    NR_UL_IND_t UL_INFO = {.frame = frame_rx, .slot = slot_rx, .module_id = gNB->Mod_id, .phy_id = gNB->CC_id};
    // Do PRACH RU processing
    UL_INFO.rach_ind.pdu_list = UL_INFO.prach_pdu_indication_list;
    UL_INFO.rach_ind.number_of_pdus = 0;
    // even if processing is late, we might collect all PRACH
    // the last PRACH's frame/slot is when all UE's appear to have accessed
    /* With a dedicated RX pool, PRACH detection runs there and the task hands its RACH
     * indication to the MAC itself (see prach_task()); otherwise it stays inline. */
    const bool prach_async = gNB->threadPoolRx == &gNB->threadPoolRxOwn && gNB->if_inst && gNB->if_inst->NR_UL_indication;
    prach_item_t p;
    while (spsc_q_get(&gNB->prach_l1rx_queue, &p, sizeof(p))) {
      if (prach_async) {
        prach_task_t *t = calloc_or_fail(1, sizeof(*t));
        t->gNB = gNB;
        t->item = p;
        t->ul = (NR_UL_IND_t){.frame = p.frame, .slot = p.slot, .module_id = gNB->Mod_id, .phy_id = gNB->CC_id};
        pushTpool(gNB->threadPoolRx, (task_t){.func = prach_task, .args = t});
      } else {
        L1_nr_prach_procedures(gNB, &p, &UL_INFO.rach_ind);
      }
    }

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

    // Call PHY Rx signal procedures
    phy_procedures_gNB_uespec_RX(gNB, frame_rx, slot_rx, &UL_INFO);

    // Call the scheduler
    START_MEAS_FULL_SLOT(&gNB->ul_indication_stats, rx_slot_type, NR_UPLINK_SLOT);
    gNB->if_inst->NR_UL_indication(&UL_INFO);
    STOP_MEAS_FULL_SLOT(&gNB->ul_indication_stats, rx_slot_type, NR_UPLINK_SLOT);

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
      res->key = slot_rx;
      LOG_D(NR_PHY, "Signaling completion for %d.%d (mod_slot %d) on L1_rx_out\n", frame_rx, slot_rx, slot_rx % RU_RX_SLOT_DEPTH);
      pushNotifiedFIFO(&gNB->L1_rx_out, res);
    }
  }

}

static void nrL1_stats_init_sorted_list(PHY_VARS_gNB *gNB, RU_t *ru, unsigned int size)
{
  init_sorted_list_meas(&gNB->l1_tx_proc, size);
  init_sorted_list_meas(&gNB->l1_rx_proc, size);
  init_sorted_list_meas(&gNB->phy_proc_tx, size);
  init_sorted_list_meas(&gNB->gnb_tx_procedures_stats, size);
  init_sorted_list_meas(&gNB->ru_tx_func_stats, size);
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
  init_sorted_list_meas(&gNB->rx_prach, size);
  init_sorted_list_meas(&gNB->rx_i0_meas, size);
  init_sorted_list_meas(&gNB->tx_encode_task_stats, size);
  init_sorted_list_meas(&gNB->tx_pdsch_generate_call, size);
  if (ru->feprx) {
    init_sorted_list_meas(&ru->ofdm_demod_stats, size);
  }
  if (ru->feptx_prec) {
    init_sorted_list_meas(&ru->precoding_stats, size);
  }
  if (ru->feptx_ofdm) {
    init_sorted_list_meas(&ru->txdataF_copy_stats, size);
    init_sorted_list_meas(&ru->ofdm_mod_stats, size);
    init_sorted_list_meas(&ru->ofdm_total_stats, size);
  }
  init_sorted_list_meas(&ru->tx_fhaul, size);
  init_sorted_list_meas(&ru->rx_decomp, size);
}

static void nrL1_stats_free_sorted_list(PHY_VARS_gNB *gNB, RU_t *ru)
{
  free_sorted_list_meas(&gNB->l1_tx_proc);
  free_sorted_list_meas(&gNB->l1_rx_proc);
  free_sorted_list_meas(&gNB->phy_proc_tx);
  free_sorted_list_meas(&gNB->gnb_tx_procedures_stats);
  free_sorted_list_meas(&gNB->ru_tx_func_stats);
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
  free_sorted_list_meas(&gNB->rx_prach);
  free_sorted_list_meas(&gNB->rx_i0_meas);
  free_sorted_list_meas(&gNB->tx_encode_task_stats);
  free_sorted_list_meas(&gNB->tx_pdsch_generate_call);
  if (ru->feprx) {
    free_sorted_list_meas(&ru->ofdm_demod_stats);
  }
  if (ru->feptx_prec) {
    free_sorted_list_meas(&ru->precoding_stats);
  }
  if (ru->feptx_ofdm) {
    free_sorted_list_meas(&ru->txdataF_copy_stats);
    free_sorted_list_meas(&ru->ofdm_mod_stats);
    free_sorted_list_meas(&ru->ofdm_total_stats);
    free_sorted_list_meas(&ru->txdataF_copy_stats);
  }
  free_sorted_list_meas(&ru->tx_fhaul);
  free_sorted_list_meas(&ru->rx_decomp);
}

static void nrL1_stats_reset(PHY_VARS_gNB *gNB, RU_t *ru)
{
  reset_meas(&gNB->l1_tx_proc);
  reset_meas(&gNB->l1_rx_proc);
  reset_meas(&gNB->phy_proc_tx);
  reset_meas(&gNB->gnb_tx_procedures_stats);
  reset_meas(&gNB->ru_tx_func_stats);
  reset_meas(&gNB->dlsch_encoding_stats);
  reset_meas(&gNB->dlsch_ldpc_encode_stats);
  reset_meas(&gNB->dlsch_scrambling_stats);
  reset_meas(&gNB->dlsch_modulation_stats);
  reset_meas(&gNB->dlsch_resource_mapping_stats);
  reset_meas(&gNB->dlsch_precoding_stats);
  reset_meas(&gNB->dlsch_pdsch_generation_stats);
  reset_meas(&gNB->phy_proc_rx);
  reset_meas(&gNB->rx_dispatch_lag);
  reset_meas(&gNB->rx_lag_txq);
  reset_meas(&gNB->rx_lag_sched);
  reset_meas(&gNB->rx_lag_rxq);
  gNB->rx_lag_txq_over500 = 0;
  reset_meas(&gNB->tx_call_all);
  reset_meas(&gNB->tx_gen_qwait);
  reset_meas(&gNB->tx_ru_qwait);
  reset_meas(&gNB->tx_grid_wait);
  gNB->tx_call_over[0] = gNB->tx_call_over[1] = 0;
  reset_meas(&gNB->pucch01_proc_rx);
  reset_meas(&gNB->pucch23_proc_rx);
  reset_meas(&gNB->ulsch_decoding_stats);
  reset_meas(&gNB->ts_ldpc_decode);
  reset_meas(&gNB->ul_indication_stats);
  reset_meas(&gNB->slot_indication_stats);
  reset_meas(&gNB->rx_pusch_stats);
  reset_meas(&gNB->ulsch_channel_estimation_stats);
  reset_meas(&gNB->pusch_channel_estimation_antenna_processing_stats);
  reset_meas(&gNB->pusch_chest_post_stats);
  reset_meas(&gNB->pusch_chest_diag_stats);
  reset_meas(&gNB->pusch_chest_avg_stats);
  reset_meas(&gNB->pusch_chest_power_stats);
  reset_meas(&gNB->rx_pusch_init_stats);
  reset_meas(&gNB->pusch_scr_init_stats);
  reset_meas(&gNB->pusch_extraction_stats);
  reset_meas(&gNB->rx_pusch_symbol_processing_stats);
  reset_meas(&gNB->rx_prach);
  reset_meas(&gNB->rx_i0_meas);
  gNB->rx_i0_skipped = 0;
  reset_meas(&gNB->tx_encode_task_stats);
  reset_meas(&gNB->tx_pdsch_generate_call);
  gNB->tx_gen_pdsch_slots = gNB->tx_gen_enc_empty = gNB->tx_gen_enc_stale = gNB->tx_gen_enc_mismatch = 0;
  memset(gNB->rx_slot_stats, 0, sizeof(gNB->rx_slot_stats));
  memset(gNB->tx_slot_stats, 0, sizeof(gNB->tx_slot_stats));
  if (ru->feprx) {
    reset_meas(&ru->ofdm_demod_stats);
  }
  if (ru->feptx_prec) {
    reset_meas(&ru->precoding_stats);
  }
  if (ru->feptx_ofdm) {
    reset_meas(&ru->txdataF_copy_stats);
    reset_meas(&ru->ofdm_mod_stats);
    reset_meas(&ru->ofdm_total_stats);
    reset_meas(&ru->txdataF_copy_stats);
  }
  reset_meas(&ru->tx_fhaul);
  reset_meas(&ru->rx_decomp);
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

  output += print_meas_log(&gNB->ru_tx_func_stats,
                           "L1 RU TX function",
                           NULL,
                           NULL,
                           output,
                           end - output);
  output += print_meas_log(&gNB->dlsch_encoding_stats, "DLSCH encoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_ldpc_encode_stats, "LDPC encoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_layer_clear_stats, "DLSCH tx_layers clear", NULL, NULL, output, end-output);
  output += print_meas_log(&gNB->dlsch_scrambling_stats, "DLSCH scrambling", NULL, NULL, output, end-output);
  output += print_meas_log(&gNB->dlsch_modulation_stats, "DLSCH modulation", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_pdsch_generation_stats, "PDSCH generation", NULL, NULL, output, end - output);
  /* The two halves of PDSCH generation.  Both are already measured per symbol task and
   * merged in nr_generate_pdsch_prepare(), they were just never printed -- so the largest
   * stage of the TX budget had no visible breakdown. Sums over the symbol tasks, so they
   * are CPU time and add up to more than the wall time of "PDSCH generation" above. */
  output += print_meas_log(&gNB->dlsch_resource_mapping_stats, "  PDSCH RE mapping", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_precoding_stats, "  PDSCH precoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->phy_proc_rx, "L1 Rx processing", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pucch01_proc_rx, "PUCCH01 reception", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pucch23_proc_rx, "PUCCH23 reception", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ulsch_decoding_stats, "ULSCH decoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_dispatch_lag, "RX dispatch lag", NULL, NULL, output, end - output);
  if (output < end)
    output += snprintf(output, end - output, "%25s:  %llu (%llu logged)\n", "RX overruns",
                       (unsigned long long)gNB->rx_overrun_count, (unsigned long long)gNB->rx_overrun_logged);
  if (gNB->rx_dispatch_lag.trials > 0 && output < end)
    output += snprintf(output, end - output,
                       "%25s:  n %llu, >200us %llu (%.2f%%), >500us %llu (%.3f%%), >1ms %llu (%.3f%%)\n",
                       "RX dispatch lag tail",
                       (unsigned long long)gNB->rx_dispatch_lag.trials,
                       (unsigned long long)gNB->rx_lag_over[0], 100.0 * gNB->rx_lag_over[0] / gNB->rx_dispatch_lag.trials,
                       (unsigned long long)gNB->rx_lag_over[1], 100.0 * gNB->rx_lag_over[1] / gNB->rx_dispatch_lag.trials,
                       (unsigned long long)gNB->rx_lag_over[2], 100.0 * gNB->rx_lag_over[2] / gNB->rx_dispatch_lag.trials);
  {
    uint64_t tot = 0;
    for (int i = 0; i < 24; i++)
      tot += gNB->early_ind_lag_hist[i];
    if (tot && output < end) {
      output += snprintf(output, end - output, "%25s: (%llu)", "early UL ind: tx - rx slots", (unsigned long long)tot);
      for (int i = 0; i < 24 && output < end; i++)
        if (gNB->early_ind_lag_hist[i])
          output += snprintf(output, end - output, " %d%s:%.1f%%", i, i == 23 ? "+" : "", 100.0 * gNB->early_ind_lag_hist[i] / tot);
      if (output < end)
        output += snprintf(output, end - output, "\n");
    }
  }
  if (gNB->dmrs_drift_n > 0 && output < end) {
    output += snprintf(output, end - output, "%25s:  n %llu, avg |phase| %.1f deg, max %.1f deg (first to last DMRS symbol)\n",
                       "PUSCH DMRS drift", (unsigned long long)gNB->dmrs_drift_n, gNB->dmrs_drift_abs_sum / gNB->dmrs_drift_n,
                       gNB->dmrs_drift_max_x10 / 10.0);
    gNB->dmrs_drift_max_x10 = 0; // per dump
  }
  if (gNB->dmrs_ant_n > 0 && output < end) {
    output += snprintf(output, end - output, "%25s:", "PUSCH DMRS per RX ant");
    for (int aa = 0; aa < min(gNB->frame_parms.nb_antennas_rx, 8) && output < end; aa++)
      output += snprintf(output, end - output, "  ant%d coh %.3f pwr %.1f dB |dphi| %.1f deg", aa,
                         gNB->dmrs_coh_sum[aa] / gNB->dmrs_ant_n, gNB->dmrs_pwr_db_sum[aa] / gNB->dmrs_ant_n,
                         gNB->dmrs_phase_abs_sum[aa] / gNB->dmrs_ant_n);
    if (output < end)
      output += snprintf(output, end - output, "\n");
  }
  if (gNB->dmrs_sb_n > 0 && output < end) {
    output += snprintf(output, end - output, "PUSCH DMRS coherence / power per 25-PRB sub-band (n %llu PUSCHs >= 200 PRB):\n",
                       (unsigned long long)gNB->dmrs_sb_n);
    for (int aa = 0; aa < min(gNB->frame_parms.nb_antennas_rx, 8) && output < end; aa++) {
      output += snprintf(output, end - output, "  ant%d:", aa);
      for (int sb = 0; sb < 11 && output < end; sb++)
        output += snprintf(output, end - output, " %.2f/%.0f", gNB->dmrs_sb_coh_sum[aa][sb] / gNB->dmrs_sb_n,
                           gNB->dmrs_sb_pwr_sum[aa][sb] / gNB->dmrs_sb_n);
      if (output < end)
        output += snprintf(output, end - output, "\n");
    }
  }
  output += print_meas_log(&gNB->rx_lag_txq, "  lag: wait TX thread", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_lag_sched, "  lag: MAC scheduler", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_lag_rxq, "  lag: wait RX thread", NULL, NULL, output, end - output);
  if (gNB->rx_lag_txq.trials > 0 && output < end)
    output += snprintf(output, end - output, "%25s:  >500us %llu (%.3f%%)\n", "  lag: wait TX tail",
                       (unsigned long long)gNB->rx_lag_txq_over500,
                       100.0 * gNB->rx_lag_txq_over500 / gNB->rx_lag_txq.trials);
  output += print_meas_log(&gNB->tx_call_all, "tx_func call (all slots)", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->tx_gen_qwait, "TX gen queue wait", NULL, NULL, output, end - output);
  if (gNB->tx_ru_split) {
    output += print_meas_log(&gNB->tx_ru_qwait, "TX RU queue wait", NULL, NULL, output, end - output);
    output += print_meas_log(&gNB->tx_grid_wait, "TX free grid wait", NULL, NULL, output, end - output);
  }
  if (gNB->tx_call_all.trials > 0 && output < end)
    output += snprintf(output, end - output, "%25s:  >500us %llu (%.3f%%), >1ms %llu (%.3f%%), window max at slot %d\n",
                       "tx_func call tail", (unsigned long long)gNB->tx_call_over[0],
                       100.0 * gNB->tx_call_over[0] / gNB->tx_call_all.trials, (unsigned long long)gNB->tx_call_over[1],
                       100.0 * gNB->tx_call_over[1] / gNB->tx_call_all.trials, gNB->tx_call_max_slot);
  output += print_meas_log(&gNB->ts_ldpc_decode, "UL segments decoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ul_indication_stats, "UL Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->slot_indication_stats, "Slot Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_pusch_stats, "PUSCH inner-receiver", NULL, NULL, output, end - output);
  {
    const uint64_t *n = gNB->pusch1_n, *f = gNB->pusch1_fail;
    output += snprintf(output, end - output,
                       "  PUSCH 1st tx vs same-UE PUCCH: without %llu fail %llu (%.2f%%) | with %llu fail %llu (%.2f%%) | by fmt",
                       (unsigned long long)n[0], (unsigned long long)f[0], n[0] ? 100.0 * f[0] / n[0] : 0.0,
                       (unsigned long long)n[1], (unsigned long long)f[1], n[1] ? 100.0 * f[1] / n[1] : 0.0);
    for (int k = 0; k < 5; k++)
      if (gNB->pusch1_fmt_n[k])
        output += snprintf(output, end - output, " F%d %llu/%llu", k, (unsigned long long)gNB->pusch1_fmt_fail[k],
                           (unsigned long long)gNB->pusch1_fmt_n[k]);
    if (gNB->pusch1_ok_sym_n) {
      output += snprintf(output, end - output, "\n  passing 1st tx symbol power vs antenna mean (dB, n %llu):", (unsigned long long)gNB->pusch1_ok_sym_n);
      for (int aa = 0; aa < 4; aa++) {
        output += snprintf(output, end - output, " a%d:", aa);
        for (int sy = 0; sy < 14; sy++)
          output += snprintf(output, end - output, " %+.1f", gNB->pusch1_ok_sym_db[aa][sy] / gNB->pusch1_ok_sym_n);
      }
    }
    output += snprintf(output, end - output, "\n  PUSCH 1st tx with same-UE F0, fail/n by ack bits:");
    for (int h = 0; h < 3; h++)
      for (int s = 0; s < 2; s++)
        if (gNB->pusch1_f0_n[h][s])
          output += snprintf(output, end - output, " ack%d%s %llu/%llu", h, s ? "+sr" : "", (unsigned long long)gNB->pusch1_f0_fail[h][s],
                             (unsigned long long)gNB->pusch1_f0_n[h][s]);
    output += snprintf(output, end - output, "\n");
  }
  output += print_meas_log(&gNB->ulsch_channel_estimation_stats, "  PUSCH chest (total)", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_channel_estimation_antenna_processing_stats, "   chest pool round", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_chest_post_stats, "   chest serial post", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_chest_diag_stats, "    post: diagnostics", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_chest_avg_stats, "    post: time avg", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_chest_power_stats, "    post: power meas", NULL, NULL, output, end - output);
  {
    extern uint64_t nr_chest_job_wait_ns, nr_chest_job_run_ns, nr_chest_job_run_max_ns, nr_chest_job_n;
    const uint64_t n = __atomic_exchange_n(&nr_chest_job_n, 0, __ATOMIC_RELAXED);
    const uint64_t w = __atomic_exchange_n(&nr_chest_job_wait_ns, 0, __ATOMIC_RELAXED);
    const uint64_t r = __atomic_exchange_n(&nr_chest_job_run_ns, 0, __ATOMIC_RELAXED);
    const uint64_t mx = __atomic_exchange_n(&nr_chest_job_run_max_ns, 0, __ATOMIC_RELAXED);
    if (n)
      output += snprintf(output, end - output, "    chest antenna job:  n %lu, queue wait avg %.1f us, run avg %.1f us, run max %.1f us\n",
                         (unsigned long)n, w / 1e3 / n, r / 1e3 / n, mx / 1e3);
  }
  output += print_meas_log(&gNB->rx_pusch_init_stats, "  PUSCH init", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_scr_init_stats, "   scrambling init", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->pusch_extraction_stats, "   init extraction", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_pusch_symbol_processing_stats, "  PUSCH symbol proc", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_prach, "PRACH RX", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_i0_meas, "I0 measurements", NULL, NULL, output, end - output);
  output += snprintf(output, end - output, "   I0 measurements skipped:  %lu UL slots (busy: PUSCH/PUCCH)\n", gNB->rx_i0_skipped);
  output += print_meas_log(&gNB->tx_encode_task_stats, "TX encode task", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->tx_pdsch_generate_call, "PDSCH generate call", NULL, NULL, output, end - output);
  output += snprintf(output,
                     end - output,
                     "   TX generate with PDSCH: %lu slots; encoded data empty %lu, stale (other slot) %lu, PDSCH count mismatch %lu\n",
                     gNB->tx_gen_pdsch_slots,
                     gNB->tx_gen_enc_empty,
                     gNB->tx_gen_enc_stale,
                     gNB->tx_gen_enc_mismatch);
  if (ru->feprx) {
    output += print_meas_log(&ru->ofdm_demod_stats, "feprx", NULL, NULL, output, end - output);
  }

  bool full_slot = ru->half_slot_parallelization == 0;
  if (ru->feptx_prec) {
    output += print_meas_log(&ru->precoding_stats,
                             full_slot ? "feptx_prec (per port)" : "feptx_prec (per port, half_slot)",
                             NULL,
                             NULL,
                             output,
                             end - output);
  }

  if (ru->feptx_ofdm) {
    output += print_meas_log(&ru->txdataF_copy_stats,"txdataF_copy",NULL,NULL, output, end - output);
    output += print_meas_log(&ru->ofdm_mod_stats,
                             full_slot ? "feptx_ofdm (per port)" : "feptx_ofdm (per port, half_slot)",
                             NULL,
                             NULL,
                             output,
                             end - output);
    output += print_meas_log(&ru->ofdm_total_stats,"feptx_total",NULL,NULL, output, end - output);
    output += print_meas_log(&ru->txdataF_copy_stats, "txdataF_copy", NULL, NULL, output, end - output);
  }

  output += print_meas_log(&ru->tx_fhaul,"tx_fhaul",NULL,NULL, output, end - output);
  output += print_meas_log(&ru->rx_decomp,"rx_decomp",NULL,NULL, output, end - output);

  /* Per-slot TX timing.  Only slots that actually carry TX appear (the UL slots of a TDD
   * pattern never run tx_func), so the table is also a readable map of the pattern.
   * Unlike the overrun alarm this counts every slot, so slots are directly comparable. */
  bool any = false;
  for (int s = 0; s < NR_MAX_SLOTS_PER_FRAME && !any; s++)
    any = gNB->tx_slot_stats[s].n > 0;
  if (any && output < end) {
    output += snprintf(output, end - output, "TX per slot [slot: n avg(gen+ru) max maxgen] (max = last dump):\n");
    for (int s = 0; s < NR_MAX_SLOTS_PER_FRAME && output < end; s++) {
      const uint64_t n = gNB->tx_slot_stats[s].n;
      if (n == 0)
        continue;
      output += snprintf(output,
                         end - output,
                         "  %3d: %8llu %6llu us (%5llu + %5llu) max %5u gen %5u\n",
                         s,
                         (unsigned long long)n,
                         (unsigned long long)(gNB->tx_slot_stats[s].tot_us / n),
                         (unsigned long long)(gNB->tx_slot_stats[s].gen_us / n),
                         (unsigned long long)(gNB->tx_slot_stats[s].ru_us / n),
                         gNB->tx_slot_stats[s].max_us,
                         gNB->tx_slot_stats[s].max_gen_us);
      /* windowed: a never-resetting max records the traffic onset forever and can then
       * never show whether steady state is clean */
      gNB->tx_slot_stats[s].max_us = 0;
      gNB->tx_slot_stats[s].max_gen_us = 0;
    }
  }

  /* Per-slot RX timing, companion to the TX table above.  Only slots that carry UL work
   * appear, so this also shows the TDD pattern from the receive side -- and separates the
   * full UL slot from the mixed slot, whose costs differ by several times. */
  bool any_rx = false;
  for (int s = 0; s < NR_MAX_SLOTS_PER_FRAME && !any_rx; s++)
    any_rx = gNB->rx_slot_stats[s].n > 0;
  if (any_rx && output < end) {
    output += snprintf(output, end - output, "RX per slot [slot: n avg max] (max = last dump):\n");
    for (int s = 0; s < NR_MAX_SLOTS_PER_FRAME && output < end; s++) {
      const uint64_t n = gNB->rx_slot_stats[s].n;
      if (n == 0)
        continue;
      output += snprintf(output,
                         end - output,
                         "  %3d: %8llu %6llu us max %5u\n",
                         s,
                         (unsigned long long)n,
                         (unsigned long long)(gNB->rx_slot_stats[s].tot_us / n),
                         gNB->rx_slot_stats[s].max_us);
      gNB->rx_slot_stats[s].max_us = 0; /* windowed, see TX table above */
    }
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
  wait_sync("L1_stats_thread");
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
    fflush(fd);
    /* Print any alarm detail the RT threads recorded. Done here, not there: LOG_W ends in
       a blocking write(2) on the calling thread, which must never be L1_tx/L1_rx. */
    l1_alarm_drain(&gNB->tx_alarms, true, gNB->tx_overrun_us);
    l1_alarm_drain(&gNB->rx_alarms, false, gNB->rx_overrun_us);
    ul_fail_drain(&gNB->ul_fails);
    /* and anything ru_thread deferred: it drives fronthaul timing and must never write */
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
  initTpool(get_softmodem_params()->threadPoolConfig, &gNB->threadPool, cpumeas(CPUMEAS_GETSTATE));
  /* UL work goes to its own pool when L1_rx_pool_cores says so, otherwise it shares the one
   * above.  Sharing is the historical behaviour and stays the default, but it lets a UL slot
   * -- which pushes one task per symbol -- fill the FIFO ahead of the next DL slot's TX
   * dispatch, which then waits for the whole batch regardless of how many workers exist. */
  if (gNB->rx_pool_cores != NULL && strlen(gNB->rx_pool_cores) > 0) {
    LOG_I(NR_PHY, "separate L1 RX thread pool on cores %s\n", gNB->rx_pool_cores);
    initTpool(gNB->rx_pool_cores, &gNB->threadPoolRxOwn, cpumeas(CPUMEAS_GETSTATE));
    gNB->threadPoolRx = &gNB->threadPoolRxOwn;
  } else {
    gNB->threadPoolRx = &gNB->threadPool;
  }
  /* DLSCH encoding gets its own pool when L1_enc_pool_cores says so.  See threadPoolEnc
   * in defs_gNB.h for why a wider shared pool cannot substitute for this. */
  if (gNB->enc_pool_cores != NULL && strlen(gNB->enc_pool_cores) > 0) {
    LOG_I(NR_PHY, "separate DLSCH encoding thread pool on cores %s\n", gNB->enc_pool_cores);
    initTpool(gNB->enc_pool_cores, &gNB->threadPoolEncOwn, cpumeas(CPUMEAS_GETSTATE));
    gNB->threadPoolEnc = &gNB->threadPoolEncOwn;
  } else {
    gNB->threadPoolEnc = &gNB->threadPool;
  }

  if (gNB->tx_pipeline) {
    /* The encode wrapper task blocks in join_task_ans() while its own ceil(C/8) sub-tasks
     * run, so a single-worker pool would deadlock: the wrapper holds the only thread and
     * the sub-tasks never get one.  Two workers merely serialise one sub-task; three run
     * the whole encode concurrently, which is what the budget needs.
     *
     * A shared pool is refused outright.  The point of the pipeline is that encoding and
     * grid filling proceed at once, and the thread pool is FIFO with no priority: on one
     * pool the generation tasks queue behind an encode task that no core count shortens. */
    AssertFatal(gNB->threadPoolEnc == &gNB->threadPoolEncOwn,
                "L1_tx_pipeline needs L1_enc_pool_cores: on the shared pool the grid fill just queues behind the encode\n");
    AssertFatal(gNB->threadPoolEnc->len_thr >= 2,
                "L1_tx_pipeline needs at least 2 cores in L1_enc_pool_cores (\"%s\" gives %zu): the encode task blocks while its sub-tasks run\n",
                gNB->enc_pool_cores,
                gNB->threadPoolEnc->len_thr);
    if (gNB->threadPoolEnc->len_thr < 3)
      LOG_W(NR_PHY,
            "L1_enc_pool_cores has %zu cores; 3 lets the encode wrapper and both of its sub-tasks run at once\n",
            gNB->threadPoolEnc->len_thr);
    LOG_I(NR_PHY, "L1 TX pipeline on: encoding slot N overlaps the grid fill of slot N-1 (costs one slot of sl_ahead)\n");
  }

  // L1 RX result FIFO
  initNotifiedFIFO(&gNB->resp_L1);
  // L1 TX result FIFO
  initNotifiedFIFO(&gNB->L1_tx_out);
  // fault the scheduler response ring in now, not on the first slots of the RT threads
  memset(tx_sched_ring, 0, sizeof(tx_sched_ring));
  AssertFatal(sem_init(&tx_sched_free, 0, TX_SCHED_RING) == 0, "sem_init: %s\n", strerror(errno));
  if (gNB->tx_split)
    initNotifiedFIFO(&gNB->L1_tx_gen);
  if (gNB->tx_ru_split) {
    /* Without L1_tx_split, tx_gen() runs on L1_tx_thread and a full TX backlog would still
     * hold up the scheduler and the RX trigger behind it */
    AssertFatal(gNB->tx_split, "L1_tx_ru_split needs L1_tx_split\n");
    const int Ptx = gNB->gNB_config.carrier_config.num_tx_ant.value;
    gNB->tx_grid[0] = gNB->common_vars.txdataF;
    gNB->tx_grid[1] = (c16_t **)malloc16_clear(Ptx * sizeof(*gNB->tx_grid[1]));
    for (int j = 0; j < Ptx; j++)
      gNB->tx_grid[1][j] = (c16_t *)malloc16_clear(gNB->frame_parms.samples_per_slot_wCP * sizeof(**gNB->tx_grid[1]));
    gNB->tx_grid_seq = 0;
    AssertFatal(sem_init(&gNB->tx_grid_free, 0, 2) == 0, "sem_init: %s\n", strerror(errno));
    initNotifiedFIFO(&gNB->L1_tx_ru);
  }
  initNotifiedFIFO(&gNB->L1_rx_out);

  // create the RX thread responsible for RX processing start event (resp_L1 msg queue), then launch rx_func()
  threadCreate(&gNB->L1_rx_thread, L1_rx_thread, (void *)gNB, "L1_rx_thread", gNB->L1_rx_thread_core, OAI_PRIORITY_RT_MAX);
  // create the TX thread responsible for TX processing start event (L1_tx_out msg queue), then launch tx_func()
  threadCreate(&gNB->L1_tx_thread, L1_tx_thread, (void *)gNB, "L1_tx_thread", gNB->L1_tx_thread_core, OAI_PRIORITY_RT_MAX);
  if (gNB->tx_split) {
    LOG_I(NR_PHY, "L1 TX split: grid fill and RU path on L1_tx_gen (core %d), scheduler and RX trigger on L1_tx_thread\n",
          gNB->L1_tx_gen_thread_core);
    threadCreate(&gNB->L1_tx_gen_thread, L1_tx_gen_thread, (void *)gNB, "L1_tx_gen", gNB->L1_tx_gen_thread_core, OAI_PRIORITY_RT_MAX);
  }
  if (gNB->tx_ru_split) {
    LOG_I(NR_PHY, "L1 TX RU split: fronthaul on L1_tx_ru (core %d), overlapping the next grid fill, two grids\n",
          gNB->L1_tx_ru_thread_core);
    /* One step below the other L1 threads: on a core it shares with ru_thread (the natural
     * place, ru_thread mostly sleeps) the slot start must preempt a fronthaul send in progress,
     * not wait up to its ~130 us behind it at equal FIFO priority. */
    threadCreate(&gNB->L1_tx_ru_thread, L1_tx_ru_thread, (void *)gNB, "L1_tx_ru", gNB->L1_tx_ru_thread_core, OAI_PRIORITY_RT_MAX - 1);
  }

  if (!IS_SOFTMODEM_NOSTATS)
    threadCreate(&proc->L1_stats_thread, nrL1_stats_thread, (void *)gNB, "L1_stats", -1, OAI_PRIORITY_RT_LOW);
}

void term_gNB_Tpool(int inst) {
  PHY_VARS_gNB *gNB = RC.gNB[inst];
  abortNotifiedFIFO(&gNB->resp_L1);
  pthread_join(gNB->L1_rx_thread, NULL);
  abortNotifiedFIFO(&gNB->L1_tx_out);
  for (int i = 0; i < TX_SCHED_RING; i++) // tx_func() may wait for a ring entry tx_gen() will not free
    sem_post(&tx_sched_free);
  pthread_join(gNB->L1_tx_thread, NULL);
  if (gNB->tx_ru_split) // tx_gen() may wait for a grid L1_tx_ru will not return
    for (int i = 0; i < 2; i++)
      sem_post(&gNB->tx_grid_free);
  if (gNB->tx_split) {
    abortNotifiedFIFO(&gNB->L1_tx_gen);
    pthread_join(gNB->L1_tx_gen_thread, NULL);
  }
  if (gNB->tx_ru_split) {
    abortNotifiedFIFO(&gNB->L1_tx_ru);
    pthread_join(gNB->L1_tx_ru_thread, NULL);
    /* phy_free_nr_gNB() frees common_vars.txdataF: give it back the grid it allocated */
    gNB->common_vars.txdataF = gNB->tx_grid[0];
    const int Ptx = gNB->gNB_config.carrier_config.num_tx_ant.value;
    for (int j = 0; j < Ptx; j++)
      free_and_zero(gNB->tx_grid[1][j]);
    free_and_zero(gNB->tx_grid[1]);
  }

  if (gNB->threadPoolRx == &gNB->threadPoolRxOwn)
    abortTpool(&gNB->threadPoolRxOwn);
  if (gNB->threadPoolEnc == &gNB->threadPoolEncOwn)
    abortTpool(&gNB->threadPoolEncOwn);
  abortTpool(&gNB->threadPool);
  abortNotifiedFIFO(&gNB->L1_rx_out);

  gNB_L1_proc_t *proc = &gNB->proc;
  pthread_join(proc->L1_stats_thread, NULL);
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
