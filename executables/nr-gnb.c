/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief Top-level threads for gNodeB
 */

#define _GNU_SOURCE
#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all

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
#include "PHY/TOOLS/tools_defs.h"
#include "PHY/defs_RU.h"
#include "PHY/defs_gNB.h"
#include "PHY/defs_nr_common.h"
#include "PHY/impl_defs_nr.h"
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
#include "time_meas.h"
#include "utils.h"

#define TICK_TO_US(ts) (ts.trials==0?0:ts.diff/ts.trials)
#define L1STATSSTRLEN 16384
static void rx_func(processingData_L1_t *param);

static void tx_func(processingData_L1tx_t *info)
{
  int frame_tx = info->frame;
  int slot_tx = info->slot;
  int frame_rx = info->frame_rx;
  int slot_rx = info->slot_rx;
  LOG_D(NR_PHY, "%d.%d running tx_func\n", frame_tx, slot_tx);
  PHY_VARS_gNB *gNB = info->gNB;
  NR_IF_Module_t *ifi = gNB->if_inst;
  nfapi_nr_config_request_scf_t *cfg = &gNB->gNB_config;

  T(T_GNB_PHY_DL_TICK, T_INT(gNB->Mod_id), T_INT(frame_tx), T_INT(slot_tx));

  if (slot_rx == 0) {
    reset_active_stats(gNB, frame_rx);
    reset_active_ulsch(gNB, frame_rx);
  }

  clear_slot_beamid(gNB, slot_tx);

  nfapi_nr_slot_indication_scf_t ind = {.sfn = frame_tx, .slot = slot_tx};
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
  LOG_D(NR_PHY, "Trigger RX for %d.%d\n", frame_rx, slot_rx);
  notifiedFIFO_elt_t *res = newNotifiedFIFO_elt(sizeof(processingData_L1_t), 0, &gNB->resp_L1, NULL);
  processingData_L1_t *syncMsg = NotifiedFifoData(res);
  syncMsg->gNB = gNB;
  syncMsg->frame_rx = frame_rx;
  syncMsg->slot_rx = slot_rx;
  syncMsg->timestamp_tx = info->timestamp_tx;
  res->key = slot_rx;
  pushNotifiedFIFO(&gNB->resp_L1, res);

  int tx_slot_type = nr_slot_select(cfg, frame_tx, slot_tx);
  // TODO check for analog_bf_vendor_ext set to 1 is a workaround while no beam API for beam selection is implemented
  if (tx_slot_type == NR_DOWNLINK_SLOT || tx_slot_type == NR_MIXED_SLOT || get_softmodem_params()->continuous_tx
      || IS_SOFTMODEM_RFSIM || cfg->analog_beamforming_ve.analog_bf_vendor_ext.value) {
    start_meas(&info->gNB->phy_proc_tx);
    /* Split the measured region so the overrun alarm can say WHICH half was slow: PDCCH/PDSCH
     * generation, or the RU path (precoding + fronthaul BFP compression).  One aggregate
     * number cannot distinguish them, and they have entirely different causes. */
    const uint64_t t_gen_start = rdtsc_oai();
    phy_procedures_gNB_TX(info->gNB, &sched_response.DL_req, &sched_response.TX_req, &sched_response.UL_dci_req, frame_tx,slot_tx);
    const uint64_t t_gen_end = rdtsc_oai();

    PHY_VARS_gNB *gNB = info->gNB;
    processingData_RU_t syncMsgRU;
    syncMsgRU.frame_tx = frame_tx;
    syncMsgRU.slot_tx = slot_tx;
    syncMsgRU.ru = gNB->RU_list[0];
    syncMsgRU.timestamp_tx = info->timestamp_tx;
    LOG_D(PHY, "gNB: %d.%d : calling RU TX function\n", syncMsgRU.frame_tx, syncMsgRU.slot_tx);
    ru_tx_func((void *)&syncMsgRU);
    const uint64_t t_ru_end = rdtsc_oai();
    stop_meas(&info->gNB->phy_proc_tx);

    /* L1 TX overrun alarm.  A TX that misses its deadline still hands xran a buffer on
     * time, so the O-RU reports nothing wrong (RX_LATE ~0, RX_CORRUPT 0) -- only the
     * CONTENTS are stale, and the UE silently fails to decode the DCI.  The aggregate
     * time_stats cannot show this: it reports a mean plus a max that start_meas() zeroes
     * every 16384 trials, so an outlier is visible but its cause is not.  Report the
     * configuration that produced the overrun instead. */
    if (cpu_meas_enabled) {
      const double ghz_ = 1000.0 * get_cpu_freq_GHz();
      const double tot_ = (double)gNB->phy_proc_tx.p_time / ghz_;
      const double gen_ = (double)(t_gen_end - t_gen_start) / ghz_;
      AssertFatal(slot_tx < NR_MAX_SLOTS_PER_FRAME, "slot_tx %d out of range\n", slot_tx);
      typeof(gNB->tx_slot_stats[0]) *st = &gNB->tx_slot_stats[slot_tx];
      st->n++;
      st->tot_us += (uint64_t)tot_;
      st->gen_us += (uint64_t)gen_;
      st->ru_us += (uint64_t)((double)(t_ru_end - t_gen_end) / ghz_);
      if ((uint32_t)tot_ > st->max_us)
        st->max_us = (uint32_t)tot_;
      if ((uint32_t)gen_ > st->max_gen_us)
        st->max_gen_us = (uint32_t)gen_;
    }

    if (gNB->tx_overrun_us > 0 && cpu_meas_enabled) {
      const double us = (double)gNB->phy_proc_tx.p_time / (1000.0 * get_cpu_freq_GHz());
      if (us > gNB->tx_overrun_us) {
        gNB->tx_overrun_count++;
        /* Rate limit: the first 10, then one in 101.  An overrun storm must not add logging
         * to a thread that is already behind.
         *
         * The modulus is prime on purpose.  With one-in-100 and a TDD pattern giving 16 TX
         * slots per frame, gcd(100,16)=4, so the sampled slot phase-locks: a run at a 100 us
         * threshold logged 16600 lines that were ALL slots 3/8/13/18 and never once visited
         * the other twelve.  Anything read off these lines is then a property of the
         * aliasing, not of the DU.  A prime cannot share a factor with the frame structure.
         * For per-slot statistics use tx_slot_stats[] below, which counts every slot. */
        if (gNB->tx_overrun_count <= 10 || (gNB->tx_overrun_count % 101) == 0) {
          gNB->tx_overrun_logged++;
          const nfapi_nr_dl_tti_request_body_t *b = &sched_response.DL_req.dl_tti_request_body;
          int n_pdsch = 0;
          char cfg[256];
          int off = 0;
          for (int i = 0; i < b->nPDUs && off < (int)sizeof(cfg) - 64; i++) {
            if (b->dl_tti_pdu_list[i].PDUType != NFAPI_NR_DL_TTI_PDSCH_PDU_TYPE)
              continue;
            const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *p = &b->dl_tti_pdu_list[i].pdsch_pdu.pdsch_pdu_rel15;
            n_pdsch++;
            off += snprintf(cfg + off,
                            sizeof(cfg) - off,
                            " [rnti %04x mcs %d rb %d+%d layers %d symb %d+%d tbs %u]",
                            p->rnti,
                            p->mcsIndex[0],
                            p->rbStart,
                            p->rbSize,
                            p->nrOfLayers,
                            p->StartSymbolIndex,
                            p->NrOfSymbols,
                            p->TBSize[0]);
          }
          /* precoding_stats and tx_fhaul carry p_time from THIS slot's call, so the RU half
           * breaks down further without extra instrumentation. */
          const double ghz = 1000.0 * get_cpu_freq_GHz();
          const RU_t *ru = gNB->RU_list[0];
          LOG_W(NR_PHY,
                "%4d.%2d L1 TX overrun: %.1f us > %d us (gen %.1f, ru %.1f = prec %.1f + fhaul %.1f)"
                " (%llu total, %d PDSCH PDU%s)%s\n",
                frame_tx,
                slot_tx,
                us,
                gNB->tx_overrun_us,
                (double)(t_gen_end - t_gen_start) / ghz,
                (double)(t_ru_end - t_gen_end) / ghz,
                (double)ru->precoding_stats.p_time / ghz,
                (double)ru->tx_fhaul.p_time / ghz,
                (unsigned long long)gNB->tx_overrun_count,
                n_pdsch,
                n_pdsch == 1 ? "" : "s",
                n_pdsch ? cfg : " none");
        }
      }
    }
  }
}

void *L1_rx_thread(void *arg) 
{
  PHY_VARS_gNB *gNB = (PHY_VARS_gNB*)arg;

  while (oai_exit == 0) {
     notifiedFIFO_elt_t *res = pullNotifiedFIFO(&gNB->resp_L1);
     if (res == NULL)
       break;
     processingData_L1_t *info = (processingData_L1_t *)NotifiedFifoData(res);
     const int slot_rx = info->slot_rx;
     start_meas(&gNB->l1_rx_proc);
     rx_func(info);
     stop_meas(&gNB->l1_rx_proc);
     if (cpu_meas_enabled) {
       AssertFatal(slot_rx < NR_MAX_SLOTS_PER_FRAME, "slot_rx %d out of range\n", slot_rx);
       const double us = (double)gNB->l1_rx_proc.p_time / (1000.0 * get_cpu_freq_GHz());
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
     start_meas(&gNB->l1_tx_proc);
     tx_func(info);
     stop_meas(&gNB->l1_tx_proc);
     delNotifiedFIFO_elt(res);
  }
  return NULL;
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
    NR_UL_IND_t UL_INFO = {.frame = frame_rx, .slot = slot_rx, .module_id = gNB->Mod_id, .CC_id = gNB->CC_id};
    // Do PRACH RU processing
    UL_INFO.rach_ind.pdu_list = UL_INFO.prach_pdu_indication_list;
    UL_INFO.rach_ind.number_of_pdus = 0;
    // even if processing is late, we might collect all PRACH
    // the last PRACH's frame/slot is when all UE's appear to have accessed
    prach_item_t p;
    while (spsc_q_get(&gNB->prach_l1rx_queue, &p, sizeof(p)))
      L1_nr_prach_procedures(gNB, &p, &UL_INFO.rach_ind);

    //WA: comment rotation in tx/rx
    if (gNB->phase_comp) {
      //apply the rx signal rotation here
      int soffset = (slot_rx % RU_RX_SLOT_DEPTH) * gNB->frame_parms.symbols_per_slot * gNB->frame_parms.ofdm_symbol_size;
      const NR_DL_FRAME_PARMS *fp = &gNB->frame_parms;
      for (int aa = 0; aa < fp->nb_antennas_rx; aa++) {
        const uint max_symb = fp->Ncp == NR_EXTENDED ? 12 : 14;
        for (int sym = 0; sym < max_symb; sym++)
          apply_nr_rotation_symbol_RX(fp->symbols_per_slot,
                                      fp->slots_per_subframe,
                                      fp->timeshift_symbol_rotation,
                                      fp->first_carrier_offset,
                                      gNB->common_vars.rxdataF[aa] + soffset + sym * fp->ofdm_symbol_size,
                                      fp->symbol_rotation[1],
                                      fp->N_RB_UL,
                                      slot_rx,
                                      sym);
      }
    }
    phy_procedures_gNB_uespec_RX(gNB, frame_rx, slot_rx, &UL_INFO);

    // Call the scheduler
    start_meas(&gNB->ul_indication_stats);
    gNB->if_inst->NR_UL_indication(&UL_INFO);
    stop_meas(&gNB->ul_indication_stats);

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

static size_t dump_L1_meas_stats(PHY_VARS_gNB *gNB, RU_t *ru, char *output, size_t outputlen) {
  const char *begin = output;
  const char *end = output + outputlen;
  output += print_meas_log(&gNB->l1_tx_proc, "L1 Tx job", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->l1_rx_proc, "L1 Rx job", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->phy_proc_tx, "L1 Tx processing", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->dlsch_encoding_stats, "DLSCH encoding", NULL, NULL, output, end - output);
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
  output += print_meas_log(&gNB->ts_deinterleave, "UL segment deinterleaving", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ts_rate_unmatch, "UL segment rate recovery", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ts_ldpc_decode, "UL segments decoding", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->ul_indication_stats, "UL Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->slot_indication_stats, "Slot Indication", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_pusch_stats, "PUSCH inner-receiver", NULL, NULL, output, end - output);
  output += print_meas_log(&gNB->rx_prach, "PRACH RX", NULL, NULL, output, end - output);
  if (ru->feprx)
    output += print_meas_log(&ru->ofdm_demod_stats, "feprx", NULL, NULL, output, end - output);

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

  return output - begin;
}

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

  reset_meas(&gNB->l1_tx_proc);
  reset_meas(&gNB->l1_rx_proc);
  reset_meas(&gNB->phy_proc_tx);
  reset_meas(&gNB->dlsch_encoding_stats);
  reset_meas(&gNB->phy_proc_rx);
  reset_meas(&gNB->ts_deinterleave);
  reset_meas(&gNB->ts_rate_unmatch);
  reset_meas(&gNB->ts_ldpc_decode);
  reset_meas(&gNB->ul_indication_stats);
  reset_meas(&gNB->slot_indication_stats);
  reset_meas(&gNB->rx_pusch_stats);
  reset_meas(&gNB->dlsch_scrambling_stats);
  reset_meas(&gNB->dlsch_modulation_stats);
  reset_meas(&gNB->dlsch_pdsch_generation_stats);
  reset_meas(&gNB->dlsch_resource_mapping_stats);
  reset_meas(&gNB->dlsch_precoding_stats);
  memset(gNB->rx_slot_stats, 0, sizeof(gNB->rx_slot_stats));
  memset(gNB->tx_slot_stats, 0, sizeof(gNB->tx_slot_stats));
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

  if (gNB->threadPoolRx == &gNB->threadPoolRxOwn)
    abortTpool(&gNB->threadPoolRxOwn);
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
