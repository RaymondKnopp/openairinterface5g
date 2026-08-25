/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef __openair_SCHED_NR_DEFS_H__
#define __openair_SCHED_NR_DEFS_H__

#include "PHY/defs_gNB.h"
#include "PHY/NR_TRANSPORT/nr_dci.h"
#include "phy_frame_config_nr.h"

void nr_set_ssb_first_subcarrier(nfapi_nr_config_request_scf_t *cfg, NR_DL_FRAME_PARMS *fp);
void phy_procedures_gNB_TX(PHY_VARS_gNB *gNB,
                           const nfapi_nr_dl_tti_request_t *DL_req,
                           const nfapi_nr_tx_data_request_t *TX_req,
                           const nfapi_nr_ul_dci_request_t *UL_dci_req,
                           int frame,
                           int slot);
void nr_save_ul_tti_req(PHY_VARS_gNB *gNB, nfapi_nr_ul_tti_request_t *UL_tti_req);
int phy_procedures_gNB_uespec_RX(PHY_VARS_gNB *gNB, int frame_rx, int slot_rx, NR_UL_IND_t *UL_INFO);

/* What the PUSCH back half needs from the front half, and nothing more -- the
 * boundary turned out to be this small. Used to defer the LDPC decode and its
 * rx_data/CRC indications so they overlap the NEXT slot's front end. */
int nr_l1_rx_defer_enabled(void);

/* Which of the two LLR buffers this slot owns.
 *
 * With the back half deferred, slot N's decode reads the LLRs while slot N+1's
 * inner receiver is already writing, so they must not be the same buffer. Keyed
 * on slot parity rather than by swapping a shared pointer, because two threads
 * are involved and a swap would race. llr_alt is only allocated when deferral is
 * enabled, so when it is NULL this returns exactly what the code used before. */
static inline int16_t *nr_pusch_llr_buf(const NR_gNB_PUSCH *pv, int slot)
{
  return (pv->llr_alt && (slot & 1)) ? pv->llr_alt : pv->ulsch_llrs;
}

/* Power measurements for the slot the back half is working on. When deferral is
 * off, pwr_d_valid is false and this returns the live values, i.e. exactly what
 * the code read before. */
static inline void nr_pusch_pwr(const NR_gNB_PUSCH *pv,
                                int slot,
                                const uint32_t **power,
                                const uint32_t **noise,
                                uint32_t *power_tot,
                                uint32_t *noise_tot)
{
  const int p = slot & 1;
  if (pv->pwr_d_valid) {
    *power = pv->ulsch_power_d[p];
    *noise = pv->ulsch_noise_power_d[p];
    *power_tot = pv->ulsch_power_tot_d[p];
    *noise_tot = pv->ulsch_noise_power_tot_d[p];
  } else {
    *power = pv->ulsch_power;
    *noise = pv->ulsch_noise_power;
    *power_tot = pv->ulsch_power_tot;
    *noise_tot = pv->ulsch_noise_power_tot;
  }
}

typedef struct {
  bool valid;
  int frame;
  int slot;
  int num_pusch;
  int ulsch_idx[MAX_UL_PDUS_PER_SLOT];
} nr_pusch_defer_t;

int phy_procedures_gNB_uespec_RX_defer(PHY_VARS_gNB *gNB,
                                       int frame_rx,
                                       int slot_rx,
                                       NR_UL_IND_t *UL_INFO,
                                       nr_pusch_defer_t *defer_out);
void nr_pusch_decode_and_indicate(PHY_VARS_gNB *gNB,
                                  int frame_rx,
                                  int slot_rx,
                                  const int *ulsch_idx_to_decode,
                                  int num_pusch,
                                  NR_UL_IND_t *UL_INFO);
void L1_nr_prach_procedures(PHY_VARS_gNB *gNB, prach_item_t *prach_id, nfapi_nr_rach_indication_t *rach_ind);
void nr_common_signal_procedures(PHY_VARS_gNB *gNB,
                                 int frame,
                                 int slot,
                                 const nfapi_nr_dl_tti_ssb_pdu *ssb_pdu,
                                 int prb_mask_words);
void nr_feptx_ofdm(RU_t *ru,int frame_tx,int tti_tx);
void nr_feptx0(RU_t *ru,int tti_tx,int first_symbol, int num_symbols, int aa);
void nr_feptx_prec(RU_t *ru,int frame_tx,int tti_tx);
void nr_feptx_prec_control(RU_t *ru,int frame,int tti_tx);
void nr_fep_tp(RU_t *ru, int slot);
void nr_feptx_tp(RU_t *ru, int frame_tx, int slot);
void feptx_prec(RU_t *ru,int frame_tx,int tti_tx);
void nr_phy_init_RU(RU_t *ru);
void nr_phy_free_RU(RU_t *ru);
void clear_slot_beamid(PHY_VARS_gNB *gNB, int slot);
void beam_index_allocation(uint16_t fapi_beam_index,
                           int ant,
                           int num_ports,
                           int symbols_per_slot,
                           int slot,
                           uint16_t bitmap_symbols,
                           int num_ant_max,
                           uint16_t **ant_beam_id_list);
uint16_t get_first_ant_idx(bool das, uint16_t num_ports_beams, uint16_t beam_id, uint16_t fapi_start_port);
#endif
