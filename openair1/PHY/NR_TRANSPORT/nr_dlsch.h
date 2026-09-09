/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief data structures for PDSCH/DLSCH/PUSCH/ULSCH physical and transport channel descriptors (TX/RX)
 */

#ifndef __NR_DLSCH__H
#define __NR_DLSCH__H

#include "PHY/defs_gNB.h"
#include "time_meas.h"

/* nr_generate_pdsch() split in two, so that the encoding of one slot can overlap the
   generation of another.  Both halves must be given the same slot's dlsch_array: the
   encode half fills freq_alloc / unav_res / pdsch_pdu, the generate half reads them. */
bool nr_dlsch_encode(PHY_VARS_gNB *gNB,
                     int n_dlsch,
                     NR_gNB_DLSCH_t *dlsch_array,
                     int frame,
                     int slot,
                     nr_dlsch_encoded_t *enc);

void nr_dlsch_generate(PHY_VARS_gNB *gNB,
                       int n_dlsch,
                       NR_gNB_DLSCH_t *dlsch_array,
                       int slot,
                       const nr_dlsch_encoded_t *enc,
                       uint64_t *pdsch_phase_comp_prb_mask,
                       int prb_mask_words);

/* Both halves back to back, on one slot. */
void nr_generate_pdsch(PHY_VARS_gNB *gNB,
                       int n_dlsch,
                       NR_gNB_DLSCH_t *dlsch_array,
                       int frame,
                       int slot,
                       uint64_t *pdsch_phase_comp_prb_mask,
                       int prb_mask_words);

int nr_dlsch_encoding(PHY_VARS_gNB *gNB,
                      int n_dlsch,
                      NR_gNB_DLSCH_t *dlsch_array,
                      int frame,
                      uint8_t slot,
                      unsigned char *output,
                      time_stats_t *tinput,
                      time_stats_t *tinput_memcpy,
                      time_stats_t *tprep,
                      time_stats_t *tparity,
                      time_stats_t *toutput,
                      time_stats_t *tconcat,
                      time_stats_t *dlsch_rate_matching_stats,
                      time_stats_t *dlsch_interleaving_stats,
                      time_stats_t *dlsch_segmentation_stats,
                      time_stats_t *dlsch_crc_stats);

void dump_pdsch_stats(FILE *fd,PHY_VARS_gNB *gNB);

#endif
