/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*!
 * \brief Top-level routines for transmission of the PDSCH 38211 v 15.2.0
 */

#include "nr_dlsch.h"
#include "nr_dci.h"
#include "nr_sch_dmrs.h"
#include "PHY/MODULATION/nr_modulation.h"
#include "PHY/NR_REFSIG/nr_refsig.h"
#include "PHY/NR_REFSIG/dmrs_nr.h"
#include "PHY/NR_REFSIG/ptrs_nr.h"
#include "common/utils/nr/nr_common.h"
#include "PHY/NR_TRANSPORT/nr_transport_common_proto.h"
#include "PHY/nr_phy_common/inc/nr_phy_meas.h"
#include "executables/softmodem-common.h"
#include "SCHED_NR/sched_nr.h"

#include "utils.h"
#include <simde/x86/avx512.h>
#define USE128BIT

static void nr_pdsch_codeword_scrambling(uint8_t *in, uint32_t size, uint8_t q, uint32_t Nid, uint32_t n_RNTI, uint32_t *out)
{
  nr_codeword_scrambling(in, size, q, Nid, n_RNTI, out);
}

static uint32_t get_block_start_sc(int block_start, int bwp_start, int symbol_sz)
{
  return (block_start + bwp_start) * NR_NB_SC_PER_RB;
}

static int do_ptrs_symbol(const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                          const freq_alloc_bitmap_t *freq_alloc,
                          int symbol_sz,
                          c16_t *txF,
                          c16_t *tx_layer,
                          int amp,
                          c16_t *mod_ptrs)
{
  int ptrs_idx = 0;
  c16_t *in = tx_layer;
  int last_rb = freq_alloc->last_rb;
  int first_rb = freq_alloc->first_rb;
  uint32_t start_sc = get_block_start_sc(freq_alloc->first_rb, rel15->BWPStart, symbol_sz);
  int k = start_sc;
  for (int j = first_rb; j <= last_rb; j++) {
    if (check_rb_in_bitmap(freq_alloc, j)) {
      for (int i = 0; i < NR_NB_SC_PER_RB; i++) {
        /* check for PTRS symbol and set flag for PTRS RE */
        bool is_ptrs_re =
            is_ptrs_subcarrier(k, rel15->rnti, rel15->PTRSFreqDensity, freq_alloc->num_rbs, rel15->PTRSReOffset, start_sc, symbol_sz);
        if (is_ptrs_re) {
          /* check if cuurent RE is PTRS RE*/
          uint16_t beta_ptrs = 1;
          txF[k] = c16mulRealShift(mod_ptrs[ptrs_idx], beta_ptrs * amp, 15);
          ptrs_idx++;
        } else {
          txF[k] = c16mulRealShift(*in++, amp, 15);
        }
        k++;
      }
    } else {
      k += NR_NB_SC_PER_RB;
    }
  }
  return in - tx_layer;
}

typedef union {
  uint64_t l;
  c16_t s[2];
} amp_t;

static inline int interleave_with_0_signal_first(c16_t *output, c16_t *mod_dmrs, const int16_t amp_dmrs, int sz)
{
  // add filler to process all as SIMD
  c16_t *out = output;
  int i = 0;
  int end = sz / 2;
#if defined(__AVX512BW__)
  simde__m512i zeros512 = simde_mm512_setzero_si512(), amp_dmrs512 = simde_mm512_set1_epi16(amp_dmrs);
  simde__m512i perml = simde_mm512_set_epi32(23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0);
  simde__m512i permh = simde_mm512_set_epi32(31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25, 9, 24, 8);
  for (; i < (end & ~15); i += 16) {
    simde__m512i d0 = simde_mm512_mulhrs_epi16(_mm512_loadu_si512((simde__m512i *)(mod_dmrs + i)), amp_dmrs512);
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(d0, perml, zeros512));
    out += 16;
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(d0, permh, zeros512));
    out += 16;
  }
#endif
#if defined(__AVX2__)
  simde__m256i zeros256 = simde_mm256_setzero_si256(), amp_dmrs256 = simde_mm256_set1_epi16(amp_dmrs);
  for (; i < (end & ~7); i += 8) {
    simde__m256i d0 = simde_mm256_mulhrs_epi16(simde_mm256_loadu_si256((simde__m256i *)(mod_dmrs + i)), amp_dmrs256);
    simde__m256i d2 = simde_mm256_unpacklo_epi32(d0, zeros256);
    simde__m256i d3 = simde_mm256_unpackhi_epi32(d0, zeros256);
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 32));
    out += 8;
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 49));
    out += 8;
  }
#endif
#if defined(USE128BIT)
  simde__m128i zeros = simde_mm_setzero_si128(), amp_dmrs128 = simde_mm_set1_epi16(amp_dmrs);
  for (; i < (end & ~3); i += 4) {
    simde__m128i d0 = simde_mm_mulhrs_epi16(simde_mm_loadu_si128((simde__m128i *)(mod_dmrs + i)), amp_dmrs128);
    simde__m128i d2 = simde_mm_unpacklo_epi32(d0, zeros);
    simde__m128i d3 = simde_mm_unpackhi_epi32(d0, zeros);
    simde_mm_storeu_si128((simde__m128i *)out, d2);
    out += 4;
    simde_mm_storeu_si128((simde__m128i *)out, d3);
    out += 4;
  }
#endif
  for (; i < end; i++) {
    *out++ = c16mulRealShift(mod_dmrs[i], amp_dmrs, 15);
    *out++ = (c16_t){};
  }
  return 0;
}

static inline int interleave_with_0_start_with_0(c16_t *output, c16_t *mod_dmrs, const int16_t amp_dmrs, int sz)
{
  c16_t *out = output;
  int i = 0;
  int end = sz / 2;
#if defined(__AVX512BW__)
  simde__m512i zeros512 = simde_mm512_setzero_si512(), amp_dmrs512 = simde_mm512_set1_epi16(amp_dmrs);
  simde__m512i perml = simde_mm512_set_epi32(23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0);
  simde__m512i permh = simde_mm512_set_epi32(31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25, 9, 24, 8);
  for (; i < (end & ~15); i += 16) {
    simde__m512i d0 = simde_mm512_mulhrs_epi16(_mm512_loadu_si512((simde__m512i *)(mod_dmrs + i)), amp_dmrs512);
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(zeros512, perml, d0));
    out += 16;
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(zeros512, permh, d0));
    out += 16;
  }
#endif
#if defined(__AVX2__)
  simde__m256i zeros256 = simde_mm256_setzero_si256(), amp_dmrs256 = simde_mm256_set1_epi16(amp_dmrs);
  for (; i < (end & ~7); i += 8) {
    simde__m256i d0 = simde_mm256_mulhrs_epi16(simde_mm256_loadu_si256((simde__m256i *)(mod_dmrs + i)), amp_dmrs256);
    simde__m256i d2 = simde_mm256_unpacklo_epi32(zeros256, d0);
    simde__m256i d3 = simde_mm256_unpackhi_epi32(zeros256, d0);
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 32));
    out += 8;
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 49));
    out += 8;
  }
#endif
#if defined(USE128BIT)
  simde__m128i zeros = simde_mm_setzero_si128(), amp_dmrs128 = simde_mm_set1_epi16(amp_dmrs);
  for (; i < (end & ~3); i += 4) {
    simde__m128i d0 = simde_mm_mulhrs_epi16(simde_mm_loadu_si128((simde__m128i *)(mod_dmrs + i)), amp_dmrs128);
    simde__m128i d2 = simde_mm_unpacklo_epi32(zeros, d0);
    simde__m128i d3 = simde_mm_unpackhi_epi32(zeros, d0);
    simde_mm_storeu_si128((simde__m128i *)out, d2);
    out += 4;
    simde_mm_storeu_si128((simde__m128i *)out, d3);
    out += 4;
  }
#endif
  for (; i < end; i++) {
    *out++ = (c16_t){};
    *out++ = c16mulRealShift(mod_dmrs[i], amp_dmrs, 15);
  }
  return 0;
}

static inline int interleave_signals(c16_t *output, c16_t *signal1, const int amp, c16_t *signal2, const int amp2, int sz)
{
    // add filler to process all as SIMD
  c16_t *out = output;
  int i = 0;
  int end = sz / 2;
#if defined(__AVX512BW__)
  simde__m512i amp2512 = simde_mm512_set1_epi16(amp2), amp512 = simde_mm512_set1_epi16(amp);
  simde__m512i perml = simde_mm512_set_epi32(23, 7, 22, 6, 21, 5, 20, 4, 19, 3, 18, 2, 17, 1, 16, 0);
  simde__m512i permh = simde_mm512_set_epi32(31, 15, 30, 14, 29, 13, 28, 12, 27, 11, 26, 10, 25, 9, 24, 8);
  for (; i < (end & ~15); i += 16) {
    simde__m512i d0 = simde_mm512_mulhrs_epi16(_mm512_loadu_si512((simde__m512i *)(signal2 + i)), amp2512);
    simde__m512i d1 = simde_mm512_mulhrs_epi16(_mm512_loadu_si512((simde__m512i *)(signal1 + i)), amp512);
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(d0, perml, d1));
    out += 16;
    simde_mm512_storeu_si512((simde__m512i *)out, simde_mm512_permutex2var_epi32(d0, permh, d1));
    out += 16;
  }
#endif
#if defined(__AVX2__)
  simde__m256i amp2256 = simde_mm256_set1_epi16(amp2), amp256 = simde_mm256_set1_epi16(amp);
  for (; i < (end & ~7); i += 8) {
    simde__m256i d0 = simde_mm256_mulhrs_epi16(simde_mm256_loadu_si256((simde__m256i *)(signal2 + i)), amp2256);
    simde__m256i d1 = simde_mm256_mulhrs_epi16(simde_mm256_loadu_si256((simde__m256i *)(signal1 + i)), amp256);
    simde__m256i d2 = simde_mm256_unpacklo_epi32(d0, d1);
    simde__m256i d3 = simde_mm256_unpackhi_epi32(d0, d1);
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 32));
    out += 8;
    simde_mm256_storeu_si256((simde__m256i *)out, simde_mm256_permute2x128_si256(d2, d3, 49));
    out += 8;
  }
#endif
#if defined(USE128BIT)
  simde__m128i amp2128 = simde_mm_set1_epi16(amp2), amp128 = simde_mm_set1_epi16(amp);
  for (; i < (end & ~3); i += 4) {
    simde__m128i d0 = simde_mm_mulhrs_epi16(simde_mm_loadu_si128((simde__m128i *)(signal2 + i)), amp2128);
    simde__m128i d1 = simde_mm_mulhrs_epi16(simde_mm_loadu_si128((simde__m128i *)(signal1 + i)), amp128);
    simde__m128i d2 = simde_mm_unpacklo_epi32(d0, d1);
    simde__m128i d3 = simde_mm_unpackhi_epi32(d0, d1);
    simde_mm_storeu_si128((simde__m128i *)out, d2);
    out += 4;
    simde_mm_storeu_si128((simde__m128i *)out, d3);
    out += 4;
  }
#endif
  for (; i < end; i++) {
    *out++ = c16mulRealShift(signal2[i], amp2, 15);
    *out++ = c16mulRealShift(signal1[i], amp, 15);
  }
  return sz / 2;
}

static inline int dmrs_case00(c16_t *output,
                              c16_t *txl,
                              c16_t *mod_dmrs,
                              const freq_alloc_bitmap_t *freq_alloc,
                              const int16_t amp_dmrs,
                              const int amp,
                              int dmrs_port,
                              const int dmrs_Type,
                              int symbol_sz,
                              int l_prime,
                              const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15)
{
  // DMRS params for this dmrs port
  int Wt[2], Wf[2];
  get_Wt(Wt, dmrs_port, dmrs_Type);
  get_Wf(Wf, dmrs_port, dmrs_Type);
  const int8_t delta = get_delta(dmrs_port, dmrs_Type);
  int dmrs_idx = 0;
  uint32_t start_sc = get_block_start_sc(freq_alloc->first_rb, rel15->BWPStart, symbol_sz);
  int k = start_sc;
  c16_t *in = txl;
  uint8_t k_prime = 0;
  uint16_t n = 0;
  int pos = freq_alloc->first_rb;
  int last_processed_rb = freq_alloc->first_rb;
  int block_start, block_end;
  while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
    // Mute skipped RBs
    int skipped_sc = (block_start - last_processed_rb) * NR_NB_SC_PER_RB;
    memset(output + k, 0, skipped_sc * sizeof(c16_t));
    k += skipped_sc;
    for (int j = block_start; j <= block_end; j++) {
      for (int i = 0; i < NR_NB_SC_PER_RB; i++) {
        if (k == ((start_sc + get_dmrs_freq_idx(n, k_prime, delta, dmrs_Type)) % (symbol_sz))) {
          output[k] = c16mulRealShift(mod_dmrs[dmrs_idx], Wt[l_prime] * Wf[k_prime] * amp_dmrs, 15);
          dmrs_idx++;
          k_prime = (k_prime + 1) & 1;
          n += (k_prime ? 0 : 1);
        } else if (allowed_xlsch_re_in_dmrs_symbol(k, start_sc, symbol_sz, rel15->numDmrsCdmGrpsNoData, dmrs_Type)) {
          /* Map PTRS Symbol */
          /* Map DATA Symbol */
          output[k] = c16mulRealShift(*in++, amp, 15);
        } else {
          output[k] = (c16_t){0};
        }
        k++;
      }
    }
    last_processed_rb = block_end + 1;
  } // RE loop
  return in - txl;
}

static inline int no_ptrs_dmrs_case(c16_t *output, c16_t *txl, const int amp, const int sz)
{
  // Loop Over SCs:
  int i = 0;
#if defined(__AVX512BW__)
  simde__m512i amp512 = simde_mm512_set1_epi16(amp);
  for (; i < (sz & ~15); i += 16) {
    const simde__m512i txL = simde_mm512_loadu_si512((simde__m512i *)(txl + i));
    simde_mm512_storeu_si512((simde__m512i *)(output + i), simde_mm512_mulhrs_epi16(amp512, txL));
  }
#endif
#if defined(__AVX2__)
  simde__m256i amp256 = simde_mm256_set1_epi16(amp);
  for (; i < (sz & ~7); i += 8) {
    const simde__m256i txL = simde_mm256_loadu_si256((simde__m256i *)(txl + i));
    simde_mm256_storeu_si256((simde__m256i *)(output + i), _mm256_mulhrs_epi16(amp256, txL));
  }
#endif
#if defined(USE128BIT)
  simde__m128i amp128 = simde_mm_set1_epi16(amp);
  for (; i < (sz & ~3); i += 4) {
    const simde__m128i txL = simde_mm_loadu_si128((simde__m128i *)(txl + i));
    simde_mm_storeu_si128((simde__m128i *)(output + i), simde_mm_mulhrs_epi16(amp128, txL));
  }
#endif
  for (; i < sz; i++) {
    output[i] = c16mulRealShift(txl[i], amp, 15);
  }
  return sz;
}

static inline void neg_dmrs(c16_t *in, c16_t *out, int sz)
{
  for (int i = 0; i < sz; i++)
    *out++ = i % 2 ? (c16_t){-in[i].r, -in[i].i} : in[i];
}

static inline void do_onelayer(NR_DL_FRAME_PARMS *frame_parms,
                               int slot,
                               const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                               const freq_alloc_bitmap_t *freq_alloc,
                               int layer,
                               c16_t *output,
                               c16_t *txl_start,
                               int symbol_sz,
                               int l_symbol,
                               uint16_t dlPtrsSymPos,
                               int n_ptrs,
                               int amp,
                               int16_t amp_dmrs,
                               int l_prime,
                               nfapi_nr_dmrs_type_e dmrs_Type,
                               c16_t *dmrs_start)
{
  c16_t *txl = txl_start;

  /* calculate if current symbol is PTRS symbols */
  int ptrs_symbol = 0;
  if (rel15->pduBitmap & 0x1) {
    ptrs_symbol = IS_BIT_SET(dlPtrsSymPos, l_symbol);
  }

  if (ptrs_symbol) {
    /* PTRS QPSK Modulation for each OFDM symbol in a slot */
    LOG_D(PHY, "Doing ptrs modulation for symbol %d, n_ptrs %d\n", l_symbol, n_ptrs);
    c16_t mod_ptrs[max(n_ptrs, 1)]
        __attribute__((aligned(64))); // max only to please sanitizer, that kills if 0 even if it is not a error
    const uint32_t *gold =
        nr_gold_pdsch(frame_parms->N_RB_DL, frame_parms->symbols_per_slot, rel15->dlDmrsScramblingId, rel15->SCID, slot, l_symbol);
    nr_modulation(gold, n_ptrs * DMRS_MOD_ORDER, DMRS_MOD_ORDER, (int16_t *)mod_ptrs);
    txl += do_ptrs_symbol(rel15, freq_alloc, symbol_sz, output, txl, amp, mod_ptrs);

  } else if (rel15->dlDmrsSymbPos & (1 << l_symbol)) {
    /* Map DMRS Symbol */
    int dmrs_port = get_dmrs_port(layer, rel15->dmrsPorts);
    if (l_prime == 0 && dmrs_Type == NFAPI_NR_DMRS_TYPE1) {
      int pos = 0;
      int block_start, block_end;
      while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
        int start_rb = block_start;
        int nb_rb = block_end - block_start + 1;
        int start_sc = get_block_start_sc(start_rb, rel15->BWPStart, symbol_sz);
        const int sz = nb_rb * NR_NB_SC_PER_RB;
        if (rel15->numDmrsCdmGrpsNoData == 2) {
          switch (dmrs_port & 3) {
            case 0:
              txl += interleave_with_0_signal_first(output + start_sc, dmrs_start, amp_dmrs, sz);
              break;
            case 1: {
              c16_t dmrs[sz / 2];
              neg_dmrs(dmrs_start, dmrs, sz / 2);
              txl += interleave_with_0_signal_first(output + start_sc, dmrs, amp_dmrs, sz);
            } break;
            case 2:
              txl += interleave_with_0_start_with_0(output + start_sc, dmrs_start, amp_dmrs, sz);
              break;
            case 3: {
              c16_t dmrs[sz / 2];
              neg_dmrs(dmrs_start, dmrs, sz / 2);
              txl += interleave_with_0_start_with_0(output + start_sc, dmrs, amp_dmrs, sz);
            } break;
          }
        } else if (rel15->numDmrsCdmGrpsNoData == 1) {
          switch (dmrs_port & 3) {
            case 0:
              txl += interleave_signals(output + start_sc, txl, amp, dmrs_start, amp_dmrs, sz);
              break;
            case 1: {
              c16_t dmrs[sz / 2];
              neg_dmrs(dmrs_start, dmrs, sz / 2);
              txl += interleave_signals(output + start_sc, txl, amp, dmrs, amp_dmrs, sz);
            } break;
            case 2:
              txl += interleave_signals(output + start_sc, dmrs_start, amp_dmrs, txl, amp, sz);
              break;
            case 3: {
              c16_t dmrs[sz / 2];
              neg_dmrs(dmrs_start, dmrs, sz / 2);
              txl += interleave_signals(output + start_sc, dmrs, amp_dmrs, txl, amp, sz);
            } break;
          }
        } else
          AssertFatal(false, "rel15->numDmrsCdmGrpsNoData is %d\n", rel15->numDmrsCdmGrpsNoData);
      }
    } else {
      txl += dmrs_case00(output,
                         txl,
                         dmrs_start,
                         freq_alloc,
                         amp_dmrs,
                         amp,
                         dmrs_port,
                         dmrs_Type,
                         symbol_sz,
                         l_prime,
                         rel15);
    } // generic DMRS case
  } else { // no PTRS or DMRS in this symbol
    int pos = 0;
    int block_start, block_end;
    while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
      int start_rb = block_start;
      int nb_rb = block_end - block_start + 1;
      int start_sc = get_block_start_sc(start_rb, rel15->BWPStart, symbol_sz);
      const int sz = nb_rb * NR_NB_SC_PER_RB;
      txl += no_ptrs_dmrs_case(output + start_sc, txl, amp, sz);
    }
  } // no DMRS/PTRS in symbol
  return;
}

static inline void do_txdataF(c16_t **txdataF,
                              int symbol_sz,
                              c16_t txdataF_precoding[][symbol_sz],
                              PHY_VARS_gNB *gNB,
                              const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                              int ant,
                              int rb_start,
                              int rb_size,
                              int txdataF_offset_per_symbol)
{
  NR_DL_FRAME_PARMS *frame_parms = &gNB->frame_parms;
  int rb = 0;
  uint16_t subCarrier = get_block_start_sc(rb_start, rel15->BWPStart, symbol_sz);
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  while (rb < rb_size) {
    // get pmi info
    const int pmi = (pb->prg_size > 0) ? (pb->prgs_list[(int)rb / pb->prg_size].pm_idx) : 0;
    const int pmi2 = (rb < (rb_size - 1) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 1) / pb->prg_size].pm_idx) : -1;
    const int pmi3 = (rb < (rb_size - 2) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 2) / pb->prg_size].pm_idx) : -1;
    const int pmi4 = (rb < (rb_size - 3) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 3) / pb->prg_size].pm_idx) : -1;

    // If pmi of next RB and pmi of current RB are the same, we do 2 RB in a row
    // if pmi differs, or current rb is the end (rb_size - 1), than we do 1 RB in a row
    int rb_step0 = pmi == pmi2 ? 2 : 1;
    const int rb_step = rb_step0 == 2 && pmi3 == pmi && pmi4 == pmi ? 4 : rb_step0;
    const int re_cnt = NR_NB_SC_PER_RB * rb_step;
    if (pmi == 0) { // unitary Precoding
      if (ant < rel15->nrOfLayers)
        memcpy(&txdataF[ant][txdataF_offset_per_symbol + subCarrier],
               &txdataF_precoding[ant][subCarrier],
               re_cnt * sizeof(**txdataF));
      else
        memset(&txdataF[ant][txdataF_offset_per_symbol + subCarrier], 0, re_cnt * sizeof(**txdataF));
      subCarrier += re_cnt;
    } else { // non-unitary Precoding
      AssertFatal(frame_parms->nb_antennas_tx > 1, "No precoding can be done with a single antenna port\n");
      // get the precoding matrix weights:
      nfapi_nr_pm_pdu_t *pmi_pdu = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1]; // pmi 0 is identity matrix
      AssertFatal(pmi == pmi_pdu->pm_idx, "PMI %d doesn't match to the one in precoding matrix %d\n", pmi, pmi_pdu->pm_idx);
      AssertFatal(ant < pmi_pdu->num_ant_ports,
                  "Antenna port index %d exceeds precoding matrix AP size %d\n",
                  ant,
                  pmi_pdu->num_ant_ports);
      AssertFatal(rel15->nrOfLayers == pmi_pdu->numLayers,
                  "Number of layers %d doesn't match to the one in precoding matrix %d\n",
                  rel15->nrOfLayers,
                  pmi_pdu->numLayers);
      nr_layer_precoder_simd(rel15->nrOfLayers,
                             symbol_sz,
                             txdataF_precoding,
                             ant,
                             pmi_pdu->weights,
                             subCarrier,
                             re_cnt,
                             &txdataF[ant][txdataF_offset_per_symbol]);
      subCarrier += re_cnt;
    } // else { // non-unitary Precoding

    rb += rb_step;
  } // RB loop: while(rb < rb_size)
}

/* The 2-port/2-layer butterfly precoder is faster than the generic per-antenna
   kernel only where the generic complex multiply-accumulate is comparatively
   expensive: on x86 (AVX2/AVX-512), and on aarch64 cores WITHOUT the fused
   rounding multiply-accumulate (vqrdmlah/vqrdmlsh, i.e. pre-ARMv8.1 such as
   Cortex-A72). On aarch64 cores WITH QRDMX (Cortex-A76+, Neoverse N/V, GH200)
   the fused MAC makes the generic path cheaper, so the fast path measurably
   regresses there and is disabled. The kernel itself stays compiled on all
   targets (validated by the nr_modulation unit test). */
#if defined(__AVX2__) || (defined(__aarch64__) && !defined(__ARM_FEATURE_QRDMX))
#define NR_PDSCH_2X2_FASTPATH 1
#endif

/* The 2x4 cross-polar fast path shares its two complex multiplies between the two
   polarisations, so unlike the 2x2 butterfly its win is fewer MACs rather than cheaper
   ones -- the QRDMX argument above does not apply to it. Measured on Cortex-A78C, where
   precoding was 304.6 us of a 403 us PDSCH generation at 273 PRB / 2 layers / 4 ports. */
#if defined(__aarch64__)
#define NR_PDSCH_2X4_FASTPATH 1
#endif

#ifdef NR_PDSCH_2X2_FASTPATH
/* Fast path of do_txdataF() for the 2 antenna-port / 2-layer case: fills both
   antenna ports in a single pass using the radix-2 butterfly precoder, sharing
   the layer loads between the two outputs. See nr_layer_precoder_2x2_simd(). */
static inline void do_txdataF_2x2(c16_t **txdataF,
                                  int symbol_sz,
                                  c16_t txdataF_precoding[][symbol_sz],
                                  PHY_VARS_gNB *gNB,
                                  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                                  int ant0,
                                  int ant1,
                                  int rb_start,
                                  int rb_size,
                                  int txdataF_offset_per_symbol)
{
  NR_DL_FRAME_PARMS *frame_parms = &gNB->frame_parms;
  int rb = 0;
  uint16_t subCarrier = get_block_start_sc(rb_start, rel15->BWPStart, symbol_sz);
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  c16_t *out0 = &txdataF[ant0][txdataF_offset_per_symbol];
  c16_t *out1 = &txdataF[ant1][txdataF_offset_per_symbol];
  while (rb < rb_size) {
    // get pmi info (identical stepping to do_txdataF: coalesce equal-PMI RBs)
    const int pmi = (pb->prg_size > 0) ? (pb->prgs_list[(int)rb / pb->prg_size].pm_idx) : 0;
    const int pmi2 = (rb < (rb_size - 1) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 1) / pb->prg_size].pm_idx) : -1;
    const int pmi3 = (rb < (rb_size - 2) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 2) / pb->prg_size].pm_idx) : -1;
    const int pmi4 = (rb < (rb_size - 3) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 3) / pb->prg_size].pm_idx) : -1;
    int rb_step0 = pmi == pmi2 ? 2 : 1;
    const int rb_step = rb_step0 == 2 && pmi3 == pmi && pmi4 == pmi ? 4 : rb_step0;
    const int re_cnt = NR_NB_SC_PER_RB * rb_step;
    if (pmi == 0) { // unitary precoding: port i carries layer i
      memcpy(&out0[subCarrier], &txdataF_precoding[0][subCarrier], re_cnt * sizeof(**txdataF));
      memcpy(&out1[subCarrier], &txdataF_precoding[1][subCarrier], re_cnt * sizeof(**txdataF));
    } else { // non-unitary precoding via 2x2 butterfly
      AssertFatal(frame_parms->nb_antennas_tx > 1, "No precoding can be done with a single antenna port\n");
      nfapi_nr_pm_pdu_t *pmi_pdu = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1]; // pmi 0 is identity matrix
      AssertFatal(pmi == pmi_pdu->pm_idx, "PMI %d doesn't match to the one in precoding matrix %d\n", pmi, pmi_pdu->pm_idx);
      AssertFatal(pmi_pdu->num_ant_ports == 2 && pmi_pdu->numLayers == 2,
                  "2x2 precoder fast path expects 2 ports / 2 layers, got %d ports / %d layers\n",
                  pmi_pdu->num_ant_ports,
                  pmi_pdu->numLayers);
      nr_layer_precoder_2x2_simd(symbol_sz, txdataF_precoding, pmi_pdu->weights, subCarrier, re_cnt, out0, out1);
    }
    subCarrier += re_cnt;
    rb += rb_step;
  } // RB loop: while(rb < rb_size)
}
#endif // NR_PDSCH_2X2_FASTPATH

/* Does the rank-2 4-port codebook entry have the cross-polar butterfly structure
 *   W[0][p+2] = +phi*W[0][p]   and   W[1][p+2] = -phi*W[1][p],   phi in {1,-1,j,-j} ?
 * The generated weights are each rounded to int16 independently, so compare with a small
 * tolerance. Returns false if it does not hold, in which case the caller must use the
 * generic per-port kernel - the fast path is an optimisation, never a requirement. */
static inline bool nr_prec2x4_classify(const nfapi_nr_pm_pdu_t *pm, int p, bool *phi_swap, bool *phi_neg)
{
  const int TOL = 2; /* LSB; independent rounding of the two entries costs about 1 */
  const c16_t a0 = pm->weights[0][p], a2 = pm->weights[0][p + 2];
  const c16_t b0 = pm->weights[1][p], b2 = pm->weights[1][p + 2];
  /* candidates: phi = +1, -1, +j, -j, expressed as (swap, neg) */
  static const bool cand_swap[4] = {false, false, true, true};
  static const bool cand_neg[4] = {false, true, false, true};
  for (int c = 0; c < 4; c++) {
    /* phi*(r,i): +1 -> (r,i); -1 -> (-r,-i); +j -> (-i,r); -j -> (i,-r) */
    c16_t pa, pb;
    if (!cand_swap[c]) {
      pa = cand_neg[c] ? (c16_t){-a0.r, -a0.i} : a0;
      pb = cand_neg[c] ? (c16_t){-b0.r, -b0.i} : b0;
    } else {
      pa = cand_neg[c] ? (c16_t){a0.i, -a0.r} : (c16_t){-a0.i, a0.r};
      pb = cand_neg[c] ? (c16_t){b0.i, -b0.r} : (c16_t){-b0.i, b0.r};
    }
    /* layer 0 takes +phi, layer 1 takes -phi */
    if (abs(pa.r - a2.r) <= TOL && abs(pa.i - a2.i) <= TOL && abs(-pb.r - b2.r) <= TOL
        && abs(-pb.i - b2.i) <= TOL) {
      *phi_swap = cand_swap[c];
      *phi_neg = cand_neg[c];
      return true;
    }
  }
  return false;
}

/* Can the 2x4 fast path handle this PDU?  The scheduler sets prg_size = rbSize so there is
   a single PMI for the whole allocation; check it once here rather than discovering a bad
   one after part of the symbol has already been written. */
static inline bool nr_pdsch_2x4_usable(PHY_VARS_gNB *gNB, const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15)
{
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  if (pb->prg_size == 0)
    return true; /* pmi 0 everywhere: the memcpy/memset path, always fine */
  const int pmi = pb->prgs_list[0].pm_idx;
  if (pmi == 0)
    return true;
  if (pmi - 1 >= gNB->gNB_config.pmi_list.num_pm_idx)
    return false;
  const nfapi_nr_pm_pdu_t *pm = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1];
  if (pmi != pm->pm_idx || pm->num_ant_ports != 4 || pm->numLayers != 2)
    return false;
  /* every RB of the allocation uses this one PMI, so one classification per pair suffices */
  bool swap, neg;
  return nr_prec2x4_classify(pm, 0, &swap, &neg) && nr_prec2x4_classify(pm, 1, &swap, &neg);
}

/* Fast path of do_txdataF() for 2 layers onto 4 antenna ports: fills the port pair
   (p, p+2) in one pass. See nr_layer_precoder_2x4_simd(). */
static inline bool do_txdataF_2x4(c16_t **txdataF,
                                  int symbol_sz,
                                  c16_t txdataF_precoding[][symbol_sz],
                                  PHY_VARS_gNB *gNB,
                                  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                                  const uint16_t *ant_to_map,
                                  int rb_start,
                                  int rb_size,
                                  int txdataF_offset_per_symbol)
{
  int rb = 0;
  uint16_t subCarrier = get_block_start_sc(rb_start, rel15->BWPStart, symbol_sz);
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  while (rb < rb_size) {
    const int pmi = (pb->prg_size > 0) ? (pb->prgs_list[(int)rb / pb->prg_size].pm_idx) : 0;
    const int pmi2 = (rb < (rb_size - 1) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 1) / pb->prg_size].pm_idx) : -1;
    const int pmi3 = (rb < (rb_size - 2) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 2) / pb->prg_size].pm_idx) : -1;
    const int pmi4 = (rb < (rb_size - 3) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 3) / pb->prg_size].pm_idx) : -1;
    int rb_step0 = pmi == pmi2 ? 2 : 1;
    const int rb_step = rb_step0 == 2 && pmi3 == pmi && pmi4 == pmi ? 4 : rb_step0;
    const int re_cnt = NR_NB_SC_PER_RB * rb_step;
    if (pmi == 0) { // unitary precoding: port i carries layer i, the others are silent
      for (int a = 0; a < 4; a++) {
        c16_t *dst = &txdataF[ant_to_map[a]][txdataF_offset_per_symbol + subCarrier];
        if (a < rel15->nrOfLayers)
          memcpy(dst, &txdataF_precoding[a][subCarrier], re_cnt * sizeof(**txdataF));
        else
          memset(dst, 0, re_cnt * sizeof(**txdataF));
      }
    } else {
      nfapi_nr_pm_pdu_t *pmi_pdu = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1];
      if (pmi != pmi_pdu->pm_idx || pmi_pdu->num_ant_ports != 4 || pmi_pdu->numLayers != 2)
        return false;
      for (int p = 0; p < 2; p++) {
        bool phi_swap, phi_neg;
        if (!nr_prec2x4_classify(pmi_pdu, p, &phi_swap, &phi_neg))
          return false; /* codebook is not of the expected form: caller falls back */
        nr_layer_precoder_2x4_simd(symbol_sz,
                                   txdataF_precoding,
                                   pmi_pdu->weights,
                                   p,
                                   phi_swap,
                                   phi_neg,
                                   subCarrier,
                                   re_cnt,
                                   &txdataF[ant_to_map[p]][txdataF_offset_per_symbol],
                                   &txdataF[ant_to_map[p + 2]][txdataF_offset_per_symbol]);
      }
    }
    subCarrier += re_cnt;
    rb += rb_step;
  }
  return true;
}

#ifdef NR_PDSCH_2X4_FASTPATH
/* Rank 3-4 generalisation of nr_prec2x4_classify(): for port pair (p, p+2) find the
   per-layer co-phasing phi_l in {+-1,+-j} with W[l][p+2] = phi_l . W[l][p].  Fills
   phi_swap[l]/phi_neg[l] for every layer; returns false (caller falls back to the generic
   kernel) if any layer does not fit the cross-polar form. */
static inline bool nr_precNx4_classify(const nfapi_nr_pm_pdu_t *pm, int p, int n_layers,
                                       bool phi_swap[NR_MAX_NB_LAYERS], bool phi_neg[NR_MAX_NB_LAYERS])
{
  const int TOL = 2; /* LSB; independent rounding of the two entries costs about 1 */
  static const bool cand_swap[4] = {false, false, true, true};
  static const bool cand_neg[4] = {false, true, false, true};
  for (int l = 0; l < n_layers; l++) {
    const c16_t a0 = pm->weights[l][p], a2 = pm->weights[l][p + 2];
    bool found = false;
    for (int c = 0; c < 4 && !found; c++) {
      /* phi*(r,i): +1->(r,i) -1->(-r,-i) +j->(-i,r) -j->(i,-r) */
      c16_t pa;
      if (!cand_swap[c])
        pa = cand_neg[c] ? (c16_t){-a0.r, -a0.i} : a0;
      else
        pa = cand_neg[c] ? (c16_t){a0.i, -a0.r} : (c16_t){-a0.i, a0.r};
      if (abs(pa.r - a2.r) <= TOL && abs(pa.i - a2.i) <= TOL) {
        phi_swap[l] = cand_swap[c];
        phi_neg[l] = cand_neg[c];
        found = true;
      }
    }
    if (!found)
      return false;
  }
  return true;
}

/* Can the Nx4 fast path handle this PDU? Single PMI for the whole allocation
   (prg_size = rbSize), r layers onto 4 ports, every port pair cross-polar. */
static inline bool nr_pdsch_Nx4_usable(PHY_VARS_gNB *gNB, const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15)
{
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  if (pb->prg_size == 0)
    return true;
  const int pmi = pb->prgs_list[0].pm_idx;
  if (pmi == 0)
    return true;
  if (pmi - 1 >= gNB->gNB_config.pmi_list.num_pm_idx)
    return false;
  const nfapi_nr_pm_pdu_t *pm = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1];
  if (pmi != pm->pm_idx || pm->num_ant_ports != 4 || pm->numLayers != rel15->nrOfLayers)
    return false;
  bool sw[NR_MAX_NB_LAYERS], ng[NR_MAX_NB_LAYERS];
  return nr_precNx4_classify(pm, 0, rel15->nrOfLayers, sw, ng)
      && nr_precNx4_classify(pm, 1, rel15->nrOfLayers, sw, ng);
}

/* Fast path of do_txdataF() for r layers (2..4) onto 4 antenna ports: fills each port
   pair (p, p+2) in one pass via nr_layer_precoder_Nx4_simd(), sharing the r layer MACs. */
static inline bool do_txdataF_Nx4(c16_t **txdataF,
                                  int symbol_sz,
                                  c16_t txdataF_precoding[][symbol_sz],
                                  PHY_VARS_gNB *gNB,
                                  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15,
                                  const uint16_t *ant_to_map,
                                  int rb_start,
                                  int rb_size,
                                  int txdataF_offset_per_symbol)
{
  const int r = rel15->nrOfLayers;
  int rb = 0;
  uint16_t subCarrier = get_block_start_sc(rb_start, rel15->BWPStart, symbol_sz);
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  while (rb < rb_size) {
    const int pmi = (pb->prg_size > 0) ? (pb->prgs_list[(int)rb / pb->prg_size].pm_idx) : 0;
    const int pmi2 = (rb < (rb_size - 1) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 1) / pb->prg_size].pm_idx) : -1;
    const int pmi3 = (rb < (rb_size - 2) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 2) / pb->prg_size].pm_idx) : -1;
    const int pmi4 = (rb < (rb_size - 3) && pb->prg_size > 0) ? (pb->prgs_list[(int)(rb + 3) / pb->prg_size].pm_idx) : -1;
    int rb_step0 = pmi == pmi2 ? 2 : 1;
    const int rb_step = rb_step0 == 2 && pmi3 == pmi && pmi4 == pmi ? 4 : rb_step0;
    const int re_cnt = NR_NB_SC_PER_RB * rb_step;
    if (pmi == 0) {
      for (int a = 0; a < 4; a++) {
        c16_t *dst = &txdataF[ant_to_map[a]][txdataF_offset_per_symbol + subCarrier];
        if (a < r)
          memcpy(dst, &txdataF_precoding[a][subCarrier], re_cnt * sizeof(**txdataF));
        else
          memset(dst, 0, re_cnt * sizeof(**txdataF));
      }
    } else {
      nfapi_nr_pm_pdu_t *pmi_pdu = &gNB->gNB_config.pmi_list.pmi_pdu[pmi - 1];
      if (pmi != pmi_pdu->pm_idx || pmi_pdu->num_ant_ports != 4 || pmi_pdu->numLayers != r)
        return false;
      for (int p = 0; p < 2; p++) {
        bool phi_swap[NR_MAX_NB_LAYERS], phi_neg[NR_MAX_NB_LAYERS];
        if (!nr_precNx4_classify(pmi_pdu, p, r, phi_swap, phi_neg))
          return false;
        nr_layer_precoder_Nx4_simd(r, symbol_sz, txdataF_precoding, pmi_pdu->weights, p,
                                   phi_swap, phi_neg, subCarrier, re_cnt,
                                   &txdataF[ant_to_map[p]][txdataF_offset_per_symbol],
                                   &txdataF[ant_to_map[p + 2]][txdataF_offset_per_symbol]);
      }
    }
    subCarrier += re_cnt;
    rb += rb_step;
  }
  return true;
}
#endif // NR_PDSCH_2X4_FASTPATH

/* Parallel scramble + modulate + layer-map.
 *
 * All three stages are elementwise/strided with no cross-element dependency:
 * scrambling is a pure XOR against a fully materialised gold sequence, modulation is a
 * per-symbol table lookup, and layer mapping is a fixed stride-n_layers deinterleave.
 * Serially they cost ~180 us of a ~525 us PDSCH generation at 273 PRB / 2 layers on a
 * Cortex-A72, and being outside the per-symbol task region they set an Amdahl floor that
 * no number of pool cores can lower.
 *
 * Chunks are cut on a granule of lcm(32, Qm * n_layers) bits so that every boundary is
 * simultaneously 32-bit aligned (the scrambler XORs whole words) and a whole number of
 * per-layer symbols (so each chunk writes a contiguous, disjoint slice of every layer).
 * Only the final chunk may be short, so only it can round its word count up -- the +4
 * word slack on scrambled_output covers that.
 *
 * gold_cache() keeps a THREAD-LOCAL table, so the sequence is fetched once by the caller
 * and the pointer handed to the workers; letting each worker call it would regenerate the
 * sequence per thread. */
typedef struct {
  const uint32_t *in;      // encoder output
  const uint32_t *seq;     // gold sequence, from the caller's gold_cache()
  uint32_t *scrambled;     // shared buffer, this chunk writes [word_off, word_off+n_words)
  c16_t *tx_base;          // &tx_layers[0][0]
  int layerSz;
  uint32_t word_off, n_words, bit_len, sym_off, n_syms;
  uint16_t Qm;
  uint8_t n_layers;
  task_ans_t *ans;
} pdschModChunk_t;

static void nr_pdsch_scramble_modulate_chunk(void *arg)
{
  pdschModChunk_t *d = (pdschModChunk_t *)arg;

  uint32_t *out = d->scrambled + d->word_off;
  const uint32_t *in = d->in + d->word_off;
  const uint32_t *seq = d->seq + d->word_off;
  for (uint32_t i = 0; i < d->n_words; i++)
    out[i] = in[i] ^ seq[i];

  c16_t mod_symbs[d->n_syms] __attribute__((aligned(64)));
  nr_modulation(out, d->bit_len, d->Qm, (int16_t *)mod_symbs);

  /* Offsetting the flat base keeps the per-layer stride, so tl[l][k] is
     tx_layers[l][sym_off / n_layers + k]. */
  c16_t (*tl)[d->layerSz] = (c16_t (*)[d->layerSz])(d->tx_base + d->sym_off / d->n_layers);
  c16_t (*ms)[d->n_syms] = &mod_symbs;
  nr_layer_mapping(1, d->n_syms, ms, d->n_layers, d->layerSz, d->n_syms, tl);

  completed_task_ans(d->ans);
}

/* Floor on chunk size, so a small allocation does not spend more on dispatch than it
   saves.  Above it, the chunk COUNT follows the pool width rather than a fixed symbol
   count: a fixed 16384 gave only 2-3 chunks at 1 layer with the allocations a real
   scheduler produces, which is why the first version measured -49% in dlsim at 2 layers
   and no gain at all on the live DU. */
#define NR_PDSCH_MOD_MIN_CHUNK_SYMS 2048

static inline uint32_t nr_gcd_u32(uint32_t a, uint32_t b)
{
  while (b) { uint32_t t = a % b; a = b; b = t; }
  return a;
}

typedef struct pdschSymbolProc_s {
  PHY_VARS_gNB *gNB;
  NR_DL_FRAME_PARMS *frame_parms;
  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15;
  freq_alloc_bitmap_t *freq_alloc;
  unsigned int slot;
  unsigned int startSymbol;
  unsigned int numSymbols;
  task_ans_t *ans;
  unsigned int layerSz2;
  unsigned int dlPtrsSymPos;
  unsigned int n_ptrs;
  uint16_t *ant_to_map;
  uint64_t *pdsch_phase_comp_prb_mask;
  int prb_mask_words;
  unsigned int re_beginning_of_symbol[14];
  c16_t *tx_layers[4];
  time_stats_t dlsch_resource_mapping_stats;
  time_stats_t dlsch_precoding_stats;
} pdschSymbolProc_t;

static inline void mark_prb_range(uint64_t *prb_mask, int prb_mask_words, int symbol, int start_prb, int nb_prb)
{
  uint64_t *symbol_mask = prb_mask + symbol * prb_mask_words;
  for (int prb = start_prb; prb < start_prb + nb_prb; prb++)
    symbol_mask[prb >> 6] |= UINT64_C(1) << (prb & 63);
}

static void nr_pdsch_symbol_processing(void *arg)
{
  pdschSymbolProc_t *rdata = (pdschSymbolProc_t *)arg;

  PHY_VARS_gNB *gNB = rdata->gNB;
  NR_DL_FRAME_PARMS *frame_parms = rdata->frame_parms;
  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15 = rdata->rel15;
  int slot = rdata->slot;
  c16_t *tx_layers[rel15->nrOfLayers];
  for (int l = 0; l < rel15->nrOfLayers; l++)
    tx_layers[l] = rdata->tx_layers[l];
  freq_alloc_bitmap_t *freq_alloc = rdata->freq_alloc;
  const int nb_re_dmrs = rel15->numDmrsCdmGrpsNoData * (rel15->dmrsConfigType == NFAPI_NR_DMRS_TYPE1 ? 6 : 4);
  const int n_dmrs = (rel15->BWPStart + freq_alloc->last_rb + 1) * nb_re_dmrs;
  // Loop Over OFDM symbols:
  c16_t mod_dmrs[(n_dmrs + 63) & ~63] __attribute__((aligned(64)));
  const int symbol_sz = frame_parms->ofdm_symbol_size;

  c16_t **txdataF = gNB->common_vars.txdataF;
  const int symb_offset = (slot % frame_parms->slots_per_subframe) * frame_parms->symbols_per_slot;

  for (int l_symbol = rdata->startSymbol; l_symbol < rdata->startSymbol + rdata->numSymbols; l_symbol++) {
    start_meas(&rdata->dlsch_resource_mapping_stats);
    int l_prime = 0; // single symbol layer 0
    int l_overline = get_l0(rel15->dlDmrsSymbPos);

    /// DMRS QPSK modulation
    if ((rel15->dlDmrsSymbPos & (1 << l_symbol))) { // DMRS time occasion
      // The reference point for is subcarrier -1 of the lowest-numbered resource block in CORESET 0 if the corresponding
      // PDCCH is associated with CORESET -1 and Type0-PDCCH common search space and is addressed to SI-RNTI
      // 2GPP TS 38.211 V15.8.0 Section 7.4.1.1.2 Mapping to physical resources
      if (l_symbol == (l_overline + 1)) // take into account the double DMRS symbols
        l_prime = 1;
      else if (l_symbol > (l_overline + 1)) { // new DMRS pair
        l_overline = l_symbol;
        l_prime = 0;
      }
      const uint32_t *gold = nr_gold_pdsch(frame_parms->N_RB_DL,
                                           frame_parms->symbols_per_slot,
                                           rel15->dlDmrsScramblingId,
                                           rel15->SCID,
                                           slot,
                                           l_symbol);
      // Qm = 1 as DMRS is QPSK modulated
      nr_modulation(gold, n_dmrs * DMRS_MOD_ORDER, DMRS_MOD_ORDER, (int16_t *)mod_dmrs);

    }
    uint32_t dmrs_idx = freq_alloc->first_rb;
    if (rel15->refPoint == 0)
      dmrs_idx += rel15->BWPStart;
    dmrs_idx *= rel15->dmrsConfigType == NFAPI_NR_DMRS_TYPE1 ? 6 : 4;
    c16_t txdataF_precoding[rel15->nrOfLayers][symbol_sz] __attribute__((aligned(64)));
    for (int layer = 0; layer < rel15->nrOfLayers; layer++) {
      do_onelayer(frame_parms,
                  slot,
                  rel15,
                  freq_alloc,
                  layer,
                  txdataF_precoding[layer],
                  tx_layers[layer] + rdata->re_beginning_of_symbol[l_symbol],
                  symbol_sz,
                  l_symbol,
                  rdata->dlPtrsSymPos,
                  rdata->n_ptrs,
                  gNB->TX_AMP,
                  min((double)gNB->TX_AMP * sqrt(rel15->numDmrsCdmGrpsNoData), INT16_MAX),
                  l_prime,
                  rel15->dmrsConfigType,
                  mod_dmrs + dmrs_idx);
    } // layer loop
    stop_meas(&rdata->dlsch_resource_mapping_stats);

    start_meas(&rdata->dlsch_precoding_stats);
    const size_t txdataF_offset_per_symbol = l_symbol * symbol_sz;
    const uint16_t num_log_ports =
        rel15->param_v4.numberCodewords ? rel15->param_v4.spatialStreamsCw[0].numSpatialStreamIndices : 0;
    // 2 ports / 2 layers: precode both antennas in one pass (radix-2 butterfly),
    // on targets where it is a win (see NR_PDSCH_2X2_FASTPATH). Otherwise, and
    // for 4-port/2-layer (num_log_ports==4), fall through to the generic path.
#ifdef NR_PDSCH_2X2_FASTPATH
    if (rel15->nrOfLayers == 2 && num_log_ports == 2) {
      const int ant0 = rdata->ant_to_map[0];
      const int ant1 = rdata->ant_to_map[1];
      int pos = 0;
      int block_start, block_end;
      while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
        do_txdataF_2x2(txdataF,
                       symbol_sz,
                       txdataF_precoding,
                       gNB,
                       rel15,
                       ant0,
                       ant1,
                       block_start,
                       block_end - block_start + 1,
                       txdataF_offset_per_symbol);

        if (gNB->phase_comp) {
          const int start_sc = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          const int nsc = (block_end - block_start + 1) * NR_NB_SC_PER_RB;
          const c16_t rot = frame_parms->symbol_rotation[0][symb_offset + l_symbol];
          c16_t *psc0 = &txdataF[ant0][txdataF_offset_per_symbol + start_sc];
          c16_t *psc1 = &txdataF[ant1][txdataF_offset_per_symbol + start_sc];
          rotate_cpx_vector(psc0, rot, psc0, nsc, 15);
          rotate_cpx_vector(psc1, rot, psc1, nsc, 15);
          mark_prb_range(rdata->pdsch_phase_comp_prb_mask, rdata->prb_mask_words, l_symbol, block_start, block_end - block_start + 1);
        }
      }
    } else
#endif // NR_PDSCH_2X2_FASTPATH
#ifdef NR_PDSCH_2X4_FASTPATH
    /* 2 layers onto 4 ports: halve the complex MACs by sharing them between the two
     * polarisations.  Decided once per symbol, never partway through: the scheduler sets
     * prg_size = rbSize, so the whole allocation carries a single PMI, and if that PMI is
     * not of the expected cross-polar form we simply use the generic kernel below. */
    if (rel15->nrOfLayers == 2 && num_log_ports == 4 && nr_pdsch_2x4_usable(gNB, rel15)) {
      int pos = 0;
      int block_start, block_end;
      while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
#ifdef NR_PDSCH_2X4_VERIFY
        /* Run the generic per-port kernel first, keep its output, then let the fused path
         * overwrite it and report the largest deviation.  The two are NOT bit-exact: the
         * codebook rounds W[l][p] and W[l][p+2] independently, so deriving one from the
         * other costs ~1 LSB.  Anything beyond a couple of LSB is a real bug. */
        {
          const int nsc_v = (block_end - block_start + 1) * NR_NB_SC_PER_RB;
          const int sc0_v = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          c16_t ref[4][nsc_v];
          for (int a = 0; a < 4; a++) {
            do_txdataF(txdataF, symbol_sz, txdataF_precoding, gNB, rel15, rdata->ant_to_map[a],
                       block_start, block_end - block_start + 1, txdataF_offset_per_symbol);
            memcpy(ref[a], &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + sc0_v],
                   nsc_v * sizeof(c16_t));
          }
          do_txdataF_2x4(txdataF, symbol_sz, txdataF_precoding, gNB, rel15, rdata->ant_to_map,
                         block_start, block_end - block_start + 1, txdataF_offset_per_symbol);
          static int worst = 0;
          for (int a = 0; a < 4; a++) {
            const c16_t *got = &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + sc0_v];
            for (int i = 0; i < nsc_v; i++) {
              int dr = abs(got[i].r - ref[a][i].r), di = abs(got[i].i - ref[a][i].i);
              int d = dr > di ? dr : di;
              if (d > worst) {
                worst = d;
                printf("2X4VERIFY worst deviation %d LSB (ant %d re %d: ref %d %d got %d %d)\n",
                       worst, a, i, ref[a][i].r, ref[a][i].i, got[i].r, got[i].i);
              }
            }
          }
        }
#endif
        bool ok = do_txdataF_2x4(txdataF,
                                 symbol_sz,
                                 txdataF_precoding,
                                 gNB,
                                 rel15,
                                 rdata->ant_to_map,
                                 block_start,
                                 block_end - block_start + 1,
                                 txdataF_offset_per_symbol);
        DevAssert(ok); /* nr_pdsch_2x4_usable() already vetted the PMI */
        if (gNB->phase_comp) {
          const int start_sc = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          const int nsc = (block_end - block_start + 1) * NR_NB_SC_PER_RB;
          const c16_t rot = frame_parms->symbol_rotation[0][symb_offset + l_symbol];
          for (int a = 0; a < 4; a++) {
            c16_t *psc = &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + start_sc];
            rotate_cpx_vector(psc, rot, psc, nsc, 15);
          }
          mark_prb_range(rdata->pdsch_phase_comp_prb_mask, rdata->prb_mask_words, l_symbol, block_start, block_end - block_start + 1);
        }
      }
    } else
#endif // NR_PDSCH_2X4_FASTPATH
#ifdef NR_PDSCH_2X4_FASTPATH
    /* 3-4 layers onto 4 ports: same cross-polar sharing as the 2x4 path, generalised to
     * rank r (halves the MACs: 2r instead of 4r per RE-pair). Decided once per symbol; if
     * the single PMI is not cross-polar the generic kernel below is used. */
    if ((rel15->nrOfLayers == 3 || rel15->nrOfLayers == 4) && num_log_ports == 4 && !getenv("NR_PDSCH_NO_NX4") && nr_pdsch_Nx4_usable(gNB, rel15)) {
      int pos = 0;
      int block_start, block_end;
      while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
#ifdef NR_PDSCH_NX4_VERIFY
        {
          const int nsc_v = (block_end - block_start + 1) * NR_NB_SC_PER_RB;
          const int sc0_v = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          c16_t ref[4][nsc_v];
          for (int a = 0; a < 4; a++) {
            do_txdataF(txdataF, symbol_sz, txdataF_precoding, gNB, rel15, rdata->ant_to_map[a],
                       block_start, block_end - block_start + 1, txdataF_offset_per_symbol);
            memcpy(ref[a], &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + sc0_v], nsc_v * sizeof(c16_t));
          }
          do_txdataF_Nx4(txdataF, symbol_sz, txdataF_precoding, gNB, rel15, rdata->ant_to_map,
                         block_start, block_end - block_start + 1, txdataF_offset_per_symbol);
          static int worst = 0;
          for (int a = 0; a < 4; a++) {
            const c16_t *got = &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + sc0_v];
            for (int i = 0; i < nsc_v; i++) {
              int dr = abs(got[i].r - ref[a][i].r), di = abs(got[i].i - ref[a][i].i);
              int d = dr > di ? dr : di;
              if (d > worst) { worst = d;
                printf("NX4VERIFY worst deviation %d LSB (layers %d ant %d re %d: ref %d %d got %d %d)\n",
                       worst, rel15->nrOfLayers, a, i, ref[a][i].r, ref[a][i].i, got[i].r, got[i].i); }
            }
          }
        }
#endif
        bool ok = do_txdataF_Nx4(txdataF, symbol_sz, txdataF_precoding, gNB, rel15, rdata->ant_to_map,
                                 block_start, block_end - block_start + 1, txdataF_offset_per_symbol);
        DevAssert(ok);
        if (gNB->phase_comp) {
          const int start_sc = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          const int nsc = (block_end - block_start + 1) * NR_NB_SC_PER_RB;
          const c16_t rot = frame_parms->symbol_rotation[0][symb_offset + l_symbol];
          for (int a = 0; a < 4; a++) {
            c16_t *psc = &txdataF[rdata->ant_to_map[a]][txdataF_offset_per_symbol + start_sc];
            rotate_cpx_vector(psc, rot, psc, nsc, 15);
          }
          mark_prb_range(rdata->pdsch_phase_comp_prb_mask, rdata->prb_mask_words, l_symbol, block_start, block_end - block_start + 1);
        }
      }
    } else
#endif // NR_PDSCH_2X4_FASTPATH
    for (int ant = 0; ant < num_log_ports; ant++) {
      int pos = 0;
      int block_start, block_end;
      while (find_next_rb_block(freq_alloc->bitmap, rel15->BWPSize, &pos, &block_start, &block_end)) {
        do_txdataF(txdataF,
                   symbol_sz,
                   txdataF_precoding,
                   gNB,
                   rel15,
                   rdata->ant_to_map[ant],
                   block_start,
                   block_end - block_start + 1,
                   txdataF_offset_per_symbol);

        if (gNB->phase_comp) {
	  int start_sc = get_block_start_sc(block_start, rel15->BWPStart, symbol_sz);
          c16_t *pdsch_sc = &txdataF[rdata->ant_to_map[ant]][txdataF_offset_per_symbol + start_sc];
          const c16_t rot = frame_parms->symbol_rotation[0][symb_offset + l_symbol];
          rotate_cpx_vector(pdsch_sc, rot, pdsch_sc,(block_end - block_start + 1)*NR_NB_SC_PER_RB, 15);
          mark_prb_range(rdata->pdsch_phase_comp_prb_mask, rdata->prb_mask_words, l_symbol, block_start, block_end - block_start + 1);
        }
      }
    }
    stop_meas(&rdata->dlsch_precoding_stats);
  }
  // Task running in // completed
  completed_task_ans(rdata->ans);
}

static int do_one_dlsch(unsigned char *input_ptr,
                        PHY_VARS_gNB *gNB,
                        const nr_pdsch_gen_info_t *info,
                        int slot,
                        uint64_t *pdsch_phase_comp_prb_mask,
                        int prb_mask_words)
{
  NR_DL_FRAME_PARMS *frame_parms = &gNB->frame_parms;

  time_stats_t *dlsch_scrambling_stats = &gNB->dlsch_scrambling_stats;
  time_stats_t *dlsch_modulation_stats = &gNB->dlsch_modulation_stats;
  const freq_alloc_bitmap_t *freq_alloc = &info->freq_alloc;
  const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15 = &info->pdsch_pdu->pdsch_pdu_rel15;
  const int layerSz = frame_parms->N_RB_DL * frame_parms->symbols_per_slot * NR_NB_SC_PER_RB;
  const int nb_re_dmrs = rel15->numDmrsCdmGrpsNoData * (rel15->dmrsConfigType == NFAPI_NR_DMRS_TYPE1 ? 6 : 4);
  const int n_dmrs = freq_alloc->num_rbs * nb_re_dmrs;

  const int xOverhead = 0;
  const int nb_re =
      (12 * rel15->NrOfSymbols - nb_re_dmrs * get_num_dmrs(rel15->dlDmrsSymbPos) - xOverhead) * freq_alloc->num_rbs * rel15->nrOfLayers;
  const int Qm = rel15->qamModOrder[0];
  const int encoded_length = nb_re * Qm;

  /* PTRS */
  uint16_t dlPtrsSymPos = 0;
  int n_ptrs = 0;
  if (rel15->pduBitmap & 0x1) {
    dlPtrsSymPos =
        get_ptrs_symb_idx(rel15->NrOfSymbols, rel15->StartSymbolIndex, 1 << rel15->PTRSTimeDensity, rel15->dlDmrsSymbPos);
    n_ptrs = (freq_alloc->num_rbs + rel15->PTRSFreqDensity - 1) / rel15->PTRSFreqDensity;
  }

#ifdef DEBUG_DLSCH
  printf("PDSCH encoding:\nPayload:\n");
  for (int i = 0; i < (info->dlsch->B >> 3); i += 16) {
    for (int j = 0; j < 16; j++)
      printf("0x%02x\t", info->dlsch->pdu[i + j]);
    printf("\n");
  }
  printf("\nEncoded payload:\n");
  for (int i = 0; i < encoded_length; i += 8) {
    for (int j = 0; j < 8; j++)
      printf("%d", (input_ptr[i >> 3] >> j) & 1);
    printf("\t");
  }
  printf("\n");
#endif

  if (IS_SOFTMODEM_DLSIM)
    memcpy(info->dlsch->f, input_ptr, (encoded_length + 7) >> 3);

  start_meas(&gNB->dlsch_pdsch_generation_stats);
  int layerSz2 = (layerSz + 63) & ~63;
  c16_t tx_layers[rel15->nrOfLayers][layerSz2] __attribute__((aligned(64)));
  /* Clear only the TAIL of each layer.  Both the chunked and the serial mapping write
     tx_layers[l][0 .. n_symbs/n_layers), and the symbol tasks read exactly that range
     (re_beginning_of_symbol accumulates to the same total), so the head is always
     written before it is read.  Clearing all of tx_layers[] was a 358 KB memset at
     273 PRB / 2 layers -- the largest remaining serial item once scramble+modulate were
     parallelised, and it got *slower* under the pool (26 -> 37 us) because the lines had
     been touched by other cores. */
  const uint32_t layer_used = (encoded_length / Qm) / rel15->nrOfLayers;
  start_meas(&gNB->dlsch_layer_clear_stats);
  if (layer_used < (uint32_t)layerSz2)
    for (int l = 0; l < rel15->nrOfLayers; l++)
      memset(&tx_layers[l][layer_used], 0, ((uint32_t)layerSz2 - layer_used) * sizeof(c16_t));
  stop_meas(&gNB->dlsch_layer_clear_stats);

  uint32_t scrambled_output[(encoded_length >> 5) + 4]; // modulator access by 4 bytes in some cases
  const uint32_t roundedSz = (encoded_length + 31) / 32;

  /* Split scramble+modulate+layer-map over the pool when it is worth it. Restricted to
     <= 2 layers because that is exactly the range where nr_modulation_layer_mapping()
     itself selects modulate-then-deinterleave; for 3-4 layers it picks a fused scalar
     loop that this chunking does not reproduce, so fall through to the serial call. */
  uint32_t nb_chunks = 1;
  const uint32_t unit = (uint32_t)Qm * rel15->nrOfLayers;
  const uint32_t granule_bits = (32u / nr_gcd_u32(32u, unit)) * unit; // lcm(32, unit)
  const uint32_t granules_total = (encoded_length + granule_bits - 1) / granule_bits;
  if (gNB->num_pdsch_symbols_per_thread > 0 && rel15->nrOfLayers <= 2) {
    /* one chunk per worker plus one for the caller, which runs the last inline */
    const uint32_t by_pool = (uint32_t)gNB->threadPool.len_thr + 1;
    const uint32_t by_size = (encoded_length / Qm) / NR_PDSCH_MOD_MIN_CHUNK_SYMS;
    nb_chunks = by_pool < by_size ? by_pool : by_size;
    if (nb_chunks > granules_total)
      nb_chunks = granules_total;
    if (nb_chunks < 1)
      nb_chunks = 1;
  }

  if (nb_chunks > 1) {
    start_meas(dlsch_scrambling_stats);
    /* only the trailing slack the modulator may over-read: the chunks tile [0, roundedSz) */
    memset(scrambled_output + roundedSz, 0, (sizeof(scrambled_output) / 4 - roundedSz) * 4);
    /* fetched once here: gold_cache() is thread-local, so per-worker calls would each
       regenerate the sequence instead of sharing it */
    const uint32_t *seq = gold_cache((rel15->rnti << 15) + (0 << 14) + rel15->dataScramblingId, roundedSz);
    stop_meas(dlsch_scrambling_stats);

    start_meas(dlsch_modulation_stats);
    pdschModChunk_t cd[nb_chunks];
    task_ans_t mans;
    init_task_ans(&mans, nb_chunks);
    for (uint32_t c = 0; c < nb_chunks; c++) {
      const uint32_t b0 = ((granules_total * c) / nb_chunks) * granule_bits;
      uint32_t b1 = ((granules_total * (c + 1)) / nb_chunks) * granule_bits;
      if (b1 > encoded_length || c == nb_chunks - 1)
        b1 = encoded_length;
      cd[c] = (pdschModChunk_t){.in = (const uint32_t *)input_ptr,
                                .seq = seq,
                                .scrambled = scrambled_output,
                                .tx_base = &tx_layers[0][0],
                                .layerSz = layerSz2,
                                .word_off = b0 / 32,
                                .n_words = ((b1 + 31) / 32) - (b0 / 32),
                                .bit_len = b1 - b0,
                                .sym_off = b0 / Qm,
                                .n_syms = (b1 - b0) / Qm,
                                .Qm = Qm,
                                .n_layers = rel15->nrOfLayers,
                                .ans = &mans};
      if (c < nb_chunks - 1) {
        task_t t = {.func = &nr_pdsch_scramble_modulate_chunk, .args = &cd[c]};
        pushTpool(&gNB->threadPool, t);
      } else {
        nr_pdsch_scramble_modulate_chunk(&cd[c]); // last one inline, like the symbol tasks
      }
    }
    join_task_ans(&mans);
    stop_meas(dlsch_modulation_stats);
  } else {
    start_meas(dlsch_scrambling_stats);
    memset(scrambled_output, 0, sizeof(scrambled_output));
    nr_pdsch_codeword_scrambling(input_ptr, encoded_length, 0, rel15->dataScramblingId, rel15->rnti, scrambled_output);
    stop_meas(dlsch_scrambling_stats);

    /* started AFTER scrambling stops: these two windows used to overlap, so
     * dlsch_modulation_stats silently included the scrambling time and the two
     * counters could not be added. */
    start_meas(dlsch_modulation_stats);
    const bool fused_ok =
        nr_modulation_layer_mapping(scrambled_output, encoded_length, Qm, rel15->nrOfLayers, layerSz2, tx_layers);
    AssertFatal(fused_ok,
                "Unsupported fused modulation/layer mapping for Qm %d, %d layers, %d codewords\n",
                Qm,
                rel15->nrOfLayers,
                rel15->NrOfCodewords);
    stop_meas(dlsch_modulation_stats);
  }

  /// Layer Precoding and Antenna port mapping
  // tx_layers 1-8 are mapped on antenna ports 1000-1007
  // The precoding info is supported by nfapi such as num_prgs, prg_size, prgs_list and pm_idx
  // The same precoding matrix is applied on prg_size RBs, Thus
  //        pmi = prgs_list[rbidx/prg_size].pm_idx, rbidx =0,...,rbSize-1
  // The Precoding matrix:
  // The Codebook Type I
  const nfapi_nr_tx_precoding_and_beamforming_t *pb = &rel15->precodingAndBeamforming;
  // beam number in multi-beam scenario (concurrent beams)
  const uint16_t symb_bitmap = SL_to_bitmap(rel15->StartSymbolIndex, rel15->NrOfSymbols);
  uint16_t ant_to_map[frame_parms->nb_antennas_tx];
  const uint16_t num_log_ports = rel15->param_v4.numberCodewords ? rel15->param_v4.spatialStreamsCw[0].numSpatialStreamIndices : 0;
  for (int ant = 0; ant < num_log_ports; ant++) {
    const uint16_t beam_id = pb->prgs_list[0].dig_bf_interface_list[ant].beam_idx;
    ant_to_map[ant] = get_first_ant_idx(gNB->enable_analog_das,
                                                  frame_parms->nb_antennas_tx / gNB->common_vars.num_beams_period,
                                                  beam_id,
                                                  rel15->param_v4.spatialStreamsCw[0].spatialStreamIndices[ant]);
    beam_index_allocation(beam_id,
                          ant_to_map[ant],
                          1,
                          frame_parms->symbols_per_slot,
                          slot,
                          symb_bitmap,
                          frame_parms->nb_antennas_tx,
                          gNB->common_vars.beam_id);
  }

  // spawn symbol threads

  int nb_tasks = 1;
  int num_pdsch_symbols_per_task = rel15->NrOfSymbols;
  if (gNB->num_pdsch_symbols_per_thread > 0) {
    // symbol processing in thread pool enabled
    num_pdsch_symbols_per_task = gNB->num_pdsch_symbols_per_thread;
    nb_tasks = rel15->NrOfSymbols / num_pdsch_symbols_per_task;
    if ((rel15->NrOfSymbols % num_pdsch_symbols_per_task) > 0)
      nb_tasks++;
  }
  pdschSymbolProc_t arr[nb_tasks];
  task_ans_t ans;
  init_task_ans(&ans, nb_tasks);
  int sz_arr = 0;
  unsigned int re_beginning_of_symbol = 0;
  int res = 0;
  for (int l_symbol = rel15->StartSymbolIndex; l_symbol < rel15->StartSymbolIndex + rel15->NrOfSymbols;
       l_symbol += num_pdsch_symbols_per_task) {
    pdschSymbolProc_t *rdata = &arr[sz_arr];
    rdata->ans = &ans;
    ++sz_arr;

    rdata->gNB = gNB;
    rdata->frame_parms = frame_parms;
    rdata->freq_alloc = freq_alloc;
    rdata->rel15 = rel15;
    rdata->slot = slot;
    rdata->startSymbol = l_symbol;
    res = rel15->NrOfSymbols - (l_symbol - rel15->StartSymbolIndex);
    if (res >= num_pdsch_symbols_per_task)
      rdata->numSymbols = num_pdsch_symbols_per_task;
    else
      rdata->numSymbols = res;
    rdata->layerSz2 = layerSz2;
    rdata->dlPtrsSymPos = dlPtrsSymPos;
    rdata->n_ptrs = n_ptrs;
    rdata->ant_to_map = ant_to_map;
    rdata->pdsch_phase_comp_prb_mask = pdsch_phase_comp_prb_mask;
    rdata->prb_mask_words = prb_mask_words;
    for (int s = l_symbol; s < l_symbol + rdata->numSymbols; s++) {
      rdata->re_beginning_of_symbol[s] = re_beginning_of_symbol;
      re_beginning_of_symbol += freq_alloc->num_rbs * NR_NB_SC_PER_RB;
      if (n_ptrs > 0 && IS_BIT_SET(dlPtrsSymPos, s)) {
        re_beginning_of_symbol -= n_ptrs;
      } else if (rel15->dlDmrsSymbPos & (1 << s)) {
        re_beginning_of_symbol -= n_dmrs;
      }
    }
    reset_meas(&rdata->dlsch_resource_mapping_stats);
    reset_meas(&rdata->dlsch_precoding_stats);
    for (int l = 0; l < rel15->nrOfLayers; l++)
      rdata->tx_layers[l] = tx_layers[l];
    if (l_symbol < rel15->StartSymbolIndex + rel15->NrOfSymbols - num_pdsch_symbols_per_task) {
      task_t t = {.func = &nr_pdsch_symbol_processing, .args = rdata};
      pushTpool(&gNB->threadPool, t);
    } else {
      nr_pdsch_symbol_processing(rdata);
    }
  }
  join_task_ans(&ans);
  for (int i = 0; i < nb_tasks; i++) {
    merge_meas(&gNB->dlsch_resource_mapping_stats, &arr[i].dlsch_resource_mapping_stats);
    merge_meas(&gNB->dlsch_precoding_stats, &arr[i].dlsch_precoding_stats);
  }
  stop_meas(&gNB->dlsch_pdsch_generation_stats);
  /* output and its parts for each dlsch should be aligned on 64 bytes (or 8 * 64 bits)
   * should remain a multiple of 8 * 64 with enough offset to fit each dlsch
   */
  uint32_t size_output_tb = freq_alloc->num_rbs * frame_parms->symbols_per_slot * NR_NB_SC_PER_RB * Qm * rel15->nrOfLayers;
  return ((size_output_tb + 511) >> 9) << 6;
}

/* First half of the old nr_generate_pdsch(): CRC, segmentation, LDPC, rate matching and
   interleaving, with the result left in enc->output for nr_dlsch_generate() to map onto
   the grid.  Split out because the two halves do not scale the same way -- encoding is
   pinned at ceil(C/8) tasks and is flat against pool width, generation scales -- so the
   only way to fit both in a slot at high MCS is to run them on different slots at once,
   and that needs the handoff to be a buffer rather than a stack VLA.

   Returns false if encoding failed, in which case enc->output holds nothing usable. */
bool nr_dlsch_encode(PHY_VARS_gNB *gNB,
                     int n_dlsch,
                     NR_gNB_DLSCH_t *dlsch_array,
                     int frame,
                     int slot,
                     nr_dlsch_encoded_t *enc)
{
  time_stats_t *dlsch_encoding_stats = &gNB->dlsch_encoding_stats;
  time_stats_t *tinput = &gNB->tinput;
  time_stats_t *tinput_memcpy = &gNB->tinput_memcpy;
  time_stats_t *tprep = &gNB->tprep;
  time_stats_t *tparity = &gNB->tparity;
  time_stats_t *toutput = &gNB->toutput;
  time_stats_t *tconcat = &gNB->tconcat;
  time_stats_t *dlsch_rate_matching_stats = &gNB->dlsch_rate_matching_stats;
  time_stats_t *dlsch_interleaving_stats = &gNB->dlsch_interleaving_stats;
  time_stats_t *dlsch_segmentation_stats = &gNB->dlsch_segmentation_stats;
  time_stats_t *dlsch_crc_stats = &gNB->dlsch_crc_stats;

  size_t size_output = 0;

  for (int i = 0; i < n_dlsch; i++) {
    NR_gNB_DLSCH_t *dlsch = &dlsch_array[i];
    const nfapi_nr_dl_tti_pdsch_pdu_rel15_t *rel15 = &dlsch->pdsch_pdu->pdsch_pdu_rel15;

    if (rel15->resourceAlloc == 0) {
      int alloc_size = (rel15->BWPSize / 8) + (rel15->BWPSize % 8 > 0);
      dlsch->freq_alloc = set_start_end_from_bitmap(rel15->BWPSize, alloc_size, rel15->rbBitmap);
    } else {
      dlsch->freq_alloc = set_bitmap_from_start_size(rel15->rbStart, rel15->rbSize);
    }
    LOG_D(PHY,
          "pdsch: BWPStart %d, BWPSize %d, rbStart %d, rbEnd %d rbsize %d\n",
          rel15->BWPStart,
          rel15->BWPSize,
          dlsch->freq_alloc.first_rb,
          dlsch->freq_alloc.last_rb,
          dlsch->freq_alloc.num_rbs);

    const int Qm = rel15->qamModOrder[0];

    /* PTRS */
    uint16_t dlPtrsSymPos = 0;
    int n_ptrs = 0;
    uint32_t ptrsSymbPerSlot = 0;
    if (rel15->pduBitmap & 0x1) {
      dlPtrsSymPos =
          get_ptrs_symb_idx(rel15->NrOfSymbols, rel15->StartSymbolIndex, 1 << rel15->PTRSTimeDensity, rel15->dlDmrsSymbPos);
      n_ptrs = (dlsch->freq_alloc.num_rbs + rel15->PTRSFreqDensity - 1) / rel15->PTRSFreqDensity;
      ptrsSymbPerSlot = get_ptrs_symbols_in_slot(dlPtrsSymPos, rel15->StartSymbolIndex, rel15->NrOfSymbols);
    }
    dlsch->unav_res = ptrsSymbPerSlot * n_ptrs;

    /// CRC, coding, interleaving and rate matching
    AssertFatal(dlsch->pdu != NULL, "%4d.%2d no PDU for PDSCH generation\n", frame, slot);

    /* output and its parts for each dlsch should be aligned on 64 bytes (or 8 * 64 bits)
     * => size_output is a sum of parts sizes rounded up to a multiple of 8 * 64
     */
    size_t size_output_tb = dlsch->freq_alloc.num_rbs * NR_SYMBOLS_PER_SLOT * NR_NB_SC_PER_RB * Qm * rel15->nrOfLayers;
    size_output += ceil_mod(size_output_tb, 8 * 64);

    /* Hand the generation stage its own copy of what it needs, so that a later encode
       overwriting dlsch[i] cannot pull the grid mapping out from under it. */
    enc->info[i].pdsch_pdu = dlsch->pdsch_pdu;
    enc->info[i].freq_alloc = dlsch->freq_alloc;
    enc->info[i].dlsch = dlsch;
  }
  enc->n_pdsch = n_dlsch;

  const size_t size_output_bytes = size_output >> 3;
  AssertFatal(size_output_bytes <= enc->capacity,
              "%4d.%2d DLSCH encoding needs %zu bytes but only %zu were allocated for the band\n",
              frame,
              slot,
              size_output_bytes,
              enc->capacity);
  unsigned char *output = enc->output;
  bzero(output, size_output_bytes);
  enc->frame = frame;
  enc->slot = slot;

  int slot_type = nr_slot_select(&gNB->gNB_config, frame, slot);
  START_MEAS_FULL_SLOT(dlsch_encoding_stats, slot_type, NR_DOWNLINK_SLOT);
  if (nr_dlsch_encoding(gNB,
                        n_dlsch,
                        dlsch_array,
                        frame,
                        slot,
                        output,
                        tinput,
                        tinput_memcpy,
                        tprep,
                        tparity,
                        toutput,
                        tconcat,
                        dlsch_rate_matching_stats,
                        dlsch_interleaving_stats,
                        dlsch_segmentation_stats,
			dlsch_crc_stats)
      == -1) {
    /* Nothing usable in enc->output; make sure the generation half skips the slot
       rather than mapping stale bits onto the grid. */
    enc->n_pdsch = 0;
    return false;
  }
  STOP_MEAS_FULL_SLOT(dlsch_encoding_stats, slot_type, NR_DOWNLINK_SLOT);
  return true;
}

/* Second half of the old nr_generate_pdsch(): scrambling, modulation, layer mapping,
   resource mapping and precoding of the bits nr_dlsch_encode() left in enc->output.
   Everything it needs comes out of enc, so it can run on a slot whose dlsch[] entries
   have since been reused by a later encode. */
void nr_dlsch_generate(PHY_VARS_gNB *gNB,
                       int slot,
                       const nr_dlsch_encoded_t *enc,
                       uint64_t *pdsch_phase_comp_prb_mask,
                       int prb_mask_words)
{
  unsigned char *output_ptr = enc->output;
  for (int i = 0; i < enc->n_pdsch; i++) {
    output_ptr += do_one_dlsch(output_ptr, gNB, &enc->info[i], slot, pdsch_phase_comp_prb_mask, prb_mask_words);
  }
}

void dump_pdsch_stats(FILE *fd, PHY_VARS_gNB *gNB)
{
  for (int i = 0; i < MAX_MOBILES_PER_GNB; i++) {
    NR_gNB_PHY_STATS_t *stats = &gNB->phy_stats[i];
    if (stats->active && stats->frame != stats->dlsch_stats.dump_frame) {
      stats->dlsch_stats.dump_frame = stats->frame;
      fprintf(fd,
              "DLSCH RNTI %x: current_Qm %d, current_RI %d, total_bytes TX %lu\n",
              stats->rnti,
              stats->dlsch_stats.current_Qm,
              stats->dlsch_stats.current_RI,
              stats->dlsch_stats.total_bytes_tx);
    }
  }
}
