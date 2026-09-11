/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2017 Intel Corporation
 */

#include "../nrLDPC_coding_interface.h"
#include "nrLDPC_coding_aal.h"
#include "PHY/sse_intrin.h"
#include "bits.h"
#include <common/utils/LOG/log.h>
#include "common/config/config_paramdesc.h"
#include "common/config/config_userapi.h"
#define NR_LDPC_ENABLE_PARITY_CHECK

#include "PHY/CODING/nrLDPC_decoder/nrLDPCdecoder_defs.h"
#include "PHY/CODING/coding_defs.h"

#include <stdint.h>
#include <getopt.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <regex.h>

#include <math.h>

#include <rte_eal.h>
#include <rte_common.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_lcore.h>
#include <rte_pdump.h>

#include <rte_dev.h>
#include <rte_launch.h>
#include <rte_bbdev.h>
#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_hexdump.h>
#include <rte_interrupts.h>

#ifdef LDPC_AAL_HAVE_LA12XX
/* NXP LA12xx (FECA) vendor API: queue-to-e200-core binding. */
#include <rte_pmd_bbdev_la12xx.h>
#endif

/* Compatibility with the NXP DPDK 19.11 fork (LA12xx / LSDK-GVLINUX).
 *
 * RTE_BBDEV_LDPC_E_MAX_MBUF was added to upstream DPDK in 20.11. The NXP
 * 19.11 fork carries the rest of the 5G LDPC bbdev API but not this macro,
 * so define it to the upstream value when absent.
 */
#ifndef RTE_BBDEV_LDPC_E_MAX_MBUF
#define RTE_BBDEV_LDPC_E_MAX_MBUF (64000)
#endif

/* DPDK 20.11 renamed the lcore roles master->main and slave->worker. */
#ifndef RTE_LCORE_FOREACH_WORKER
#define RTE_LCORE_FOREACH_WORKER(i) RTE_LCORE_FOREACH_SLAVE(i)
#endif

// this socket is the NUMA socket, so the hardware CPU id (numa is complex)
#define GET_SOCKET(socket_id) (((socket_id) == SOCKET_ID_ANY) ? 0 : (socket_id))
#define MAX_QUEUES 32
#define OPS_CACHE_SIZE 256U
#define OPS_POOL_SIZE_MIN 511U /* 0.5K per queue */
#define SYNC_WAIT 0
#define SYNC_START 1
#define TIME_OUT_POLL 1e8
/* Headroom for filler LLRs insertion in HARQ buffer */
#define FILLER_HEADROOM 1024
/* TB-mode decode needs a whole transport block's LLRs in ONE buffer (the PMD
 * advertises no SCATTER_GATHER, so a chained mbuf is not an option), and that
 * is far more than an mbuf can hold: rte_pktmbuf_pool_create()'s
 * data_room_size is a **uint16_t**, so 64 KiB is the hard ceiling -- asking for
 * 1 MiB silently truncates to 4224 B. Measured need: 78624 B at 273 PRB MCS 9,
 * 235872 B at MCS 27 (1 layer), ~944 kB at 4 layers.
 *
 * So take the mbuf room to just under the uint16_t limit and, above that, use
 * the same oversized-buffer trick the CB path already uses for
 * E > RTE_BBDEV_LDPC_E_MAX_MBUF: point the mbuf at an rte_malloc() buffer,
 * which is still hugepage memory and therefore DMA-able by the modem.
 */
/* nb_TBs per slot is 1 in the simulators and small in the softmodem. */
#define LDPC_AAL_TB_MAX_TBS 8u

/* Fine-grained encode-path timing (LDPC_AAL_DEBUG_ENCT=1). "LDPC input
 * preparation" is a single counter covering buffer init, op alloc and
 * set_ldpc_enc_op; this splits it so the ~53 ms can be attributed. */
static uint64_t enc_t[7];
static int enc_dbg = -1;

static uint64_t enc_now_us(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* TB mode needs one contiguous buffer per transport block for the LLRs and one
 * for the hard output, both larger than an mbuf can hold. Allocating them per
 * slot cost ~4.2 ms at 273 PRB MCS 27 (a 235 kB rte_malloc plus a 235 kB copy
 * to gather the per-CB LLRs) and leaked every slot. Keep them instead: one
 * allocation per TB index, grown only if a bigger slot arrives, and have the
 * LLR conversion write straight into them so the gather copy disappears too.
 * Never freed by design -- they live for the process.
 */
static uint8_t *tb_llr_buf[LDPC_AAL_TB_MAX_TBS];
static size_t tb_llr_cap[LDPC_AAL_TB_MAX_TBS];
static uint8_t *tb_out_buf[LDPC_AAL_TB_MAX_TBS];
static size_t tb_out_cap[LDPC_AAL_TB_MAX_TBS];

static uint8_t *tb_enc_in_buf[LDPC_AAL_TB_MAX_TBS];
static size_t tb_enc_in_cap[LDPC_AAL_TB_MAX_TBS];
static uint8_t *tb_enc_out_buf[LDPC_AAL_TB_MAX_TBS];
static size_t tb_enc_out_cap[LDPC_AAL_TB_MAX_TBS];

static unsigned tb_buf_allocs; /* TEST 2: must stop growing after slot 1 */

static uint8_t *tb_buf(uint8_t **slot, size_t *cap, size_t need)
{
  if (*cap < need) {
    rte_free(*slot);
    *slot = rte_malloc(NULL, need, RTE_CACHE_LINE_SIZE);
    AssertFatal(*slot != NULL, "rte_malloc(%zu) failed for a TB-mode buffer", need);
    tb_buf_allocs++;
    if (getenv("LDPC_AAL_DEBUG_ENCT"))
      printf("[tb-buf] realloc #%u: cap %zu -> %zu\n", tb_buf_allocs, *cap, need);
    *cap = need;
  }
  return *slot;
}

/* sum(E) over the code blocks of one transport block */
static size_t tb_llr_bytes(const nrLDPC_TB_decoding_parameters_t *p)
{
  size_t total = 0;

  for (int i = 0; i < p->C; ++i)
    total += (i < p->first_rE2) ? (size_t)p->E : (size_t)p->E2;
  return total;
}
/* Spare tailroom rte_pmd_la12xx_ldpc_enc_adj_bbuf() needs to shift the buffer. */
#define LA12XX_ENC_ADJ_TAILROOM 4096

pthread_mutex_t encode_mutex;
pthread_mutex_t decode_mutex;

/* we assume one active baseband device only */
struct active_device {
  const char *driver_name;
  uint8_t dev_id;
  struct rte_bbdev_info info;
  bool is_t2;
  /* Saturate int16 LLRs to int8 instead of rescaling them. True for the T2 and
   * for the NXP LA12xx, whose FECA expects plain S8.0 two's-complement LLRs. */
  bool saturate_llrs;
  /* Advertised LDPC enc/dec capability_flags, used to mask op_flags. */
  uint32_t enc_cap_flags;
  uint32_t dec_cap_flags;
  uint32_t num_harq_codeblock;
  /* Persistent data structure to keep track of HARQ-related information */
  // Note: This is used to store/keep track of the combined output information across iterations
  struct rte_bbdev_op_data *harq_buffers;
  bool support_internal_harq_memory;
  int dec_queue;
  int enc_queue;
  uint16_t queue_ids[MAX_QUEUES];
  uint16_t nb_queues;
  struct rte_mempool *bbdev_dec_op_pool;
  struct rte_mempool *bbdev_enc_op_pool;
  struct rte_mempool *in_mbuf_pool;
  struct rte_mempool *hard_out_mbuf_pool;
  struct rte_mempool *harq_in_mbuf_pool;
  struct rte_mempool *harq_out_mbuf_pool;
} active_dev;

/* Data buffers used by BBDEV ops */
struct data_buffers {
  struct rte_bbdev_op_data *inputs;
  struct rte_bbdev_op_data *hard_outputs;
  struct rte_bbdev_op_data *harq_outputs;
};

/* Operation parameters specific for given test case */
struct test_op_params {
  uint16_t num_lcores;
  rte_atomic16_t sync;
};

/* Contains per lcore params */
struct thread_params {
  uint8_t dev_id;
  uint16_t queue_id;
  uint32_t lcore_id;
  nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters;
  nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters;
  uint8_t iter_count;
  struct test_op_params *op_params;
  struct data_buffers *data_buffers;
  struct rte_mempool *bbdev_op_pool;
};

/* Last per-slot DPDK heap allocation. Every rte_zmalloc/rte_free pair on the
 * slot path costs ~66.5 ms once EAL starts syncing memseg changes over the
 * multi-process socket -- removing the op_data pair simply moved the two stalls
 * from the buffer-init/free windows into this alloc and its matching free. Keep
 * one array per direction instead; num_lcores is 1 in practice and never grows
 * mid-run.
 */
static struct thread_params *t_params_persist(unsigned which, uint16_t num_lcores)
{
  static struct thread_params *buf[2];
  static uint16_t cap[2];

  if (cap[which] < num_lcores) {
    rte_free(buf[which]);
    buf[which] = rte_zmalloc(NULL, (size_t)num_lcores * sizeof(struct thread_params), RTE_CACHE_LINE_SIZE);
    AssertFatal(buf[which] != NULL, "Couldn't allocate thread_params for %u lcores", num_lcores);
    cap[which] = num_lcores;
  } else {
    memset(buf[which], 0, (size_t)num_lcores * sizeof(struct thread_params));
  }
  return buf[which];
}


/* TB-mode decode: submit one bbdev op per transport block instead of one per
 * code block. The LA12xx PMD's decode path is TB-native (prepare_ldpc_dec_op()
 * reads tb_params.c unconditionally) and 23 of the 32 shipped decode vectors
 * are TB mode, so this is the better-tested path. It also collapses the op
 * count from C (up to 119) to 1, removing ~8 us of per-op overhead per code
 * block, and the device then returns exactly the TBS (it strips the per-CB and
 * TB CRCs itself) instead of C buffers that still carry their CRCs.
 *
 * Enable with LDPC_AAL_TB_DEC=1, together with LDPC_AAL_NO_HARQ=1: HARQ in TB
 * mode needs one contiguous region covering all C code blocks, which this does
 * not yet build.
 */
/* Transport-block mode, for BOTH directions. One operation carries the whole
 * transport block instead of one operation per code block, which is what makes
 * the per-operation round-trip cost (measured 13.4 us on decode, 11.9 us on
 * encode) amortise over C code blocks instead of being paid C times.
 * LDPC_AAL_TB_DEC is still accepted so existing scripts keep working. */
static bool tb_mode(void)
{
  static int v = -1;

  if (v < 0)
    v = (getenv("LDPC_AAL_TB") != NULL) || (getenv("LDPC_AAL_TB_DEC") != NULL);
  return v != 0;
}

/* sum(E) over the code blocks of one transport block, in bytes */
static size_t tb_enc_bytes(const nrLDPC_TB_encoding_parameters_t *p)
{
  size_t bits = 0;

  for (uint32_t i = 0; i < p->C; ++i)
    bits += (size_t)p->segments[i].E;
  return (bits + 7) / 8;
}

/* Payload bits A of a transport block, inverted from 38.212 5.2.2.
 *
 * TBs[].A cannot be used: nr_dlsch_coding.c sets it in bits but
 * nr_ulsch_coding.c sets it to pusch_data.tb_size / 8, so its units depend on
 * the caller. K, F and C are unambiguous on both paths, and
 *   K - F = K',  K' = (B + 24C)/C for C > 1,  K' = B for C == 1,  B = A + crc
 * inverts to the expression below. Verified against both DL configurations:
 * C=25 K=8448 F=64 -> 208976 and C=7 K=7744 F=256 -> 52224.
 */
static uint32_t tb_enc_A_bits(const nrLDPC_TB_encoding_parameters_t *p)
{
  uint32_t K_dash, B;

  if (p->C == 0 || p->K <= p->F)
    return 0;
  K_dash = p->K - p->F;
  B = (p->C > 1) ? (K_dash * p->C - 24 * p->C) : K_dash;
  /* crc_bits is 24 above 3824 payload bits and 16 at or below it; B still
   * carries it at this point, so decide on the larger candidate first. */
  if (B > 3824 + 24)
    return B - 24;
  return (B > 16) ? B - 16 : 0;
}

/* Can this transport block go to FECA as a single encode operation?
 *
 * The PMD takes A from input.length * 8 and then does the whole of 38.212
 * 5.2.2 itself: B = A + crc_bits, C = ceil(B/(K_cb-24)), K' = (B + 24C)/C
 * (bbdev_la12xx_feca_param.c LDPC_evaluate_parameters). So the input must be
 * the UNSEGMENTED payload and the device must be willing to attach the TB CRC
 * -- feeding it OAI's already-segmented, already-CRC'd code blocks would make
 * it derive a different C and K'. It also refuses the job unless
 * B % (8*C) == 0 (its TBS_VALID check), so verify that here rather than let
 * the device reject it.
 */
static bool tb_enc_ok(const nrLDPC_TB_encoding_parameters_t *p)
{
  static int warned;
  const char *why = NULL;
  uint32_t A = tb_enc_A_bits(p);
  uint32_t B = A + ((A > 3824) ? 24 : 16);

  if (p->a == NULL)
    why = "caller did not provide the unsegmented TB (TBs[].a is NULL)";
  else if (A < 24 || (A % 8) != 0)
    why = "could not derive a byte-aligned A from K/F/C";
  else if ((B % (8 * p->C)) != 0)
    why = "B % (8*C) != 0, the device's TBS_VALID check would fail";
  else if (!(active_dev.enc_cap_flags & RTE_BBDEV_LDPC_CRC_24A_ATTACH))
    why = "device does not advertise CRC_24A_ATTACH, so it cannot build B = A + 24";

  if (why != NULL) {
    if (!warned++)
      LOG_W(PHY, "LDPC TB-mode encode unavailable (%s); using code-block mode\n", why);
    return false;
  }
  return true;
}

/* TB-mode encode is only taken when every TB in the slot qualifies, so that
 * one slot never mixes the two layouts. */
static bool tb_enc_mode(const nrLDPC_slot_encoding_parameters_t *sp)
{
  if (!tb_mode())
    return false;
  for (int h = 0; h < sp->nb_TBs; ++h)
    if (!tb_enc_ok(&sp->TBs[h]))
      return false;
  return true;
}

/* Total code blocks in the slot. The LLR staging buffer (l_ol) is always
 * per-code-block at a LDPC_MAX_CB_SIZE stride, whichever mode we submit in, so
 * it must be sized with this and NOT with nb_segments_decoding() -- in TB mode
 * that counts transport blocks, and undersizing l_ol smashes the stack. */
static uint16_t nb_cbs_decoding(nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  uint16_t n = 0;

  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h)
    n += nrLDPC_slot_decoding_parameters->TBs[h].C;
  return n;
}

static uint16_t nb_segments_decoding(nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  uint16_t nb_segments = 0;

  if (tb_mode())
    return nrLDPC_slot_decoding_parameters->nb_TBs;
  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
    nb_segments += nrLDPC_slot_decoding_parameters->TBs[h].C;
  }
  return nb_segments;
}

static uint16_t nb_segments_encoding(nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters)
{
  uint16_t nb_segments = 0;

  if (tb_enc_mode(nrLDPC_slot_encoding_parameters))
    return nrLDPC_slot_encoding_parameters->nb_TBs;
  for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
    nb_segments += nrLDPC_slot_encoding_parameters->TBs[h].C;
  }
  return nb_segments;
}

/* Read flag value 0/1 from bitmap */
// DPDK BBDEV copy
static inline bool check_bit(uint32_t bitmap, uint32_t bitmask)
{
  return bitmap & bitmask;
}

/* rte_bbdev_info.drv.capabilities is a list terminated by RTE_BBDEV_OP_NONE,
 * NOT an array indexable by rte_bbdev_op_type. Look the entry up by type.
 *
 * Getting this wrong is silent: on the LA12xx the list is ordered
 * [0]=LDPC_ENC [1]=LDPC_DEC [2]=POLAR_DEC [3]=POLAR_ENC [4]=RAW [5]=LA12XX_VSPA,
 * so capabilities[RTE_BBDEV_OP_LDPC_DEC] (== capabilities[3]) reads the
 * POLAR_ENC entry's union instead of the LDPC decoder's.
 */
static const struct rte_bbdev_op_cap *find_bbdev_cap(const struct rte_bbdev_info *info, enum rte_bbdev_op_type type)
{
  for (int i = 0; info->drv.capabilities[i].type != RTE_BBDEV_OP_NONE; i++)
    if (info->drv.capabilities[i].type == type)
      return &info->drv.capabilities[i];
  return NULL;
}

/* The NXP LA12xx offloads LDPC to the FECA hardware block. Its PMD advertises
 * only the capability bits it needs the application to act on, so several flags
 * OAI treats as mandatory read back as 0 even though FECA performs the
 * operation unconditionally. */
static bool dev_is_la12xx(const struct rte_bbdev_info *info)
{
  return info->drv.driver_name && strcmp(info->drv.driver_name, "baseband_la12xx") == 0;
}

/* calculates optimal mempool size not smaller than the val */
// DPDK BBDEV copy
static unsigned int optimal_mempool_size(unsigned int val)
{
  return rte_align32pow2(val + 1) - 1;
}

// based on DPDK BBDEV create_mempools
static int create_mempools(struct active_device *ad, int socket_id, uint16_t num_ops, int out_buff_sz, int in_max_sz)
{
  unsigned int ops_pool_size, mbuf_pool_size, data_room_size = 0;
  num_ops = 1;
  uint8_t nb_segments = 1;
  ops_pool_size = optimal_mempool_size(RTE_MAX(
      /* Ops used plus 1 reference op */
      RTE_MAX((unsigned int)(ad->nb_queues * num_ops + 1),
              /* Minimal cache size plus 1 reference op */
              (unsigned int)(1.5 * rte_lcore_count() * OPS_CACHE_SIZE + 1)),
      OPS_POOL_SIZE_MIN));

  /* Decoder ops mempool */
  ad->bbdev_dec_op_pool = rte_bbdev_op_pool_create("bbdev_op_pool_dec",
                                                   RTE_BBDEV_OP_LDPC_DEC,
                                                   /* Encoder ops mempool */ ops_pool_size,
                                                   OPS_CACHE_SIZE,
                                                   socket_id);
  ad->bbdev_enc_op_pool =
      rte_bbdev_op_pool_create("bbdev_op_pool_enc", RTE_BBDEV_OP_LDPC_ENC, ops_pool_size, OPS_CACHE_SIZE, socket_id);

  if ((ad->bbdev_dec_op_pool == NULL) || (ad->bbdev_enc_op_pool == NULL))
    AssertFatal(1 == 0, "ERROR Failed to create %u items ops pool for dev %u on socket %d.", ops_pool_size, ad->dev_id, socket_id);

  /* Inputs */
  mbuf_pool_size = optimal_mempool_size(ops_pool_size * nb_segments);
  data_room_size = RTE_MAX(in_max_sz + RTE_PKTMBUF_HEADROOM + FILLER_HEADROOM, (unsigned int)RTE_MBUF_DEFAULT_BUF_SIZE);
  /* No TB-mode special-casing here on purpose. TB-mode decode uses direct
   * memory (persistent rte_malloc buffers, is_direct_mem = 1) and allocates no
   * mbufs at all, so these pools are used only by the CB-mode encode path with
   * ~1 kB buffers. Enlarging the room and clamping the count to 192 for TB mode
   * starved that path: measured 66.8 ms in mbuf alloc and 66.5 ms in mbuf free
   * per slot at C=25 (fast for the first two slots, then degrading), against
   * ~365 us for the actual FECA work.
   */
#ifdef LDPC_AAL_HAVE_LA12XX
  /* rte_pmd_la12xx_ldpc_enc_adj_bbuf() (the LA12xx Rev A0 FECA-encode erratum
   * workaround) requires 4096 spare bytes of tailroom in the input bbuf. */
  if (dev_is_la12xx(&active_dev.info))
    data_room_size += LA12XX_ENC_ADJ_TAILROOM;
#endif
  LOG_I(NR_PHY, "bbdev input pool: %u mbufs x %u B room (%u MiB)\n",
        mbuf_pool_size, data_room_size, (mbuf_pool_size * data_room_size) >> 20);
  ad->in_mbuf_pool = rte_pktmbuf_pool_create("in_mbuf_pool", mbuf_pool_size, 0, 0, data_room_size, socket_id);
  AssertFatal(ad->in_mbuf_pool != NULL,
              "ERROR Failed to create %u items input pktmbuf pool for dev %u on socket %d.",
              mbuf_pool_size,
              ad->dev_id,
              socket_id);

  /* Hard outputs */
  data_room_size = RTE_MAX(out_buff_sz + RTE_PKTMBUF_HEADROOM + FILLER_HEADROOM, (unsigned int)RTE_MBUF_DEFAULT_BUF_SIZE);
  ad->hard_out_mbuf_pool = rte_pktmbuf_pool_create("hard_out_mbuf_pool", mbuf_pool_size, 0, 0, data_room_size, socket_id);
  AssertFatal(ad->hard_out_mbuf_pool != NULL,
              "ERROR Failed to create %u items hard output pktmbuf pool for dev %u on socket %d.",
              mbuf_pool_size,
              ad->dev_id,
              socket_id);

  /* HARQ outputs */
  data_room_size = LDPC_MAX_CB_SIZE;
  ad->harq_out_mbuf_pool = rte_pktmbuf_pool_create("harq_out_mbuf_pool", mbuf_pool_size, 0, 0, data_room_size, socket_id);
  AssertFatal(ad->harq_out_mbuf_pool != NULL,
              "ERROR Failed to create %u items harq output pktmbuf pool for dev %u on socket %d.",
              mbuf_pool_size,
              ad->dev_id,
              socket_id);

  /* HARQ inputs */
  // Note: This is used as our harq buffer to store the combined outputs across iterations
  data_room_size = LDPC_MAX_CB_SIZE;
  ad->harq_in_mbuf_pool =
      rte_pktmbuf_pool_create("harq_in_mbuf_pool", active_dev.num_harq_codeblock, 0, 0, data_room_size, socket_id);
  AssertFatal(ad->harq_in_mbuf_pool != NULL,
              "ERROR Failed to create %u items harq input pktmbuf pool for dev %u on socket %d.",
              active_dev.num_harq_codeblock,
              ad->dev_id,
              socket_id);

  return 0;
}

const char *ldpcenc_flag_bitmask[] = {
    /** Set for bit-level interleaver bypass on output stream. */
    "RTE_BBDEV_LDPC_INTERLEAVER_BYPASS",
    /** If rate matching is to be performed */
    "RTE_BBDEV_LDPC_RATE_MATCH",
    /** Set for transport block CRC-24A attach */
    "RTE_BBDEV_LDPC_CRC_24A_ATTACH",
    /** Set for code block CRC-24B attach */
    "RTE_BBDEV_LDPC_CRC_24B_ATTACH",
    /** Set for code block CRC-16 attach */
    "RTE_BBDEV_LDPC_CRC_16_ATTACH",
    /** Set if a device supports encoder dequeue interrupts. */
    "RTE_BBDEV_LDPC_ENC_INTERRUPTS",
    /** Set if a device supports scatter-gather functionality. */
    "RTE_BBDEV_LDPC_ENC_SCATTER_GATHER",
    /** Set if a device supports concatenation of non byte aligned output */
    "RTE_BBDEV_LDPC_ENC_CONCATENATION",
};

const char *ldpcdec_flag_bitmask[] = {
    /** Set for transport block CRC-24A checking */
    "RTE_BBDEV_LDPC_CRC_TYPE_24A_CHECK",
    /** Set for code block CRC-24B checking */
    "RTE_BBDEV_LDPC_CRC_TYPE_24B_CHECK",
    /** Set to drop the last CRC bits decoding output */
    "RTE_BBDEV_LDPC_CRC_TYPE_24B_DROP"
    /** Set for bit-level de-interleaver bypass on Rx stream. */
    "RTE_BBDEV_LDPC_DEINTERLEAVER_BYPASS",
    /** Set for HARQ combined input stream enable. */
    "RTE_BBDEV_LDPC_HQ_COMBINE_IN_ENABLE",
    /** Set for HARQ combined output stream enable. */
    "RTE_BBDEV_LDPC_HQ_COMBINE_OUT_ENABLE",
    /** Set for LDPC decoder bypass.
     *  RTE_BBDEV_LDPC_HQ_COMBINE_OUT_ENABLE must be set.
     */
    "RTE_BBDEV_LDPC_DECODE_BYPASS",
    /** Set for soft-output stream enable */
    "RTE_BBDEV_LDPC_SOFT_OUT_ENABLE",
    /** Set for Rate-Matching bypass on soft-out stream. */
    "RTE_BBDEV_LDPC_SOFT_OUT_RM_BYPASS",
    /** Set for bit-level de-interleaver bypass on soft-output stream. */
    "RTE_BBDEV_LDPC_SOFT_OUT_DEINTERLEAVER_BYPASS",
    /** Set for iteration stopping on successful decode condition
     *  i.e. a successful syndrome check.
     */
    "RTE_BBDEV_LDPC_ITERATION_STOP_ENABLE",
    /** Set if a device supports decoder dequeue interrupts. */
    "RTE_BBDEV_LDPC_DEC_INTERRUPTS",
    /** Set if a device supports scatter-gather functionality. */
    "RTE_BBDEV_LDPC_DEC_SCATTER_GATHER",
    /** Set if a device supports input/output HARQ compression. */
    "RTE_BBDEV_LDPC_HARQ_6BIT_COMPRESSION",
    /** Set if a device supports input LLR compression. */
    "RTE_BBDEV_LDPC_LLR_COMPRESSION",
    /** Set if a device supports HARQ input from
     *  device's internal memory.
     */
    "RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_IN_ENABLE",
    /** Set if a device supports HARQ output to
     *  device's internal memory.
     */
    "RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_OUT_ENABLE",
    /** Set if a device supports loop-back access to
     *  HARQ internal memory. Intended for troubleshooting.
     */
    "RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_LOOPBACK",
    /** Set if a device includes LLR filler bits in the circular buffer
     *  for HARQ memory. If not set, it is assumed the filler bits are not
     *  in HARQ memory and handled directly by the LDPC decoder.
     */
    "RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_FILLERS",
};

void debug_dev_capabilities(uint8_t dev_id, struct rte_bbdev_info *info)
{
  /* Display for debug the capabilities of the card */
  for (int i = 0; info->drv.capabilities[i].type != RTE_BBDEV_OP_NONE; i++) {
    LOG_D(NR_PHY, "device: %d, capability[%d]=%s\n", dev_id, i, rte_bbdev_op_type_str(info->drv.capabilities[i].type));
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_ENC) {
      const struct rte_bbdev_op_cap_ldpc_enc cap = info->drv.capabilities[i].cap.ldpc_enc;
      LOG_D(NR_PHY, "    buffers: src = %d, dst = %d\n   capabilites: ", cap.num_buffers_src, cap.num_buffers_dst);
      for (int j = 0; j < sizeof(cap.capability_flags) * 8; j++)
        if (cap.capability_flags & (1ULL << j))
          LOG_D(NR_PHY, "%s ", ldpcenc_flag_bitmask[j]);
      LOG_D(NR_PHY, "\n");
    }
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_DEC) {
      const struct rte_bbdev_op_cap_ldpc_dec cap = info->drv.capabilities[i].cap.ldpc_dec;
      LOG_D(NR_PHY,
            "    buffers: src = %d, hard out = %d, soft_out %d, llr size %d, llr decimals %d \n   capabilities: ",
            cap.num_buffers_src,
            cap.num_buffers_hard_out,
            cap.num_buffers_soft_out,
            cap.llr_size,
            cap.llr_decimals);
      for (int j = 0; j < sizeof(cap.capability_flags) * 8; j++)
        if (cap.capability_flags & (1ULL << j))
          LOG_D(NR_PHY, "%s ", ldpcdec_flag_bitmask[j]);
      LOG_D(NR_PHY, "\n");
    }
  }
}

void check_required_dev_capabilities(struct rte_bbdev_info *info)
{
  // check ldpc enc/ dec support
  bool ldpc_enc = false;
  bool ldpc_dec = false;
  for (int i = 0; info->drv.capabilities[i].type != RTE_BBDEV_OP_NONE; i++) {
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_ENC) {
      ldpc_enc = true;
    }
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_DEC) {
      ldpc_dec = true;
    }
  }
  AssertFatal(ldpc_enc, "ERROR: bbdev device does not support LDPC encoding\n");
  AssertFatal(ldpc_dec, "ERROR: bbdev device does not support LDPC decoding\n");

  for (int i = 0; info->drv.capabilities[i].type != RTE_BBDEV_OP_NONE; i++) {
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_ENC) {
      // check encoding capabilities
      bool rate_match = check_bit(info->drv.capabilities[i].cap.ldpc_enc.capability_flags, RTE_BBDEV_LDPC_RATE_MATCH);
      if (!rate_match && dev_is_la12xx(info)) {
        /* FECA always rate-matches: NXP's own encode vectors carry n_cb/ea/eb
         * and rv_index while setting only CRC_24B_ATTACH in op_flags. */
        LOG_W(NR_PHY, "%s does not advertise LDPC_RATE_MATCH; FECA rate-matches unconditionally, continuing\n", info->drv.driver_name);
      } else {
        AssertFatal(rate_match, "ERROR: bbdev device does not support LDPC encoding with rate matching\n");
      }
    }
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_DEC) {
      // check decoding capabilities
      bool iter_stop = check_bit(info->drv.capabilities[i].cap.ldpc_dec.capability_flags, RTE_BBDEV_LDPC_ITERATION_STOP_ENABLE);
      if (!iter_stop && dev_is_la12xx(info)) {
        /* Early termination is a decoder-internal decision on FECA; the flag is
         * simply not advertised. Worst case the decoder runs iter_max always. */
        LOG_W(NR_PHY, "%s does not advertise LDPC_ITERATION_STOP_ENABLE; continuing\n", info->drv.driver_name);
      } else {
        AssertFatal(iter_stop, "ERROR: bbdev device does not support LDPC decoding with iteration stop\n");
      }

      bool crc_24b_drop = check_bit(info->drv.capabilities[i].cap.ldpc_dec.capability_flags, RTE_BBDEV_LDPC_CRC_TYPE_24B_DROP);
      AssertFatal(crc_24b_drop, "ERROR: bbdev device does not support LDPC decoding with CRC-24B drop\n");

      bool crc_24b_check = check_bit(info->drv.capabilities[i].cap.ldpc_dec.capability_flags, RTE_BBDEV_LDPC_CRC_TYPE_24B_CHECK);
      AssertFatal(crc_24b_check, "ERROR: bbdev device does not support LDPC decoding with CRC-24B check\n");
    }
  }
}

bool check_internal_harq_memory_capabilities(struct rte_bbdev_info *info)
{
  for (int i = 0; info->drv.capabilities[i].type != RTE_BBDEV_OP_NONE; i++) {
    if (info->drv.capabilities[i].type == RTE_BBDEV_OP_LDPC_DEC) {
      bool harq_in =
          check_bit(info->drv.capabilities[i].cap.ldpc_dec.capability_flags, RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_IN_ENABLE);
      bool harq_out =
          check_bit(info->drv.capabilities[i].cap.ldpc_dec.capability_flags, RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_OUT_ENABLE);
      bool internal_harq_memory_support = harq_in & harq_out;
      if (internal_harq_memory_support) {
        LOG_I(NR_PHY, "bbdev device supports internal HARQ memory\n");
      }
      return internal_harq_memory_support;
    }
  }
  return false;
}

// based on DPDK BBDEV add_bbdev_dev
static int add_dev(uint8_t dev_id, bool is_t2, uint32_t num_harq_codeblock)
{
  int ret;
  unsigned int nb_queues;

  // retrieve device capabilities
  rte_bbdev_info_get(dev_id, &active_dev.info);
  LOG_I(NR_PHY, "using bbdev %d: %s\n", dev_id, active_dev.info.dev_name);

  active_dev.driver_name = active_dev.info.drv.driver_name;
  active_dev.dev_id = dev_id;

  nb_queues = RTE_MIN(rte_lcore_count(), active_dev.info.drv.max_num_queues);
  nb_queues = RTE_MIN(nb_queues, (unsigned int)MAX_QUEUES);

  // debug device capabilities
  debug_dev_capabilities(dev_id, &active_dev.info);

  // check required device capabilities
  check_required_dev_capabilities(&active_dev.info);

  // check internal harq memory capabilities
  active_dev.support_internal_harq_memory = check_internal_harq_memory_capabilities(&active_dev.info);

  // setup harq buffers
  active_dev.num_harq_codeblock = num_harq_codeblock;
  active_dev.harq_buffers = malloc(sizeof(struct rte_bbdev_op_data) * active_dev.num_harq_codeblock);

  // is device T2?
  active_dev.is_t2 = is_t2;
  /* The LA12xx/FECA expects plain S8.0 two's-complement LLRs, exactly what the
   * T2 saturation path produces (simde_mm_packs_epi16 clamps to [-128,127]).
   * NXP's own decode vectors are one signed byte per LLR saturating at +/-127,
   * with graduated soft values well inside int8 and no fractional bits. Taking
   * the saturation path also avoids llr_scaling() entirely, which this device
   * cannot feed correctly (llr_size is reported as 0). */
  /* Remember what the device actually advertises, so we never request an
   * op_flag it does not claim. The LA12xx silently never completes such ops:
   * they enqueue fine and the dequeue loop spins until the caller's
   * "time_out <= 1e8" assertion fires. */
  const struct rte_bbdev_op_cap *_ec = find_bbdev_cap(&active_dev.info, RTE_BBDEV_OP_LDPC_ENC);
  const struct rte_bbdev_op_cap *_dc = find_bbdev_cap(&active_dev.info, RTE_BBDEV_OP_LDPC_DEC);
  active_dev.enc_cap_flags = _ec ? _ec->cap.ldpc_enc.capability_flags : 0xffffffffu;
  active_dev.dec_cap_flags = _dc ? _dc->cap.ldpc_dec.capability_flags : 0xffffffffu;

  /* Saturation (plain int16 -> int8 clamp) is a T2 convention. The LA12xx
   * advertises llr_size = 8 with llr_decimals = 1, i.e. S7.1 fixed point, so
   * its LLRs must be *rescaled* by llr_scaling() rather than clamped. */
  active_dev.saturate_llrs = is_t2;
  if (active_dev.saturate_llrs && !is_t2)
    LOG_I(NR_PHY, "%s: saturating LLRs to int8 instead of rescaling\n", active_dev.info.drv.driver_name);

  // device setup
  ret = rte_bbdev_setup_queues(dev_id, nb_queues, active_dev.info.socket_id);
  AssertFatal(ret == 0, "rte_bbdev_setup_queues(%u, %u, %d) ret %i\n", dev_id, nb_queues, active_dev.info.socket_id, ret);

  /* setup device queues */
  struct rte_bbdev_queue_conf qconf = {
      .socket = active_dev.info.socket_id,
      .queue_size = active_dev.info.drv.default_queue_conf.queue_size,
  };

  // Search a queue linked to HW capability ldpc decoding
  qconf.op_type = RTE_BBDEV_OP_LDPC_ENC;
  int queue_id;
  for (queue_id = 0; queue_id < nb_queues; ++queue_id) {
    ret = rte_bbdev_queue_configure(dev_id, queue_id, &qconf);
    if (ret == 0) {
      LOG_I(NR_PHY, "Found LDPC encoding queue (id=%u) at prio%u on dev%u\n", queue_id, qconf.priority, dev_id);
      qconf.priority++;
      active_dev.enc_queue = queue_id;
      active_dev.queue_ids[queue_id] = queue_id;
      break;
    }
  }
  AssertFatal(queue_id != nb_queues, "ERROR Failed to configure encoding queues on dev %u", dev_id);

  // Search a queue linked to HW capability ldpc encoding
  qconf.op_type = RTE_BBDEV_OP_LDPC_DEC;
  for (queue_id++; queue_id < nb_queues; ++queue_id) {
    ret = rte_bbdev_queue_configure(dev_id, queue_id, &qconf);
    if (ret == 0) {
      LOG_I(NR_PHY, "Found LDPC decoding queue (id=%u) at prio%u on dev%u\n", queue_id, qconf.priority, dev_id);
      qconf.priority++;
      active_dev.dec_queue = queue_id;
      active_dev.queue_ids[queue_id] = queue_id;
      break;
    }
  }
  AssertFatal(queue_id != nb_queues, "ERROR Failed to configure encoding queues on dev %u", dev_id);
  active_dev.nb_queues = 2;

#ifdef LDPC_AAL_HAVE_LA12XX
  if (dev_is_la12xx(&active_dev.info)) {
    /* On the LA12xx the bbdev queues are serviced by the modem's e200 cores,
     * and the binding is explicit: without it ops are enqueued but never
     * picked up, so the dequeue loop spins until pmd_lcore_ldpc_dec() trips
     * its "time_out <= 1e8" assertion. NXP's own test-bbdev does the same
     * call. Bind queue i to e200 core i; the modem-side agent is started
     * across cores 0-3 ("test bbdev_ipc1 0xf 1"), which covers the two queues
     * OAI configures (0 = encode, 1 = decode).
     */
    uint16_t core_ids[MAX_QUEUES];
    for (uint16_t q = 0; q < active_dev.nb_queues; q++)
      core_ids[q] = q;
    uint16_t rc = rte_pmd_la12xx_queue_core_config(dev_id, active_dev.queue_ids, core_ids, active_dev.nb_queues);
    AssertFatal(rc == 0, "rte_pmd_la12xx_queue_core_config() failed (%u): LA12xx queues not bound to e200 cores\n", rc);
    LOG_I(NR_PHY, "bound %u bbdev queues to LA12xx e200 cores 0..%u\n", active_dev.nb_queues, active_dev.nb_queues - 1);

    /* Use a single QDMA for the FECA shared-channel-decode input. The PMD
     * defaults to multi-QDMA; NXP's own test-bbdev switches to single QDMA and
     * is the only configuration known to work on this board. On the default
     * path the modem raises a QDMA fault as soon as a decode op is picked up
     * ("QdmaErrorHandler Irq_num 42", NXP_QDMA_DEDR 0x10000000) and never
     * returns the op.
     *
     * NXP warns this can misbehave when FECA falls behind and the QDMA
     * overwrites not-yet-read data (they call out some DEMUX cases), so it is
     * worth revisiting if decode errors appear under load.
     */
    rte_pmd_la12xx_ldpc_dec_single_input_dma(dev_id);
    LOG_I(NR_PHY, "LA12xx: enabled single-QDMA for FECA SD input\n");
  }
#endif
  return 0;
}

static int init_op_data_objs_harq(struct rte_bbdev_op_data *bufs, struct rte_mempool *mbuf_pool)
{
  for (int i = 0; i < active_dev.num_harq_codeblock; i++) {
    struct rte_mbuf *m_head = rte_pktmbuf_alloc(mbuf_pool);
    AssertFatal(m_head != NULL,
                "Not enough mbufs in HARQ mbuf pool (needed %u, available %u)",
                active_dev.num_harq_codeblock,
                mbuf_pool->size);
    bufs[i].data = m_head;
    bufs[i].offset = 0;
    bufs[i].length = 0;
  }
  return 0;
}

// based on DPDK BBDEV init_op_data_objs
static int init_op_data_objs_dec(struct rte_bbdev_op_data *bufs,
                                 uint8_t *input,
                                 nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters,
                                 struct rte_mempool *mbuf_pool,
                                 enum op_data_type op_type,
                                 uint16_t min_alignment)
{
  bool large_input = false;
  int j = 0;

  if (tb_mode()) {
    /* One buffer per TB holding all C code blocks contiguously. The LLR
     * conversion writes each CB at a LDPC_MAX_CB_SIZE stride, so gather them. */
    int cb_base = 0;
    for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
      uint32_t total = (uint32_t)tb_llr_bytes(p);
      AssertFatal((unsigned)h < LDPC_AAL_TB_MAX_TBS, "nb_TBs %d exceeds LDPC_AAL_TB_MAX_TBS", h + 1);

      /* No mbuf in TB mode. A transport block does not fit one (the room is a
       * uint16_t), and the trick of pointing an mbuf at foreign memory breaks
       * the pool: the decoder's cleanup calls rte_pktmbuf_free() on it, which
       * returns a mangled mbuf to the mempool -- measured as 127 ms/slot and a
       * segfault. Use the PMD's direct-memory path instead, which is what
       * dpdk-test-bbdev does for this device: hand it the pointer and the
       * length, and it never looks at an mbuf. The cleanup loop skips the free
       * in TB mode for the same reason.
       */
      bufs[h].offset = 0;
      if (op_type == DATA_INPUT) {
        /* Gather the per-CB aligned slots into one contiguous block. The buffer
         * is persistent, so this costs a single ~235 kB memcpy per slot and no
         * allocation. (The copy itself cannot be removed -- see the alignment
         * note at the LLR conversion.) */
        uint8_t *dst = tb_buf(&tb_llr_buf[h], &tb_llr_cap[h], total);
        uint32_t off = 0;
        for (int i = 0; i < p->C; ++i) {
          uint32_t len = (i < p->first_rE2) ? (uint32_t)p->E : (uint32_t)p->E2;
          rte_memcpy(dst + off, &input[(cb_base + i) * LDPC_MAX_CB_SIZE], len);
          off += len;
        }
        bufs[h].mem = dst;
        bufs[h].length = total;
      } else {
        bufs[h].mem = tb_buf(&tb_out_buf[h], &tb_out_cap[h], (size_t)p->A / 8 + 64);
        bufs[h].length = 0;
      }
      bufs[h].is_direct_mem = 1;
      cb_base += p->C;
    }
    return 0;
  }

  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
    nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
    for (int i = 0; i < p->C; ++i) {
      uint32_t data_len = i < p->first_rE2 ? p->E : p->E2;
      char *data;
      struct rte_mbuf *m_head = rte_pktmbuf_alloc(mbuf_pool);
      AssertFatal(m_head != NULL,
                  "Not enough mbufs in %d data type mbuf pool (needed %u, available %u)",
                  op_type,
                  nb_segments_decoding(nrLDPC_slot_decoding_parameters),
                  mbuf_pool->size);

      if (data_len > RTE_BBDEV_LDPC_E_MAX_MBUF) {
        printf("Warning: Larger input size than DPDK mbuf %u\n", data_len);
        large_input = true;
      }
      bufs[j].data = m_head;
      bufs[j].offset = 0;
      bufs[j].length = 0;

      if (op_type == DATA_INPUT) {
        if (large_input) {
          /* Allocate a fake overused mbuf */
          data = rte_malloc(NULL, data_len, 0);
          AssertFatal(data != NULL, "rte malloc failed with %u bytes", data_len);
          memcpy(data, &input[j * LDPC_MAX_CB_SIZE], data_len);
          m_head->buf_addr = data;
          m_head->buf_iova = rte_malloc_virt2iova(data);
          m_head->data_off = 0;
          m_head->data_len = data_len;
        } else {
          rte_pktmbuf_reset(m_head);
          data = rte_pktmbuf_append(m_head, data_len);
          AssertFatal(data != NULL, "Couldn't append %u bytes to mbuf from %d data type mbuf pool", data_len, op_type);
          AssertFatal(data == RTE_PTR_ALIGN(data, min_alignment),
                      "Data addr in mbuf (%p) is not aligned to device min alignment (%u)",
                      data,
                      min_alignment);
          rte_memcpy(data, &input[j * LDPC_MAX_CB_SIZE], data_len);
        }
        bufs[j].length += data_len;
      }
      ++j;
    }
  }
  return 0;
}

// based on DPDK BBDEV init_op_data_objs
static int init_op_data_objs_enc(struct rte_bbdev_op_data *bufs,
                                 nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters,
                                 struct rte_mempool *mbuf_pool,
                                 enum op_data_type op_type,
                                 uint16_t min_alignment)
{
  bool large_input = false;
  int j = 0;

  if (tb_enc_mode(nrLDPC_slot_encoding_parameters)) {
    /* One buffer per TB. No mbuf: a transport block does not fit one (the room
     * is a uint16_t) and pointing an mbuf at foreign memory corrupts the pool
     * on free -- same reasoning as the decode path. Use the PMD's direct-memory
     * path, which is what dpdk-test-bbdev does for this device.
     *
     * The input is the A payload bits only. TBs[].a holds payload followed by
     * the TB CRC, and the device attaches its own (CRC_24A_ATTACH), so copy
     * just A/8 bytes. It has to be copied rather than referenced because
     * is_direct_mem needs DPDK-registered memory with a valid IOVA, and the
     * caller's buffer is plain malloc. One memcpy per TB replaces C of them. */
    for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_encoding_parameters_t *p = &nrLDPC_slot_encoding_parameters->TBs[h];
      AssertFatal((unsigned)h < LDPC_AAL_TB_MAX_TBS, "nb_TBs %d exceeds LDPC_AAL_TB_MAX_TBS", h + 1);

      bufs[h].offset = 0;
      bufs[h].is_direct_mem = 1;
      if (op_type == DATA_INPUT) {
        uint32_t len = tb_enc_A_bits(p) / 8;
        uint8_t *dst = tb_buf(&tb_enc_in_buf[h], &tb_enc_in_cap[h], len);
        rte_memcpy(dst, p->a, len);
        bufs[h].mem = dst;
        bufs[h].length = len;
      } else {
        /* Room for sum(E) plus slack: the device reports what it produced. */
        bufs[h].mem = tb_buf(&tb_enc_out_buf[h], &tb_enc_out_cap[h], tb_enc_bytes(p) + 64);
        bufs[h].length = 0;
      }
    }
    return 0;
  }

  for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
    for (int i = 0; i < nrLDPC_slot_encoding_parameters->TBs[h].C; ++i) {
      uint32_t data_len = (nrLDPC_slot_encoding_parameters->TBs[h].K - nrLDPC_slot_encoding_parameters->TBs[h].F + 7) / 8;
      char *data;
      struct rte_mbuf *m_head = rte_pktmbuf_alloc(mbuf_pool);
      AssertFatal(m_head != NULL,
                  "Not enough mbufs in %d data type mbuf pool (needed %u, available %u)",
                  op_type,
                  nb_segments_encoding(nrLDPC_slot_encoding_parameters),
                  mbuf_pool->size);

      if (data_len > RTE_BBDEV_LDPC_E_MAX_MBUF) {
        printf("Warning: Larger input size than DPDK mbuf %u\n", data_len);
        large_input = true;
      }
      bufs[j].data = m_head;
      bufs[j].offset = 0;
      bufs[j].length = 0;

      if (op_type == DATA_INPUT) {
        if (large_input) {
          /* Allocate a fake overused mbuf */
          data = rte_malloc(NULL, data_len, 0);
          AssertFatal(data != NULL, "rte malloc failed with %u bytes", data_len);
          memcpy(data, nrLDPC_slot_encoding_parameters->TBs[h].segments[i].c, data_len);
          m_head->buf_addr = data;
          m_head->buf_iova = rte_malloc_virt2iova(data);
          m_head->data_off = 0;
          m_head->data_len = data_len;
        } else {
          rte_pktmbuf_reset(m_head);
          /* No rte_pmd_la12xx_ldpc_enc_adj_bbuf() here: that shift works around
           * the Rev A0 FECA-encode erratum (UG10384 10.2.14.1). This board is
           * Rev-B -- the yami driver logs "LA1224-RDB Rev-B (SVR:0x81520010)"
           * and programs GEUL_HOST_REVB_VAL -- so the erratum does not apply
           * and shifting the buffer only misaligns the encode input.
           */
          data = rte_pktmbuf_append(m_head, data_len);
          AssertFatal(data != NULL, "Couldn't append %u bytes to mbuf from %d data type mbuf pool", data_len, op_type);
          AssertFatal(data == RTE_PTR_ALIGN(data, min_alignment),
                      "Data addr in mbuf (%p) is not aligned to device min alignment (%u)",
                      data,
                      min_alignment);
          rte_memcpy(data, nrLDPC_slot_encoding_parameters->TBs[h].segments[i].c, data_len);
        }
        bufs[j].length += data_len;
      }
      ++j;
    }
  }
  return 0;
}

// DPDK BBEV copy
/* Persistent op_data arrays.
 *
 * These used to be rte_zmalloc'd and rte_free'd on every slot -- two per encode
 * plus four per decode. DPDK's heap alloc/free syncs memseg changes over the
 * multi-process channel, and once that starts happening each call blocks on
 * /var/run/dpdk/<prefix>/mp_socket until it times out: sampled kernel stacks
 * sat in __skb_wait_for_more_packets/unix_dgram_recvmsg, costing ~66.5 ms in
 * the buffer-init window and another ~66.5 ms in the free window, from the
 * third slot onwards, against ~365 us for the actual FECA work.
 *
 * The mempools are healthy throughout (avail_count stays at 1023 on every
 * slot, stalled or not), so the mbufs were never involved. Allocate these once
 * and grow only if a bigger slot arrives.
 */
static struct rte_bbdev_op_data *op_data_buf[2][DATA_NUM_TYPES];
static unsigned op_data_cap[2][DATA_NUM_TYPES];

static struct rte_bbdev_op_data *persistent_op_data(unsigned which, unsigned type, unsigned n, int socket)
{
  size_t bytes = (size_t)n * sizeof(struct rte_bbdev_op_data);

  if (op_data_cap[which][type] < n) {
    rte_free(op_data_buf[which][type]);
    op_data_buf[which][type] = rte_zmalloc_socket(NULL, bytes, RTE_CACHE_LINE_SIZE, socket);
    AssertFatal(op_data_buf[which][type] != NULL, "Couldn't allocate %zu B of rte_bbdev_op_data", bytes);
    op_data_cap[which][type] = n;
  } else {
    memset(op_data_buf[which][type], 0, bytes);
  }
  return op_data_buf[which][type];
}

static int allocate_buffers_on_socket(struct rte_bbdev_op_data **buffers, const int len, const int socket)
{
  int i;

  *buffers = rte_zmalloc_socket(NULL, len, 0, socket);
  if (*buffers == NULL) {
    printf("WARNING: Failed to allocate op_data on socket %d\n", socket);
    /* try to allocate memory on other detected sockets */
    for (i = 0; i < socket; i++) {
      *buffers = rte_zmalloc_socket(NULL, len, 0, i);
      if (*buffers != NULL)
        break;
    }
  }

  return (*buffers == NULL) ? -1 : 0;
}

// DPDK BBDEV copy
static void free_mempools(struct active_device *ad)
{
  rte_mempool_free(ad->bbdev_dec_op_pool);
  rte_mempool_free(ad->bbdev_enc_op_pool);
  rte_mempool_free(ad->in_mbuf_pool);
  rte_mempool_free(ad->hard_out_mbuf_pool);
  rte_mempool_free(ad->harq_in_mbuf_pool);
  rte_mempool_free(ad->harq_out_mbuf_pool);
}

// based on DPDK BBDEV copy_reference_ldpc_dec_op
static void set_ldpc_dec_op(struct rte_bbdev_dec_op **ops,
                            struct rte_bbdev_op_data *inputs,
                            struct rte_bbdev_op_data *outputs,
                            struct rte_bbdev_op_data *harq_outputs,
                            nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  int j = 0;

  if (tb_mode()) {
    for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
      struct rte_bbdev_op_ldpc_dec *d = &ops[h]->ldpc_dec;

      d->basegraph = p->BG;
      d->z_c = p->Z;
      d->q_m = p->Qm;
      d->n_filler = p->F;
      d->n_cb = (p->BG == 1) ? (66 * p->Z) : (50 * p->Z);
      d->iter_max = p->max_ldpc_iterations;
      d->rv_index = p->rv_index;
      d->op_flags = RTE_BBDEV_LDPC_ITERATION_STOP_ENABLE;
      *p->processedSegments = 0;

      /* Describe the whole transport block: C code blocks, the first
       * first_rE2 of them with E bits (ea) and the rest with E2 (eb). */
      d->code_block_mode = 0;
      d->tb_params.c = p->C;
      d->tb_params.cab = p->first_rE2;
      d->tb_params.ea = p->E;
      d->tb_params.eb = p->E2;
      d->tb_params.r = 0;
      if (p->C > 1) {
        d->op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_DROP;
        d->op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_CHECK;
      }

      /* Both already carry is_direct_mem and the persistent pointer. */
      d->input = inputs[h];
      d->hard_output = outputs[h];
    }
    return;
  }
  // The T2 only supports CB mode, and does not TB mode special case handling.
  bool special_case_tb_mode = !active_dev.is_t2 && (nrLDPC_slot_decoding_parameters->nb_TBs == 1)
                              && (nb_segments_decoding(nrLDPC_slot_decoding_parameters) == 1);
#ifdef LDPC_AAL_HAVE_LA12XX
  /* The LA12xx decode path in the PMD only understands the TB-mode view of the
   * tb_params/cb_params union: it reads tb_params.c unconditionally, with no
   * code_block_mode check. In CB mode we only write cb_params.e, which aliases
   * tb_params.ea, so tb_params.c reads back as 0 -- the PMD then sets
   * hard_output.length = 0 and builds a null descriptor, and the device returns
   * 0 iterations and no data (observed on every config, UL and DL).
   *
   * We enqueue every code block individually, so describe each as a partial TB
   * with c = 1, which is what the bbdev programming guide prescribes for a CB
   * enqueued on its own, and what the branch below already does for the
   * single-CB case.
   */
  if (dev_is_la12xx(&active_dev.info))
    special_case_tb_mode = true;
#endif
  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
    nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
    for (int i = 0; i < p->C; ++i) {
      ops[j]->ldpc_dec.basegraph = p->BG;
      ops[j]->ldpc_dec.z_c = p->Z;
      ops[j]->ldpc_dec.q_m = p->Qm;
      ops[j]->ldpc_dec.n_filler = p->F;
      ops[j]->ldpc_dec.n_cb = (p->BG == 1) ? (66 * p->Z) : (50 * p->Z);
      ops[j]->ldpc_dec.iter_max = p->max_ldpc_iterations;
      ops[j]->ldpc_dec.rv_index = p->rv_index;
      ops[j]->ldpc_dec.op_flags = RTE_BBDEV_LDPC_ITERATION_STOP_ENABLE | RTE_BBDEV_LDPC_HQ_COMBINE_OUT_ENABLE;
      if (p->d_to_be_cleared) {
        if (active_dev.is_t2)
          *p->processedSegments = 0;
      } else {
        ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_HQ_COMBINE_IN_ENABLE;
        if (active_dev.support_internal_harq_memory) {
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_IN_ENABLE;
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_INTERNAL_HARQ_MEMORY_OUT_ENABLE;
        }
      }

      if (!active_dev.is_t2)
        *p->processedSegments = 0;

      if (!special_case_tb_mode) {
        ops[j]->ldpc_dec.code_block_mode = 1;
        ops[j]->ldpc_dec.cb_params.e = i < p->first_rE2 ? p->E : p->E2;
        if (p->C > 1) {
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_DROP;
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_CHECK;
        }
      } else {
        /**
         * This is a special case when #TB = 1 and #CB = 1
         * In this case, we must use TB mode
         * Quoted from: https://doc.dpdk.org/guides-23.11/prog_guide/bbdev.html#bbdev-ldpc-decode-operation
         * The case when one CB belongs to TB and is being enqueued individually to BBDEV, this case is considered as a
         * special case of partial TB where its number of CBs is 1. Therefore, it requires to get processed in TB-mode.
         */
        ops[j]->ldpc_dec.code_block_mode = 0;
        ops[j]->ldpc_dec.tb_params.c = 1;
        ops[j]->ldpc_dec.tb_params.r = 0;
        ops[j]->ldpc_dec.tb_params.cab = 1;
        ops[j]->ldpc_dec.tb_params.ea = i < p->first_rE2 ? p->E : p->E2;
        ops[j]->ldpc_dec.tb_params.eb = i < p->first_rE2 ? p->E : p->E2;
        /* A CB of a multi-CB TB still carries its own 24B CRC; ask the device
         * to check and strip it, which is also what populates crc_stat. */
        if (p->C > 1) {
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_DROP;
          ops[j]->ldpc_dec.op_flags |= RTE_BBDEV_LDPC_CRC_TYPE_24B_CHECK;
        }
      }
      // Calculate offset in the HARQ combined buffers
      // Unique segment offset
      uint32_t segment_offset = (p->harq_unique_pid * NR_LDPC_MAX_NUM_CB) + i;
      // Prune to avoid shooting above maximum id
      uint32_t pruned_segment_offset = segment_offset % active_dev.num_harq_codeblock;
      // Segment offset to byte offset
      uint32_t harq_combined_offset = pruned_segment_offset * LDPC_MAX_CB_SIZE;

      if (active_dev.support_internal_harq_memory) {
        // retrieve corresponding HARQ output information from previous iteration, especially the length
        ops[j]->ldpc_dec.harq_combined_input = active_dev.harq_buffers[pruned_segment_offset];
        // Note: When using INTERNAL_HARQ memory, the "offset" is used to point to a particular address
        // within the BBDEV's onboard memory, and the address should be multiples of 32K.
        harq_outputs[j].offset = harq_combined_offset;
        ops[j]->ldpc_dec.harq_combined_output = harq_outputs[j];
      } else {
        // retrieve corresponding HARQ buffers from previous iteration
        ops[j]->ldpc_dec.harq_combined_input = active_dev.harq_buffers[pruned_segment_offset];
        ops[j]->ldpc_dec.harq_combined_output = harq_outputs[j];
      }
      ops[j]->ldpc_dec.hard_output = outputs[j];
      ops[j]->ldpc_dec.input = inputs[j];
#ifdef LDPC_AAL_HAVE_LA12XX
      if (dev_is_la12xx(&active_dev.info)) {
        /* The PMD reaches the payload through get_data_ptr(), which returns
         * op_data->mem when is_direct_mem is set and rte_bbuf_mtod(bdata)
         * otherwise. In the bbuf path it is also responsible for growing the
         * output mbuf (rte_bbuf_append), and that append is not happening for
         * us -- the descriptor is built and the CRC comes back OK, but the
         * mbuf keeps data_len = 0 so there is nothing to copy out.
         *
         * Declare direct memory instead, which is what dpdk-test-bbdev does
         * for this device: hand the PMD the data pointer itself and take the
         * size from hard_output.length, which the PMD fills in. Note the
         * offset is NOT applied on top of ->mem, so fold it in here.
         *
         * Deliberately not done for the HARQ buffers: with internal HARQ
         * memory their .offset addresses device-side memory rather than a
         * buffer offset.
         */
        struct rte_mbuf *out_m = ops[j]->ldpc_dec.hard_output.data;
        uint32_t out_off = ops[j]->ldpc_dec.hard_output.offset;
        ops[j]->ldpc_dec.hard_output.mem = rte_pktmbuf_mtod_offset(out_m, void *, out_off);
        ops[j]->ldpc_dec.hard_output.is_direct_mem = 1;
        ops[j]->ldpc_dec.hard_output.length = 0;
      }
#endif

      ++j;
    }
    p->d_to_be_cleared = false;
  }
}

// based on DPDK BBDEV copy_reference_ldpc_enc_op
static void set_ldpc_enc_op(struct rte_bbdev_enc_op **ops,
                            struct rte_bbdev_op_data *inputs,
                            struct rte_bbdev_op_data *outputs,
                            nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters)
{
  int j = 0;

  if (tb_enc_mode(nrLDPC_slot_encoding_parameters)) {
    for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_encoding_parameters_t *p = &nrLDPC_slot_encoding_parameters->TBs[h];
      struct rte_bbdev_op_ldpc_enc *e = &ops[h]->ldpc_enc;
      uint32_t cab;

      e->basegraph = p->BG;
      e->z_c = p->Z;
      e->q_m = p->Qm;
      e->n_filler = p->F;
      e->n_cb = (p->BG == 1) ? (66 * p->Z) : (50 * p->Z);
      if (p->tbslbrm != 0) {
        uint32_t Nref = 3 * p->tbslbrm / (2 * p->C);
        e->n_cb = min(e->n_cb, Nref);
      }
      e->rv_index = p->rv_index;
      /* CRC_24A_ATTACH is what makes the PMD use B = A + 24 for the TB it is
       * handed; the per-code-block 24B CRC is then added inside
       * LDPC_evaluate_parameters unconditionally for C > 1, so it needs no
       * flag of its own. */
      e->op_flags = RTE_BBDEV_LDPC_RATE_MATCH | RTE_BBDEV_LDPC_CRC_24A_ATTACH;

      /* ea applies to the first cab code blocks, eb to the rest. OAI gives E
       * per segment rather than an (E, E2, first_rE2) triple as on decode, so
       * recover the split by counting the leading run. */
      for (cab = 1; cab < p->C && p->segments[cab].E == p->segments[0].E; ++cab)
        ;
      e->code_block_mode = 0;
      e->tb_params.c = p->C;
      e->tb_params.r = 0;
      e->tb_params.cab = cab;
      e->tb_params.ea = p->segments[0].E;
      e->tb_params.eb = p->segments[p->C - 1].E;

      e->input = inputs[h];
      e->output = outputs[h];
    }
    return;
  }
  // The T2 only supports CB mode, and does not TB mode special case handling.
  bool special_case_tb_mode = !active_dev.is_t2 && (nrLDPC_slot_encoding_parameters->nb_TBs == 1)
                              && (nb_segments_encoding(nrLDPC_slot_encoding_parameters) == 1);
  for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
    for (int i = 0; i < nrLDPC_slot_encoding_parameters->TBs[h].C; ++i) {
      ops[j]->ldpc_enc.basegraph = nrLDPC_slot_encoding_parameters->TBs[h].BG;
      ops[j]->ldpc_enc.z_c = nrLDPC_slot_encoding_parameters->TBs[h].Z;
      ops[j]->ldpc_enc.q_m = nrLDPC_slot_encoding_parameters->TBs[h].Qm;
      ops[j]->ldpc_enc.n_filler = nrLDPC_slot_encoding_parameters->TBs[h].F;
      ops[j]->ldpc_enc.n_cb = (nrLDPC_slot_encoding_parameters->TBs[h].BG == 1) ? (66 * nrLDPC_slot_encoding_parameters->TBs[h].Z)
                                                                                : (50 * nrLDPC_slot_encoding_parameters->TBs[h].Z);
      if (nrLDPC_slot_encoding_parameters->TBs[h].tbslbrm != 0) {
        uint32_t Nref = 3 * nrLDPC_slot_encoding_parameters->TBs[h].tbslbrm / (2 * nrLDPC_slot_encoding_parameters->TBs[h].C);
        ops[j]->ldpc_enc.n_cb = min(ops[j]->ldpc_enc.n_cb, Nref);
      }
      ops[j]->ldpc_enc.rv_index = nrLDPC_slot_encoding_parameters->TBs[h].rv_index;
      ops[j]->ldpc_enc.op_flags = RTE_BBDEV_LDPC_RATE_MATCH;
      if (!special_case_tb_mode) {
        ops[j]->ldpc_enc.code_block_mode = 1;
        ops[j]->ldpc_enc.cb_params.e = nrLDPC_slot_encoding_parameters->TBs[h].segments[i].E;
      } else {
        /**
         * This is a special case when #TB = 1 and #CB = 1
         * In this case, we must use TB mode
         * Quoted from: https://doc.dpdk.org/guides-23.11/prog_guide/bbdev.html#bbdev-ldpc-decode-operation
         * The case when one CB belongs to TB and is being enqueued individually to BBDEV, this case is considered as a
         * special case of partial TB where its number of CBs is 1. Therefore, it requires to get processed in TB-mode.
         */
        ops[j]->ldpc_enc.code_block_mode = 0;
        ops[j]->ldpc_enc.tb_params.c = 1;
        ops[j]->ldpc_enc.tb_params.r = 0;
        ops[j]->ldpc_enc.tb_params.cab = 1;
        ops[j]->ldpc_enc.tb_params.ea = nrLDPC_slot_encoding_parameters->TBs[h].segments[i].E;
        ops[j]->ldpc_enc.tb_params.eb = nrLDPC_slot_encoding_parameters->TBs[h].segments[i].E;
      }
      ops[j]->ldpc_enc.output = outputs[j];
      ops[j]->ldpc_enc.input = inputs[j];
      ++j;
    }
  }
}

static int retrieve_ldpc_dec_op(struct rte_bbdev_dec_op **ops, nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  int j = 0;

  if (tb_mode()) {
    for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
      struct rte_bbdev_op_data *hard_output = &ops[h]->ldpc_dec.hard_output;
      uint32_t data_len;
      uint8_t *data;

      if (hard_output->is_direct_mem) {
        data_len = hard_output->length;
        data = (uint8_t *)hard_output->mem;
      } else {
        struct rte_mbuf *m = hard_output->data;
        uint32_t mbuf_len = rte_pktmbuf_data_len(m);
        data_len = mbuf_len ? (mbuf_len - hard_output->offset) : hard_output->length;
        data = rte_pktmbuf_mtod_offset(m, uint8_t *, hard_output->offset);
      }
      if (getenv("LDPC_AAL_DEBUG_DEC"))
        printf("[aal-dec-tb] TB%d C=%u c=%u cab=%u ea=%d eb=%d out_len=%u crc_stat=%02x%02x status=0x%x iter=%u first4=%02x%02x%02x%02x\n",
               h, p->C, ops[h]->ldpc_dec.tb_params.c, ops[h]->ldpc_dec.tb_params.cab,
               p->E, p->E2, data_len, ops[h]->crc_stat[1], ops[h]->crc_stat[0],
               ops[h]->status, ops[h]->ldpc_dec.iter_count,
               data[0], data[1], data[2], data[3]);
      memcpy(p->c, data, data_len);
    }
    return 0;
  }
  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
    nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
    size_t data_off = 0;
    for (int i = 0; i < p->C; ++i) {
      struct rte_bbdev_op_data *hard_output = &ops[j]->ldpc_dec.hard_output;
      uint32_t data_len;
      uint8_t *data;
      if (hard_output->is_direct_mem) {
        /* ->mem already points at the data (offset folded in at enqueue) and
         * the PMD wrote the produced size into ->length. Do not touch ->data:
         * it shares the union with ->mem and is no longer an mbuf pointer. */
        data_len = hard_output->length;
        data = (uint8_t *)hard_output->mem;
      } else {
        struct rte_mbuf *m = hard_output->data;
        data_len = rte_pktmbuf_data_len(m) - hard_output->offset;
        data = rte_pktmbuf_mtod_offset(m, uint8_t *, hard_output->offset);
      }
      /* Temporary instrumentation: set LDPC_AAL_DEBUG_DEC to dump what the
       * device actually returned per code block. */
      if (getenv("LDPC_AAL_DEBUG_DEC"))
        printf("[aal-dec] TB%d CB%d/%u cbmode=%u hard_len=%u mbuf_data_len=%u off=%u data_len=%u "
               "crc_stat0=0x%02x status=0x%x iter=%u K=%u F=%u E=%u dmem=%u in_dmem=%u first4=%02x%02x%02x%02x\n",
               h, i, p->C, ops[j]->ldpc_dec.code_block_mode, ops[j]->ldpc_dec.hard_output.length,
               0u, hard_output->offset, data_len,
               ops[j]->crc_stat[0], ops[j]->status, ops[j]->ldpc_dec.iter_count,
               p->K, p->F, p->E, hard_output->is_direct_mem,
               ops[j]->ldpc_dec.input.is_direct_mem,
               data[0], data[1], data[2], data[3]);
      memcpy(p->c + data_off, data, data_len);

      uint32_t segment_offset = (p->harq_unique_pid * NR_LDPC_MAX_NUM_CB) + i;
      uint32_t pruned_segment_offset = segment_offset % active_dev.num_harq_codeblock;
      struct rte_bbdev_op_data *harq_output = &ops[j]->ldpc_dec.harq_combined_output;
      if (!active_dev.support_internal_harq_memory) {
        struct rte_mbuf *m_src = harq_output->data;
        uint8_t *data_src = rte_pktmbuf_mtod_offset(m_src, uint8_t *, 0);
        struct rte_mbuf *m_dst = active_dev.harq_buffers[pruned_segment_offset].data;
        uint8_t *data_dst = rte_pktmbuf_mtod_offset(m_dst, uint8_t *, 0);
        rte_memcpy(data_dst, data_src, harq_output->length);
      }
      active_dev.harq_buffers[pruned_segment_offset].offset = harq_output->offset;
      active_dev.harq_buffers[pruned_segment_offset].length = harq_output->length;
      ++j;
      data_off += p->K >> 3;
    }
  }
  return 0;
}

static int retrieve_ldpc_enc_op(struct rte_bbdev_enc_op **ops, nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters)
{
  uint8_t *p_out = NULL;
  int j = 0;

  if (tb_enc_mode(nrLDPC_slot_encoding_parameters)) {
    /* The device concatenates the rate-matched code blocks itself, so the
     * output is already the G bits OAI wants -- one copy, and none of the
     * per-code-block bit-offset stitching the CB path has to do. */
    for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_encoding_parameters_t *p = &nrLDPC_slot_encoding_parameters->TBs[h];
      struct rte_bbdev_op_data *output = &ops[h]->ldpc_enc.output;
      uint8_t *data = output->mem;
      size_t want = tb_enc_bytes(p);
      uint32_t got = output->length;

      AssertFatal(data != NULL, "TB-mode encode output has no buffer");
      if (got == 0 || got > want)
        got = (uint32_t)want;
      if (getenv("LDPC_AAL_FORCE_BITREV"))
        reverse_bits_u8(data, got, data);
      memcpy(p->output, data, got);
      if (got < want)
        memset(p->output + got, 0, want - got);
    }
    return 0;
  }

  for (int h = 0; h < nrLDPC_slot_encoding_parameters->nb_TBs; ++h) {
    int E_sum = 0;
    int bit_offset = 0;
    int byte_offset = 0;
    p_out = nrLDPC_slot_encoding_parameters->TBs[h].output;
    for (int r = 0; r < nrLDPC_slot_encoding_parameters->TBs[h].C; ++r) {
      struct rte_bbdev_op_data *output = &ops[j]->ldpc_enc.output;
      struct rte_mbuf *m = output->data;
      /* The LA12xx PMD reports the produced size in output->length and does not
       * grow the mbuf (its rte_bbuf_append() is conditional and does not fire
       * for us), so rte_pktmbuf_data_len() stays 0 and we would copy nothing --
       * emitting an all-zero codeword while FECA had in fact written the real
       * one into this very buffer. Prefer the length the device reported. */
      uint32_t mbuf_len = rte_pktmbuf_data_len(m);
      uint32_t data_len = mbuf_len ? (mbuf_len - output->offset) : output->length;
      uint8_t *data = rte_pktmbuf_mtod_offset(m, uint8_t *, output->offset);
      /* Byte-level bit reversal of the FECA encode output was an adaptation for
       * the old FECA / DPDK 19.11 combination. Gated so it can be toggled
       * without a rebuild while the convention of this FECA is established.
       * VERIFIED WRONG on this Rev-B/DPDK25.11 board (2026-09-11): reversing corrupts
       * the codeword -- offload-encode + CPU-decode gives 8/8 neg-CRC with it, 0/8
       * without. Kept opt-in via LDPC_AAL_FORCE_BITREV for the legacy 19.11 path. */
      if (getenv("LDPC_AAL_FORCE_BITREV"))
        reverse_bits_u8(data, data_len, data);
      if (bit_offset == 0) {
        memcpy(&p_out[byte_offset], data, data_len);
      } else {
        size_t i = 0;
        for (; i < (data_len & ~0x7); i += 8) {
          uint8_t carry = *data << bit_offset;
          p_out[byte_offset + i - 1] |= carry;

          simde__m64 current = *((simde__m64 *)data);
          data += 8;
          current = simde_mm_srli_si64(current, 8 - bit_offset);
          *(simde__m64 *)&p_out[byte_offset + i] = current;
        }
        for (; i < data_len; i++) {
          uint8_t current = *data++;

          uint8_t carry = current << bit_offset;
          p_out[byte_offset + i - 1] |= carry;

          p_out[byte_offset + i] = (current >> (8 - bit_offset));
        }
      }
      E_sum += nrLDPC_slot_encoding_parameters->TBs[h].segments[r].E;
      byte_offset = (E_sum + 7) / 8;
      bit_offset = E_sum % 8;
      ++j;
    }
  }
  return 0;
}

// based on DPDK BBDEV throughput_pmd_lcore_ldpc_dec
static int pmd_lcore_ldpc_dec(void *arg)
{
  struct thread_params *tp = arg;
  nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters = tp->nrLDPC_slot_decoding_parameters;
  int time_out = 0;
  const uint16_t queue_id = tp->queue_id;
  const uint16_t num_segments = nb_segments_decoding(nrLDPC_slot_decoding_parameters);
  struct rte_bbdev_dec_op *ops_enq[num_segments];
  struct rte_bbdev_dec_op *ops_deq[num_segments];
  struct data_buffers *bufs = tp->data_buffers;

  AssertFatal((num_segments < MAX_BURST), "BURST_SIZE should be <= %u", MAX_BURST);

  while (rte_atomic16_read(&tp->op_params->sync) == SYNC_WAIT)
    rte_pause();

  int ret = rte_bbdev_dec_op_alloc_bulk(tp->bbdev_op_pool, ops_enq, num_segments);
  AssertFatal(ret == 0, "Allocation failed for %d ops", num_segments);
  set_ldpc_dec_op(ops_enq, bufs->inputs, bufs->hard_outputs, bufs->harq_outputs, nrLDPC_slot_decoding_parameters);

  // Start timer
  // We report timing only once in (0,0) since the timers are merged at the end
  start_meas(&nrLDPC_slot_decoding_parameters->TBs[0].ts_ldpc_decode);

  uint16_t enq = 0, deq = 0;
  while (enq < num_segments) {
    uint16_t num_to_enq = num_segments - enq;
    /* Drop any op_flag the device does not advertise (see enc/dec_cap_flags). */
    uint32_t dec_mask = active_dev.dec_cap_flags;
    if (getenv("LDPC_AAL_NO_HARQ"))
      dec_mask &= ~(uint32_t)(RTE_BBDEV_LDPC_HQ_COMBINE_IN_ENABLE | RTE_BBDEV_LDPC_HQ_COMBINE_OUT_ENABLE);
    for (uint16_t k = 0; k < num_to_enq; k++)
      ops_enq[enq + k]->ldpc_dec.op_flags &= dec_mask;
    enq += rte_bbdev_enqueue_ldpc_dec_ops(tp->dev_id, queue_id, &ops_enq[enq], num_to_enq);
    deq += rte_bbdev_dequeue_ldpc_dec_ops(tp->dev_id, queue_id, &ops_deq[deq], enq - deq);
  }
  /* dequeue the remaining */
  while (deq < enq) {
    deq += rte_bbdev_dequeue_ldpc_dec_ops(tp->dev_id, queue_id, &ops_deq[deq], enq - deq);
    time_out++;
    DevAssert(time_out <= TIME_OUT_POLL);
  }

  // Stop timer
  // We report timing only once in (0,0) since the timers are merged at the end
  stop_meas(&nrLDPC_slot_decoding_parameters->TBs[0].ts_ldpc_decode);

  if (deq == enq) {
    ret = retrieve_ldpc_dec_op(ops_deq, nrLDPC_slot_decoding_parameters);
    AssertFatal(ret == 0, "LDPC offload decoder failed!");
    tp->iter_count = 0;
    /* get the max of iter_count for all dequeued ops */
    int j = 0;
    for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
      nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
      for (int i = 0; i < p->C; ++i) {
        bool *status = &p->decodeSuccess[i];

        if (tb_mode()) {
          /* One op for the whole TB; crc_stat bit i is code block i's CRC
           * result (set = OK), and bit c is the TB-level CRC. */
          struct rte_bbdev_dec_op *dop = ops_deq[h];
          tp->iter_count = RTE_MAX(dop->ldpc_dec.iter_count, tp->iter_count);
          *status = (dop->crc_stat[i >> 3] >> (i & 7)) & 1;
          if (*status)
            *p->processedSegments = *p->processedSegments + 1;
          continue;
        }
        tp->iter_count = RTE_MAX(ops_enq[j]->ldpc_dec.iter_count, tp->iter_count);

        // Check if CRC is available otherwise rely on ops_enq[j]->status to detect decoding success
        // CRC is NOT available if the CRC type is 24_B which is when C is greater than 1
        if (p->C > 1) {
#ifdef LDPC_AAL_HAVE_LA12XX
          if (dev_is_la12xx(&active_dev.info)) {
            /* The LA12xx PMD only folds the hardware CRC status into
             * op->status when neither HARQ-combine flag is set (see the
             * op_flags test in its dequeue path), and we unconditionally
             * request HQ_COMBINE_OUT_ENABLE above. op->status is therefore
             * permanently 0 and every code block would be reported as decoded
             * regardless of the data -- pure noise then reads back as a
             * perfect decode with BLER 0.
             *
             * Use the per-CB CRC status bitmap the PMD populates
             * unconditionally instead; a set bit means CRC OK. Both paths we
             * enqueue (CB mode, and the c==1 TB-mode special case) make the
             * PMD select tb_crc = 0, so the bit for this op is bit 0. Read it
             * from the dequeued op, which is what retrieve_ldpc_dec_op() used
             * to fill p->c.
             */
            /* The PMD skips a descriptor whose hard_output.length is 0
             * *before* it copies crc_stat in, so in that case crc_stat still
             * holds whatever was in the mempool. Treat "no output" as a
             * failure rather than reading stale memory. */
            *status = ops_deq[j]->ldpc_dec.hard_output.length != 0
                      && (ops_deq[j]->crc_stat[0] & 1) != 0;
          } else {
            *status = (ops_enq[j]->status == 0);
          }
#else
          *status = (ops_enq[j]->status == 0);
#endif
        } else {
          uint8_t *decoded_bytes = p->c;
          uint8_t crc_type = crcType(p->C, p->A);
          uint32_t len_with_crc = lenWithCrc(p->C, p->A);
          *status = check_crc(decoded_bytes, len_with_crc, crc_type);
        }

        if (*status) {
          *p->processedSegments = *p->processedSegments + 1;
        }
        ++j;
      }
    }
  }

  rte_bbdev_dec_op_free_bulk(ops_enq, num_segments);
  // Return the worst decoding number of iterations for all segments
  return tp->iter_count;
}

// based on DPDK BBDEV throughput_pmd_lcore_ldpc_enc
static int pmd_lcore_ldpc_enc(void *arg)
{
  struct thread_params *tp = arg;
  nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters = tp->nrLDPC_slot_encoding_parameters;
  int time_out = 0;
  const uint16_t queue_id = tp->queue_id;
  const uint16_t num_segments = nb_segments_encoding(nrLDPC_slot_encoding_parameters);
  struct rte_bbdev_enc_op *ops_enq[num_segments];
  struct rte_bbdev_enc_op *ops_deq[num_segments];
  struct data_buffers *bufs = tp->data_buffers;

  AssertFatal((num_segments < MAX_BURST), "BURST_SIZE should be <= %u", MAX_BURST);

  while (rte_atomic16_read(&tp->op_params->sync) == SYNC_WAIT)
    rte_pause();
  enc_t[2] = enc_now_us();

  int ret = rte_bbdev_enc_op_alloc_bulk(tp->bbdev_op_pool, ops_enq, num_segments);
  AssertFatal(ret == 0, "Allocation failed for %d ops", num_segments);
  enc_t[3] = enc_now_us();
  set_ldpc_enc_op(ops_enq, bufs->inputs, bufs->hard_outputs, nrLDPC_slot_encoding_parameters);
  enc_t[4] = enc_now_us();

  /* The offload round trip, reported through the one encode counter that still
   * exists in nrLDPC_coding_interface.h. Upstream deleted tprep / tparity /
   * toutput (and ts_interleave / ts_rate_match / ts_output), which is what the
   * old "LDPC parity generation time" line came from, so without this the
   * accelerator time is invisible in -P output. segments[0] of TBs[0] is the
   * slot's reporting slot: nr_dlsch_coding.c merges every segment's counter
   * into gNB->dlsch_ldpc_encode_stats, printed as "LDPC encoding time". */
  start_meas(&nrLDPC_slot_encoding_parameters->TBs[0].segments[0].ts_ldpc_encode);

  uint16_t enq = 0, deq = 0;
  while (enq < num_segments) {
    uint16_t num_to_enq = num_segments - enq;
    /* Drop any op_flag the device does not advertise (see enc/dec_cap_flags). */
    for (uint16_t k = 0; k < num_to_enq; k++)
      ops_enq[enq + k]->ldpc_enc.op_flags &= active_dev.enc_cap_flags;
    enq += rte_bbdev_enqueue_ldpc_enc_ops(tp->dev_id, queue_id, &ops_enq[enq], num_to_enq);
    deq += rte_bbdev_dequeue_ldpc_enc_ops(tp->dev_id, queue_id, &ops_deq[deq], enq - deq);
  }
  /* dequeue the remaining */
  while (deq < enq) {
    deq += rte_bbdev_dequeue_ldpc_enc_ops(tp->dev_id, queue_id, &ops_deq[deq], enq - deq);
    time_out++;
    DevAssert(time_out <= TIME_OUT_POLL);
  }
  stop_meas(&nrLDPC_slot_encoding_parameters->TBs[0].segments[0].ts_ldpc_encode);
  ret = retrieve_ldpc_enc_op(ops_deq, nrLDPC_slot_encoding_parameters);
  AssertFatal(ret == 0, "Failed to retrieve LDPC encoding op!");
  rte_bbdev_enc_op_free_bulk(ops_enq, num_segments);
  return ret;
}

// based on DPDK BBDEV throughput_pmd_lcore_dec
int start_pmd_dec(struct active_device *ad,
                  struct test_op_params *op_params,
                  struct data_buffers *data_buffers,
                  nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  unsigned int lcore_id, used_cores = 0;
  /* Set number of lcores */
  int num_lcores = (ad->nb_queues < (op_params->num_lcores)) ? ad->nb_queues : op_params->num_lcores;
  /* Allocate memory for thread parameters structure */
  struct thread_params *t_params = t_params_persist(0, num_lcores);
  AssertFatal(t_params != 0,
              "Failed to alloc %zuB for t_params",
              RTE_ALIGN(sizeof(struct thread_params) * num_lcores, RTE_CACHE_LINE_SIZE));
  rte_atomic16_set(&op_params->sync, SYNC_WAIT);
  /* Master core is set at first entry */
  t_params[0].dev_id = ad->dev_id;
  t_params[0].lcore_id = rte_lcore_id();
  t_params[0].op_params = op_params;
  t_params[0].data_buffers = data_buffers;
  t_params[0].bbdev_op_pool = ad->bbdev_dec_op_pool;
  t_params[0].queue_id = ad->dec_queue;
  t_params[0].iter_count = 0;
  t_params[0].nrLDPC_slot_decoding_parameters = nrLDPC_slot_decoding_parameters;
  used_cores++;
  // For now, we never enter here, we don't use the DPDK thread pool
  RTE_LCORE_FOREACH_WORKER(lcore_id)
  {
    if (used_cores >= num_lcores)
      break;
    t_params[used_cores].dev_id = ad->dev_id;
    t_params[used_cores].lcore_id = lcore_id;
    t_params[used_cores].op_params = op_params;
    t_params[used_cores].data_buffers = data_buffers;
    t_params[used_cores].bbdev_op_pool = ad->bbdev_dec_op_pool;
    t_params[used_cores].queue_id = ad->queue_ids[used_cores];
    t_params[used_cores].iter_count = 0;
    t_params[used_cores].nrLDPC_slot_decoding_parameters = nrLDPC_slot_decoding_parameters;
    rte_eal_remote_launch(pmd_lcore_ldpc_dec, &t_params[used_cores++], lcore_id);
  }
  rte_atomic16_set(&op_params->sync, SYNC_START);
  int ret = pmd_lcore_ldpc_dec(&t_params[0]);
  /* Master core is always used */
  // for (used_cores = 1; used_cores < num_lcores; used_cores++)
  //	ret |= rte_eal_wait_lcore(t_params[used_cores].lcore_id);
  return ret;
}

// based on DPDK BBDEV throughput_pmd_lcore_enc
int32_t start_pmd_enc(struct active_device *ad,
                      struct test_op_params *op_params,
                      struct data_buffers *data_buffers,
                      nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters)
{
  unsigned int lcore_id, used_cores = 0;
  uint16_t num_lcores;
  int ret;
  num_lcores = (ad->nb_queues < (op_params->num_lcores)) ? ad->nb_queues : op_params->num_lcores;
  struct thread_params *t_params = t_params_persist(1, num_lcores);
  rte_atomic16_set(&op_params->sync, SYNC_WAIT);
  t_params[0].dev_id = ad->dev_id;
  t_params[0].lcore_id = rte_lcore_id() + 1;
  t_params[0].op_params = op_params;
  t_params[0].data_buffers = data_buffers;
  t_params[0].bbdev_op_pool = ad->bbdev_enc_op_pool;
  t_params[0].queue_id = ad->enc_queue;
  t_params[0].iter_count = 0;
  t_params[0].nrLDPC_slot_encoding_parameters = nrLDPC_slot_encoding_parameters;
  used_cores++;
  // For now, we never enter here, we don't use the DPDK thread pool
  RTE_LCORE_FOREACH_WORKER(lcore_id)
  {
    if (used_cores >= num_lcores)
      break;
    t_params[used_cores].dev_id = ad->dev_id;
    t_params[used_cores].lcore_id = lcore_id;
    t_params[used_cores].op_params = op_params;
    t_params[used_cores].data_buffers = data_buffers;
    t_params[used_cores].bbdev_op_pool = ad->bbdev_enc_op_pool;
    t_params[used_cores].queue_id = ad->queue_ids[1];
    t_params[used_cores].iter_count = 0;
    t_params[used_cores].nrLDPC_slot_encoding_parameters = nrLDPC_slot_encoding_parameters;
    rte_eal_remote_launch(pmd_lcore_ldpc_enc, &t_params[used_cores++], lcore_id);
  }
  rte_atomic16_set(&op_params->sync, SYNC_START);
  ret = pmd_lcore_ldpc_enc(&t_params[0]);
  return ret;
}

struct test_op_params *op_params = NULL;

/* True when \p input is not a PCI BDF, i.e. it names a DPDK virtual device
 * (vdev) such as "bbdev_la12xx" or "bbdev_la12xx,modem=0".
 *
 * The NXP LA12xx baseband is exposed as a vdev, not as a PCI device: the
 * LA12xx sits behind a PCI endpoint owned by the yami kernel driver, and the
 * bbdev PMD is instantiated on top of that driver rather than bound to the
 * PCI function itself.
 */
static bool dpdk_dev_is_vdev(const char *input)
{
  regex_t re;
  /* PCI BDF, with or without the leading domain */
  const char *pattern_bdf = "^([0-9a-fA-F]{4}:)?[0-9a-fA-F]{2}:[0-9a-fA-F]{2}\\.[0-7]$";
  regcomp(&re, pattern_bdf, REG_EXTENDED);
  int reti = regexec(&re, input, 0, NULL, 0);
  regfree(&re);
  return reti != 0;
}

/* Copy the bare device name (the part before any vdev argument list) of
 * \p input into \p output. rte_bbdev_info_get() reports a vdev without its
 * arguments, so "bbdev_la12xx,modem=0" must be matched as "bbdev_la12xx".
 */
static void dpdk_dev_basename(const char *input, char *output, size_t out_len)
{
  size_t n = strcspn(input, ",");
  if (n >= out_len)
    n = out_len - 1;
  memcpy(output, input, n);
  output[n] = '\0';
}

static int normalize_dpdk_dev(const char *input, char *output, size_t out_len)
{
  regex_t regex_full, regex_short;
  int reti;

  /* A vdev name is passed through untouched; only PCI addresses get
   * normalised to the full 0000:bb:dd.f form.
   */
  if (dpdk_dev_is_vdev(input)) {
    strncpy(output, input, out_len - 1);
    output[out_len - 1] = '\0';
    return 0;
  }

  // patterns
  const char *pattern_full = "^[0]{4}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}\\.[0-7]$";
  const char *pattern_short = "^[0-9a-fA-F]{2}:[0-9a-fA-F]{2}\\.[0-7]$";
  regcomp(&regex_full, pattern_full, REG_EXTENDED);
  regcomp(&regex_short, pattern_short, REG_EXTENDED);

  // check full format
  reti = regexec(&regex_full, input, 0, NULL, 0);
  if (reti == 0) {
    // already in full format
    strncpy(output, input, out_len - 1);
    output[out_len - 1] = '\0';
  } else {
    // check short format
    reti = regexec(&regex_short, input, 0, NULL, 0);
    if (reti == 0) {
      // convert to full format
      snprintf(output, out_len, "0000:%s", input);
    } else {
      // invalid format
      regfree(&regex_full);
      regfree(&regex_short);
      return -1;
    }
  }
  regfree(&regex_full);
  regfree(&regex_short);
  return 0;
}

// OAI CODE
int32_t nrLDPC_coding_init(int max_num_pxsch)
{
  pthread_mutex_init(&encode_mutex, NULL);
  pthread_mutex_init(&decode_mutex, NULL);

  int ret;
  int dev_id = -1;

  char *dpdk_dev = NULL; // PCI address of the card
  char *dpdk_core_list = NULL; // cores used by DPDK for bbdev
  char *dpdk_file_prefix = NULL;
  char *vfio_vf_token = NULL; // vfio token for the bbdev card
  uint32_t num_harq_codeblock = 0; // size of the HARQ buffer in terms of the number of 32K blocks
  uint32_t is_t2 = 0;
  paramdef_t LoaderParams[] = {
      {"dpdk_dev", NULL, 0, .strptr = &dpdk_dev, .defstrval = NULL, TYPE_STRING, 0, NULL},
      {"dpdk_core_list", NULL, 0, .strptr = &dpdk_core_list, .defstrval = NULL, TYPE_STRING, 0, NULL},
      {"dpdk_file_prefix", NULL, 0, .strptr = &dpdk_file_prefix, .defstrval = "b6", TYPE_STRING, 0, NULL},
      {"vfio_vf_token", NULL, 0, .strptr = &vfio_vf_token, .defstrval = NULL, TYPE_STRING, 0, NULL},
      {"num_harq_codeblock", NULL, 0, .uptr = &num_harq_codeblock, .defintval = 512, TYPE_UINT32, 0, NULL},
      {"is_t2", NULL, 0, .uptr = &is_t2, .defintval = 0, TYPE_UINT8, 0, NULL},
  };
  config_get(config_get_if(), LoaderParams, sizeofArray(LoaderParams), "nrLDPC_coding_aal");
  if (dpdk_dev == NULL)
    LOG_E(NR_PHY,
          "could not find mandatory --nrLDPC_coding_aal.dpdk_dev. If you used --nrLDPC_coding_t2.*, please rename all options to "
          "--nrLDPC_coding_aal.*\n");
  AssertFatal(dpdk_dev != NULL, "nrLDPC_coding_aal.dpdk_dev was not provided");

  char dpdk_dev_full[32];
  if (normalize_dpdk_dev(dpdk_dev, dpdk_dev_full, sizeof(dpdk_dev_full)) != 0) {
    LOG_E(NR_PHY, "invalid DPDK device format: %s\n", dpdk_dev);
    return -1;
  }

  // Detect if EAL was initialized by probing the device
  LOG_I(NR_PHY, "Probing DPDK device %s to know if EAL is initialized. This may generate EAL error messages\n", dpdk_dev_full);
  /* A vdev does not exist until EAL creates it from --vdev, so it cannot be
   * probed beforehand the way a PCI device can. For a vdev we therefore go
   * straight to rte_eal_init(); if EAL is already up it returns < 0 and we
   * fall through to looking the device up among the bbdevs already present.
   */
  bool dev_is_vdev = dpdk_dev_is_vdev(dpdk_dev_full);
  bool eal_ready = false;

  if (!dev_is_vdev)
    eal_ready = (rte_dev_probe(dpdk_dev_full) == 0);

  if (eal_ready) {
    // EAL was already initialized
    LOG_I(NR_PHY, "Probing DPDK device %s succeeded, skipping EAL initialization\n", dpdk_dev_full);
  } else {
    if (!dev_is_vdev)
      LOG_I(NR_PHY, "Probing DPDK device %s failed, initializing EAL before continuing\n", dpdk_dev_full);
    else
      LOG_I(NR_PHY, "DPDK device %s is a vdev, initializing EAL with --vdev\n", dpdk_dev_full);
    // EAL was not initialized yet
    // We initialize EAL
    AssertFatal(dpdk_core_list != NULL, "nrLDPC_coding_aal.dpdk_core_list was not provided");

    /* PCI devices are allow-listed with -a <bdf> (two tokens); a vdev is
     * created with --vdev=<name>[,<args>] (a single token). Build argv
     * incrementally so the two forms cannot desynchronise argc.
     */
    char vdev_arg[64];
    char *argv[12];
    int argc = 0;
    argv[argc++] = "bbdev";
    argv[argc++] = "-l";
    argv[argc++] = dpdk_core_list;
    if (dev_is_vdev) {
      snprintf(vdev_arg, sizeof(vdev_arg), "--vdev=%s", dpdk_dev_full);
      argv[argc++] = vdev_arg;
    } else {
      argv[argc++] = "-a";
      argv[argc++] = dpdk_dev_full;
    }
    argv[argc++] = "--file-prefix";
    argv[argc++] = dpdk_file_prefix;
    /* We are a single, self-contained DPDK process: nothing ever attaches as a
     * secondary. Leaving the multi-process channel enabled is actively harmful
     * here, because every run uses the same --file-prefix and so shares
     * /var/run/dpdk/<prefix>/mp_socket: a lingering peer from a previous run
     * makes EAL's memory subsystem do an mp handshake on memseg alloc/free and
     * block until it times out. Sampled kernel stacks showed threads parked in
     * __skb_wait_for_more_packets / unix_dgram_recvmsg, costing a repeatable
     * ~66.5 ms in mbuf alloc and another ~66.5 ms in mbuf free -- ~133 ms per
     * slot, intermittently, against ~365 us for the actual FECA work.
     * --no-shconf drops the runtime directory and that socket. Deliberately not
     * --in-memory, which also implies --huge-unlink and would remove the
     * hugepage backing files the modem's DMA mapping relies on.
     *
     * Opt-in (LDPC_AAL_NO_SHCONF=1) rather than default: with the socket gone
     * the same trial-3 memory event stops stalling and instead SIGSEGVs, so the
     * mp handshake is only where the time is *spent* -- the real trigger is
     * still an unidentified per-slot memory event. Kept as a switch because it
     * is the cleanest way to reproduce that trigger without the timeout hiding
     * it.
     */
    if (getenv("LDPC_AAL_NO_SHCONF"))
      argv[argc++] = "--no-shconf";
    if (vfio_vf_token != NULL) {
      argv[argc++] = "--vfio-vf-token";
      argv[argc++] = vfio_vf_token;
    }
    argv[argc] = NULL;
    DevAssert(argc < (int)(sizeof(argv) / sizeof(argv[0])));
    ret = rte_eal_init(argc, argv);
    if (ret < 0) {
      /* For a vdev this may simply mean EAL is already running (e.g. another
       * OAI component initialized it); the device lookup below decides.
       */
      if (!dev_is_vdev) {
        LOG_E(NR_PHY, "EAL initialization failed\n");
        return (-1);
      }
      LOG_W(NR_PHY, "rte_eal_init() failed for vdev %s; assuming EAL is already initialized\n", dpdk_dev_full);
    }
  }
  uint16_t nb_bbdevs = rte_bbdev_count();
  AssertFatal(nb_bbdevs > 0, "no bbdev found");

  // find the baseband device that matches the dpdk_dev specified in the configurations
  /* rte_bbdev_info_get() reports a vdev without its argument list, so match
   * on the bare device name ("bbdev_la12xx" for "bbdev_la12xx,modem=0").
   */
  char dpdk_dev_name[32];
  dpdk_dev_basename(dpdk_dev_full, dpdk_dev_name, sizeof(dpdk_dev_name));

  struct rte_bbdev_info info;
  LOG_I(NR_PHY, "detected %u bbdev.\n", nb_bbdevs);
  for (uint16_t device_id = 0; device_id < nb_bbdevs; device_id++) {
    rte_bbdev_info_get(device_id, &info);
    // check if info matches the dpdk_dev that we are looking for
    if (strcmp(info.dev_name, dpdk_dev_name) == 0) {
      LOG_I(NR_PHY, "bbdev %s found.\n", info.dev_name);
      dev_id = device_id;
      break;
    }
  }
  AssertFatal(dev_id != -1, "bbdev %s not found.", dpdk_dev_name);

  AssertFatal(add_dev(dev_id, is_t2, num_harq_codeblock) == 0, "Failed to setup bbdev");
  AssertFatal(rte_bbdev_stats_reset(dev_id) == 0, "Failed to reset stats of bbdev %u", dev_id);
  AssertFatal(rte_bbdev_start(dev_id) == 0, "Failed to start bbdev %u", dev_id);

  // the previous calls have populated this global variable (beurk)
  //  One more global to remove, not thread safe global op_params
  op_params = rte_zmalloc(NULL, sizeof(struct test_op_params), RTE_CACHE_LINE_SIZE);
  AssertFatal(op_params != NULL,
              "Failed to alloc %zuB for op_params",
              RTE_ALIGN(sizeof(struct test_op_params), RTE_CACHE_LINE_SIZE));

  int socket_id = GET_SOCKET(info.socket_id);
  int out_max_sz = 8448; // max code block size (for BG1), 22 * 384
  int in_max_sz = LDPC_MAX_CB_SIZE; // max number of encoded bits (for BG2 and MCS0)
  int num_queues = 1;
  int f_ret = create_mempools(&active_dev, socket_id, num_queues, out_max_sz, in_max_sz);

#ifdef LDPC_AAL_HAVE_LA12XX
  if (dev_is_la12xx(&active_dev.info)) {
    /* The modem reaches host memory only through the PCIe ATU window that the
     * yami driver programs. DPDK mempools live in ordinary hugepages outside
     * that window, so FECA's DMA engine faults on them -- visible on the modem
     * console as "DMA error capture DECCD*R ..." while the host side simply
     * spins in the dequeue loop until its timeout assertion. Map the hugepage
     * region backing each pool over PCI first.
     */
    struct rte_mempool *pools[] = {active_dev.in_mbuf_pool,
                                   active_dev.hard_out_mbuf_pool,
                                   active_dev.harq_in_mbuf_pool,
                                   active_dev.harq_out_mbuf_pool};
    for (unsigned int i = 0; i < sizeof(pools) / sizeof(pools[0]); i++) {
      if (pools[i] == NULL)
        continue;
      struct rte_mbuf *m = rte_pktmbuf_alloc(pools[i]);
      if (m == NULL)
        continue;
      int msz = rte_pmd_la12xx_map_hugepage_addr(active_dev.dev_id, m->buf_addr);
      rte_pktmbuf_free(m);
      AssertFatal(msz >= 0, "rte_pmd_la12xx_map_hugepage_addr() failed (%d) for pool %u\n", msz, i);
      LOG_I(NR_PHY, "mapped hugepage region for pool %u over PCI (%d bytes)\n", i, msz);
    }
  }
#endif
  if (f_ret != 0) {
    printf("Couldn't create mempools");
    return -1;
  }

  // initialize persistent data structure to keep track of HARQ-related information
  init_op_data_objs_harq(active_dev.harq_buffers, active_dev.harq_in_mbuf_pool);

  op_params->num_lcores = 1;
  return 0;
}

int32_t nrLDPC_coding_shutdown()
{
  int dev_id = 0;
  struct rte_bbdev_stats stats;
  free(active_dev.harq_buffers);
  free_mempools(&active_dev);
  rte_free(op_params);
  rte_bbdev_stats_get(dev_id, &stats);
  rte_bbdev_stop(dev_id);
  rte_bbdev_close(dev_id);
  memset(&active_dev, 0, sizeof(active_dev));
  return 0;
}

static void
llr_scaling(int16_t *llr, int llr_len, uint8_t *llr_scaled, int8_t llr_size, int8_t llr_decimal, int8_t nb_layers, int8_t Qm)
{
  const int16_t llr_max = (1 << (llr_size - 1)) - 1;
  const int16_t llr_min = -llr_max;

  // Step 1: Find the max absolute LLR
  int16_t max_abs = 1; // prevent divide-by-zero
  // SCALAR IMPLEMENTATION
  // for (int i = 0; i < llr_len; i++) {
  //     int16_t abs_val = abs(llr[i]);
  //     if (abs_val > max_abs) max_abs = abs_val;
  // }
  // VECTORIZED IMPLEMENTATION
  simde__m128i max_vec = simde_mm_set1_epi16(1);
  for (int i = 0; i < llr_len; i += 8) {
    simde__m128i llr_vec = simde_mm_loadu_si128((simde__m128i *)&llr[i]);
    simde__m128i abs_vec = simde_mm_abs_epi16(llr_vec);
    max_vec = simde_mm_max_epi16(max_vec, abs_vec);
  }
  // reduce max_vec to single max_abs
  int16_t temp[8];
  simde_mm_storeu_si128((simde__m128i *)temp, max_vec);
  for (int i = 0; i < 8; i++) {
    if (temp[i] > max_abs)
      max_abs = temp[i];
  }

  // Step 2: Compute dynamic scale factor
  float fixed_point_range = (float)llr_max / (1 << llr_decimal);
  float scale = fixed_point_range / (float)max_abs;
  if (max_abs < fixed_point_range) {
    scale = 1.0f;
  }

  // Step 3: Scale and saturate
  // SCALAR IMPLEMENTATION
  // for (int i = 0; i < llr_len; i++) {
  //     float scaled = (float)llr[i] * scale; // map into fixed-point domain
  //     scaled = (int8_t)roundf(scaled * (1 << llr_decimal));
  //     // Clamp to [-128, 127]
  //     if (scaled > llr_max) llr_scaled[i] = llr_max;
  //     else if (scaled < llr_min) llr_scaled[i] = llr_min;
  // }
  // VECTORIZED IMPLEMENTATION
  simde__m128 scale_vec = simde_mm_set1_ps(scale);
  simde__m128i llr_max_vec = simde_mm_set1_epi16(llr_max);
  simde__m128i llr_min_vec = simde_mm_set1_epi16(llr_min);
  simde__m128i decimal_shift = simde_mm_set1_epi16(1 << llr_decimal);
  for (int i = 0; i < llr_len; i += 8) {
    // load LLR values
    simde__m128i llr_vec = simde_mm_loadu_si128((simde__m128i *)&llr[i]);

    // convert to float for scaling
    simde__m128i llr_lo = simde_mm_cvtepi16_epi32(llr_vec);
    simde__m128i llr_hi = simde_mm_cvtepi16_epi32(simde_mm_srli_si128(llr_vec, 8));
    simde__m128 float_lo = simde_mm_cvtepi32_ps(llr_lo);
    simde__m128 float_hi = simde_mm_cvtepi32_ps(llr_hi);

    // scale
    float_lo = simde_mm_mul_ps(float_lo, scale_vec);
    float_hi = simde_mm_mul_ps(float_hi, scale_vec);

    // convert back to int16 with saturation
    llr_lo = simde_mm_cvtps_epi32(float_lo);
    llr_hi = simde_mm_cvtps_epi32(float_hi);
    simde__m128i scaled_vec = simde_mm_packs_epi32(llr_lo, llr_hi);

    scaled_vec = simde_mm_mullo_epi16(scaled_vec, decimal_shift);

    // clamp to [llr_min, llr_max]
    scaled_vec = simde_mm_min_epi16(scaled_vec, llr_max_vec);
    scaled_vec = simde_mm_max_epi16(scaled_vec, llr_min_vec);

    // Pack to int8 and store
    simde__m128i result = simde_mm_packs_epi16(scaled_vec, simde_mm_setzero_si128());
    simde_mm_storeu_si128((simde__m128i *)&llr_scaled[i], result);
  }
}

int32_t nrLDPC_coding_decoder(nrLDPC_slot_decoding_parameters_t *nrLDPC_slot_decoding_parameters)
{
  pthread_mutex_lock(&decode_mutex);

  int ret;
  int socket_id = active_dev.info.socket_id;

  const uint16_t num_segments = nb_segments_decoding(nrLDPC_slot_decoding_parameters);

  /* It is not unlikely that l_ol becomes big enough to overflow the stack
   * If you observe this behavior then move it to the heap
   * Then you would better do a persistent allocation to limit the overhead
   */
    uint8_t l_ol[nb_cbs_decoding(nrLDPC_slot_decoding_parameters) * LDPC_MAX_CB_SIZE] __attribute__((aligned(16)));

  // fill_queue_buffers -> init_op_data_objs
  struct rte_mempool *mbuf_pools[DATA_NUM_TYPES] = {active_dev.in_mbuf_pool,
                                                    active_dev.hard_out_mbuf_pool,
                                                    active_dev.harq_out_mbuf_pool};
  struct data_buffers data_buffers;
  struct rte_bbdev_op_data **queue_ops[DATA_NUM_TYPES] = {&data_buffers.inputs,
                                                          &data_buffers.hard_outputs,
                                                          &data_buffers.harq_outputs};
  /* Look the decoder capability up by type: capabilities[] is a terminated
   * list, not an array indexed by rte_bbdev_op_type (see find_bbdev_cap()). */
  const struct rte_bbdev_op_cap *dec_cap = find_bbdev_cap(&active_dev.info, RTE_BBDEV_OP_LDPC_DEC);
  int8_t llr_size = dec_cap ? dec_cap->cap.ldpc_dec.llr_size : 0;
  int8_t llr_decimal = dec_cap ? dec_cap->cap.ldpc_dec.llr_decimals : 0;
  /* A device may leave these at 0 (the LA12xx PMD does). llr_scaling() then
   * evaluates (1 << (llr_size - 1)) == (1 << -1), which is undefined behaviour
   * and yields a garbage saturation bound. Fall back to the common S8.0 LLR
   * format rather than computing with a bad value. */
  if (llr_size <= 0) {
    LOG_W(NR_PHY, "bbdev reports llr_size=%d; assuming 8-bit two's-complement LLRs with %d decimals\n", llr_size, 0);
    llr_size = 8;
    llr_decimal = 0;
  }
  int offset = 0;
  for (int h = 0; h < nrLDPC_slot_decoding_parameters->nb_TBs; ++h) {
    nrLDPC_TB_decoding_parameters_t *p = &nrLDPC_slot_decoding_parameters->TBs[h];
    size_t data_off = 0;
    /* Both LLR paths write with 16-byte SIMD stores, so the destination has to
     * stay 16-byte aligned. E is not always a multiple of 16 (e.g. 9414 at 273
     * PRB MCS 27), so code blocks CANNOT be converted straight into a packed
     * per-TB buffer: the second one onwards starts unaligned, and llr_scaling()
     * then faults or crawls (measured 127 ms/slot and a SIGSEGV). Keep
     * converting into l_ol's aligned slots; TB mode gathers them afterwards.
     */
    for (int r = 0; r < p->C; r++) {
      uint32_t E = r < p->first_rE2 ? p->E : p->E2;
      uint8_t *dst = &l_ol[offset];

      if (active_dev.saturate_llrs) {
        // Saturate int16 -> int8 rather than rescaling (T2 and LA12xx).
        uint16_t z_ol[LDPC_MAX_CB_SIZE] __attribute__((aligned(16)));
        memcpy(z_ol, p->llr + data_off, E * sizeof(uint16_t));
        simde__m128i *pv_ol128 = (simde__m128i *)z_ol;
        simde__m128i *pl_ol128 = (simde__m128i *)dst;
        for (int i = 0, j = 0; j < ((E + 15) >> 4); i += 2, j++) {
          pl_ol128[j] = simde_mm_packs_epi16(pv_ol128[i], pv_ol128[i + 1]);
        }
      } else {
        llr_scaling(p->llr + data_off, E, dst, llr_size, llr_decimal, p->nb_layers, p->Qm);
      }
      offset += LDPC_MAX_CB_SIZE;
      data_off += E;
    }
  }

  for (enum op_data_type type = DATA_INPUT; type < DATA_NUM_TYPES; ++type) {
    *queue_ops[type] = persistent_op_data(1, type, num_segments, socket_id);
    ret = init_op_data_objs_dec(*queue_ops[type],
                                l_ol,
                                nrLDPC_slot_decoding_parameters,
                                mbuf_pools[type],
                                type,
                                active_dev.info.drv.min_alignment);
    AssertFatal(ret == 0, "Couldn't init rte_bbdev_op_data structs");
  }

  ret = start_pmd_dec(&active_dev, op_params, &data_buffers, nrLDPC_slot_decoding_parameters);
  if (ret < 0) {
    LOG_E(NR_PHY, "Couldn't start pmd dec\n");
  }

  for (enum op_data_type type = DATA_INPUT; type < DATA_NUM_TYPES; ++type) {
    /* TB mode holds no mbufs: the buffers are persistent direct memory. */
    if (!tb_mode())
      for (int segment = 0; segment < num_segments; ++segment)
        rte_pktmbuf_free((*queue_ops[type])[segment].data);
  }

  pthread_mutex_unlock(&decode_mutex);
  return 0;
}

int32_t nrLDPC_coding_encoder(nrLDPC_slot_encoding_parameters_t *nrLDPC_slot_encoding_parameters)
{
  pthread_mutex_lock(&encode_mutex);

  if (enc_dbg < 0)
    enc_dbg = (getenv("LDPC_AAL_DEBUG_ENCT") != NULL);
  enc_t[0] = enc_now_us();

  const uint16_t num_segments = nb_segments_encoding(nrLDPC_slot_encoding_parameters);

  int ret;
  int socket_id = active_dev.info.socket_id;

  // fill_queue_buffers -> init_op_data_objs
  struct rte_mempool *mbuf_pools[2] = {active_dev.in_mbuf_pool, active_dev.hard_out_mbuf_pool};
  struct data_buffers data_buffers;
  struct rte_bbdev_op_data **queue_ops[2] = {&data_buffers.inputs, &data_buffers.hard_outputs};

  for (enum op_data_type type = DATA_INPUT; type < 2; ++type) {
    *queue_ops[type] = persistent_op_data(0, type, num_segments, socket_id);
    ret = init_op_data_objs_enc(*queue_ops[type],
                                nrLDPC_slot_encoding_parameters,
                                mbuf_pools[type],
                                type,
                                active_dev.info.drv.min_alignment);
    AssertFatal(ret == 0, "Couldn't init rte_bbdev_op_data structs");
  }

  enc_t[1] = enc_now_us();
  ret = start_pmd_enc(&active_dev, op_params, &data_buffers, nrLDPC_slot_encoding_parameters);
  enc_t[5] = enc_now_us();

  if (!tb_enc_mode(nrLDPC_slot_encoding_parameters))
    for (enum op_data_type type = DATA_INPUT; type < 2; ++type)
      for (int segment = 0; segment < num_segments; ++segment)
        rte_pktmbuf_free((*queue_ops[type])[segment].data);
  enc_t[6] = enc_now_us();

  if (enc_dbg)
    printf("[pool] in=%u hard_out=%u harq_out=%u encop=%u decop=%u tb_allocs=%u\n",
           rte_mempool_avail_count(active_dev.in_mbuf_pool),
           rte_mempool_avail_count(active_dev.hard_out_mbuf_pool),
           rte_mempool_avail_count(active_dev.harq_out_mbuf_pool),
           rte_mempool_avail_count(active_dev.bbdev_enc_op_pool),
           rte_mempool_avail_count(active_dev.bbdev_dec_op_pool),
           tb_buf_allocs);
  if (enc_dbg)
    printf("[enc-t] C=%u bufinit=%lu  launch->sync=%lu  opalloc=%lu  setop=%lu  enq..retr=%lu  free=%lu  TOTAL=%lu\n",
           num_segments,
           (unsigned long)(enc_t[1] - enc_t[0]),
           (unsigned long)(enc_t[2] - enc_t[1]),
           (unsigned long)(enc_t[3] - enc_t[2]),
           (unsigned long)(enc_t[4] - enc_t[3]),
           (unsigned long)(enc_t[5] - enc_t[4]),
           (unsigned long)(enc_t[6] - enc_t[5]),
           (unsigned long)(enc_t[6] - enc_t[0]));

  pthread_mutex_unlock(&encode_mutex);
  return ret;
}
