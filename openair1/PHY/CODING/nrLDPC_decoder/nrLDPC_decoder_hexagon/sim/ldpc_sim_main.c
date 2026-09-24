// SPDX-License-Identifier: LicenseRef-CSSL-1.0
//
// ldpc_sim_main.c — Standalone Hexagon simulator profiling harness.
//
// Mirrors the phase breakdown of ldpc_scalar_core() on the device:
//   scatter  (1× per codeword)   — hvx_llr2llrProcBuf + hvx_llr2CnProcBuf_BG1
//   cnProc   (N_ITER× accum)     — cnProc_fused
//   bnProcPc (N_ITER× accum)     — hvx_bnProcPc
//   bnProc   (N_ITER× accum)     — bnProc_fused
//   cnProcPc (N_ITER× accum)     — cnProcPc_fused
//   gather   (1× per codeword)   — nrLDPC_llrRes2llrOut
//
// Reports both raw simulator cycles and estimated µs at TURBO clock
// (1459 MHz, IQ-9075 signed-PD) for direct comparison with on-device
// HAP_perf_get_time_us() measurements.
//
// Build:   cd sim/ && make
// Run:     make sim

#define LDPC_SIM_STANDALONE 1
#include "../src/ldpc_hexagon_imp.c"
#include <hexagon_sim_timer.h>

#define N_WARMUP    2       // un-timed warm-up iterations (prime caches)
#ifndef N_ITER
#define N_ITER      8       // timed iterations — matches device numMaxIter at SNR=3dB
#endif
#ifndef SIM_R
#define SIM_R       13      // rate code: 13=1/3, 23=2/3, 89=8/9 (BG1)
#endif

/* PMU isolation.  hexagon-sim --statsfile aggregates over the entire run, so the only
 * way to attribute packets/stalls to a single kernel is to make it the only thing in
 * the loop.  Build one binary per kernel and diff the statsfiles.
 *   0 = all phases (the default: relative cycle profile, matches the device order)
 *   1 = cnProc_fused only   2 = hvx_bnProcPc only
 *   3 = bnProc_fused only   4 = cnProcPc_fused only
 * Skipping a kernel freezes its neighbours' input, which does not change the cycle
 * count: the HVX kernels are branch-free over the data. */
#ifndef SIM_KERNEL
#define SIM_KERNEL  0
#endif

/* Lifting size. 384 is what the 273-PRB / MCS 20 case runs, but the cnProc Pass-2 fold has
 * a separate code path per Z/128, so 128 / 256 / 384 each need their FNV64 compared against
 * a -DLDPC_CNPROC_FOLD=0 build. */
#ifndef SIM_Z
#define SIM_Z       384
#endif

/* K' for the CRC termination test: 8448 is what a 273-PRB transport block uses. */
#ifndef SIM_KPRIME
#define SIM_KPRIME  8448
#endif
#define RUN_K(n) (SIM_KERNEL == 0 || SIM_KERNEL == (n))
/* 5 = the scatter (hvx_llr2llrProcBuf + hvx_llr2CnProcBuf_BG1), repeated N_ITER times
 * 6 = the CRC termination test (ldpc_llr2bits + ldpc_check_crc24), the `termtest` phase
 *     under the path production actually takes. SIM_KERNEL=4 is the OTHER termination
 *     test, cnProcPc's parity check, which use_crc means the device never runs. */
#define SIM_R_STR (SIM_R == 13 ? "1/3" : SIM_R == 23 ? "2/3" : SIM_R == 89 ? "8/9" : "?")
#define CLK_MHZ     1459.0  // TURBO clock on IQ-9075 (signed-PD)

// Device reference (IQ-9075, signed-PD TURBO, BG1 Z=384, 8 iters, SNR=3dB, QTIMER).
// These are for a codeword that runs the full 8 iterations.  The in-service profile
// (273 PRB / MCS 20) is roughly half of each, because the segment CRC terminates
// early -- do not compare these against that.  Its termination test is also the CRC,
// not cnProcPc; cnProcPc is profiled here purely as a kernel.
#define DEV_SCATTER_US    113
#define DEV_CNPROC_US     197
#define DEV_BNPROCPC_US    64
#define DEV_BNPROC_US     192
#define DEV_CNPROCPC_US    30
#define DEV_GATHER_US       8
#define DEV_TOTAL_US      604

// Weak LLRs that do not converge easily — ensures all N_ITER run.
static void fill_llr(int8_t *buf, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        buf[i] = (int8_t)((i & 3) ? 4 : -4);
}

int main(void)
{
    t_nrLDPC_dec_params dp;
    memset(&dp, 0, sizeof(dp));
    dp.BG = 1; dp.Z = SIM_Z; dp.R = SIM_R; dp.numMaxIter = N_WARMUP + N_ITER + 4;

    t_nrLDPC_lut lut;
    memset(&lut, 0, sizeof(lut));
    uint32_t numLLR = nrLDPC_init(&dp, &lut);

    /* SIM_AC=0 runs the kernels with no address cache (they compute inline), 1 with it.
     * The FNV64 must be identical either way -- that is what proves the cached edge order
     * matches the order bnProc_fused consumes. */
#ifndef SIM_AC
#define SIM_AC 1
#endif
    const ldpc_addr_cache_t *sim_ac = NULL;
#if SIM_AC
    ldpc_addr_cache_build(&g_addr_cache[0], &lut, 1, SIM_Z, (uint8_t)SIM_R);
    sim_ac = &g_addr_cache[0];
#endif
    if (!numLLR) { printf("nrLDPC_init failed\n"); return 1; }

    ldpc_worker_t ctx;   /* the working set moved here from ldpc_dsp_ctx_t with the worker pool */
    /* +128 of slack each: the kernels round the chunk count UP and let the last vector
     * run past Z, so a read can reach just beyond the last group. Matches the skel. */
    ctx.cn_msg     = calloc(NR_LDPC_SIZE_CN_PROC_BUF + 128, 1);
    ctx.bn_msg     = calloc(NR_LDPC_SIZE_BN_PROC_BUF + 128, 1);
    ctx.llrRes     = calloc(NR_LDPC_MAX_NUM_LLR + 128, 1);
    ctx.llrProcBuf = calloc(NR_LDPC_MAX_NUM_LLR + 128, 1);
    int8_t  *llr    = malloc(numLLR);
    uint8_t *llrout = malloc(numLLR);

    if (!ctx.cn_msg || !ctx.bn_msg || !ctx.llrRes || !ctx.llrProcBuf || !llr || !llrout) {
        printf("alloc failed\n"); return 1;
    }
    fill_llr(llr, numLLR);

    int8_t *cn_msg     = ctx.cn_msg;
    int8_t *bn_msg     = ctx.bn_msg;
    int8_t *llrRes     = ctx.llrRes;
    int8_t *llrProcBuf = ctx.llrProcBuf;

    /* ---- HVX copy / scatter / gather vs the OAI reference --------------------------
     * hvx_memcpy, hvx_llr2llrProcBuf and hvx_llrRes2llrOut live in ldpc_scalar_core, which
     * this harness never calls, so the FNV64 below cannot see them. Check them directly
     * against nrLDPC_mPass.h, which is what they replaced. */
    {
        int bad = 0;
        uint32_t n = numLLR;
        int8_t *a = calloc(NR_LDPC_MAX_NUM_LLR + 128, 1);
        int8_t *b = calloc(NR_LDPC_MAX_NUM_LLR + 128, 1);
        int8_t *src = calloc(NR_LDPC_MAX_NUM_LLR + 128, 1);
        for (uint32_t i = 0; i < NR_LDPC_MAX_NUM_LLR; i++)
            src[i] = (int8_t)((i * 37u + (i >> 5)) & 0xff);

        /* plain block copy, at a few awkward lengths as well as numLLR */
        static const uint32_t lens[] = {1, 127, 128, 129, 383, 384, 385};
        for (unsigned li = 0; li < sizeof lens / sizeof lens[0]; li++) {
            memset(a, 0x5a, NR_LDPC_MAX_NUM_LLR); memset(b, 0x5a, NR_LDPC_MAX_NUM_LLR);
            memcpy(a, src, lens[li]);
            hvx_memcpy(b, src, lens[li]);
            if (memcmp(a, b, NR_LDPC_MAX_NUM_LLR)) { printf("  hvx_memcpy DIFF at n=%lu\n",
                                                            (unsigned long)lens[li]); bad++; }
        }
        memset(a, 0x5a, NR_LDPC_MAX_NUM_LLR); memset(b, 0x5a, NR_LDPC_MAX_NUM_LLR);
        memcpy(a, src, n); hvx_memcpy(b, src, n);
        if (memcmp(a, b, NR_LDPC_MAX_NUM_LLR)) { printf("  hvx_memcpy DIFF at numLLR\n"); bad++; }

        /* scatter and gather */
        memset(a, 0, NR_LDPC_MAX_NUM_LLR); memset(b, 0, NR_LDPC_MAX_NUM_LLR);
        nrLDPC_llr2llrProcBuf(&lut, src, a, SIM_Z, 1);
        hvx_llr2llrProcBuf(&lut, src, b, SIM_Z, 1);
        if (memcmp(a, b, NR_LDPC_MAX_NUM_LLR)) { printf("  hvx_llr2llrProcBuf DIFF\n"); bad++; }

        memset(a, 0, NR_LDPC_MAX_NUM_LLR); memset(b, 0, NR_LDPC_MAX_NUM_LLR);
        nrLDPC_llrRes2llrOut(&lut, a, src, SIM_Z, 1);
        hvx_llrRes2llrOut(&lut, b, src, SIM_Z, 1);
        if (memcmp(a, b, NR_LDPC_MAX_NUM_LLR)) { printf("  hvx_llrRes2llrOut DIFF\n"); bad++; }

        /* The scatter to cn_msg. This harness used to time the REFERENCE here while the
         * device ran hvx_llr2CnProcBuf_BG1, so the kernel actually shipped had never been
         * exercised in the sim at all. */
        {
            int8_t *ca = calloc(NR_LDPC_SIZE_CN_PROC_BUF + 128, 1);
            int8_t *cb = calloc(NR_LDPC_SIZE_CN_PROC_BUF + 128, 1);
            if (ca && cb) {
                nrLDPC_llr2CnProcBuf_BG1(&lut, src, ca, SIM_Z);
                hvx_llr2CnProcBuf_BG1(&lut, src, cb, SIM_Z);
                if (memcmp(ca, cb, NR_LDPC_SIZE_CN_PROC_BUF)) {
                    printf("  hvx_llr2CnProcBuf_BG1 DIFF\n"); bad++;
                }
            }
            free(ca); free(cb);
        }

        printf("  HVX copy/scatter/gather vs reference (Z=%d): %s\n", SIM_Z,
               bad ? "FAIL" : "identical");
        free(a); free(b); free(src);
    }

    /* ---- CRC-24 slicing-by-4 vs the byte-at-a-time reference ----------------------
     * Exact equality is the whole requirement: a CRC that is merely usually right is a
     * decoder that merely usually terminates correctly. Random data, every bit length that
     * exercises the 4-byte main loop, the byte tail and the partial-byte residue. */
    {
        int bad = 0;
        ldpc_crc_tabs_init();
        uint8_t *buf = calloc(2048, 1);
        uint32_t st = 0x12345678u;
        for (int i = 0; i < 2048; i++) { st = st * 1664525u + 1013904223u; buf[i] = (uint8_t)(st >> 24); }
        for (int bitlen = 1; bitlen <= 1200; bitlen++) {
            for (int t = 0; t < 2; t++) {
                const uint32_t (*tab)[256] = t ? g_crc24b_tab : g_crc24a_tab;
                if (ldpc_crc24(buf, bitlen, tab) != ldpc_crc24_ref(buf, bitlen, tab)) {
                    if (bad < 4) printf("  CRC24 DIFF at bitlen=%d poly=%c\n", bitlen, t ? 'B' : 'A');
                    bad++;
                }
            }
        }
        /* and the production length */
        for (int t = 0; t < 2; t++) {
            const uint32_t (*tab)[256] = t ? g_crc24b_tab : g_crc24a_tab;
            if (ldpc_crc24(buf, 8448 - 24, tab) != ldpc_crc24_ref(buf, 8448 - 24, tab)) bad++;
        }
        printf("  CRC24 slicing-by-4 vs reference: %s (%d mismatches over 2400 lengths)\n",
               bad ? "FAIL" : "exact", bad);
        free(buf);
    }

    /* ---- cnProcPc ring test -------------------------------------------------------
     * Comparing the flag on the harness's own LLRs is nearly worthless: they are chosen
     * NOT to converge, so parity always fails and any implementation that just returned 1
     * would pass. This plants ONE sign bit at a time and requires the HVX version and the
     * scalar oracle to agree on every position, which is a direct test of the ring
     * indexing -- a wrong offset looks at the wrong byte and disagrees.
     *
     * Zero llrRes must give "pass"; that is the only way the pass path gets exercised. */
    {
        int bad = 0, saw_pass = 0, saw_fail = 0;
        memset(llrRes, 0, NR_LDPC_MAX_NUM_LLR);
        if (cnProcPc_fused(llrRes, &lut, SIM_Z, 1, sim_ac) != 0 ||
            cnProcPc_ref(llrRes, &lut, SIM_Z, 1) != 0) {
            printf("  cnProcPc: all-zero llrRes must pass, it does not\n");
            bad++;
        }
        /* Positions clustered on the boundaries where a ring bug lives: slot starts, the
         * 128-byte chunk edges, and the wrap itself. */
        for (uint32_t base = 0; base + SIM_Z <= NR_LDPC_MAX_NUM_LLR; base += 7u * SIM_Z) {
            static const int off[] = {0, 1, 126, 127, 128, 129, 254, 255, 256, 257};
            for (unsigned oi = 0; oi < sizeof off / sizeof off[0]; oi++) {
                int32_t o = off[oi];
                if (o >= SIM_Z) o = SIM_Z - 1 - (o % (SIM_Z ? SIM_Z : 1));
                if (o < 0) continue;
                uint32_t idx = base + (uint32_t)o;
                llrRes[idx] = -1;
                int32_t a = cnProcPc_fused(llrRes, &lut, SIM_Z, 1, sim_ac);
                int32_t b = cnProcPc_ref(llrRes, &lut, SIM_Z, 1);
                llrRes[idx] = 0;
                if (a != b) {
                    if (bad < 6) printf("  cnProcPc MISMATCH: llrRes[%lu]=-1 hvx=%d ref=%d\n",
                                        (unsigned long)idx, (int)a, (int)b);
                    bad++;
                }
                if (a) saw_fail++; else saw_pass++;
            }
        }
        printf("  cnProcPc ring test (Z=%d): %s  %d mismatches, %d fail / %d pass outcomes\n",
               SIM_Z, bad ? "FAIL" : "ok", bad, saw_fail, saw_pass);
        memset(llrRes, 0, NR_LDPC_MAX_NUM_LLR);
    }

    // Warm-up: prime caches, not timed (mirrors steady-state on device where
    // buffers are warm from previous codewords in the same slot).
    hvx_llr2llrProcBuf(&lut, llr, llrProcBuf, SIM_Z, 1);
    hvx_llr2CnProcBuf_BG1(&lut, llr, cn_msg, SIM_Z);   /* what the device runs */
    for (int w = 0; w < N_WARMUP; w++) {
        if (RUN_K(1)) cnProc_fused(cn_msg, bn_msg, &lut, SIM_Z, 1);
        if (RUN_K(2)) hvx_bnProcPc(&lut, bn_msg, llrProcBuf, llrRes, SIM_Z);
        if (RUN_K(3)) bnProc_fused(bn_msg, llrRes, cn_msg, &lut, SIM_Z, sim_ac);
        if (RUN_K(4)) cnProcPc_fused(llrRes, &lut, SIM_Z, 1, sim_ac);
    }
    nrLDPC_llrRes2llrOut(&lut, (int8_t *)llrout, llrRes, SIM_Z, 1);

    // --- Timed region — mirrors ldpc_scalar_core() structure ---
    uint64_t t0, t1;

    // scatter (1x per codeword). SIM_KERNEL=5 isolates it, and repeats it N_ITER times so
    // it dominates the run -- it is a once-per-codeword phase, too small to attribute
    // against init otherwise.
    memset(llrProcBuf, 0, NR_LDPC_MAX_NUM_LLR);
#if SIM_KERNEL == 5
    for (int r = 0; r < N_ITER; r++) {
        hvx_llr2llrProcBuf(&lut, llr, llrProcBuf, SIM_Z, 1);
        hvx_llr2CnProcBuf_BG1(&lut, llr, cn_msg, SIM_Z);
    }
#endif
    t0 = hexagon_sim_read_pcycles();
    hvx_llr2llrProcBuf(&lut, llr, llrProcBuf, SIM_Z, 1);
    hvx_llr2CnProcBuf_BG1(&lut, llr, cn_msg, SIM_Z);   /* what the device runs */
    t1 = hexagon_sim_read_pcycles();
    uint64_t cyc_scatter = t1 - t0;

#if SIM_KERNEL == 6
    /* termtest, CRC form. Kprime is the production value; llrout is the gather's output,
     * which is what the device hands to llr2bits. */
    {
        uint8_t *crc_bits = memalign(128, ((SIM_KPRIME + 7) / 8 + 127) & ~127u);
        volatile int sink = 0;
        ldpc_crc_tabs_init();
        for (int r = 0; r < N_ITER; r++) {
            ldpc_llr2bits(crc_bits, (const int8_t *)llrout, SIM_KPRIME);
            sink += ldpc_check_crc24(crc_bits, SIM_KPRIME, 0);
        }
        (void)sink;
        free(crc_bits);
    }
#endif

    // BP iterations (accumulated, as on device) — matches ldpc_scalar_core() order:
    // cnProcPc runs immediately after bnProcPc; bnProc_fused skipped on last iter.
    uint64_t cyc_cnProc = 0, cyc_bnProcPc = 0, cyc_bnProc = 0, cyc_cnProcPc = 0;
    int pc_bad = 0; unsigned pc_seen = 0;
    for (int it = 0; it < N_ITER; it++) {
        if (RUN_K(1)) {
            t0 = hexagon_sim_read_pcycles();
            cnProc_fused(cn_msg, bn_msg, &lut, SIM_Z, 1);
            t1 = hexagon_sim_read_pcycles();
            cyc_cnProc += t1 - t0;
        }

        if (RUN_K(2)) {
            t0 = hexagon_sim_read_pcycles();
            hvx_bnProcPc(&lut, bn_msg, llrProcBuf, llrRes, SIM_Z);
            t1 = hexagon_sim_read_pcycles();
            cyc_bnProcPc += t1 - t0;
        }

        if (RUN_K(4)) {
            t0 = hexagon_sim_read_pcycles();
            int32_t pc = cnProcPc_fused(llrRes, &lut, SIM_Z, 1, sim_ac);
            t1 = hexagon_sim_read_pcycles();
            cyc_cnProcPc += t1 - t0;
            /* cnProcPc writes no buffer, so the FNV64 below cannot see it. Check it
             * against the scalar oracle every iteration instead -- this is the only
             * thing standing between a wrong parity ring and silence. */
            int32_t pc_ref = cnProcPc_ref(llrRes, &lut, SIM_Z, 1);
            if (pc != pc_ref) {
                printf("  cnProcPc MISMATCH at iter %d: hvx=%d ref=%d (Z=%d)\n",
                       it, (int)pc, (int)pc_ref, SIM_Z);
                pc_bad++;
            }
            pc_seen |= (pc ? 2u : 1u);
        }

        if (RUN_K(3) && it < N_ITER - 1) {  // skip bnProc on last iter (matches device)
            t0 = hexagon_sim_read_pcycles();
            bnProc_fused(bn_msg, llrRes, cn_msg, &lut, SIM_Z, sim_ac);
            t1 = hexagon_sim_read_pcycles();
            cyc_bnProc += t1 - t0;
        }
    }

    // gather (1× per codeword) — volatile sink prevents DCE of llrout writes
    t0 = hexagon_sim_read_pcycles();
    nrLDPC_llrRes2llrOut(&lut, (int8_t *)llrout, llrRes, SIM_Z, 1);
    t1 = hexagon_sim_read_pcycles();
    uint64_t cyc_gather = t1 - t0;
    volatile uint8_t _sink = llrout[0]; (void)_sink;

    /* Bit-exactness check across builds: any restructuring of the kernels must leave
     * every buffer identical.  Compare this line between two builds, do not eyeball
     * the cycle counts and assume the maths held. */
    {
        uint64_t h = 1469598103934665603ULL;
        const struct { const void *p; uint32_t n; } regions[] = {
            { cn_msg, NR_LDPC_SIZE_CN_PROC_BUF }, { bn_msg, NR_LDPC_SIZE_BN_PROC_BUF },
            { llrRes, NR_LDPC_MAX_NUM_LLR },      { llrout, numLLR },
        };
        for (unsigned r = 0; r < 4; r++) {
            const uint8_t *q = regions[r].p;
            for (uint32_t i = 0; i < regions[r].n; i++) { h ^= q[i]; h *= 1099511628211ULL; }
        }
        printf("  FNV64(cn_msg,bn_msg,llrRes,llrout) = %016llx\n", (unsigned long long)h);
        printf("  cnProcPc vs scalar oracle: %s  (saw %s%s)\n",
               pc_bad ? "MISMATCH" : "match",
               (pc_seen & 1u) ? "pass " : "", (pc_seen & 2u) ? "fail" : "");
    }

    uint64_t cyc_total = cyc_scatter + cyc_cnProc + cyc_bnProcPc +
                         cyc_bnProc + cyc_cnProcPc + cyc_gather;

#define US(c)  ((double)(c) / CLK_MHZ)
#define PCT(c) (100.0 * (double)(c) / (double)cyc_total)

    printf("\n=== LDPC phase profile: BG1 Z=%d R=%s, %d BP iters, -O3, sim @%.0f MHz ===\n",
           SIM_Z, SIM_R_STR,
           N_ITER, CLK_MHZ);
    printf("  %-32s  %8s  %6s  %10s  %10s\n",
           "phase", "cycles", "%", "sim µs", "dev µs");
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "scatter (1× per cw)",
           (unsigned long long)cyc_scatter, PCT(cyc_scatter),
           US(cyc_scatter), DEV_SCATTER_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "cnProc_fused (×8 accum)",
           (unsigned long long)cyc_cnProc, PCT(cyc_cnProc),
           US(cyc_cnProc), DEV_CNPROC_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "bnProcPc (×8 accum)",
           (unsigned long long)cyc_bnProcPc, PCT(cyc_bnProcPc),
           US(cyc_bnProcPc), DEV_BNPROCPC_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "bnProc_fused (×8 accum)",
           (unsigned long long)cyc_bnProc, PCT(cyc_bnProc),
           US(cyc_bnProc), DEV_BNPROC_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "cnProcPc (×8 accum)",
           (unsigned long long)cyc_cnProcPc, PCT(cyc_cnProcPc),
           US(cyc_cnProcPc), DEV_CNPROCPC_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "gather (1× per cw)",
           (unsigned long long)cyc_gather, PCT(cyc_gather),
           US(cyc_gather), DEV_GATHER_US);
    printf("  %-32s  %8llu  %5.1f%%  %10.1f  %10d\n",
           "TOTAL (codeword compute)",
           (unsigned long long)cyc_total, 100.0,
           US(cyc_total), DEV_TOTAL_US);
    printf("\n  per-iter avg (inner 4 phases): %.0f cycles  (%.1f µs@TURBO)\n",
           (double)(cyc_cnProc + cyc_bnProcPc + cyc_bnProc + cyc_cnProcPc) / N_ITER,
           US(cyc_cnProc + cyc_bnProcPc + cyc_bnProc + cyc_cnProcPc) / N_ITER);
    printf("  dev/sim ratio: scatter %.2fx  cnProc %.2fx  bnProc %.2fx  cnProcPc %.2fx\n",
           (double)DEV_SCATTER_US / US(cyc_scatter),
           (double)DEV_CNPROC_US  / US(cyc_cnProc),
           (double)DEV_BNPROC_US  / US(cyc_bnProc),
           (double)DEV_CNPROCPC_US / US(cyc_cnProcPc));
    printf("  (dev < sim for iter phases → early termination at SNR=3dB; dev > sim → cache miss penalty)\n");

    return 0;
}
