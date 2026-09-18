<!-- SPDX-License-Identifier: CC-BY-4.0 -->

# Where the time goes in the AAL LDPC offload

Measured on an Intel Xeon Gold 6433N (Sapphire Rapids, TSC 2.0 GHz) with an
on-package ACC200/VRB1, and on an AccelerComm T2 (Xilinx, `igb_uio`). Numbers come
from temporary rdtsc instrumentation, since reverted.

> Methodology note, because it cost a wrong first answer: a shared object sees
> `cpu_freq_GHz = 1.000`, because it never runs OAI's TSC calibration. Any
> rdtsc-based timing computed inside `libldpc_aal.so` is therefore off by the TSC
> rate. Cross-check against an OAI counter covering an identical region before
> believing such numbers.

## Encode, per transport block

`nr_dlsim -R 273 -b 273 -q1 -e27 -x4 -y4 -z4`, TBS 1 081 512 bits, **C = 129**.
`DLSCH encoding time` 91.3 us total.

| phase | us | % | rate over the TB |
|---|---:|---:|---|
| `crc24a` over the TB | 15.8 | 17 % | 8.6 GB/s (PCLMULQDQ path) |
| `memcpy` a -> `dlsch->b` | 5.7 | 6 % | 23.6 GB/s |
| `nr_segmentation`, b -> 129 x `c[r]` | 8.3 | 9 % | |
| `rte_memcpy` c -> DPDK mbufs | 20.4 | 22 % | **6.6 GB/s** |
| op alloc + `set_ldpc_enc_op` | 1.4 | 2 % | |
| **enqueue/dequeue = device + DMA** | **19.6** | **21 %** | ~16 GB/s effective |
| `reverse_bits_u8` over the output | 7.0 | 8 % | 26.1 GB/s (GFNI) |
| concat / copy into `output` | 5.8 | 6 % | |
| `rte_pktmbuf_free` x258 + `rte_free` | 4.2 | 5 % | per slot |
| sum | 87.7 | 96 % | vs 91.3 measured |

**The offload is not PCIe-bound.** `ts_ldpc_encode` brackets only the bbdev
enqueue/dequeue loop, i.e. the whole device round trip including DMA in and out:
19.6 us of 91.3, 21 %. The rest is host staging, and the transport block is walked
five times:

    a -> b -> c[r] -> mbuf -> DEVICE -> mbuf -> output

Note this cost is paid **per HARQ round**, not per transport block: encoding is
stateless and redone from `a` every round.

### Consequences worth discussing

1. **The mbuf fill runs at 6.6 GB/s, 3.6x slower than a plain `memcpy` of the same
   135 kB at 23.6 GB/s.** The gap is per-code-block mbuf management (mempool get,
   `rte_pktmbuf_append`, 129 small copies), not data movement.
2. **Three of the five copies are pure staging.** If `nr_segmentation` wrote
   directly into persistent DPDK mbufs, both `dlsch->b` and `c[r]` disappear:
   ~29 us, 32 % of the encode. This needs a coding-interface hook so the backend
   hands out the `segments[r].c` destinations instead of the caller owning them.
   That is not merely stylistic: **the T2 reports `min_alignment = 64` where the
   ACC200 reports 1**, so only the backend knows where a code block may start.
   `init_op_data_objs_enc` already asserts this
   (`data == RTE_PTR_ALIGN(data, min_alignment)`).
3. **The mbufs are allocated and freed every slot.** 258 per TB, ~4.2 us, for
   buffers that could be allocated at init and released at cleanup. Note the pool
   is currently sized `optimal_mempool_size(ops_pool_size)` where `ops_pool_size`
   derives from lcore count and `OPS_POOL_SIZE_MIN` (511) -- i.e. sized by queues
   and lcores, **not** by code blocks (~1023 mbufs). Persistent mbufs must instead
   be sized by `MAX_NUM_NR_DLSCH_SEGMENTS_PER_LAYER * layers * nb_TBs`, or the pool
   silently exhausts with several UEs.
4. `memcpy(dlsch->b, a, (A / 8) + 4)` is redundant: `crc24a` already writes the CRC
   into `a` in place, `nr_segmentation` only reads from `b`, and there is no HARQ
   reason to keep `b` since encoding is redone from `a` each round. ~5.7 us, and
   it needs no interface change.

### Why the device cannot do the segmentation

bbdev TB mode (`code_block_mode = 0`) would let the caller pass the CRC-24A
augmented TB and have the device segment, attach CB CRCs, encode, rate-match and
concatenate. The ACC200 PMD implements it, and OAI already uses it for the
degenerate C == 1 case. It is unusable for C > 1 on both devices because neither
advertises `ENC_SCATTER_GATHER` (so the TB must be one mbuf) or
`ENC_CONCATENATION` (needed whenever E is not byte aligned; `nr_get_E` returns a
multiple of `Nl * Qm`), and DPDK caps a single mbuf at 64 KiB while a loaded TB is
132 kB in / 179 kB out.

## Per-CB CRC offload (capability-driven)

With the backend publishing `NRLDPC_CODING_CAP_ENC_CB_CRC`, `nr_dlsch_encoding`
skips the per-CB CRC when the device attaches it. `DLSCH encoding time`:

| configuration | CPU computes | device attaches | delta |
|---|---|---|---|
| ACC200, 273 PRB, 2 layers, MCS 20 (C = 16) | 21.7 us | 19.9 us | -8.2 % |
| ACC200, 273 PRB, 4 layers, MCS 27 (C = 129) | 104.1 us | 91.1 us | -12.4 % |
| T2, 273 PRB, 2 layers, MCS 20 (C = 16) | 51.6 us | 48.6 us | -5.7 % |

`LDPC encoding time` is unchanged across arms in every case, confirming the saving
is in segmentation rather than the offload. Both devices decode cleanly with the
capability active (BLER 0, 0 of 47 M bits over 200 trials x 2 runs).

## Decode: HARQ combining costs about 1 dB on both devices

`nr_ulsim -R 273 -r 273 -m 16 -v 4`, TBS 96264, C = 12, 200 trials per point.
Round 1 never decodes for any backend, so round-2 BLER measures HARQ combining
alone.

| SNR | CPU | ACC200 | T2 |
|---|---|---|---|
| 8.0 dB | 0.045 / 0.020 | 0.335 | 0.345 |
| 9.0 dB | 0.000 | 0.060 | 0.055 |
| 10.0 dB | 0.000 | 0.000 | 0.000 |

But single shot (`-v 1`, no combining), in the waterfall, the device decoder is
about 0.5 dB **better** than the CPU decoder:

| SNR | CPU BLER | ACC200 BLER |
|---|---|---|
| 11.0 dB | 0.990 | 0.890 |
| 11.5 dB | 0.345 | 0.065 |

So the decoder and the LLR feed are fine; the entire round-2 deficit is in the
combining path. Ruled out by measurement: device fixed-point precision (the two
devices are 8/1 and 6/2 yet degrade identically), LLR scaling (they take
*different* preparation paths -- `llr_scaling()` vs plain saturation -- yet degrade
identically), 8-bit quantisation (the CPU path rescales to 8 bit too), buffer
truncation (`LDPC_MAX_CB_SIZE` 32768 > max `N_cb` 25344), and HARQ memory
architecture (opposite on the two devices). Not root-caused.

## Other observations

* `(harq_unique_pid * NR_LDPC_MAX_NUM_CB + i) % num_harq_codeblock` with
  `NR_LDPC_MAX_NUM_CB = 144` and `num_harq_codeblock` defaulting to 512 aliases
  from `harq_unique_pid = 4` onward: pid 3's code blocks 80..143 land on pid 0's
  buffers 0..63, silently corrupting HARQ combining. Not reachable in ulsim, but
  reachable in a gNB with enough concurrent HARQ processes.
* In `init_op_data_objs_enc` / `_dec`, `bool large_input` is declared outside the
  code-block loop and never reset, so once any block exceeds
  `RTE_BBDEV_LDPC_E_MAX_MBUF` every later block in the slot takes the fake-mbuf
  path; that `rte_malloc`'d buffer is also never freed, since cleanup calls
  `rte_pktmbuf_free` on an mbuf whose `buf_addr` was overwritten. Unreachable today
  for encode input (<= 1056 bytes per CB), but it would fire if TB mode were used.
* `is_t2` is a hand-set configuration flag defaulting to 0, and still drives four
  behavioural branches (TB-mode special case on encode and decode,
  `processedSegments` reset semantics, and LLR preparation). Running a T2 without
  `--nrLDPC_coding_aal.is_t2 1` silently selects the wrong paths. The driver name
  is available in `info.drv.driver_name`. Note also that the T2 branch bypasses
  `llr_scaling()` entirely and saturates to 8 bit, ignoring the `llr_size = 6,
  llr_decimals = 2` the device itself advertises.
