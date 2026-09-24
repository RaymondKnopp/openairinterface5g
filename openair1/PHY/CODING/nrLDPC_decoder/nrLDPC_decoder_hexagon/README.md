# LDPC Decoder — Hexagon DSP Offload

Offloads 5G NR LDPC belief-propagation decoding from the A78 application cores
to the Qualcomm Hexagon Compute DSP (cDSP) on the DragonWing IQ-9075 via
FastRPC.

## Directory layout

```
nrLDPC_decoder_hexagon/
├── inc/
│   └── ldpc_hexagon.idl      FastRPC IDL (qaic → stub + skel + shared header)
└── src/
    ├── ldpc_hexagon_arm.c    ARM stub  → libldpc_hexagon.so
    └── ldpc_hexagon_imp.c    DSP skel  → libldpc_hexagon_skel.so
```

## Interface

The IDL defines a single RPC function:

```
long decode(in   sequence<uint8> params,    // 8-byte packed t_nrLDPC_dec_params
            in   sequence<uint8> llr,        // input LLRs (numLLR × int8)
            rout sequence<uint8> llr_out,    // posterior LLRs (numLLR × int8)
            rout sequence<uint8> meta);      // 4-byte LE uint32 = iterations completed
```

`params` wire format (8 bytes):

| Offset | Field      | Type    | Notes                                  |
|--------|------------|---------|----------------------------------------|
| 0      | BG         | uint8   | Base graph 1 or 2                      |
| 1      | R          | uint8   | Rate: 13, 23, 89 (BG1); 15, 13, 23 (BG2) |
| 2      | numMaxIter | uint8   | Maximum BP iterations                  |
| 3      | diag       | uint8   | 0=memcpy passthrough, 1=scatter/gather roundtrip, 2+=full BP |
| 4–5    | Z          | uint16  | Lifting size (2–384), little-endian    |
| 6–7    | —          | uint16  | Reserved                               |

The DSP always returns posterior LLRs in `llr_out`. The ARM stub converts to the
requested `outMode` (bit-packed / int8 / LLR pass-through) locally, and the
`check_crc` function pointer never crosses the FastRPC boundary.

## ARM stub (`ldpc_hexagon_arm.c` + `ldpc_encoder_optim8segmulti.c`)

`libldpc_hexagon.so` exports the same four symbols as `libldpc.so` so it is a
drop-in replacement for testing with `ldpctest` and the segmentation layer:

| Symbol         | Source                               | Description                       |
|----------------|--------------------------------------|-----------------------------------|
| `LDPCinit()`   | ldpc_hexagon_arm.c                   | Opens FastRPC session, rpcmem init |
| `LDPCshutdown()` | ldpc_hexagon_arm.c                 | Closes session, releases rpcmem   |
| `LDPCdecoder()` | ldpc_hexagon_arm.c                  | Offloads decode to cDSP            |
| `LDPCencoder()` | ldpc_encoder_optim8segmulti.c       | Runs on ARM (not offloaded)        |

The encoder is included unchanged from the standard `libldpc.so` build and
links against `ldpc_segment` and `ldpc_gen_HEADERS` exactly as it does there.
Offloading the encoder to the DSP may be evaluated separately.

`LDPCdecoder` flow:
1. Calls `nrLDPC_init` to derive `numLLR` from BG/Z/R.
2. Uses four ION-backed rpcmem buffers pre-allocated in `LDPCinit` (params, LLR in, LLR out, meta).
3. Copies input LLRs, calls `ldpc_hexagon_decode` via FastRPC.
4. Reads `numIter` from meta; sets abort flag if `numIter >= numMaxIter`.
5. Converts `llr_out` to the requested output format with a scalar loop.

## DSP implementation (`ldpc_hexagon_imp.c`)

Scalar min-sum belief propagation using the existing portable OAI headers:

| Header reused        | Role                                             |
|----------------------|--------------------------------------------------|
| `nrLDPC_types.h`     | Struct definitions (`t_nrLDPC_lut`, etc.)        |
| `nrLDPCdecoder_defs.h` | Constants (ZMAX, buffer sizes, group counts)   |
| `nrLDPC_lut.h`       | All static LUT arrays (circShift, BN addresses…) |
| `nrLDPC_init.h`      | LUT pointer setup (`nrLDPC_init`)                |
| `nrLDPC_mPass.h`     | Circular memcpy (`nrLDPC_llr2CnProcBuf`, etc.)   |

`#define CODEGEN 1` suppresses the `time_meas.h` dependency inside
`nrLDPC_types.h`. `sizeofArray` is defined locally before including
`nrLDPC_mPass.h` to avoid pulling in `common/utils/utils.h`.

### Algorithm

Standard flooding-schedule min-sum BP:

**Initialisation**
- `nrLDPC_llr2llrProcBuf` — scatter channel LLRs into LLR processing buffer
- `nrLDPC_llr2CnProcBuf_BG{1,2}` — scatter channel LLRs into CN processing buffer

**Each iteration**
1. `scalar_cnProc` — CN step (min-sum, scalar)
2. `nrLDPC_cn2bnProcBuf_BG{1,2}` — CN results → BN processing buffer (circular memcpy)
3. `scalar_bnProcPc` — BN posterior sum: `llrRes = sat8(llrProcBuf + Σ_k bnProcBuf[k])`
4. `scalar_bnProc` — BN extrinsic: `bnProcBufRes[k] = sat8(llrRes − bnProcBuf[k])`
5. `nrLDPC_bn2cnProcBuf_BG{1,2}` — extrinsics → CN processing buffer
6. `scalar_cnProcPc` — parity check: XOR of sign bits at each CN; returns 0 if all pass

**Output**
- `nrLDPC_llrRes2llrOut` — gather `llrRes` into external LLR order

### CN processing detail

`cnProc_group` processes one CN group (e.g. G3, G4, …, G19 for BG1):

```
for each (CN i, lifting symbol z):
    Pass 1 over all numBN positions k:
        sgn_all ^= cnProcBuf[startAddr + k*bitOff + i*Z + z]
        track min1, min2, min1_k
    Pass 2 — write extrinsic for each position j:
        mag_j  = (j == min1_k) ? min2 : min1
        sgn_j  = sgn_all ^ cnProcBuf[..j..]   // product of signs excluding j
        result = (sgn_j & 0x80) ? -mag_j : mag_j
```

`bitOff = numCN × NR_LDPC_ZMAX` — stride between BN position layers (bytes).

### Buffer layout in `bnProcBuf` / `bnProcBufRes`

BN groups are indexed by connectivity degree (`cnidx` = number of connected CNs − 1):

```
bnProcBuf[lut_startAddrBnGroups[g] + k * numBn * ZMAX + b]
                                      ↑ CN layer k         ↑ BN sample b ∈ [0, numBn*Z)
```

Only `numBn × Z` bytes are valid within each `numBn × ZMAX`-byte layer; the
extra `(ZMAX − Z) × numBn` bytes provide alignment for all lifting sizes without
recomputing offsets.

### Working buffers

All six processing buffers are heap-allocated on the DSP to stay within the
cDSP thread stack limit (~64 KB):

| Buffer         | Size (bytes)              |
|----------------|---------------------------|
| cnProcBuf      | 316 × 384 = 121 344       |
| cnProcBufRes   | 316 × 384 = 121 344       |
| bnProcBuf      | 316 × 384 = 121 344       |
| bnProcBufRes   | 316 × 384 = 121 344       |
| llrRes         | 27 000                    |
| llrProcBuf     | 27 000                    |
| **Total**      | **~540 KB**               |

## Build

### ARM stub — integrated OAI build (recommended)

Add `-DHEXAGON_LDPC=ON` to the normal DragonWing top-level cmake invocation:

```sh
cd openairinterface5g
cmake -B build-dragonwing \
    -DCMAKE_TOOLCHAIN_FILE=cmake_targets/cross-arm-dragonwing.cmake \
    -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
    -DHEXAGON_LDPC=ON \
    .
cmake --build build-dragonwing --target ldpc_hexagon --target ldpctest
```

**Important:** always pass `--target ldpc_hexagon` (and optionally `--target ldpctest`).
A bare `cmake --build build-dragonwing` attempts to build the full OAI tree and
fails on host-native tool targets (`_check_vcd`, `ldpc_generators`) that require
a prior native build with `-DNATIVE_DIR`.

CMake may print: *"Manually-specified variables were not used by the project:
CMAKE_TOOLCHAIN_FILE"* — this warning is harmless.  The toolchain is applied
correctly (the build targets aarch64); cmake emits the warning because the
variable is consumed implicitly before the project() call.

`libldpc_hexagon.so` is placed alongside the other OAI loadable modules in
`build-dragonwing/`. The integrated path uses standard CMake directly (no
`hexagon_fun.cmake`) so it works correctly within the OAI parent project.

### ARM stub — standalone build

```sh
cmake -B build-arm \
    -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
    -DOAI_SRC_ROOT=/path/to/openairinterface5g \
    -DCMAKE_TOOLCHAIN_FILE=<oai>/cmake_targets/cross-arm-dragonwing.cmake \
    -DOS_TYPE=HLOS \
    <oai>/openair1/PHY/CODING/nrLDPC_decoder/nrLDPC_decoder_hexagon
cmake --build build-arm
```

### DSP skel (always built separately with hexagon-clang)

```sh
cmake -B build-dsp \
    -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
    -DHEXAGON_TOOLS_ROOT=/opt/Hexagon_SDK/6.4.0.2/tools/HEXAGON_Tools/19.0.04 \
    -DOAI_SRC_ROOT=/path/to/openairinterface5g \
    -DCMAKE_TOOLCHAIN_FILE=/opt/Hexagon_SDK/6.4.0.2/build/cmake/hexagon_toolchain.cmake \
    -DDSP_VERSION=v73 \
    <oai>/openair1/PHY/CODING/nrLDPC_decoder/nrLDPC_decoder_hexagon
cmake --build build-dsp
```

On a device with a testsig (signed-PD) add `-DLDPC_HEX_VTCM=ON` to enable VTCM
and the spinning dspqueue worker:

```sh
cmake -B build-dsp \
    -DHEXAGON_SDK_ROOT=/opt/Hexagon_SDK/6.4.0.2 \
    -DHEXAGON_TOOLS_ROOT=/opt/Hexagon_SDK/6.4.0.2/tools/HEXAGON_Tools/19.0.04 \
    -DOAI_SRC_ROOT=/path/to/openairinterface5g \
    -DCMAKE_TOOLCHAIN_FILE=/opt/Hexagon_SDK/6.4.0.2/build/cmake/hexagon_toolchain.cmake \
    -DDSP_VERSION=v73 \
    -DLDPC_HEX_VTCM=ON \
    <oai>/openair1/PHY/CODING/nrLDPC_decoder/nrLDPC_decoder_hexagon
cmake --build build-dsp
```

Do **not** pass `-DLDPC_HEX_VTCM=ON` for unsigned-PD builds — the
`compute_resource_*` symbols are absent from the unsigned DSP runtime and will
cause `AEE_EUNABLETOLOAD` at skel dlopen time.

Confirm the correct DSP ISA version for the IQ-9075:
```sh
adb shell cat /sys/devices/soc0/soc_id
```

### Deploy

Always specify the **full destination path** (not a trailing-slash directory) to
avoid `adb: remote couldn't create file: Is a directory` errors:

```sh
adb push build-dragonwing/libldpc_hexagon.so   /usr/lib/rfsa/adsp/libldpc_hexagon.so
adb push build-dsp/libldpc_hexagon_skel.so     /usr/lib/rfsa/adsp/libldpc_hexagon_skel.so
```

`/usr/lib/rfsa/adsp/` is the first FastRPC library directory scanned by **both**
the ARM linker and the DSP skel loader on DragonWing.  Push both the ARM stub
and the DSP skel there.  `/vendor/lib/rfsa/adsp/` is scanned second; if a stale
skel exists there it will be shadowed by the `/usr/lib/` copy, but pushing to
`/vendor/` while `/usr/lib/` has an older version will silently load the wrong
binary.

### Test with ldpctest

`ldpctest` is an aarch64 test binary built alongside `libldpc_hexagon.so`.  It
dynamically loads an LDPC library by suffix; `-v _hexagon` selects
`libldpc_hexagon.so`.

Push and run:

```sh
adb push build-dragonwing/ldpctest  /data/local/oai/ldpctest
adb shell chmod +x /data/local/oai/ldpctest

# BG1, K'=8448 (max transport block segment), rate 1/3, 100 trials, SNR=3 dB
adb shell /data/local/oai/ldpctest -v _hexagon -l 8448 -r 1 -d 3 -n 100 -s 3
```

Key options:

| Flag | Meaning | Default |
|------|---------|---------|
| `-v _hexagon` | Load `libldpc_hexagon.so` | `""` (loads `libldpc.so`) |
| `-l <K'>` | Payload bits per segment (K' per 38.212 §5.2.2) | 8448 |
| `-r <n>` / `-d <d>` | Code rate = n/d; supported: 1/3, 2/3, 22/25 (BG1), 1/5, 1/3, 2/3 (BG2) | 1/3 |
| `-i <N>` | Max decoder iterations | 5 |
| `-n <N>` | Number of simulation trials | 1 |
| `-s <dB>` | SNR per information bit (Eb/N₀) | −2 |
| `-S <N>` | Number of segments (max 8) | 1 |

`ldpctest` prints per-iteration BLER/BER and a timing summary.  If the Hexagon
skel is not found (`AEE_EUNABLETOLOAD`), verify that
`libldpc_hexagon_skel.so` is in `/vendor/lib/rfsa/adsp/` and that the skel
ABI matches (v73 for IQ-9075, soc_id 0x60D).

## Deep idle wrecks the measurement (check this FIRST)

The ARM thread sleeps waiting for the DSP, so every batch pays one wake. On this platform
that wake is dominated by cpuidle exit latency, not by the queue:

| cpuidle state | exit latency |
|---|---|
| `WFI` | 1 us |
| `cpu-sleep-0` | **1450 us** |
| `cpu-sleep-1` | 1763 us |

Measured at 273 PRB / 1 layer / MCS 22, `OAI_LDPC_WORKERS=4`, batch of 18 code blocks:

| | wait | DSP wall | wake | segments decoding | Total PHY proc rx |
|---|---|---|---|---|---|
| stock idle | 3000 us | 1837 us | **1163 us** | 3008 us | 5627 us |
| `state[1-9]` disabled | 2050 us | 1845 us | **205 us** | 2057 us | **3146 us** |
| polling (`OAI_LDPC_SLEEP=0`) | 1951 us | 1884 us | 67 us | 1958 us | 3803 us |

Disabling deep idle cuts the wake 5.7x and costs nothing. It also speeds up the ARM inner
receiver (1300 -> 1039 us), which pays the same exit latency. Note the last row: with deep
idle off, SLEEPING is within 100 us of POLLING while still releasing the ARM core, so the
usual "poll for latency, sleep for CPU" trade-off mostly disappears -- do not switch the
default to polling on the strength of a stock-idle measurement.

    for c in /sys/devices/system/cpu/cpu[0-9]*/cpuidle; do
      for s in $c/state[1-9]; do [ -d "$s" ] && echo 1 > "$s"/disable; done
    done

This is a sysfs write and does NOT survive a reboot. `setup-sriov.sh` applies it, but that
script is for the fronthaul, and the CDSP work runs on a kernel where the fronthaul does
not -- so on a CDSP-only boot nothing applies it and every timing is ~1000 us pessimistic.
Check `cpu-sleep-0`'s `disable` before trusting any number here; a partially-applied state
(only `cpu-sleep-1` disabled) has been observed.

## Every lifting size works now (it did not until 2026-08-28)

The decoder used to work only for **Zc in {128, 256, 384}**. Every other lifting size
returned BLER 1.0 while the ARM reference decoded the same vectors at BLER 0, and the
LDPC offload had been carried for months in that state -- unnoticed because every
allocation we normally run lands on 384 or 256 (273/106/51 PRB give Z=384, 24 PRB gives
Z=256). A small UL grant failed outright.

**The bug was one instruction.** `hvx_circ_copy_any` wrote its residual tail through
`Q6_vmaskedstoreq_QAV`, on the stated assumption that "dst is 128-aligned, so dst+full is
too". It is not: dst is `&bn_msg[lut_sb[..] + bnPos*Z]`, a multiple of Z, not of 128 -- for
Z = 128/256/384 those coincide and the assumption holds, for anything else it fails. The
intrinsic takes an `HVX_VectorAddress`, so the address is truncated to a vector boundary
and the tail landed up to 127 bytes BELOW where it belonged, corrupting the previous BN's
messages.

**Fixing correctness exposed that the odd-Z paths were also unusably slow** -- 8181 / 7085
/ 15507 us per codeword at Zc = 120 / 176 / 240, against ~150 for the aligned sizes. Every
path that only ran when `Z % 128 != 0` was scalar, and cn_msg/bn_msg are in VTCM where a
scalar access costs ~20 cycles. In the sim, Z=120 cost 12692958 cycles against Z=128's
452205 -- 28x more for less data.

The fix throughout is **round the chunk count UP and let the last vector run past Z**,
instead of dropping to a scalar tail:

- HVX is lane-wise, so lanes past Z compute garbage that never disturbs the valid lanes.
- Where the garbage lands in our own scratch (`tmp[]` in cnProc and bnProc), it is simply
  never read back -- the rotation copies only Z bytes out.
- Where it would land in a shared buffer, `hvx_store_partial` blends the valid bytes over
  a re-read of the destination. Unlike a masked store that needs no alignment.
- Reads can run up to 127 bytes past the last group, so every buffer carries `HVX_VLEN` of
  slack. Cheaper than bounds-checking the inner loops.

The rotation itself was the last scalar holdout: `hvx_circ_copy_any` assembled each rotated
copy with two scalar `memcpy`s into a stack buffer. `hvx_circ_copy_pad` instead writes one
vector of wrap replica after the ring (`src[Z..Z+127] = src[0..127]`), which turns every
destination chunk into a single unaligned load at `src + ((m*128 + r) mod Z)` -- no valign,
no pair selection, no scalar pass. `hvx_circ_copy_ro` keeps the staging copy for the one
caller whose source is read-only (the channel-LLR scatter, once per codeword).

Sim cycles per codeword, and the FNV64 that had to stay fixed through all of it:

| Z | broken/slow | now | | FNV64 |
|---|---|---|---|---|
| 120 | 12692958 | **557994** | 22.7x | `10484ad9dd0c60b3` |
| 176 | 6324873 | **778116** | 8.1x | `f1be8884279db4a3` |
| 240 | 12764616 | **837912** | 15.2x | `8b7c5df650393803` |
| 128 | 452205 | 452205 | -- | `3d0aa6f542c16123` |
| 256 | 677169 | 675633 | -- | `3288cf385d2e5f23` |
| 384 | 877785 | 876681 | -- | `b4424e5eb7542f23` |

On hardware, `ldpctest -n 300 -i 5 -s 3` BG2 rate 1/5, **BLER 0 at every lifting size**:
Zc = 120 / 128 / 160 / 176 / 208 / 240 / 256 / 320 / 384 at 257 / 112 / 262 / 270 / 278 /
286 / 158 / 321 / 145 us. Odd sizes settle ~1.8x the nearest aligned size, because they
still use the `tmp[]` staging rather than cnProc's register fold -- against 54x slower and
wrong before. The production path is unchanged: 273 PRB / 1 layer / MCS 20 gives decode
178 us/codeword, ULSCH segments decoding 1304 us/slot, 802 dual-CDSP, and `nr_ulsim -r 11`
(Z=240), which used to fail outright, decodes at BLER 0 in 322 us/slot.

### cnProcPc_fused's ring: was only right for Z=384, now right for all of them

`cnProcPc_fused` loaded exactly three vectors at `col`, `col+128`, `col+256` and valigned
them as if the ring were always 3x128, gating the accumulation with `full`. For **any Z
below 384** -- 256 and 128 included, not just the non-multiples -- the wrap region then read
the NEXT `llrRes` slot instead of the ring's start. It erred toward reporting failure, so it
cost iterations rather than accuracy: the parity check simply never terminated early.

The ring is now handled for any Z. Chunk m wants the 128 bytes at ring offset
`o = (m*128 + cs) mod Z`, of which only `n = min(128, Z - m*128)` lanes are real, and
`w = Z - o` bytes are available before the wrap:

- `w >= n`: one unaligned load, the part we use does not wrap.
- `w < n`: lanes `[0,w)` from that load, lanes `[w,n)` from the ring start shifted right by
  w. `valign(vstart, vzero, 128-w)` does the shift and `vsetq2(w)` selects between them.

At most one wrap per chunk, since `n <= Z`. The last chunk's lanes past Z are masked to zero
before the reduction -- otherwise a padding lane can report a parity failure that is not one.

Effect, `ldpctest -n 300 -i 5 -s 3` (mean iterations, decode us):

| ldpctest | Zc | before | after |
|---|---|---|---|
| `-l 1280 -r 1 -d 5` | 128 | 4.00 / 112.5 | **1.06 / 81.9** |
| `-l 2560 -r 1 -d 5` | 256 | 4.00 / 158.0 | **1.03 / 114.7** |
| `-l 2400 -r 1 -d 5` | 240 | 4.00 / 285.8 | **1.01 / 190.0** |
| `-l 3840 -r 1 -d 5` | 384 | 1.01 / 144.6 | 1.05 / 148.3 |
| `-l 8448 -r 1 -d 3` | 384 | 4.00 / 315.8 | 4.00 / 315.2 |
| `-l 8448 -r 22 -d 25` | 384 | 2.99 / 106.9 | 3.11 / 109.3 |

Z=384 is unchanged, as it must be -- it was the one size that already worked. Production is
untouched either way, since `use_crc` is true for every real transport block and the CRC
path runs instead; `OAI_LDPC_CRC=0` now decodes at BLER 0 in the same 1306 us/slot as the
CRC path.

**How this hid for months, and what now stops it.** The sim harness FNV64 covers the message
buffers, and `cnProcPc` returns a flag and writes nothing -- so it had no check on it at
all. Comparing the flag on the harness's own LLRs is nearly worthless, too: they are chosen
*not* to converge, so parity always fails and an implementation that just returned 1 would
pass. The harness now carries `cnProcPc_ref`, a deliberately dumb scalar transcription of
the definition, and a test that plants ONE sign bit at a time and requires the two to agree
on every position, clustered on the slot and 128-byte boundaries where a ring bug lives.
That test was itself validated by reintroducing the bug: with the wrap correction disabled
it reports mismatches at Z=240 (`hvx=1 ref=0`, the false-failure signature) and stays clean
at Z=384.

## Every software prefetch in the decoder was a net loss

`DCFETCH128` appeared in three places -- the incoming codeword LLRs in the scatter,
`cn_msg` in `cnProc_fused_group`, and `bn_msg`/`llrRes` in `hvx_bnProcPc` and
`bnProc_fused`. All three were removed. 273 PRB / 1 layer / MCS 20 / SNR 13.2 / 200 slots,
arms alternated inside one batch and reproduced over two reps, us/codeword:

| phase | with prefetch | without |
|---|---|---|
| scatter | 29 | **15** |
| cnProc | 97 | **77** |
| cn2bn | 14 | 13 |
| bnProcPc | 32 | **18** |
| bnProc | 76 | **42** |
| bn2cn | 15 | 15 |
| termtest | 7 | 7 |
| gather | 15 | 15 |
| **total** | **285** | **202** |

(the Pass-2 fold below then took cnProc 77 -> 58 and the total to 185)

ULSCH segments decoding 1725 -> 1403 us/slot (-19%); Total PHY proc rx 2815 -> 2492
(-11%); dual-CDSP 1 layer 854 us/slot. BLER 0 in every arm -- prefetch is semantically
neutral, so all of this was pure cost. The `hexagon-sim` harness confirms the output is
bit-identical (same FNV64 over `cn_msg`/`bn_msg`/`llrRes`/`llrout`).

Two reasons, and neither is "prefetching is bad":

1. **`cn_msg` and `bn_msg` moved to VTCM and nobody revisited the prefetch.** VTCM is not
   backed by L1/L2, so there is nothing for a `dcfetch` to prime, but the instruction still
   issues. A 36%-of-kernel optimisation became pure overhead through a change made
   somewhere else entirely.
2. **`DCFETCH128` is four `dcfetch` instructions**, issued in nested loops. `bnProc_fused`'s
   look-ahead fired 3 offsets x 2 buffers = 24 scalar instructions per CN position to cover
   loads that hit L2 anyway. `llrRes` genuinely *is* cached, and its prefetch was still the
   single biggest loss (`bnProc` -36%).

`-DLDPC_HEX_MSG_DCFETCH=1`, `-DLDPC_HEX_LLR_DCFETCH=1`, `-DLDPC_HEX_IN_DCFETCH=1` restore
them individually (the `dsp/Makefile` takes `EXTRA_CFLAGS=`). If a buffer moves back out of
VTCM, re-measure rather than re-enabling on principle.

## libc memcpy is not vectorised here, and three phases were nothing but memcpy

Two of the profile's phase names were lying, which is why this went unlooked-at: slots 2 and
5 were labelled `cn2bn` / `bn2cn`, for a message-passing pair that no longer exists --
`cnProc_fused` and `bnProc_fused` absorbed both -- and had been quietly repurposed as
timers for the two input-side copies. They are now `llrcopy` and `llr2proc`. Reading the
profile as "cn2bn 13 us" sends you looking for message passing; reading it as "a 26 KB
memcpy costs 13 us" does not.

Once relabelled, three of the eight phases turned out to be pure `memcpy`:

- `llrcopy`: `memcpy(llrRes, p_llr, numLLR)`, one 26 KB block.
- `llr2proc`: `nrLDPC_llr2llrProcBuf`, one bulk copy plus 22 copies of Z bytes.
- `gather`: `nrLDPC_llrRes2llrOut`, the same shape inverted -- and it runs EVERY iteration
  under CRC termination.

**libc's `memcpy` is not using the vector unit on this DSP.** The 26 KB copy measured 13 us,
about 1.4 bytes per cycle. A 128-byte-at-a-time loop (`hvx_memcpy`) is the entire fix; the
two reference functions get HVX twins here rather than edits to `nrLDPC_mPass.h`, which the
ARM decoder shares.

| phase | before | after |
|---|---|---|
| llrcopy | 13 | **3** |
| llr2proc | 15 | **2** |
| gather | 15 | **2** |
| **DSP compute total** | **164** | **127 us/codeword** |

ULSCH segments decoding 1229 -> 1084 us/slot, dual-CDSP 764 -> 687, 2 layers 2278 -> 1972,
Total PHY proc rx 2324 -> 2175. BLER 0 at every lifting size and on both termination paths.

The residual is copied as bytes rather than with a partial vector store, so `hvx_memcpy`
neither reads nor writes outside its buffers -- the input is FastRPC-mapped and the last
segment's end is a real boundary. Correctness is checked against the reference directly in
the sim harness (`hvx_memcpy` at lengths 1/127/128/129/383/384/385 and numLLR, plus both
scatter and gather), because `ldpc_scalar_core` is not exercised there and the FNV64 cannot
see any of it.

This also settles the untimed `memset`s at the top of `ldpc_scalar_core` (269 KB across
cn_msg, bn_msg and llrProcBuf): DSP compute is 127 us against a phase sum of 123, so they
cost about 4 us and are not worth pursuing.

### Getting diagnostics off the DSP at all

None of the usual FARF channels work on this platform, and it is worth knowing that before
spending an afternoon on it: `printf` forwarding over FastRPC is not active, `FARF(ERROR)`
does not reach `/dev/kmsg` (and `dmesg` here is a stub -- read `/dev/kmsg` directly, it
works), and `mini_dm` is not in the SDK tree. See `FARF_DEBUG.md` for the option list.

What does work is **the batch response struct**, which the ARM already prints. `vtcm_ok`
now carries the number of VTCM bytes actually granted rather than a flag, because "the
acquire returned a context" is not the same as "we got what we asked for". That one change
turned a guess into a measurement.

### llrRes and llrProcBuf now live in VTCM

They were deliberately kept in heap because the gather read them with a scalar memcpy, and a
scalar VTCM access bypasses L1/L2 for ~20 cycles. Vectorising the copies killed that reason:
every access is now a vector access, and an HVX read of VTCM is 1 cycle.

**The first attempt broke** -- BLER 1.0 and everything ~2x slower -- and the obvious
explanation was wrong. Reporting the grant showed **4 MB handed back for a 1.43 MB request**
(`single_page` rounds up to a 4 MB page), so it was never short. The real cause was
alignment: `NR_LDPC_MAX_NUM_LLR` is 27000, not a multiple of 128, so carving the two llr
buffers out at their natural size misaligned everything after them -- including `gv_f`,
whose vgather/vscatter staging requires an aligned VTCM address. Every slice is rounded to
`HVX_VLEN` now.

**The second trap was the copy tail.** `hvx_memcpy` copies its residual as bytes, which is
right for the FastRPC-mapped input but catastrophic once the destination is VTCM: the
scatter and gather do 22 copies of Z bytes each, so at Zc=240 the byte tails alone took the
decode from 171 to 279 us. `hvx_memcpy_slack` finishes with a masked vector RMW instead, and
is used only on buffers we own (which carry `HVX_VLEN` of slack).

| | heap | VTCM |
|---|---|---|
| scatter | 15 | **14** |
| llrcopy | 3 | **1** |
| llr2proc | 2 | **1** |
| gather | 2 | **1** |
| bnProc | 31 | **30** |
| **DSP compute** | **127** | **120 us/codeword** |

ULSCH segments decoding 1077 -> 1036 us/slot, dual-CDSP 687 -> 635, 2 layers 1972 -> 1893,
the Zc=240 grant 313 -> 248. Odd lifting sizes gain the most, because they were paying the
scalar tails: Zc = 120 / 176 / 240 / 320 go 133 / 162 / 171 / 186 -> 109 / 137 / 142 / 159
us. BLER 0 everywhere.

The scatter itself barely moved (15 -> 14), so its cost is NOT L2 latency on `llrRes` as
guessed. `--packet_analyze` settled what it is: see below.

### What the scatter is actually waiting on

Profiling it needed two fixes to the harness first. The sim was timing the **reference**
`nrLDPC_llr2CnProcBuf_BG1` while the device ran `hvx_llr2CnProcBuf_BG1`, so the kernel that
ships had never been exercised there at all; it now runs the device one and checks it
against the reference. And `THREAD_OFF_PVIEW_CYCLES`, which looked like 50% of the scatter's
cycles, is attributed to address `0xdeadbeef` with **zero commits** -- it is the simulator's
accounting for cycles with no thread running, not a stall in our code. Excluding it, the
scatter is 9830 cycles per codeword (4396 commits + 5434 stalls), not 22591.

Per repetition it is 2826 permutes, 1878 vector loads, 1896 vector stores and **zero**
`HVXPIPE_ALU` -- pure data movement. Only 32% of committed packets contain any HVX; the
other 2983 carry 6479 scalar instructions, about 20 per edge, of address arithmetic and loop
control.

The two worst packets are named exactly:

    r19 += mpyi(r11,r24)     219 commits, 438 stalls   2.00/commit   CU_BUSY
    v22 = vmemu(r19+#0x0)    219 commits, 438 stalls   2.00/commit

which is `src = llr + lut_pos[i] * Z` feeding a vector load address immediately: the load
cannot issue until the multiply retires.

**Hoisting the Z dispatch out of the per-edge path works** -- `hvx_circ_copy` re-tested Z
against 128/256/384 on every one of the 316 edges, on a value invariant for the whole
codeword. Specialised on `NV = Z/HVX_VLEN` and dispatched once: **scatter 14 -> 11 us**,
DSP compute 120 -> 118, ULSCH 1041 -> 1032 us/slot.

Note the sim called that change *neutral* (15072 cycles either way) and the device made it a
21% win. Third time this session the two have disagreed on a verdict; the device decides.

**TRIED AND SLOWER -- breaking the multiply-to-load chain.** Computing edge i+1's source
address before copying edge i, the same one-step-behind shape that works in the cnProc
rotation, is worse at every Z: 15072 -> 15807 (Z=384), 12942 -> 14082 (256), 27450 -> 29331
(240). The extra bound check and live pointer cost more than the chain does. So the stall
`--packet_analyze` pointed at is real but not removable this way.

### bnProc: scalar-bound, and the fix is not in the vector code

`--packet_analyze` on `bnProc_fused` in the device configuration, per call (one BP
iteration): **7878 commits + 8845 stalls = 16723 cycles**, 2.12 cycles per packet.

| stall cause | % of stalls | % of cycles |
|---|---|---|
| `COPROC_BUSY` -- HVX busy | 40.3 | 21.3 |
| `CU_BUSY` -- register interlock | 35.1 | 18.6 |
| `DU_BUSY_OTHER` -- DU replay/bubble | 13.9 | 7.4 |
| `REDIRECT` -- mispredict | 10.0 | 5.3 |
| `DU_CACHE_MISS` | 0.4 | 0.2 |

`DCACHE_DEMAND_MISS` is **2**. There is no memory problem. What there is:

- **Only 23% of committed packets contain any HVX** -- 1781 of 7878. The other ~6100 carry
  **14312 scalar instructions**, about 45 per edge.
- The vector work is small and well packed: 3288 permutes, 822 ALU, 3288 loads, 1644
  stores, 5.1 HVX ops per HVX packet.
- Worst single packet is `r11 = add(r17,r3)` at 4.00 stall cycles per commit, feeding the
  `vmemu(r11)` two packets later -- address arithmetic gating a vector load again. Next is
  `r21 = memuh(r27+r21<<#0x1)`, the `lut_cs[i]` halfword load, at 2.09.
- All 308 DU replays are `DU_BANK_CONFLICT_REPLAY`: the paired `llrRes` and `bn_msg` loads
  hitting the same bank. Now that both are in VTCM their relative offset is fixed, so this
  is tunable with padding -- but at ~308 replays it is worth a few percent, not more.

**So bnProc is not waiting on its arithmetic; it is drowning in address arithmetic.** The
largest identified slice of that is work it repeats needlessly: every call rebuilds
`slot_to_bngrp[316]` and then, for each of the 316 edges, recomputes `llrPos_tab[i]` and
`bn_addr_tab[i]` -- a divide by `NR_LDPC_ZMAX`, a table lookup and two multiplies by Z.
**All of it depends only on the LUT and Z, both invariant across BP iterations**, and
`cnProcPc_fused` rebuilds the same slot table again. That is roughly 4800 of the 14312
scalar instructions, call after call.

**Done.** The tables are now built once per (BG, Z, R) by the dispatching thread, before it
releases the workers -- building them lazily inside the kernel would have four workers
racing to fill the same arrays. Two entries, because a transport block uses at most two
rates (segments `r < first_rE2` take R, the rest R2). The diag paths pass NULL and `bnProc_fused` computes inline exactly as
before. The per-segment dspqueue handler and the synchronous RPC entry build it on demand
via `ldpc_addr_cache_get`, which is safe there because each runs on one thread at a time --
**pool workers must never call it**, which is the whole reason the batch path builds ahead
of releasing them.

That distinction matters for measurement, because **`ldpctest` uses the per-segment path,
not the batch path** -- no "batch path active" line, and it decodes one segment at a time.
Until the per-segment path got the cache, `ldpctest` numbers reflected every other
optimisation but not this one. With it: `-l 8448 -r 1 -d 3` 246 -> 226 us, `-r 2 -d 3`
133 -> 120 us, both at an unchanged 4.00 iterations, while the batch path stays exactly
where it was (bnProc 22 us, DSP compute 109).

**bnProc 31 -> 23 us/codeword (-26%), DSP compute 118 -> 112.** BLER 0 at every lifting size,
both termination paths, 2 layers and dual-CDSP; the sim FNV64 is identical with the cache on
and off at every Z, which is what proves the cached edge order matches the order the kernel
consumes it in.

Two things worth knowing before touching it. `ldpc_addr_cache_build` walks `(g, j, i)` in
**exactly** the order `bnProc_fused` consumes them, and the two share `LDPC_BNS_WRITE` so
they cannot disagree about the group loop -- but nothing enforces the rest, so
`-DSIM_AC=0/1` and the FNV64 are the guard if that nest is ever changed. And
**hexagon-sim reports this change 35% SLOWER** (239547 -> 322350 cycles at Z=384) where the
device makes it 26% faster. That is the fourth sim/device disagreement of the session and
the widest; the sim holds every buffer in heap, so trading scalar arithmetic for table loads
looks bad there in a way it is not when cn_msg/bn_msg/llrRes are in VTCM. `restrict` on the
table pointers changed nothing either way.

**Two profiling traps caught in the act, both already documented above and both walked into
anyway.** The first bnProc profile measured 28488 cycles per call with `dcfetch` packets as
the top four stall sites at 5.00 cycles per commit -- because the sim build does not define
`LDPC_HEX_VTCM`, so `LDPC_HEX_MSG_DCFETCH` defaults back to 1 and the prefetches the device
does not run were dominating. With `-DLDPC_HEX_MSG_DCFETCH=0` the real figure is 16723 and
the dcfetches vanish. The second: differencing per-address profiles between two `N_ITER`
builds is invalid, because `N_ITER` is a compile-time constant and the two binaries have
different code layouts -- the aggregate delta is fine, per-address is not. Restrict to one
build and one symbol's address range instead.

### termtest: the CRC was the whole phase, and it was a dependency chain

`termtest` is whichever termination test runs -- and under `use_crc`, which is every real
transport block, that is `ldpc_llr2bits` + `ldpc_check_crc24`. The sim harness had never
exercised that path: `SIM_KERNEL=4` runs `cnProcPc_fused`, the OTHER test, the one the device
never takes. `SIM_KERNEL=6` runs the CRC form.

Split across the two, per call:

| | commits | stalls | cycles | HVX packets |
|---|---|---|---|---|
| `ldpc_llr2bits` | 454 | 472 | 926 | 22% |
| `ldpc_check_crc24` | 3246 | 4455 | **7701** | **0%** |

The bit packing is fine. The CRC was **eight times larger and entirely scalar**, with
`CU_BUSY` at 94% of stalls -- a register interlock, which is exactly what a byte-at-a-time
table CRC is: `crc = (crc << 8) ^ tab[*in++ ^ (crc >> 24)]`, where each step's index needs
the previous step's result, over 1056 bytes, every BP iteration. The worst packets were all
the `immext` carrying the table base address.

**Slicing-by-4** folds four bytes into one dependent step: four table lookups that do not
depend on each other, and only their XOR carries forward, so the chain is a quarter as long.
Three extra tables per polynomial, built by the same recurrence
(`t[k][i] = (t[k-1][i] << 8) ^ t[0][t[k-1][i] >> 24]`, one more zero byte pushed through),
8 KB total. The identity is the standard one for an MSB-first 32-bit register, which this is:
the 24-bit CRC sits in bits [31:8] and the low byte is always zero because `LDPC_POLY24A` is
the polynomial pre-shifted by 8.

**termtest 9 -> 5 us/codeword, DSP compute 112 -> 109.** In the sim the CRC goes 7701 -> 5320
cycles and `CU_BUSY` falls from 94% to 48% of stalls, with `DU_BUSY_OTHER` rising to 45% --
the interlock is gone and what is left is the four clustered table loads, which is why
slicing further would buy less than the first step did.

Exactness is the whole requirement here -- a CRC that is merely usually right is a decoder
that merely usually terminates correctly -- so the byte-at-a-time version is kept as
`ldpc_crc24_ref` (sim-only) and the harness compares the two over random data at **every bit
length from 1 to 1200 for both polynomials**, plus the production length. 0 mismatches.

### bnProcPc is already near the machine, and that is the answer

`--packet_analyze` on `hvx_bnProcPc`, device configuration, per call:
**2637 commits + 2899 stalls = 5536 cycles**, 2.10 cycles per packet.

| stall cause | % of stalls | % of cycles |
|---|---|---|
| `COPROC_BUSY` -- HVX busy | **90.0** | **47.1** |
| `REDIRECT` | 6.3 | 3.3 |
| `CU_BUSY` | 3.1 | 1.6 |
| `DU_CACHE_MISS` | 0.0 | 0.0 |

This is the opposite shape to bnProc. There is no scalar problem and no memory problem
(`DCACHE_DEMAND_MISS` 0, two DU replays, fourteen mispredicts in the whole call). 44% of
committed packets contain HVX -- against bnProc's 23% -- at **6.7 HVX ops per HVX packet**,
and the kernel sustains 1.41 HVX ops per cycle across 3570 ALU, 1512 permute, 2304 load and
408 store operations. It is packed about as tightly as this machine allows.

**So there is no scheduling slack to recover, and the only remaining lever is doing fewer
HVX operations.** The ALU count comes from the accumulator being halfword-wide: each of the
up to 20 messages per BN is sign-extended into a vector pair and added twice, four ops per
message, because the sum reaches +-2688 and cannot live in a byte.

That is reducible, but not cheaply or safely:

- Saturating byte accumulation is wrong -- saturation is not associative. `sat8(sat8(100+100)-100)`
  is 27 where the true sum is 100.
- HVX has no signed widening byte add; `Q6_Wh_vadd_VubVub` is unsigned only. Using it needs
  every operand biased by 128 (an extra `vxor` each) and a constant correction at the end,
  which nets about 5 ops per two messages against the current 8 -- roughly a third off the
  ALU count, so perhaps 2-3 us of the 112.
- `Q6_Wh_vmpaacc_WhWubRb` could fold two messages into one accumulate, but it is a
  byte-pair dot product with its own operand ordering, and it is also unsigned.

Both routes change the arithmetic domain of the decoder's message passing for a few
microseconds. **Left alone deliberately**: the measurement says this kernel is not where the
remaining time is, and the cheap structural wins that worked elsewhere -- hoisted dispatch,
cached tables, native instructions -- have no equivalent here.

**Profiling note:** `hvx_bnProcPc` had been inlined into `main`, and `--packet_analyze`
reports per packet *address*, so there was no range to attribute to. Build the harness with
`-DLDPC_PROFILE_NOINLINE` to keep the kernels as real symbols. It changes inlining, so use
it to find *where* the cycles go and measure the verdict on the device.

## Reducing ALU ops, once the profile said that is what is scarce

`--packet_analyze` established that these kernels are bound by HVX issue, and the failed
register-fusion experiment narrowed that further: it is the **ALU** pipe specifically
(`HVXPIPE_ALU` 80514 against 28440 vector loads). Loads are nearly free; instructions are
not. Three changes follow from that, all bit-exact -- the sim FNV64 is unchanged at every Z:

**1. The 2-best update in three ops instead of four.** cnProc's Pass 1 tracked the two
smallest magnitudes with `cmp -> mux -> min -> min`. It is exactly:

    cand  = max(vmin1, vabs)     // the OLD vmin1, so this must come first
    vmin2 = min(vmin2, cand)
    vmin1 = min(vmin1, vabs)

If `vabs < vmin1` the max IS the old `vmin1`, which is what the mux selected; otherwise it
is `vabs`, likewise; ties agree too.

**2. `hvx_sat8_add`/`hvx_sat8_sub` were about twelve instructions each.** They sign-extended
both operands into halfword pairs, added, clamped with `vmin`/`vmax` against splatted
+127/-128, packed the low bytes and shuffled back. HVX does saturating signed-byte
arithmetic natively -- `Vd.b = vadd(Vu.b,Vv.b):sat`, CVI_VA, SLOT0123 -- in **one**. These
run per edge per 128-byte chunk in `bnProc_fused` and `hvx_bnProcPc`.

**3. `hvx_bnProcPc`'s clamp-and-pack in one op.** Its group accumulator genuinely needs
halfword width (it sums up to 20 messages), but the narrowing back to bytes was four
`vmin`/`vmax` plus a plain `vpacke`. `Q6_Vb_vpack_VhVh_sat` (`vpackhb:sat`) clamps and
narrows in a single instruction.

Measured, 273 PRB / 1 layer / MCS 20 / 200 slots, two reps, us/codeword:

| phase | before | after |
|---|---|---|
| scatter | 16 | 16 |
| cnProc | 52 | **49** |
| cn2bn | 13 | 13 |
| bnProcPc | 18 | **14** |
| bnProc | 40 | **31** |
| bn2cn | 15 | 15 |
| termtest | 7 | 7 |
| gather | 15 | 15 |
| **total** | **181** | **160** |

ULSCH segments decoding 1290 -> 1229 us/slot, dual-CDSP 802 -> 764, 2 layers 2424 -> 2278,
Total PHY proc rx 2384 -> 2322. BLER 0 at every lifting size, on both termination paths.

Pass 2 of cnProc was worked through for the same kind of saving and has none: precomputing
+-m1/+-m2, sign-mask arithmetic instead of `cmp`+`sub`+`mux`, a 4-way select, and dropping
`vabs` in favour of an argmin tracked in Pass 1 all come out at the same op count or worse.
It stands at 7 ops, Pass 1 at 5.

**The general lesson: check whether an emulated sequence has a native instruction before
optimising around it.** Two of these three were multi-instruction emulations of things the
ISA does in one, sitting in the hottest loops, and no amount of restructuring the
surrounding code would have found them.

## cnProc Pass-2 fold: the rotated extrinsic goes straight to bn_msg

`cnProc_fused_group` used to stage each code block's extrinsics in a stack `tmp[]` and then
run a circular scatter that loaded them straight back out, rotated, into `bn_msg`. Since the
kernel is bound by HVX issue bandwidth (see below), that round-trip was pure cost:
`cnProc_fused_row` now produces the rotation from registers as Pass 2 computes it.

Per BP iteration this removes ~1400 vector stores and ~1400 vector loads of the kernel's
~7100 vector memory ops. Measured, us/codeword, 273 PRB / 1 layer / MCS 20 / 200 slots:

| | before | after |
|---|---|---|
| cnProc | 77 | **58** (-25%) |
| decode total | 202 | **185** |
| ULSCH segments decoding, us/slot | 1403 | **1325** |
| ... dual CDSP | 854 | **812** |

`ldpctest` reaches every lifting size, but is NOT comparable to the profile above -- one
codeword on one worker, parity termination instead of the segment CRC, and a rate you choose
rather than the one the scheduler picked.

**`ldpctest -n` below ~100 reports garbage, and it does not look like garbage.** Its
"Decoding time mean" is contaminated by a cold first call that its "max" accumulator never
sees, so the mean can come out ABOVE the max -- 622 us mean against 435 us max at `-n 10`,
549 us mean against 391 us max at `-n 20`. Both are plausible-looking numbers. Always check
mean < max, and use `-n 300`; at that length the same configuration repeats to ~1%. Its mean
iteration count also drifts, so compare us/iteration when it does.

At `-n 300 -i 5 -s 3`, decode us (us/iteration), across all three steps:

| ldpctest | Zc | original | prefetch removed | + Pass-2 fold |
|---|---|---|---|---|
| `-l 8448 -r 1 -d 3`   | 384 | 454.8 (113.7) | 363.4 (90.9) | **330.3 (82.6)** |
| `-l 8448 -r 2 -d 3`   | 384 | 233.6 ( 58.4) | 190.5 (47.6) | **173.6 (43.4)** |
| `-l 8448 -r 22 -d 25` | 384 | 143.8 ( 43.9) | 118.2 (38.0) | **110.2 (35.6)** |
| `-l 2560 -r 1 -d 5`   | 256 | 220.7 ( 55.2) | 185.3 (46.3) | **162.5 (40.6)** |
| `-l 1280 -r 1 -d 5`   | 128 | 155.1 ( 38.8) | 125.1 (31.3) | **116.1 (29.0)** |

BLER 0 in every arm (the 22/25 rows ran 3.28 / 3.11 / 3.10 iterations, hence per-iteration).
The fold alone is 9% at rate 1/3 and 9% at 2/3; the two changes together are 27% and 26%.
Note the fold's share of the whole decode here is far smaller than its 25% cut of cnProc,
because ldpctest at these rates spends much of the decode outside cnProc.

Two things make this safe to have done:

- **Pass 2 had to be turned inside out.** A rotated store needs several chunks of the SAME
  k at once, so Pass 2 runs k-outer / chunk-inner, which makes the 2-best state
  chunk-indexed. `NV = Z/HVX_VLEN` is therefore a literal at each call site (1, 2, 3) --
  with a variable bound the compiler spills `m1`/`m2`/`sg` and the reloads cost more than
  the fold saves. Same trap as the 2-layer 16QAM metric loop in `inner_rx_ml2_hvx.c`.
- **The rotation is derived independently of `hvx_circ_copy`**, so the two can drift. The
  guard is the sim harness FNV64, which must be identical with `-DLDPC_CNPROC_FOLD=0`.
  It is, at every Z (and the cycle counts are not): Z=128 `3d0aa6f542c16123`, 144600 ->
  113976; Z=256 `3288cf385d2e5f23`, 306888 -> 174552; Z=384 `b4424e5eb7542f23`, 422760 ->
  232056. Use `-DSIM_Z=` to pick.

Only three vectors stay live rather than all NV: dst chunk m is `valign(e[q+1], e[q])` with
q = (m + r/128) mod NV, so walking q upward emits one store per chunk one step behind the
producer, and the ring's wrap needs chunk 0 kept to the end -- first / prev / cur. `m` is
only ever a store offset, never a register index.

## Reading the PMU, and one thing it does NOT mean

`sim/` builds the kernels for `hexagon-sim`, which is where the above started.
`make sim` gives the relative phase profile; `--timing` runs the microarchitecture model
and populates the PMU counters, without which almost everything reads zero:

    cd sim && make clean && make EXTRA_CFLAGS="-DSIM_KERNEL=1 -DN_ITER=32"
    hexagon-sim -mv73 --timing --statsfile pmu.txt ./ldpc_sim_test

`SIM_KERNEL=1..4` isolates one kernel (the whole-run counters cannot be attributed
otherwise); `LDPC_CNPROC_STAGES` selects which stages of `cnProc_fused_group` are emitted.
Take counter *deltas* between two `N_ITER` values -- that removes init, scatter and gather
exactly. Beware that disabling a stage lets the compiler delete its consumers: dropping the
scatter makes Pass 2's stores dead, so that arm measures nothing useful.

For stall *attribution* rather than counting, use `--packet_analyze`, which writes a JSON
carrying per-packet-address stall cycles split by cause:

    hexagon-sim -mv73 --timing --packet_analyze pa.json ./ldpc_sim_test
    hexagon-profiler --json=pa.json --elf=ldpc_sim_test --packet_analyze -o pa.html

The JSON is more useful than the HTML if you want to aggregate: `core_packet_profile` is
keyed by packet address, and `core_packet_profile_help` documents every counter. Map
addresses to functions with `hexagon-nm --print-size` and `hexagon-llvm-objdump -d`.
**The sim build does not define `LDPC_HEX_VTCM`, so `LDPC_HEX_MSG_DCFETCH` defaults back
to 1 there** -- pass `-DLDPC_HEX_MSG_DCFETCH=0` or you are profiling a configuration the
device does not run. (Doing this by accident was useful once: the three `dcfetch` packets
came out as the top three stall sites at 5 stall cycles per commit, cause
`DU_BUSY_OTHER`, i.e. dcfetch generating DU replays -- an independent confirmation of the
device measurement, and `DU_BUSY_OTHER` then collapsed from 57812 to 892 cycles with the
prefetch off.)

Per BP iteration, `cnProc`: `ICACHE_DEMAND_MISS` 0, `DCACHE_DEMAND_MISS` 61,
`CU_PKT_READY_NOT_DISPATCHED` 0 -- yet 61% of cycles are bubbles, 64% of committed packets
carry **exactly one** bubble, and packets hold 2.12 instructions of 4 slots. So: not
memory, not issue slots.

**One bubble per packet is not proof of a dependency chain.** The obvious reading is that
`m1`/`m2`/`sgn` carry across `k` (`m2`'s path `vcmp -> vmux -> vmin` is three dependent ops
deep), so the 2-best reduction was split into W independent streams and merged with
`min1 = min(a1,b1)`, `min2 = min(max(a1,b1), min(a2,b2))` -- exact including multiplicity,
and the sim checksum was identical for W=1..5. It was also monotonically **slower**:
1427717 cycles at W=1 against 1478943 / 1483389 / 1496196 / 1487942 for W=2..5. Do not
redo it.

`--packet_analyze` says why, in the configuration the device runs (prefetch off, 4 BP
iterations, `cnProc_fused_group` only):

| stall cause | % of stalls | % of cycles |
|---|---|---|
| `COPROC_BUSY` -- HVX coprocessor busy | **72.4** | **37.5** |
| `REDIRECT` -- branch mispredict | 12.7 | 6.6 |
| `CU_BUSY` -- register interlock, no-B2B-HVX | 9.8 | 5.1 |
| `DU_CACHE_MISS` | 3.5 | 1.8 |
| `DU_BUSY_OTHER` -- DU replay/bubble | 1.1 | 0.6 |

`CU_BUSY` is the category that *contains* register interlocks, and it is 5% of cycles --
that is why splitting the reduction did nothing. The kernel is waiting on **HVX issue
bandwidth**: 30507 of 76026 committed packets contain HVX, `HVXPIPE_ALU` 80514 and
`HVXPIPE_PERM` 34128 against `HVXPIPE_MPY` 54, and packets containing a vector load or
store are 35% of all cycles. The inlined circular scatter (`hvx_circ_copy`) alone is
**39% of the kernel**.

Dynamic vector memory ops per BP iteration, from the disassembly weighted by commits:

| region | loads | stores | valign |
|---|---|---|---|
| Pass 1 + Pass 2 + scalar tail | 2844 | 1353 | 0 |
| inlined circular scatter | 1422 | 1491 | 1422 |

Pass 1 and Pass 2 load exactly 1422 each -- Pass 2 re-reads every `vk` Pass 1 already had
and recomputes its `vabs`. And the scatter's 1422 loads are re-reads of the `tmp[]` Pass 2
had just written. So two levers, both aimed at HVX op *count*, which is the thing that is
actually scarce:

1. **Fold the rotation into Pass 2** and store to `bn_msg` directly, dropping `tmp[]`:
   removes ~1400 stores and 1422 loads per iteration, ~40% of the kernel's vector memory
   ops. Needs a one-chunk lag, since a rotated store spans two source vectors (Z=384 is
   only 3 chunks, so the pipeline is short).
2. **Fuse Pass 1 and Pass 2 for low-degree CN groups**, keeping `vk` in registers: removes
   up to 1422 loads. Only possible while `numBN` vectors plus accumulators fit in the 32
   HVX registers, so groups of degree 3/4/5 yes, degree 19 no -- Pass 2 needs `vmin1`/
   `vmin2`/`vsgn_all`, which are not known until Pass 1 has seen every `k`.

Lever 1 is done (see the Pass-2 fold above). **Lever 2 was tried and loses** -- kept here
because the reasoning above is what made it look attractive, and it is wrong.

Holding the row's `cn_msg` vectors in registers so Pass 2 need not reload them fits only for
low-degree CN groups (NV*numBN vectors plus 3*NV of 2-best state, against 32 registers), but
degree <= 6 is 51% of BG1 R=1/3's 316 edges, so the coverage looked worthwhile. On hardware
it loses, monotonically in the degree covered:

| fused up to degree | cnProc us/codeword | ULSCH us/slot |
|---|---|---|
| off | **52** | **1289 / 1294** |
| 6 | 53 | 1312 / 1305 |
| 7 | 57 | 1321 / 1318 |

Not spills: no variant has a stack-relative `vmem` in its disassembly. The cause is that
cnProc is **ALU-bound, not load-bound** -- `HVXPIPE_ALU` 80514 against 28440 vector loads --
and on the device `cn_msg` is in VTCM, where a load is ~1 cycle and rides in the same packet
as the ALU work that is the real constraint. Removing loads frees nothing, while holding
12-21 vectors live costs the scheduler its freedom to interleave.

So the sentence that motivated it -- "a vector load costs because it consumes HVX issue" --
is only half right. It does consume issue, but not the *contended* issue. `DU_CACHE_MISS` at
1.8% correctly ruled out the cache argument; what it did not establish is that loads were on
the critical resource, and they are not.

**Two measurement traps here, both worth remembering.** The probe that sold the idea deleted
Pass 2's load by substituting a register and showed 652205 -> 549369 cycles (15.8%) -- but
that let the compiler fold the dependent `vabs` and `vcmp_eq` too, so it measured "remove the
load *and its ALU work*". A probe that removes a value must keep the arithmetic that consumed
it. And `hexagon-sim` agreed only in sign and only barely (2.3% at best, where hardware lost
10%), because it holds these buffers in heap and models no VTCM, so it systematically
overvalues a removed load.

## CRC termination on the DSP

The decoder terminates on the **segment CRC**, not on the parity check. A
`check_crc` function pointer cannot cross FastRPC, but it does not need to: the
CRC is fully determined by `Kprime` and `crc_type`, so those two fields go over
the wire in `ldpc_q_req_t` and the DSP builds its own CRC-24A/B tables
(polynomials copied from `crc_byte.c`). Each iteration gathers, hard-decides,
packs and checks; `cnProcPc` is then **not computed at all** — parity is only
ever a termination test, and belief propagation does not need it.

`OAI_LDPC_CRC=0` reverts to parity termination on the same binary, which is how
the two were compared. `ldpctest` passes `check_crc = NULL`, so it keeps the
parity path unconditionally and is unaffected (measured identical: BLER 0,
162.99 us, 3.81 iterations at rate 2/3, K'=8448).

273 PRB, mu=1, MCS 20, 2 layers, 4 Rx, `-C1`, SNR 13.2, 200 slots, one batch:

| termination | segment decode us/slot | DSP us/codeword | mean iterations |
|-------------|-----------------------|-----------------|-----------------|
| segment CRC | **14,419**            | **416**         | **3.7**         |
| parity      | 17,206                | 499             | 4.8             |

**16% faster on segment decoding, 10% on total RX, and lower BLER.** The BLER
gain is not a coincidence: parity is satisfied strictly later than the CRC, late
enough that a correct codeword routinely used the last iteration and returned
`numMaxIter`, which OAI reads as a failure. At SNR 13.0 over 300 slots the gap
is BLER 0.090 against 0.170 (throughput 95.5% against 91.5%). Below the
waterfall both report BLER 1.0 — CRC termination adds no false successes.

Do not compare timings across batches: the DSP phase profile drifts ~20% with
clock state, enough to invert a 12% conclusion. And `nr_ulsim` is not
deterministic run to run, so single-run BLER at 50 slots is worth little — the
parity path, which never calls the packer at all, moved between 0.06 and 0.08 on
identical invocations. Use >= 200 slots and compare within one batch.

Cost of the test, per codeword-iteration (hexagon-sim, converted at 1459 MHz):

| bit-packing                                  | pack  | + CRC |
|----------------------------------------------|-------|-------|
| bit-at-a-time                                | 70.0  | 76.5  |
| 64-bit multiply trick                        | 17.5  | 24.1  |
| `vcmpb.gt` + `tfrpr` + `brev` + `swiz`       | 13.1  | 19.7  |
| HVX `vcmp` + `vrmpy` + `vpacke` (tiered)     | 3.5   | 10.1  |

The CRC table walk itself is only 6.6 us, so the packing dominated every scalar
version — which is why `ldpc_llr2bits` is tiered: HVX for whole 1024-LLR blocks,
the scalar byte-vector path for whole 32-LLR blocks, a bit loop for the last few.
The middle tier is not redundant: K' is not a multiple of 1024 (8448 leaves 256),
and finishing that remainder one bit at a time costs more than the vector part.
On hardware this took the pack+CRC phase from 44 to 16 us/codeword.
`LDPC_CRC_FIRST_ITER` additionally skips the test on iterations that never
converge anyway.

### This decoder is still slower than the A78 CPU decoder

Same batch, same operating point, 200 slots — segment decoding per slot:

| decoder                        | us/slot   |
|--------------------------------|-----------|
| native CPU (`ldpc_optim8seg`)  | **8,500** |
| Hexagon, CRC termination       | 14,322    |

So CRC termination is a large win *within* this decoder and the decoder is still
**1.7x behind the CPU**. Enabling the LDPC offload costs throughput; its only
current argument is moving work off the ARM cores, which pays only where the ARM
is the bottleneck and the CDSP is otherwise idle. Treat the offload as
not-for-production until that gap closes. The measurement is stable, at least:
BLER held flat over 50/200/400 slots (0.000 / 0.045 / 0.013).

### Verify Hexagon bit manipulation under hexagon-sim, not on hardware

`vitpack` looks like the instruction for concatenating two byte-compare
predicates. It is not — it takes the **even** bits of `Ps` and the **odd** bits
of `Pt` into one byte. `Q6_R_equals_p` (`C2_tfrpr`) is the predicate-to-register
move. Probe before trusting either:

```
$SDK/tools/HEXAGON_Tools/*/Tools/bin/hexagon-clang -mv73 -O2 -o probe probe.c \
    -I$SDK/tools/HEXAGON_Tools/*/Tools/target/hexagon/include
$SDK/tools/HEXAGON_Tools/*/Tools/bin/hexagon-sim -mv73 ./probe
```

This matters more here than it looks. A wrong bit order in the packer does not
fail anything visibly: the CRC simply never matches, every segment runs to
`numMaxIter`, the ARM-side CRC re-check still passes the correct output up to
OAI, and the only symptom is a decode that is quietly 25% slower. It cost a full
build/deploy/measure cycle to notice. The simulator settles the question in
seconds, and single-bit response tables pin down a mapping exactly.

## Limitations / future work

- The decoder is 1.7x slower than the A78 CPU decoder (above). Closing that gap
  is the only thing that makes this offload worth enabling for throughput.
  `cnProc` (34%) and `bnProc` (30%) are where the time is; the termination test
  is now down to 4%.
- The scalar baseline has not yet been benchmarked on hardware; expected
  throughput on the cDSP scalar pipeline is comparable to A78 NEON before HVX
  vectorisation.

## Pairing with the inner-RX offload

When inner-RX and LDPC both run on the CDSP the stages are kept SEPARATE, in one FastRPC
dispatch, with the descrambled LLRs left in VTCM as int8 (this batch path's input is already
`const int8_t *`). Fusing inner-RX and decode per segment was considered and rejected; the
reasoning and the on-target measurement that settled it are in
`openair1/PHY/nr_phy_common/hexagon_inner_rx/README.md`, section "Fusing inner-RX with LDPC".

The number that matters here: the batch leaves **28% of worker-time idle** while it drains
(steady wall 1505 us against a 1080 us balanced ideal, gap ~1.5 codewords). That gap is
decode-time variance and the ragged tail, NOT pooling overhead -- there are only 15 claims
per batch, so per-claim cost is ~1% at most. Do not try to fix it by making work units
coarser; the fix is finer granularity and filling the tail with the next stage's work.

## When FastRPC opens start failing for everything

Symptom: `ldpc_hexagon_open failed: -1`, or `dspqueue_read` returning `0xc` (AEE_EEXPIRED,
a timed-out wait) -- and crucially it happens for configurations that worked minutes
earlier, including single-device ones. It reads as "the DSP is broken". It is not; it is
resource exhaustion.

Cause: both DSP workers SPIN on `dspqueue_read_noblock`, and before the idle watchdog they
left that loop only on an explicit STOP packet. Any ARM-side death that skips STOP -- a
`timeout` kill, a crash, Ctrl-C -- left the thread spinning inside the blocking
`worker_start` FastRPC call, so the PD could never be reclaimed. Each abandoned run leaked
one, and a sweep of back-to-back runs exhausted the DSP.

The idle watchdog (180 s of silence -> the worker returns) makes this self-healing: 18 runs
killed mid-flight no longer degrade anything. If you see the symptom anyway, WAIT three
minutes before concluding something is wrong.

### Do NOT restart the subsystem to clear it

That is what turns a recoverable state into a reboot:

    echo stop  > /sys/class/remoteproc/remoteproc2/state   # cdsp; remoteproc3 is cdsp1
    echo start > /sys/class/remoteproc/remoteproc2/state

The stop times out waiting for a shutdown response (the spinning worker never answers) but
eventually completes. The start then takes MINUTES to boot `cdsp0.mbn`. Retrying it, or
killing the write because it looks hung, interrupts the boot:

    remoteproc remoteproc2: can't lock rproc cdsp: -4     <- EINTR
    remoteproc remoteproc2: Boot failed: -4

and leaves the writer stuck in D state holding the rproc lock. `kill -9` will not clear a D
state, every later `start` blocks on the same lock, and the subsystem stays `offline`. At
that point only a reboot recovers it: `echo b > /proc/sysrq-trigger`, which takes ~25 s.
Plain `reboot` hangs on this board and needs a power cycle.

If you do restart a subsystem, issue the start ONCE and let it run to completion however
long it takes.

### After any reboot, re-apply the RT tuning

Nothing survives. A fresh boot comes up `schedutil` at 1632 MHz with cpu-sleep-0/1 ENABLED,
and every DSP measurement taken in that state is noise -- cpu-sleep-1 costs 1763 us to exit
against 50-200 us of per-task work. Run:

    adb shell sh /data/oai/dsp-rt-tune.sh

That is the CDSP-only equivalent of the CPU tuning in `setup-sriov.sh`; the latter also does
SR-IOV/DPDK/hugepage work that is wrong on this kernel, but its CPU tuning is mandatory for
both. Verify before trusting numbers:

    gov=performance sleep0=1 sleep1=1
