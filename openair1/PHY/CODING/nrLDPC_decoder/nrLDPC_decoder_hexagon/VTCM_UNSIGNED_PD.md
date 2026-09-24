# VTCM Allocation from Unsigned-PD on IQ-9075 (IQ-9075)

**Platform:** IQ-9075 / IQ-9075 DragonWing  
**Hexagon SDK:** 6.4.0.2  
**DSP:** Hexagon v73 (CDSP), 1.42 GHz TURBO, 8 MB VTCM per HTP  
**Mode:** Unsigned protected domain (unsigned-PD) via `DSPRPC_CONTROL_UNSIGNED_MODULE`

---

## Motivation

We are offloading 5G NR LDPC decoding (BG1, Z=384) from ARM Cortex-A78 to the
Hexagon CDSP to free ARM cores for other tasks. The LDPC working set is 538 KB:

| Buffer | Size |
|--------|------|
| cnProcBuf | 121,344 B |
| cnProcBufRes | 121,344 B |
| bnProcBuf | 121,344 B |
| bnProcBufRes | 121,344 B |
| llrRes | 27,000 B |
| llrProcBuf | 27,000 B |
| **Total** | **538,376 B** |

The CDSP has 8 MB of VTCM scratchpad, which would hold the entire working set with
near-register latency.  With TURBO clock (1.42 GHz) and VTCM we estimate ~0.97 ms
per 4 BP iterations vs. ~3.3 ms with DRAM (at 1.46 GHz measured).

---

## What We Tried

### Approach 1: HAP_compute_res.h (CRM API)

Including `HAP_compute_res.h` and calling the standard API:

```c
compute_res_attr_t attr;
HAP_compute_res_attr_init(&attr);
HAP_compute_res_attr_set_vtcm_param(&attr, 539392, 1);
unsigned int ctx_id = HAP_compute_res_acquire(&attr, 0);
void *vtcm = HAP_compute_res_attr_get_vtcm_ptr(&attr);
```

**Result:** The skel fails to load (`AEE_EUNABLETOLOAD`, 0x80000406).

**Root cause:** `HAP_compute_res.h` declares the underlying functions with
`__attribute__((weak))`.  These weak references appear in the skel's dynamic symbol
table (`.dynsym`):

```
w compute_resource_acquire
w compute_resource_attr_get_vtcm_ptr
w compute_resource_attr_init
w compute_resource_attr_init_v2
w compute_resource_attr_set_vtcm_param
w compute_resource_release
```

The IQ-9075 RTLD rejects the skel at `dlopen` time when any weak symbol in `.dynsym`
cannot be resolved from the currently-loaded DSP runtime libraries.  This is
non-standard ELF behavior — POSIX requires unresolvable weak symbols to resolve to
NULL; the IQ-9075 RTLD returns `AEE_EUNABLETOLOAD` instead.

These symbols are **not exported** by the IQ-9075 DSP runtime in unsigned-PD mode.

### Approach 2: dlopen/dlsym at Runtime

To avoid adding weak references to the skel's `.dynsym` at link time, we tried
resolving the CRM functions at runtime:

```c
#include <dlfcn.h>
typedef unsigned int (*crm_acquire_fn)(crm_attr_t *, unsigned int);
crm_acquire_fn fn = dlsym(RTLD_DEFAULT, "compute_resource_acquire");
```

**Result:** Same failure — `AEE_EUNABLETOLOAD` (0x80000406).

**Root cause:** `dlsym` itself is not exported by the IQ-9075 DSP runtime.  Including
`<dlfcn.h>` and calling `dlsym` adds `w dlsym` to the skel's `.dynsym`, which the
RTLD rejects for the same reason.

---

## Key Finding

**The IQ-9075 DSP RTLD (in unsigned-PD mode) fails `dlopen` if any weak symbol in
the skel's `.dynsym` cannot be resolved from a currently-loaded library.**

This is reproducible and deterministic.  Only weak symbols that ARE exported by the
DSP runtime are safe to use:

| Symbol | Available |
|--------|-----------|
| `HAP_debug` | ✓ |
| `HAP_debug_runtime` | ✓ |
| `HAP_debug_v2` | ✓ |
| `__cxa_finalize` | ✓ |
| `HAP_power_set` | ✓ (strong) |
| `HAP_power_request_abs` | ✓ (strong) |
| `HAP_power_get` | ✓ (strong) |
| `compute_resource_acquire` | ✗ — not in unsigned-PD runtime |
| `dlsym` | ✗ — not in unsigned-PD runtime |

---

## Questions for QCOM

1. **Is VTCM allocation supported in unsigned-PD on IQ-9075?**  
   If yes, which API should be used?  `compute_resource_acquire` (CRM)?
   `HAP_request_VTCM` (older sysmon API)?  Something else?

2. **If `compute_resource_acquire` is available in unsigned-PD, why is it not
   exported by the DSP runtime on this device?**  
   Is there a firmware update, a build flag, or a signing step that enables it?

3. **Can the RTLD behavior be changed so that truly-unresolvable weak symbols
   are set to NULL instead of causing `AEE_EUNABLETOLOAD`?**  
   This would allow the standard SDK pattern (include `HAP_compute_res.h`, check
   function pointer for NULL before calling) to work correctly in unsigned-PD.

4. **Is `HAP_power_request` + `HAP_VTCM_resource` available as an alternative?**  
   We see no VTCM-related fields in `HAP_power.h` (SDK 6.4.0.2).

5. **What is the recommended path to VTCM access on IQ-9075 without code signing?**  
   We are using unsigned-PD as we do not yet have a QCOM signing workflow set up.

---

## Current Workaround

We malloc the working set on the CDSP heap (DRAM-backed).  The 538 KB working set
exceeds the 32 KB L1 cache but fits entirely in the 1 MB L2 cache after warmup.
At 1.46 GHz TURBO with L2 hits, decode time is ~3.1 ms per codeword (1 BP iteration
at SNR=10 dB).  ARM Cortex-A78 reference: ~0.22 ms.  The gap is the permutation
phase (cn2bn/bn2cn, 81% of cycles in simulator profile) which is latency-bound on
DRAM and would benefit most from VTCM.

---

## Platform / Software Details

| Item | Value |
|------|-------|
| SoC | IQ-9075 |
| Board | IQ-9075 DragonWing |
| Hexagon SDK | 6.4.0.2 |
| Hexagon toolchain | 19.0.04 |
| DSP ISA | v73 |
| DSP TURBO clock | 1459 MHz (measured via FastRPC at session open) |
| VTCM per HTP | 8 MB |
| L2 per HTP | 1 MB |
| FastRPC mode | Unsigned-PD (`DSPRPC_CONTROL_UNSIGNED_MODULE`) |
| Error code | `AEE_EUNABLETOLOAD` = 0x80000406 |
