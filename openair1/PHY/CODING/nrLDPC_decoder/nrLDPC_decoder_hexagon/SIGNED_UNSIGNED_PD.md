# Signed vs Unsigned Protection Domains on IQ-9075

## What is a protection domain?

The Hexagon DSP runs user code in isolated processes called protection domains (PDs).
Two types exist for the cDSP:

- **Signed PD**: the shared object (.so) loaded on the DSP must carry a digital
  signature verified at load time. Full access to HAP services: VTCM, clock voting
  (HAP_power_set), compute resource management, L2 cache locking, etc.

- **Unsigned PD**: no signature required — the skel loads freely. The PD runs in a
  sandbox with reduced privileges. Only available on the cDSP (not aDSP/mDSP).

## What is blocked in unsigned PD (IQ-9075)

| Feature | Unsigned PD | Notes |
|---------|-------------|-------|
| HVX contexts | available | |
| VTCM (HAP_vtcm_request) | **blocked** | RTLD rejects `compute_resource_*` weak symbols |
| Clock voting (HAP_power_set) | **silently no-op** | Returns AEE_SUCCESS but has no effect |
| HAP_power_get (clock query) | returns 0 | |
| Hardware cycle counters (C15:14) | return 0 | |
| L2 cache locking | blocked | |
| UBWCDMA / Camera Streamer | blocked | |
| QuRT timer APIs | blocked | use qurt_signal_wait_timed() instead |

The practical effect: in unsigned PD the CDSP runs at the 19.2 MHz crystal oscillator
(XO) regardless of HAP_power_set calls. This was confirmed on IQ-9075 via
`clk_summary` showing `26300000.remoteproc` consuming `xo`.

## Is this restriction technical or commercial?

**Both, and the two are intertwined.**

### The security argument is genuine

The CDSP shares the SoC with the modem baseband, secure enclaves, camera ISP, and
safety-critical subsystems. On the IQ-9075 (automotive/industrial target), a
compromised or runaway DSP binary that could vote for TURBO clocks, lock VTCM, or
access DMA could affect system stability or leak data from adjacent subsystems. The
sandbox is a real isolation boundary.

### But the mechanism also serves commercial interests

Signing requires going through Qualcomm. That relationship:
- Gives Qualcomm visibility into who deploys DSP code in production
- Ties production use to a business agreement
- Ensures OEMs cannot ship products with arbitrary third-party DSP binaries without
  Qualcomm's knowledge

The clearest evidence that it is not *purely* security: `HAP_power_set` returns
`AEE_SUCCESS` in unsigned PD but silently does nothing. A clean security model would
return `AEE_EACCESS`. A silent no-op is what you get when a feature is gated for
non-security reasons and nobody wanted to break caller code that checks the return value.

## Getting signed PD on a DragonWing development board

Signed PD is not a fundamental hardware capability that is locked out — it is an
access control gate on the software path. For research/academic partners (e.g. OAI)
there is usually a path via Qualcomm engineering to obtain either:
- A signing key for the development board
- Pre-signed development firmware with relaxed restrictions

This is worth asking your Qualcomm contact directly. The answer is often
"yes, here is the process" rather than a hard no.

## SDK documentation references (local)

```
/opt/Hexagon_SDK/6.4.0.2/docs/software/ipc/rpc.html
```
Section "DSP protection domains" → "Signed and unsigned PDs", "Unsigned PD limitations".

```
/opt/Hexagon_SDK/6.4.0.2/docs/software/ipc/dspqueue.html
```
Asynchronous DSP Packet Queue API — the recommended low-overhead alternative to
synchronous FastRPC for persistent worker patterns. Handles cache maintenance and
has an "early wakeup" mechanism to hide ARM interrupt latency on the return path.

```
/opt/Hexagon_SDK/6.4.0.2/docs/pdf/80-VB419-178_D_Qualcomm_Hexagon_QuRT_RTOS_User_Guide_SDK.pdf
```
QuRT thread scheduling — explains the IPC interrupt latency discussed below.

## FastRPC overhead and the spinning worker pattern

At 19.2 MHz XO, synchronous FastRPC costs ~400 µs per call. The overhead is not
data transfer (~27 KB LLR at DRAM bandwidth = <1 µs) but the IPC machinery:

1. ARM syscall + fastrpc driver
2. ARM D-cache flush for input buffers
3. Doorbell interrupt to DSP
4. QuRT interrupt handler + thread scheduling (at 19.2 MHz: ~74× slower than TURBO)
5. RPC unmarshal + skel dispatch
6. [compute]
7. DSP completion interrupt to ARM
8. ARM D-cache invalidate for output buffers
9. ARM userspace wake

Steps 4 and 7–8 alone account for most of the 400 µs.

### Recommended approach: dspqueue + batching

Use the `dspqueue` API (see reference above) to implement a persistent DSP worker:

- `LDPCinit()`: open fastrpc handle, allocate shared ring buffer, start DSP worker
  (calls into skel once; skel enters `dspqueue_read()` loop and never returns to RPC)
- `LDPCdecoder()`: write codeword packet to queue via `dspqueue_write()` (non-blocking),
  poll for completion packet — zero syscalls, zero interrupt round-trips per codeword
- `LDPCshutdown()`: send stop packet, DSP exits loop, close handle

Batch multiple codewords per slot into the queue before polling. ARM fills N slots,
flushes cache once, DSP drains the queue. Per-codeword IPC overhead drops from
~400 µs to ~2–5 µs (cache flush time for 27 KB).

Even at 19.2 MHz this removes the IPC tax entirely. At TURBO (once signed PD is
available) the compute itself drops to ~0.24 ms/iter, making the IPC overhead
completely negligible.
