# Viewing FARF Output on Linux HLOS (IQ-9075 / IQ-9075)

On Linux HLOS there is no logcat. These are the options in order of ease.

---

## 1. `printf` via FastRPC stdio forwarding

Recent fastrpc drivers forward DSP stdout back to the ARM process. Just use
`printf` directly in the skel — no FARF needed:

```c
printf("[DSP] clk = %u Hz\n", ctx->dsp_clk_hz);
```

Output appears on the ARM-side `ldpctest` stdout. Try this first.

---

## 2. Runtime FARF via `.farf` config file

Create a file named after the skel (no `.so` extension) with a FARF level bitmask:

```bash
echo "0x1f" > /vendor/lib/rfsa/adsp/ldpc_hexagon.farf
```

`0x1f` enables all runtime levels. Then use `FARF(RUNTIME_HIGH, ...)` in the
DSP code instead of `FARF(HIGH, ...)` or `FARF(ALWAYS, ...)`. Output goes to
the DIAG stream — capture with `mini_dm` (option 4).

---

## 3. `dmesg` — ERROR level reaches the kernel log

`FARF(ERROR, ...)` is forwarded to the kernel ring buffer:

```bash
dmesg | grep -i cdsp
```

Useful for one-off diagnostics that must be visible.

---

## 4. `mini_dm` — full DSP message capture (Hexagon SDK)

`mini_dm` reads the DSP DIAG stream and prints all FARF output.

```bash
# Push once
adb push $HEXAGON_SDK_ROOT/tools/mini_dm/Linux_Debug/mini_dm /data/local/tmp/

# Run in one shell
adb shell /data/local/tmp/mini_dm

# Run ldpctest in another shell
adb shell "cd /data/local/tmp && LD_LIBRARY_PATH=... ./ldpctest ..."
```

Captures all FARF levels. The standard Qualcomm tool for this scenario.

---

## 5. debugfs fastrpc log (kernel 5.x+)

```bash
cat /sys/kernel/debug/fastrpc/cdsp*/log 2>/dev/null || \
cat /sys/kernel/debug/qcom_fastrpc/*/log 2>/dev/null
```

Path varies by kernel version. No setup required — worth a quick check.
