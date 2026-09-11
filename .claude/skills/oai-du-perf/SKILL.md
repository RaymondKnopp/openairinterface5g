---
name: oai-du-perf
description: Measure and debug OAI gNB DU real-time performance on constrained platforms - per-slot L1 timing, DL/UL throughput ceilings, link adaptation, and offline nr_dlsim/nr_ulsim characterisation. Use when a DU drops traffic, oscillates in MCS, overruns its slot budget, or when a change needs proving not to have regressed the PHY.
---

# OAI DU performance work

## The rule that matters most

**Never trust a lifetime counter.** Every stats file OAI writes is truncate-and-rewrite
once per second, and every counter in it is cumulative from startup. A mean over a
10-minute run that contained 20 seconds of traffic tells you about the idle period.
Difference two snapshots from the same load window, or you will draw a conclusion about
nothing. This has produced wrong answers repeatedly, including an invented "84,033 job
backlog" from two counters read 42 seconds apart, and a "3-8x tail" that was an artefact
of comparing a load window against a lifetime average.

Corollary: read both counters of a comparison from the **same** snapshot.

## Capture

`capture.sh`, next to this file, runs the DU and samples the stats files at 2 Hz into
`stats-stream.txt`, timestamped, skipping unchanged reads. Without it the history is
destroyed every second. Paths come from the environment so it works on any host:

    CONF=/path/to/gnb.conf POOL=10,11 .claude/skills/oai-du-perf/capture.sh

It records the config, kernel cmdline and git HEAD alongside each run, and refuses to
start if a DU is already running. What it takes care of, and what you must supply if you
run nr-softmodem by hand:

- **`-q`** on the nr-softmodem command line. This sets `cpu_meas_enabled`; without it
  `nrL1_stats.log` contains no timing tables at all. It is not a "quiet" flag.
- the right `--thread-pool`, which must not overlap any other pool or pinned thread.

Analysis pattern: split `stats-stream.txt` on `### t=<epoch> file=<name>`, find the load
window by `goodput DL > threshold`, then difference the first and last sample inside it.
For a mean over an interval from cumulative `diff`/`trials`:
`(mean_b * n_b - mean_a * n_a) / (n_b - n_a)`.

## Core assignment

Check what threads are actually on which core. Do not read it off the config file: the
conf pins single threads (`L1_rx_thread_core`, `L1_tx_thread_core`, `ru_thread_core`) as
well as pools (`L1_rx_pool_cores`, `L1_enc_pool_cores`, `tp_cores`, `io_core`,
`worker_cores`, `--thread-pool`), and it is easy to grep only the pools and conclude a
core is free when a critical RT thread is sitting on it.

    pid=$(for p in /proc/[0-9]*; do c=$(cat $p/comm 2>/dev/null); [ "$c" = nr-softmodem ] && echo ${p#/proc/}; done | head -1)
    for t in /proc/$pid/task/*; do printf "%-18s %s\n" "$(cat $t/comm)" "$(awk '{print $39}' $t/stat)"; done | sort -k2 -n

Doubling two RT threads (priority -98) onto one isolated core produces overruns that look
exactly like a real finding and are pure artefact.

## Offline characterisation with nr_dlsim / nr_ulsim

**Stop the DU first.** Pool threads are SCHED_RR priority 97 and pinned; with nohz_full
and `sched_rt_runtime_us=-1`, a worker spinning in userspace never takes SIGKILL and the
whole process group parks in `do_exit`. Recovery is `chrt -o -p 0 <tid>`, which lets the
pending kill land.

**Run under `chrt -f 80 taskset -c <isolated core>`.** As SCHED_OTHER the sim lands on the
non-isolated cores and its p99.9 is milliseconds of OS preemption, not compute. Pinned and
RT the same measurement is deterministic - std of a few us, max within 10% of the median.
An unpinned tail is noise and says nothing about the DU.

**`OAI_RNGSEED=<n>`** (read by `randominit()`) makes runs reproducible: BER identical to
every digit. Use it to prove a refactor bit-exact - stash the change, rebuild, seeded run,
unstash, rebuild, compare BER. Without it BER scatters 0.2-1% between runs and cannot serve
as a fingerprint. Any claim of "no functional change" should carry this evidence.

Flags that have caused confusion: `-x` is layers, `-e` is MCS (`-m` is nr_ulsim's), `-q` is
the MCS table index, `-Y` is symbols per PDSCH thread, `-X` the pool cores, `-P` prints the
performance tables. `-s` is start SNR and `-S` is **end** SNR with the loop `SNR < snr1`,
so `-s29 -S29` runs zero iterations; use `-S29.1`. Omitting `-S` sweeps 50 points.

`printDistribution()` prints std/min/q1/median/q3/max, backed by `time_stats_sorted_list_t`
in `common/utils/time_meas.h` (insert/merge/reset plus `get_min`/`get_median`/`get_q1`/
`get_q3`), which the DU also uses under `TIME_STATS_ADVANCED_MODE`. There is no p99
accessor, so the question "does the 400 us case happen once a minute or once every ten
slots" is still unanswerable from it - adding `get_p99` alongside the others is the natural
fix, and would serve the DU as well as the sims. Historically these sims called
`printDistribution()` on a varArray nothing ever appended to and printed quantiles read out
of unwritten malloc; if you are on a branch predating the sorted-list rework, check that
before trusting a median.

## Predicting DL throughput

Delivered DL goodput is linear in 3GPP spectral efficiency (38.214 table 5.1.3.1-2 for the
256QAM MCS table). Calibrate the constant once from a clean, MCS-pinned run, then predict
instead of guessing. At 273 PRB / 1 layer / TDD DDDSU 2.5 ms it is 56.0 Mbps per unit SE
(validated blind to 0.1%). The constant follows from bandwidth and TDD pattern, not the
CPU, so it carries across platforms.

Use it to tell "we hit the modulation ceiling" from "something is broken": a flat rate at
exactly the predicted value for the MCS cap is saturation, not failure.

## Link adaptation

The effective DL MCS cap is `min(sched_ctrl->dl_max_mcs, max_mcs_table, bo->max_mcs)`
(`gNB_scheduler_dlsch.c`), and `sched_ctrl->dl_max_mcs` is **written from the reported CQI**
by `get_mcs_from_cqi()`. So an uncapped config hands control to the UE's CQI, and MCS
oscillation may be the UE's report moving rather than the outer loop misbehaving.

The `wide-band CQI distribution` and PRB-weighted `PDSCH MCS distribution` tables in
`nrMAC_stats.log` are the instrument: the CQI is the cap the UE asked for, the MCS is what
went out after the outer loop. CQI pinned with MCS spread means the outer loop is pulling
back; both spread means the report is moving.

Chasing a high MCS can *lower* throughput: transmitting above what the link carries causes
decode failures, the reported CQI follows those failures down, and the cap slams. Capping at
the sustainable MCS has measured +14% mean throughput and 8x better stability. Find the
ceiling by raising `dl_max_mcs` one step at a time and watching when CQI stops pinning.

**Do not read the `SNR` on the `dlsch_rounds` line as downlink.** It is
`nr_mac_get_snr(&sched_ctrl->pucch_pc)` - uplink power control. Same for both RSSI values.
The only real DL measurements are RSRP and CQI; SSB-SINR needs a Rel-16 `reportQuantity`
that the CSI-ReportConfig does not request.

## Per-slot L1 budget

Budget per TX slot is the TDD period divided by the TX slots in it - at mu=1 DDDSU 2.5 ms
that is 625 us for 4 TX slots, not the 500 us slot length. `tx_slot_stats[]` gives per-slot
`gen` and `ru` for every slot (unlike the overrun alarm, which is rate-limited and whose
sampled slots can phase-lock to the TDD pattern if the modulus shares a factor with it).

Split the measured region so an overrun says which half was slow. `gen` is grid fill;
`ru` is precoding plus fronthaul BFP compression and is roughly fixed (~180-200 us at
273 PRB / 2 antennas here). A large `ru` with a trivial `gen` is a fronthaul event, not a
compute one, and they have unrelated causes.

## Logging from RT threads

`LOG_*` ends in a synchronous `write(2)` on the calling thread unless `--log-mem` is set;
measured worst case 400 ms for one write and 22.9 s of accumulated stall. Never log from
`L1_tx_thread`, `L1_rx_thread`, `ru_thread` or a pool worker. Use `RT_LOG_DEFER` (claims a
slot lock-free, drained by a stats thread) or record-only counters. `-q` turns on the
measurement tables *and* unmasks `LOG_W` from RT paths, which is why it can appear to
destabilise scheduling.

Prefer threshold counters ("n above 200us / 500us / 1ms") over avg+max for reporting L1
timing: a mean plus a max that `start_meas()` zeroes every 16384 trials cannot show whether
a deadline is missed rarely or constantly.

## What is upstream and what is not

This file is method, and most of it applies anywhere. But several facilities it names live
on the `a72-timing-ldpc-optim` branch rather than in upstream OAI, and you will not find
them on `develop`:

- `RT_LOG_DEFER` - the deferred logging ring for RT threads
- `tx_slot_stats[]` / `rx_slot_stats[]` - the per-slot TX and RX timing tables
- `L1_rx_pool_cores`, `L1_enc_pool_cores`, `L1_tx_pipeline` - the extra L1 thread pools and
  the encode/generate pipeline
- the wide-band CQI and PRB-weighted PDSCH MCS distribution dumps

On upstream the equivalent question usually has to be answered with `print_meas_log()` and
`printDistribution()` output plus the sampling discipline above, which is weaker: those are
per-counter, not per-slot, so they cannot show that one slot of a TDD pattern is the one
missing its deadline.

## Discipline

- One variable per run. Mixing a code change with a config change costs a run and produces
  a wrong attribution.
- Prefer an experiment that can falsify the hypothesis over one that can confirm it. Capping
  MCS at the level a suspect CQI authorised, and finding the errors did *not* appear, settled
  in one run what argument had not.
- Elaborate mechanisms proposed from counter data have a poor record here. State them as
  hypotheses, name the measurement that would refute them, and run it.
- `pkill -f <pattern>` matching the DU also matches the shell running it. Scan `/proc/*/comm`
  and kill by PID.
