# NPE-010: fast debug loop, board access, and the 4T4R findings

Companion to `README.nxp-npe010.md`.  That one covers building and running the
image; this one covers iterating on OAI code quickly once an image is on the board,
and what was learned on 2026-10-10 bringing a 4T4R cell up with a Quectel module.

## 1. Fast debug loop: overlay binaries instead of rebuilding images

Rebuilding and shipping the image takes ~15–20 min per change (QEMU on bulbul) or ~12
min (native on the GH200), plus a ~750 MB `docker load` on a board that only has ~1 GB
free on `/build`.  For OAI code changes none of that is needed: DPDK, xran and RAL do
not change, and the runtime image takes exactly nine files from the build stage:

```
/opt/oai-gnb/bin/nr-softmodem
/usr/local/lib/{liboran_fhlib_5g,libcoding,libdfts,libldpc,libldpc_aal,libldpc_orig,
               libparams_libconfig,libparams_yaml}.so
```

So build those nine incrementally and bind-mount them over the image's copies.

```
  source branch            build host (aarch64)                 NPE-010 board
  bulbul ~/knopp/genvisio  GH200, container "oai-dev"           /build/dev/{bin,lib}
  genvisio-w39   --git-->  = ran-build-fhi72-nxp:latest  --tar-> mounted read-only over
                 bundle    ninja <targets>, A72 ISA check        the image by a compose
                           strip, keep unstripped in /tmp/dbg    override; DU restarted
```

**Nothing runs on the GH200.**  It only compiles.  The DU, fronthaul, PTP and the
LA1200 are all on the board, exactly as before.

Measured: commit → binaries on the board **14 s**; DU restart ~45 s.

### One-time setup

1. **Build the images on an aarch64 host** (native, no QEMU) — `docker/dev/build-native-arm64.sh`.
   Needs an arm64 `ran-base:latest` locally; see the script header.
2. **Ship the runtime image once** and keep the previous one as a rollback:
   ```
   ssh $BOARD 'docker tag oai-gnb-nxp:latest oai-gnb-nxp:prev'
   ssh $GH 'docker save oai-gnb-nxp:latest | gzip -1' | ssh $BOARD 'gunzip | docker load'
   ```
3. **Start the build container from the *same* build image**, so the overlay matches the
   runtime image's DPDK/xran ABI:
   ```
   ssh $GH 'docker run -d --name oai-dev --entrypoint sleep ran-build-fhi72-nxp:latest infinity'
   ```
   It contains `/oai-ran` (a git checkout) with a configured, fully built `build/`.  A
   no-op `ninja` there takes 0.5 s.
4. **Board overrides** — copy `docker/dev/du-options.override.yaml` and
   `docker/dev/overlay.override.yaml` to `/build/dev/` on the board.  The first carries the
   DU command-line options (LA1200, `-q` timers); edit it there to change options.

### Use

```
BULBUL=... GH=... BOARD=... docker/dev/dev-ship.sh             # sync, build, ship, restart DU
docker/dev/dev-ship.sh --no-restart                             # ... but leave the DU running
docker/dev/dev-ship.sh --restart                                # restart with the current overlay
docker/dev/dev-ship.sh --off                                    # back to the plain image
```

It ships whatever is **committed** on `$BRANCH` in `$SRC_REPO` on `$BULBUL`.

**Check what is actually running** (the overlay is easy to forget):
```
P=$(pidof nr-softmodem | tr ' ' '\n' | while read p; do echo "$(ls /proc/$p/task | wc -l) $p"; done | sort -rn | head -1 | cut -d' ' -f2)
md5sum < /proc/$P/exe; md5sum < /build/dev/bin/nr-softmodem; cat /build/dev/COMMIT
```

### Gotchas

- **Rebuild the image, and restart `oai-dev` from it, whenever DPDK/xran/RAL or the
  Dockerfiles change.**  An overlay built against a different image is not guaranteed
  to be ABI-compatible.
- **The board's sshd gets starved while the DU runs** (RT threads).  A single SSH
  timeout is not a hang; the script retries.
- **bulbul's arm64 QEMU handler has been unregistered** since Oct 8 (no reboot), so
  `build-nxp-arm64.sh` won't run there until someone runs
  `docker run --privileged --rm tonistiigi/binfmt --install arm64`.
- **A from-scratch `ran-base` fails** at the asn1c/simde clones in `build_oai -I`.  Not
  the network or the git proxy line (both checked); unresolved.  Use the cached one.
- **`LD_PRELOAD` in the runtime image points at `/usr/local/lib/aach64-linux-gnu/...`**
  (`aach64`, missing an `r`), so `libforce_no_lse.so` has never actually been preloaded.
  Every `docker exec` prints the ld.so error.

## 2. Board SSH access survives reboots

`/` on the board is an initramfs: `/root/.ssh` and `/etc` vanish at every reboot.  Keys
now live on the persistent partition and are reinstalled by bringup:

- one public key per file in `/mnt/backup/authorized_keys.d/*.pub`
- `/mnt/backup/setup-keys.sh` rewrites `/root/.ssh/authorized_keys` from that directory
- `npe010-bringup.sh` calls it at the end (original saved as `npe010-bringup.sh.bak`)

To grant or revoke access, add or remove a `.pub` file and rerun `setup-keys.sh`.
**Until bringup has run after a reboot nothing is mounted**, so the first login after a
reboot needs the root password or the console.

## 3. What was found on 2026-10-10

### UE stuck at Msg1 after the w39 merge — fixed (`16470125b7`)

develop's `d0b0de4c93` made `rxdataF` flat (first RB at index 0) and changed the 7.2 RX
path in `oaioran.c` to decompress straight into it.  The merge kept this branch's
`oaioran.c` (for the tp-batch work) and with it the old DC-centred staging, while every
RX reader taken from develop reads flat.  Every PUSCH subcarrier landed in the wrong
bin; PRACH has its own buffer, so the cell looked alive and the UE reached Msg1.

Why it was hard to see: the simulators never go through `oaioran.c`, and MAC drops a
Msg3 NACK **silently** when its frame/slot doesn't match the scheduled Msg3.  At the
default log level, the reliable test is to count `Scheduling retransmission of Msg3`
(INFO) and `Reached msg3 max harq rounds` (WARN): both zero across many attempts means
MAC never acted on any Msg3 result.

### 4T4R throughput: one RX thread does all the UL work

2T2R had sustained 750 Mbps DL.  At 4T4R the DL collapsed above ~35 Mbps.  It is not a
merge regression (the pre-merge build broke too), not the LA1200, and not core sharing.

`L1_rx_thread` (core 2) does all UL processing serially; the dedicated RX pool
(`L1_rx_pool_cores`, cores 7,8,13,14) was ~idle.  When that thread falls behind, the
fronthaul overwrites `rxdataF` slots L1 hasn't read: PUCCH HARQ-ACKs and PUSCH become
DTX, DL BLER rises, MCS drops to 0, and closed-loop power control drives the UE to
maximum power (UL RSSI −35 → −9 dB).

| change | commit | DL breakpoint |
|---|---|---|
| none | — | ~35 Mbps |
| noise estimate only on UL slots with no PUSCH and no PUCCH | `2f96654cbf` | ~175 Mbps (clean at 150, degrades at 200) |
| + PRACH detection on the RX pool | `32c5aeb0c7` | **inconclusive**: one run failed at 150 Mbps (UE at rank 1, CQI 14, vs rank 2/CQI 15 before); RX thread only 31% busy but TX overrunning -- needs an A/B against `2f96654cbf` in one session |

- **`gNB_I0_measurements()`** made ~15 000 per-PRB `signal_energy_nodc()` calls per UL
  slot (every unused PRB × every UL symbol × 4 RX), outside every timer: ~690 µs per
  full UL slot, keeping the RX thread 53% busy with no UE.  It is an EMA (0.1), so it
  now runs only on UL slots with neither PUSCH nor PUCCH.  This assumes no
  load-dependent inter-cell interference.
- **PRACH detection** costs ~340 µs per occasion at 4 antennas.  It has no slot
  deadline of its own but delayed the PUCCH/PUSCH work behind it; it now runs on the RX
  pool and delivers its own RACH indication (MAC's RACH entry point takes `sched_lock`
  itself).
- **After moving PRACH, the RX thread was no longer the limit** (31% busy under load),
  yet RX jobs still waited ~1 ms to start and the TX thread was at 69% with TX job peaks
  of ~770 us against a 500 us slot.  RX start is gated on TX scheduling by design, so a
  late TX thread delays RX: the bottleneck may have moved to TX.
- **Still open at 200 Mbps:** PUCCH F2/F3 reception 191 µs avg / 469 µs max on the RX
  thread; the UL indication occasionally blocking up to ~1.4 ms (suspected `sched_lock`
  contention with the DL scheduler); and the TX side starting to overrun (TX job max
  ~760 µs against a 500 µs slot).

### How to measure this yourself

- Run the DU with `-q`.  Timers land in `/opt/oai-gnb/nrL1_stats.log` inside the
  container (`I0 measurements`, `PRACH RX`, `PUCCH23 reception`, `UL Indication`, `RX
  dispatch lag tail`, ...).  `L1 Rx job` covers full UL slots only, and the values are
  cumulative since start.
- **The RX overrun alarm is armed at 1000 µs, twice the 500 µs slot**, so lag can build
  up without it firing.  The reliable signals are the counts of `RX slot overwrite` and
  `PRACH job is in the past` in `docker logs oai-du`.
- **Per-thread CPU without trusting any timer:** sample fields 14+15 of
  `/proc/<pid>/task/*/stat` twice, 5 s apart (100 ticks/s).
- **Ramp DL in steps** (10, 25, 50, 100, 150, 200, ... Mbps, ~30 s each) and watch
  goodput, BLER, PUCCH/PUSCH DTX and those two counts.
