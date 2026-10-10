# NPE-010: fast debug loop and board access

Companion to `README.nxp-npe010.md`.  That one covers building and running the
image; this one covers iterating on OAI code quickly once an image is on the board.

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
