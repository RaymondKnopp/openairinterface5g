# NPE-010 (GeneVisio / LX2160A) — gNB with FHI 7.2 fronthaul + LA1200 LDPC offload

Branch **`genvisio-fhi72`**, based on `rk/a72-timing-ldpc-rx` @ `b4bb4bdd09`
(github.com/RaymondKnopp/openairinterface5g) — the same commit running on the
board, carrying the LDPC offload and the A72 receiver work.

## What was assembled here

| item | source | state |
|---|---|---|
| base branch | `rk/a72-timing-ldpc-rx` @ `b4bb4bdd09` | checked out |
| VLAN/PCP | `o-ran-nxp.patch` (a72-timing tree) | **applied**, kept as `o-ran-nxp-vlan-pcp.patch` |
| libxran | `o-du-phy/` @ `a53d8e9` | already present, byte-identical to the a72-timing copy |
| DPDK patches | `docker/dpdk-patches/nxp-lx2160a/` | 23 patches + `strip_lse.py` |
| build image | `Dockerfile.build.fhi72.ubuntu.nxp` | modified — see below |
| target image | `Dockerfile.gNB.fhi72.ubuntu.nxp` | **new**, modelled on the t2 variant |
| cross-build | `build-nxp-arm64.sh` | **new** |

### VLAN/PCP: only two of four hunks were taken

`o-ran-nxp.patch` touches three files. The two `radio/fhi_72/oaioran.c` hunks
relax `DevAssert(xran_queue_prach_length == 0)` — **obsolete on this branch**,
because the RX work already replaced those counters with `atomic_int` and
removed the asserts. Applying them fails. Only the genuinely new work was taken:

- `oran-params.h` — new `ORAN_CONFIG_VLAN_PCP` ("vlan_pcp")
- `oran-config.c` — composes `PCP|VID` into the 802.1Q TCI and writes it to
  `fh_config->cp_vlan_tag` / `up_vlan_tag`, defaulting PCP to 7 (O-RAN class).

This is needed because the DPAA2 PMD cannot offload VLAN insert/strip — DPDK
patch `0009-net-dpaa2-remove-broken-VLAN-insert-offload.patch` removes the
broken offload, and the o-du-phy fork inserts the tag in software instead.

### `npe010.patch` is superseded — do not apply it

It is the earlier LDPC/AAL work generated against w36 (`b3930e39f5`). The base
branch already contains all of it plus Raymond's three-defect decode fix
(`fbd846dd73`, `c990b63423`). Kept only for history.

### DPDK: NXP's fork, not upstream

The build uses **`github.com/nxp/dpdk` @ `la12xx-sdk4.0`** on aarch64 — the same
tree `package/genevisio/dpdk-nxp` builds for the Buildroot image and the board,
so the container matches what is validated on hardware. It needs
`github.com/nxp-qoriq/geul_common_headers` @ the same tag, cloned to
`/opt/geul_common_headers` and passed as `COMMON_HEADERS_DIR`. Meson options
mirror `dpdk-nxp.mk`: `platform=dpaa`, `enable_driver_sdk=true`,
`cpu_instruction_set=generic`. x86_64 still uses stock 24.11.4.

Patches applied, from `docker/dpdk-patches/nxp-la12xx-sdk4.0/`:

| | |
|---|---|
| 0001 | meson `common-headers-dir` option |
| 0002 | IPC instance debug |
| 0003 | la12xx CB-mode LDPC encode |
| 0004 | la12xx honour CRC attach flags |
| 0005 | lower the fslmc MC version floor to 10.33 — **new**, see below |

**Trade-off you should know about.** The other series,
`docker/dpdk-patches/nxp-lx2160a/` (23 dpaa2 fixes), was written against
*upstream* DPDK. **None of the 23 apply to the NXP fork** — verified by dry-run
against the board's tree. They fail on context divergence, not because the fixes
are already present: NXP's fork still carries, for example, the broken VLAN
insert offload that patch 0009 removes (`dpaa2_ethdev.c:61`,
`dpaa2_rxtx.c:1691`). So choosing the NXP fork means going without those fixes —
memory leaks on port close, link-not-up after stop/start, checksum-flag and
packet-type bugs, the Tx zero-length FD fix. The series is kept in the tree; if
any of them matter under fronthaul traffic they need rebasing onto NXP's dpaa2.

Note the software VLAN path is unaffected: the o-du-phy fork inserts the 802.1Q
tag itself from `cp_vlan_tag`/`up_vlan_tag`, so it does not rely on the PMD
offload either way.

### Change to the build Dockerfile

The final OAI step was `./build_oai` producing `cmake_targets/ran_build/build/`.
It now uses `cmake -B build/` — matching `Dockerfile.build.fhi72.t2.ubuntu`, so
the slim target image can copy from `/oai-ran/build/` — and adds
**`-DENABLE_LDPC_AAL=ON`**, without which there is no LDPC offload. NXP's fork provides both the
bbdev la12xx PMD and the DPAA2 fronthaul PMD from one tree, and the build fails
fast if `librte_baseband_la12xx.so` is missing.

## Building

The board has ~2 GB free and the build image is several GB, so **build on an
x86_64 workstation** and ship only the runtime image.

```sh
docker/build-nxp-arm64.sh all      # ran-base -> ran-build-fhi72-nxp -> oai-gnb-nxp
docker/build-nxp-arm64.sh ship     # save | ssh genvisioext docker load
```

It uses `buildx --platform linux/arm64` with the `qemu-aarch64` binfmt handler,
so `uname -m` returns aarch64 inside the build and the Dockerfile's existing
NXP branches run unchanged. **That is deliberate.** Those branches carry the
Cortex-A72 gcc wrapper (`-march=armv8-a -mno-outline-atomics`), the LSE-atomic
stripping of DPDK's arm meson config, the `libforce_no_lse.so` shim and the
`objdump` check that no LSE atomic reaches user code. The LX2160A is ARMv8.0-A
and **SIGILLs on LSE atomics**, so none of it may be skipped.

`Dockerfile.base.ubuntu.cross-arm64` would be far faster (real cross toolchain,
no emulation) but `uname -m` returns x86_64 under it, so every NXP branch would
be silently skipped and the result would fault on the board. Converting to a
true cross build means replacing each `uname -m` test with `TARGETARCH` and
giving meson a cross file. Not done.

Emulated compilation is slow — budget hours.

## Runtime prerequisites on the board

1. `/build/npe010-dpaa2-ports.sh all` — creates `eth2` (PTP) and binds `dprc.2`
   to `vfio-fsl-mc` for the fronthaul port (`dpni.2` -> `dpmac.6`).
2. `/build/ptp-docker/run-ptp.sh daemon` — PTP; also disciplines the clock,
   which the board needs since it has no RTC.
3. `export DPRC=dprc.2`; gNB config: `dpdk_devices = ("dpni.2")`,
   `du_addr = ("00:18:7d:3f:3c:cd")`, plus `vlan_tag` and the new `vlan_pcp`.
4. `LD_PRELOAD=/usr/local/lib/aarch64-linux-gnu/libforce_no_lse.so`.

**Open blocker:** DPDK's fslmc bus requires MC firmware >= 10.39.0 and the board
ships 10.33.0. Lowering `MC_VER_MINOR` to 33 in
`drivers/bus/fslmc/mc/fsl_dpmng.h` makes `dpni.2` probe and create the ethdev,
verified on the board; that change is **not** in this tree's DPDK patch series
yet. The durable fix is updating the board's MC firmware (Buildroot package
`qoriq-mc-binary`, `mc_lx2160a_10.39.0.itb`, flash offset `0xa00000`).

## Not yet done

- The image has never been built — `build-nxp-arm64.sh` is untested end to end.
- Nothing has run against a real O-RU.
