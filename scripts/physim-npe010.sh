#!/bin/bash
# Build and run nr_dlsim / nr_ulsim on a GeneVisio NPE-010 (aarch64, LX2160A).
#
# Runs everything inside the oai-physim:npe010 toolchain container, because the
# board's Ubuntu 18.04 rootfs has gcc 7.5 / no cmake and OAI needs C++17 and
# cmake >= 3.19. Binaries built in the container link against its glibc, so they
# are also RUN in the container -- do not copy them onto the bare board.
#
#   ./physim-npe010.sh build          # configure + build $TARGETS
#   ./physim-npe010.sh build-aal      # same, with LDPC offload to LA12xx FECA
#   ./physim-npe010.sh shell          # interactive shell in the build env
#   ./physim-npe010.sh run <args...>  # run a binary, with baseband devices mapped
#
set -euo pipefail

IMAGE=oai-physim:npe010
SRC=${SRC:-$(cd "$(dirname "$0")/.." && pwd)}
# The board's / has little room; BUILD_DIR should live on a partition with space.
# mmcblk1p3 ("backup", ~4.6 GB, empty by default) is a good candidate:
#   mkdir -p /build && mount /dev/mmcblk1p3 /build
BUILD_DIR=${BUILD_DIR:-/build/oai-physim}
# Default to the *ch*sim targets: nr_ulschsim/nr_dlschsim exercise the same
# UL-SCH/DL-SCH coding chain (and support the AAL offload -- see the AAL
# CMakeLists add_dependencies list) but link only asn1_*_hdrs. nr_ulsim/nr_dlsim
# additionally pull L2_NR -> nr_rrc -> the *compiled* asn1_nr_rrc/asn1_lte_rrc
# plus F1AP/NGAP/S1AP/X2AP/E1AP, which is the bulk of an OAI build.
#
# IMPORTANT: the *ch*sims cannot test HARQ. Both hardcode rvidx=0 and have no
# retransmission loop, so they only ever exercise the first transmission.
# nr_ulsim/nr_dlsim have a real HARQ loop and a "-v <max rounds>" option, so
# testing HARQ combining on FECA requires them (and the full ASN.1 build):
#   TARGETS="nr_dlsim nr_ulsim" ./physim-npe010.sh build-aal
TARGETS=${TARGETS:-"nr_dlschsim nr_ulschsim"}
JOBS=${JOBS:-$(nproc)}

# NXP DPDK 19.11 from the board rootfs, needed only for the AAL/bbdev offload.
DPDK_HOST_INC=${DPDK_HOST_INC:-/usr/local/include/dpdk}
DPDK_HOST_LIB=${DPDK_HOST_LIB:-/usr/local/lib}

# Mount those OUTSIDE /usr/local inside the container. The board's
# /usr/local/lib carries OpenSSL 1.1.1 (libcrypto.so.1.1 plus an openssl.pc
# reading "Version: 1.1.1d"), and bind-mounting it over the container's
# /usr/local shadows Ubuntu 22.04's OpenSSL 3.0.2 -- which openair3/SECU needs
# (EVP_MAC_fetch, EVP_MAC_CTX_new, OSSL_PARAM_construct_*, ...). Keeping DPDK
# under /opt leaves the container's own /usr/local untouched.
DPDK_INC=/opt/nxp-dpdk/include/dpdk
DPDK_LIB=/opt/nxp-dpdk/lib

# Relocating the mount is not enough on its own: the board's /usr/local/lib
# mixes the DPDK archives with OpenSSL 1.1.1 (libcrypto.so, libssl.so), and any
# directory handed to the linker via -L is searched before the system paths --
# so -lcrypto would still resolve to 1.1.1 and openair3/SECU would fail to link
# even though pkg-config reports OpenSSL 3. Stage a directory holding *only*
# the DPDK archives (121 x librte_*.a + libdpdk.a, ~15 MB) and use that as
# DPDK_LIB_DIR. libdpdk.a is a GNU ld GROUP() script that references nothing
# but librte_*.a, so this set is complete.
DPDK_LIB_STAGE=${DPDK_LIB_STAGE:-/build/nxp-dpdk-lib}

stage_dpdk_libs() {
  mkdir -p "$DPDK_LIB_STAGE"
  if [ ! -e "$DPDK_LIB_STAGE/libdpdk.a" ] \
     || [ "$DPDK_HOST_LIB/libdpdk.a" -nt "$DPDK_LIB_STAGE/libdpdk.a" ]; then
    echo "staging DPDK archives into $DPDK_LIB_STAGE"
    cp -f "$DPDK_HOST_LIB"/librte_*.a "$DPDK_HOST_LIB"/libdpdk.a "$DPDK_LIB_STAGE"/
  fi
}

dpdk_mounts=(-v "$DPDK_HOST_INC":"$DPDK_INC":ro -v "$DPDK_LIB_STAGE":"$DPDK_LIB":ro)

mkdir -p "$BUILD_DIR"

common_docker_args=(
  --rm
  -v "$SRC":/oai
  -v "$BUILD_DIR":/build
  -w /build
)

# Devices the LA12xx bbdev PMD and DPDK need. yami must already be loaded and the
# modem armed on the host; the container only talks to the existing device nodes.
# DPDK is linked statically into the executables, so nothing from the board's
# DPDK install is needed at run time -- only the device nodes and hugepages.
run_docker_args=(
  "${common_docker_args[@]}"
  -v /dev/hugepages:/dev/hugepages
  -v /var/run/dpdk:/var/run/dpdk
  # /dev/mem: the bbdev_la12xx PMD mmaps the modem's PCI space through it
  # ("bbdev_la12xx: Error: Cannot open /dev/mem" without it), which also needs
  # CAP_SYS_RAWIO. The gul* nodes are yami's IPC / CLI / TTI channels.
  --device /dev/mem
  --device /dev/gulipcgul0 --device /dev/gulipcgul1 --device /dev/gulmodinfo
  --device /dev/gul_cli_dev0 --device /dev/gul_cli_dev1
  --cap-add IPC_LOCK --cap-add SYS_ADMIN --cap-add SYS_RAWIO
)

configure() {
  local extra=("$@")
  docker run "${common_docker_args[@]}" "$IMAGE" \
    cmake -GNinja /oai \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DAUTO_DOWNLOAD_ASN1C=ON \
      "${extra[@]}"
}

case "${1:-build}" in
  build)
    configure
    docker run "${common_docker_args[@]}" "$IMAGE" ninja -j"$JOBS" $TARGETS
    ;;
  build-aal)
    stage_dpdk_libs
    # DPDK headers/libs must be visible at configure time, so mount them here too.
    docker run "${common_docker_args[@]}" "${dpdk_mounts[@]}" "$IMAGE" \
      cmake -GNinja /oai \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DAUTO_DOWNLOAD_ASN1C=ON \
        -DENABLE_LDPC_AAL=ON \
        -DDPDK_INCLUDE_DIR="$DPDK_INC" \
        -DDPDK_LIB_DIR="$DPDK_LIB"
    docker run "${common_docker_args[@]}" "${dpdk_mounts[@]}" "$IMAGE" \
      ninja -j"$JOBS" $TARGETS ldpc_aal
    ;;
  shell)
    docker run -it "${run_docker_args[@]}" "$IMAGE" bash
    ;;
  run)
    shift
    # Only allocate a TTY when we actually have one, so this works under
    # ssh/cron/CI ("the input device is not a TTY") as well as interactively.
    tty_args=()
    [ -t 0 ] && tty_args=(-it)
    docker run "${tty_args[@]}" "${run_docker_args[@]}" "$IMAGE" "$@"
    ;;
  *)
    echo "usage: $0 {build|build-aal|shell|run <args...>}" >&2
    exit 2
    ;;
esac
