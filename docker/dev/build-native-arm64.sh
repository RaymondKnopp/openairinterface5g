#!/bin/bash
# Native arm64 equivalent of docker/build-nxp-arm64.sh build+target, for an aarch64 host
# (e.g. the GH200).  build-nxp-arm64.sh's preflight insists on a qemu-aarch64 binfmt
# handler even when none is needed, so the stages are run directly here.
#
# Needs ran-base:latest (arm64) present locally.  A from-scratch ran-base build currently
# fails at the asn1c/simde clones inside build_oai -I, so load a known-good one instead:
#   ssh bulbul 'docker save ran-base:latest | gzip -1' | gunzip | docker load
set -o pipefail
cd "$(dirname "$0")/../.."
bx() { docker buildx build --platform linux/arm64 --load "$@"; }
docker image inspect ran-base:latest >/dev/null 2>&1 || { echo "ran-base:latest missing (see header)" >&2; exit 1; }
echo "=== BUILD $(date +%T) ==="
bx -t ran-build-fhi72-nxp:latest -f docker/Dockerfile.build.fhi72.ubuntu.nxp . || exit 1
echo "=== TARGET $(date +%T) ==="
bx -t oai-gnb-nxp:latest -f docker/Dockerfile.gNB.fhi72.ubuntu.nxp . || exit 1
echo "=== DONE $(date +%T) ==="
