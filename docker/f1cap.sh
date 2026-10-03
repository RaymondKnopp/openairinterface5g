#!/bin/sh
# Capture F1 between this DU and the CU.  /build/f1cap.sh [watch|file]
#
# START THIS BEFORE the DU/ping, not after.
#
# F1 is ordinary IP on eth1, NOT fronthaul: dpni.2/dpmac.6 is bound to
# vfio-fsl-mc and has no netdev, so tcpdump can never see fronthaul.
#
# Two peers, and they are DIFFERENT hosts - filtering on the CU-CP alone misses
# all the user data:
#   CU-CP 172.21.6.90  F1-C, SCTP  (remote_n_address in gnb.conf)
#   CU-UP 172.21.6.91  F1-U, GTP-U UDP 2153  (learned via F1AP, NOT in gnb.conf)
#
# No taskset on this busybox, so the capture cannot be pinned from here. F1 is
# low rate and that is fine; for anything at fronthaul rate, pin it by running
# tcpdump in a container with --cpuset-cpus=14 instead, because
# isolcpus=...,2-15 + irqaffinity=0-1 leaves only cores 0-1 for unpinned work
# and saturating those two wedges the board with no kernel trace
# (nosoftlockup, nmi_watchdog=0).
set -u
CUCP=${CUCP:-172.21.6.90}
CUUP=${CUUP:-172.21.6.91}
IF=${IF:-eth1}
F="host $CUCP or host $CUUP"

case "${1:-watch}" in
watch) exec tcpdump -i "$IF" -nn -v -s 300 -X "$F" ;;
file)
    OUT=${OUT:-/build/f1-$(date -u +%Y%m%d-%H%M%S).pcap}
    echo "writing $OUT - ctrl-C to stop"
    exec tcpdump -i "$IF" -nn -s 0 -w "$OUT" "$F"
    ;;
teid)
    # TEIDs per direction, from an existing capture. GTP-U TEID is payload
    # bytes 4..7; UL here has E=0 and DL has E=1 (NR RAN Container), so read
    # them from the hex rather than trusting a fixed payload offset.
    P=${2:-/build/f1.pcap}
    echo "DL (CU-UP -> DU):"; tcpdump -nn -r "$P" -X "src $CUUP and udp port 2153" 2>/dev/null | grep -A1 "0x0010" | head -4
    echo "UL (DU -> CU-UP):"; tcpdump -nn -r "$P" -X "dst $CUUP and udp port 2153" 2>/dev/null | grep -A1 "0x0010" | head -4
    ;;
*)  echo "usage: $0 [watch|file|teid <pcap>]" >&2; exit 1 ;;
esac
