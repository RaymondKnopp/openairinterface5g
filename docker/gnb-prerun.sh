#!/bin/sh
# Run before EVERY gNB start:  /build/gnb-prerun.sh && docker compose up -d
set -e

# 1. Free the hugepages. xran runs EAL with --file-prefix wls_0, so its map files
#    are /mnt/huge/wls_0map_*, NOT rtemap_* - and rtemap_* is the only glob that
#    /etc/init.d/bbdev_la12xx removes. A crashed gNB therefore leaks all 8 GB and
#    the next run starts with 1 of 9 hugepages free.
# 2. Reset the modem. An aborted run leaves the FECA/e200 queues wedged: the
#    bbdev keeps accepting enqueues and never completes them, so the first
#    encode spins out in pmd_lcore_ldpc_enc() (TIME_OUT_POLL assert).
docker rm -f oai-du >/dev/null 2>&1 || true
rm -f /mnt/huge/*
/etc/init.d/bbdev_la12xx restart
sleep 8

# 3. Make sure dprc.2 has enough DPIOs. On DPAA2 every thread that frees an mbuf
#    needs its own QBMan software portal (DPIO). The DPL gives dprc.2 only 4; the
#    3 EAL lcores take one each, so the first non-EAL thread wins the last one and
#    every other one fails with
#        DPAA2_BUS: No software portal resource left
#        DPAA2_MEMPOOL: affine portal err: -1
#    Those threads then cannot return RX mbufs to the hardware buffer pool, xran's
#    65535-mbuf RX pool drains in ~3 s and fronthaul RX stops dead for good
#    (rx counter frozen, pps 0). 16 = one per core. Idempotent.
WANT=16
HAVE=$(restool dprc show dprc.2 | grep -c dpio)
if [ "$HAVE" -lt "$WANT" ]; then
    echo "dprc.2 has $HAVE dpio, adding $((WANT - HAVE))"
    if [ -e /sys/bus/fsl-mc/devices/dprc.2/driver ]; then
        echo dprc.2 > /sys/bus/fsl-mc/drivers/vfio-fsl-mc/unbind
        sleep 1
    fi
    i=$HAVE
    while [ "$i" -lt "$WANT" ]; do
        d=$(restool -s dpio create --container=dprc.2 \
                --channel-mode=DPIO_LOCAL_CHANNEL --num-priorities=8 2>&1 | tail -1)
        case "$d" in dpio.*) ;; *) echo "dpio create failed: $d" >&2; exit 1 ;; esac
        # created objects come up unplugged; VFIO only exposes plugged ones
        restool dprc assign dprc.2 --object="$d" --plugged=1
        i=$((i + 1))
    done
    restool dprc sync 2>/dev/null || true
    echo vfio-fsl-mc > /sys/bus/fsl-mc/devices/dprc.2/driver_override
    echo dprc.2 > /sys/bus/fsl-mc/drivers/vfio-fsl-mc/bind
    sleep 2
fi

echo "dpio in dprc.2: $(restool dprc show dprc.2 | grep -c dpio)"
grep -E 'HugePages_(Free|Total)' /proc/meminfo
