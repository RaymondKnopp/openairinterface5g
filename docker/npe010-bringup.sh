#!/bin/sh
# NPE-010 single-command bring-up. Run after EVERY boot.
#
#   mkdir -p /mnt/backup && mount /dev/mmcblk1p3 /mnt/backup && /mnt/backup/npe010-bringup.sh
#
# (that one line is the whole procedure: the script lives on the eMMC, which is
# the only thing that survives a reboot - the rootfs is a ramdisk)
#
# The mkdir is not optional. /mnt/backup is itself on the ramdisk, so it is gone
# after every boot, and mount reports the missing MOUNT POINT as
# "No such file or directory" - which reads like the device is missing when it is
# not. Only /mnt/huge survives, because the kernel cmdline creates it.
#
# Does, in dependency order:
#   1 clock      - no RTC on this board; a 1970 clock breaks docker and logs
#   2 storage    - /dev/mmcblk1p3 -> /build, /var/lib/docker, /root/genvisio
#   3 cpu        - performance governor @ 2.2 GHz (boots at 1.1 GHz/schedutil)
#   4 docker     - DOCKER_RAMDISK, daemon.json, dockerd
#   5 compose    - plugin reinstalled from the eMMC (/usr/local is on the ramdisk)
#   6 yami       - LA12xx modems, for the LDPC offload
#   7 dpaa2      - eth2 (PTP) + dprc.2 -> vfio-fsl-mc (fronthaul)
#   7b eth1      - management port: all RX on core 0, no TX pause (see do_eth1)
#   8 ptp        - native ptp4l/phc2sys on eth2; also sets the clock (no RTC)
#
#   npe010-bringup.sh          everything
#   npe010-bringup.sh status   report only, change nothing
#   npe010-bringup.sh ptp      restart PTP only
#   npe010-bringup.sh eth1     re-apply the eth1 steering only
set -u

EMMC=/dev/mmcblk1p3
DNS=172.21.3.100
# No RTC: seed a plausible time so docker and logs behave. phc2sys corrects it
# to the grandmaster within seconds of step 8. Override: DATE="..." bringup.sh
DATE="${DATE:-2026-09-17 10:00:00}"

say()  { printf '%s\n' "$*"; }
step() { printf '\n=== %s ===\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*" >&2; }

# ---------------------------------------------------------------- 1 clock
do_clock() {
    step "1/8 clock"
    if [ "$(date +%Y)" -lt 2020 ]; then
        date -s "$DATE" >/dev/null && say "  seeded: $(date -u)  (phc2sys will correct it)"
    else
        say "  already sane: $(date -u)"
    fi
}

# -------------------------------------------------------------- 2 storage
do_storage() {
    step "2/8 storage"
    mount | grep -q " /mnt/backup " || { mkdir -p /mnt/backup; mount "$EMMC" /mnt/backup; }
    for pair in "/mnt/backup:/build" "/mnt/backup/docker:/var/lib/docker" "/mnt/backup/genvisio:/root/genvisio"; do
        src=${pair%%:*}; dst=${pair##*:}
        if mount | grep -q " $dst "; then
            say "  $dst already bound"
        else
            mkdir -p "$dst" && mount --bind "$src" "$dst" && say "  $src -> $dst"
        fi
    done
    df -h /build | tail -1 | sed 's/^/  /'
}

# ------------------------------------------------------------------ 3 cpu
do_cpu() {
    step "3/8 cpu clock"
    for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        [ -w "$g" ] && echo performance > "$g" 2>/dev/null
    done
    say "  governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
    say "  freq    : $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null) kHz"
    [ "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)" = 2200000 ] \
        || warn "not at 2200000 kHz - every measurement assumes 2.2 GHz"
}

# --------------------------------------------------------------- 4 docker
do_docker() {
    step "4/8 docker"
    if docker info >/dev/null 2>&1; then say "  already running"; return 0; fi
    # /etc/default does not exist on a fresh boot, and without DOCKER_RAMDISK
    # dockerd wedges on pivot_root against the ramfs rootfs.
    mkdir -p /etc/default /etc/docker
    echo 'export DOCKER_RAMDISK=1' > /etc/default/dockerd
    # no systemd -> cgroupfs; no zfs -> overlay2; log rotation is not optional,
    # an unrotated json-file log fills the eMMC during a long run.
    cat > /etc/docker/daemon.json <<JSON
{
  "exec-opts": ["native.cgroupdriver=cgroupfs"],
  "log-driver": "json-file",
  "log-opts": { "max-size": "100m", "max-file": "3" },
  "dns": ["$DNS"],
  "storage-driver": "overlay2",
  "data-root": "/var/lib/docker"
}
JSON
    . /etc/default/dockerd
    dockerd >/var/log/dockerd.log 2>&1 &
    i=0; while ! docker info >/dev/null 2>&1 && [ $i -lt 40 ]; do i=$((i+1)); sleep 1; done
    docker info >/dev/null 2>&1 && say "  started ($(docker images -q | wc -l) images)" \
                                || warn "dockerd did not come up - see /var/log/dockerd.log"
}

# -------------------------------------------------------------- 5 compose
do_compose() {
    step "5/8 docker compose plugin"
    if docker compose version >/dev/null 2>&1; then
        say "  present: $(docker compose version)"
    elif [ -x /build/docker-compose ]; then
        mkdir -p /usr/local/lib/docker/cli-plugins
        cp /build/docker-compose /usr/local/lib/docker/cli-plugins/docker-compose
        chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
        say "  installed: $(docker compose version 2>/dev/null)"
    else
        warn "/build/docker-compose missing - compose unavailable"
    fi
}

# ----------------------------------------------------------------- 6 yami
do_yami() {
    step "6/8 yami / LA12xx modems"
    if lsmod | grep -q '^yami'; then
        say "  already loaded"
    else
        /etc/init.d/bbdev_la12xx start >/dev/null 2>&1
        sleep 8
    fi
    n=$(dmesg | grep -c "Probed OK")
    say "  modems probed: $n/2"
    say "  ipc devices  : $(ls /dev/gulipcgul* 2>/dev/null | wc -l)/2"
    [ "$n" -ge 2 ] || warn "expected 2 modem probes - LDPC offload needs both"
}

# ---------------------------------------------------------------- 7 dpaa2
do_dpaa2() {
    step "7/8 dpaa2 ports (eth2 PTP + dprc.2 fronthaul)"
    if [ -x /build/npe010-dpaa2-ports.sh ]; then
        /build/npe010-dpaa2-ports.sh all 2>&1 | sed 's/^/  /'
    else
        warn "/build/npe010-dpaa2-ports.sh missing"
    fi
}

# ----------------------------------------------------------- 7b eth1 (mgmt)
# eth1 (dpni.0, management + F1 to the CU) hashes RX flows over 16 queues, one
# per core, each serviced by that core's irq/NNN-dpio.N thread at FIFO 50
# (PREEMPT_RT). On isolated cores 2-15 the DU runs RT 97/98 threads that never
# yield, so a flow hashed to one of them is never serviced. Its queue fills, the
# MAC then sends PAUSE frames (tx pause is on by default) and the switch stops
# sending ANYTHING to the board: F1, ssh and even ping/ARP die until the DU stops.
# Which core a flow lands on is luck of the hash, so it looked like random DU
# crashes. Fix: steer all IPv4 RX to queue 0 (core 0, housekeeping) and turn TX
# pause off so one stuck queue can never freeze the whole link again.
# Busybox has no ethtool; the ptp image does.
ETHTOOL="docker run --rm --net=host --privileged --entrypoint ethtool ptp:latest"
do_eth1() {
    step "7b/8 eth1 management port (RX on core 0, no TX pause)"
    if ! docker image inspect ptp:latest >/dev/null 2>&1; then
        warn "ptp:latest image missing - no ethtool, eth1 left at defaults (DU load can freeze it)"
        return 0
    fi
    $ETHTOOL -A eth1 tx off rx on >/dev/null 2>&1
    if ! $ETHTOOL -n eth1 2>/dev/null | grep -q "Direct to queue 0"; then
        $ETHTOOL -N eth1 flow-type ip4 action 0 >/dev/null 2>&1
    fi
    tx=$($ETHTOOL -a eth1 2>/dev/null | sed -n 's/^TX:[[:space:]]*//p')
    q0=$($ETHTOOL -n eth1 2>/dev/null | grep -c "Direct to queue 0")
    say "  tx pause: ${tx:-?}   rules to queue 0: $q0"
    [ "$tx" = "off" ] && [ "$q0" -ge 1 ] || warn "eth1 steering not applied - DU load can freeze the management port"
}

# ------------------------------------------------------------------ 8 ptp
# Native linuxptp, not the container. Same settings the ptp-docker image used
# (domain 24, L2, client-only, hardware timestamping) - that one locked to
# ~10 ns RMS, so the parameters are known good; only the packaging changes.
PTP_IF="${PTP_IF:-eth2}"
PTP_DOMAIN="${PTP_DOMAIN:-24}"
PTP_CFG=/build/ptp4l.cfg

do_ptp() {
    step "8/8 PTP (native ptp4l/phc2sys on $PTP_IF)"

    PTP4L=$(command -v ptp4l); PHC2SYS=$(command -v phc2sys)
    if [ -z "$PTP4L" ] || [ -z "$PHC2SYS" ]; then
        warn "ptp4l/phc2sys not on PATH - fall back to /build/ptp-docker/run-ptp.sh daemon"
        return 1
    fi

    # Buildroot's S65ptp4l/S66phc2sys fire at boot, long before step 7 has
    # created eth2, so whatever they started is bound to nothing. Clear them
    # out before starting our own, or two ptp4l instances fight over the PHC.
    [ -x /etc/init.d/S65ptp4l ]   && /etc/init.d/S65ptp4l stop   >/dev/null 2>&1
    [ -x /etc/init.d/S66phc2sys ] && /etc/init.d/S66phc2sys stop >/dev/null 2>&1
    killall ptp4l phc2sys >/dev/null 2>&1
    sleep 1

    if ! ip link show "$PTP_IF" >/dev/null 2>&1; then
        warn "$PTP_IF does not exist - step 7 (dpaa2) has to succeed first"
        return 1
    fi

    # Written here rather than shipped so the interface name stays in one place.
    cat > "$PTP_CFG" <<CFG
[global]
domainNumber            $PTP_DOMAIN
clientOnly               1
verbose                 1
network_transport       L2
logAnnounceInterval     1
logSyncInterval         -4
time_stamping           hardware
tx_timestamp_timeout    50
[$PTP_IF]
CFG

    nohup "$PTP4L" -m -f "$PTP_CFG" >/var/log/ptp4l.log 2>&1 &
    sleep 2
    # -w waits for ptp4l to lock before stepping; no -c, so the sink is
    # CLOCK_REALTIME, which is what gives this RTC-less board a real clock.
    nohup "$PHC2SYS" -s "$PTP_IF" -w -n "$PTP_DOMAIN" -m -R 8 >/var/log/phc2sys.log 2>&1 &

    i=0
    while [ $i -lt 30 ]; do
        grep -q "rms" /var/log/phc2sys.log 2>/dev/null && break
        i=$((i+1)); sleep 1
    done

    say "  ptp4l   : $(pidof ptp4l   >/dev/null && echo running || echo DOWN)  (/var/log/ptp4l.log)"
    say "  phc2sys : $(pidof phc2sys >/dev/null && echo running || echo DOWN)  (/var/log/phc2sys.log)"
    tail -2 /var/log/ptp4l.log   2>/dev/null | sed 's/^/    /'
    tail -2 /var/log/phc2sys.log 2>/dev/null | sed 's/^/    /'
    say "  clock now: $(date -u)"
    pidof ptp4l >/dev/null && pidof phc2sys >/dev/null \
        || warn "PTP did not come up - without it the O-RU and DU do not share time"
}

# --------------------------------------------------------------- summary
summary() {
    step "summary"
    printf '  MC firmware : %s\n' "$(dmesg | sed -n 's/.*MC firmware version: //p' | head -1)"
    printf '  cpu         : %s kHz / %s\n' \
        "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)" \
        "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
    printf '  docker      : %s\n' "$(docker info >/dev/null 2>&1 && echo up || echo DOWN)"
    printf '  compose     : %s\n' "$(docker compose version 2>/dev/null || echo MISSING)"
    printf '  yami        : %s modem probes\n' "$(dmesg | grep -c 'Probed OK')"
    for i in 3 4 5 6; do
        ep=$(restool dpmac info dpmac.$i 2>/dev/null | sed -n 's/^endpoint: //p' | cut -d, -f1)
        case "$ep" in dpni.*) ;; *) ep="<none>" ;; esac
        nd=""; [ "${ep#dpni.}" != "$ep" ] && nd=$(ls /sys/bus/fsl-mc/devices/$ep/net/ 2>/dev/null | head -1)
        printf '  dpmac.%-2s    : %-10s %s\n' "$i" "$ep" "${nd:-}"
    done
    printf '  vfio        : %s\n' "$(ls /dev/vfio/ 2>/dev/null | grep -v vfio | tr '\n' ' ')"
    printf '  ptp         : ptp4l %s, phc2sys %s\n' \
        "$(pidof ptp4l   >/dev/null && echo up || echo DOWN)" \
        "$(pidof phc2sys >/dev/null && echo up || echo DOWN)"
    printf '  clock       : %s\n' "$(date -u)"
    say ""
    say "  next: cd /build/docker-compose-ran && docker compose up"
}

case "${1:-all}" in
status) summary ;;
ptp)    do_ptp ;;
all)    do_clock; do_storage; do_cpu; do_docker; do_compose; do_yami; do_dpaa2; do_eth1; do_ptp; summary ;;
eth1)   do_eth1 ;;
*)      echo "usage: $0 [all|status|ptp|eth1]" >&2; exit 1 ;;
esac

# reinstall SSH keys (/root is on the initramfs)
/mnt/backup/setup-keys.sh
