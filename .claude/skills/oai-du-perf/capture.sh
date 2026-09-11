#!/bin/bash
# Capture a full nr-softmodem run: console log plus 2 Hz snapshots of the stats files.
#
# The stats files are truncate-and-rewrite once per second, so without this sampler every
# second of history is destroyed and only the final cumulative state survives.  The
# snapshots keep the cumulative diff/trials at each point in time, which is what lets an
# analyser difference two samples from the same load window into a per-interval mean.
# Reading a lifetime counter instead is the single most common way to reach a wrong
# conclusion about a DU.
#
# Everything is overridable from the environment so the same script works on any host:
#   REPO   repo root                (default: three levels up from this script)
#   BUILD  directory holding nr-softmodem and the stats files it writes
#   CONF   gNB configuration file   (no default; must be set or passed as $1)
#   POOL   --thread-pool core list
#   OUT    directory to write runs into
#
#   CONF=/path/gnb.conf POOL=10,11 ./capture.sh
set -u

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=${REPO:-$(cd "$HERE/../../.." && pwd)}
BUILD=${BUILD:-$REPO/cmake_targets/ran_build/build}
CONF=${CONF:-${1:-}}
POOL=${POOL:-}
OUT=${OUT:-$REPO/../oai-debug}

if [ -z "$CONF" ]; then
  echo "usage: CONF=<gnb.conf> [POOL=10,11] [OUT=dir] $0" >&2
  exit 2
fi
[ -r "$CONF" ]              || { echo "cannot read CONF: $CONF" >&2; exit 2; }
[ -x "$BUILD/nr-softmodem" ] || { echo "no nr-softmodem in BUILD: $BUILD" >&2; exit 2; }

# Refuse to start a second DU: two of them fight over the same isolated cores at RT
# priority and neither run means anything.
for p in /proc/[0-9]*; do
  [ "$(cat "$p/comm" 2>/dev/null)" = "nr-softmodem" ] && {
    echo "nr-softmodem already running as PID ${p#/proc/}; stop it first" >&2; exit 1; }
done

RUN=$OUT/run-$(date +%Y%m%d-%H%M%S)
mkdir -p "$RUN" || exit 1
echo "$RUN"

cd "$BUILD" || exit 1
{ echo "conf=$CONF"; echo "pool=$POOL"; echo "build=$BUILD"; echo "start=$(date -Is)";
  uname -a; echo "cmdline=$(cat /proc/cmdline)";
  echo "git=$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null) $(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null)";
} > "$RUN/meta.txt"

# Record the config verbatim.  Runs get compared weeks apart and "which conf was that?"
# is otherwise unanswerable.
cp "$CONF" "$RUN/conf" 2>/dev/null

# 2 Hz sampler: timestamped, skipping unchanged or empty reads so the stream only grows
# when something actually changed.
(
  prev_l1=""; prev_mac=""
  while :; do
    now=$(date +%s.%3N)
    printf '@@@ t=%s gnblines=%s\n' "$now" "$(wc -l < "$RUN/gnb.log" 2>/dev/null || echo 0)"
    for f in nrL1_stats.log nrMAC_stats.log; do
      [ -s "$f" ] || continue
      c=$(cat "$f" 2>/dev/null)
      [ -z "$c" ] && continue
      if [ "$f" = nrL1_stats.log ]; then
        [ "$c" = "$prev_l1" ] && continue; prev_l1=$c
      else
        [ "$c" = "$prev_mac" ] && continue; prev_mac=$c
      fi
      printf '### t=%s file=%s\n%s\n' "$now" "$f" "$c"
    done
    sleep 0.5
  done
) > "$RUN/stats-stream.txt" 2>/dev/null &
sampler=$!
echo "$sampler" > "$RUN/sampler.pid"
trap 'kill "$sampler" 2>/dev/null' EXIT

# -q sets cpu_meas_enabled.  It is not a "quiet" flag: without it nrL1_stats.log contains
# no timing tables at all.
set -- -q -O "$CONF"
[ -n "$POOL" ] && set -- "$@" --thread-pool "$POOL"
stdbuf -oL -eL ./nr-softmodem "$@" > "$RUN/gnb.log" 2>&1
rc=$?

kill "$sampler" 2>/dev/null
{ echo "end=$(date -Is)"; echo "rc=$rc"; } >> "$RUN/meta.txt"
exit "$rc"
