#!/bin/bash
# Fast debug loop for the Genvisio DU: build the OAI parts incrementally on the GH200
# inside oai-dev (same image the board's runtime image was built from), then overlay the
# nine binaries on the board via a compose override.  No image rebuild, no image ship.
#
#   dev-ship.sh            sync genvisio-w39 from bulbul, build, ship, restart DU
#   dev-ship.sh --no-restart   ... but leave the DU as it is
#   dev-ship.sh --restart  restart the DU with the overlay already on the board
#   dev-ship.sh --off      drop the overlay: restart the DU on the plain image
set -eo pipefail
BULBUL=${BULBUL:-bulbul}            # host holding the source branch
GH=${GH:-knopp@172.21.19.111}       # native arm64 build host running the oai-dev container
BOARD=${BOARD:-root@172.21.17.21}   # the NPE-010
SRC_REPO=${SRC_REPO:-~/knopp/genvisio}
BRANCH=${BRANCH:-genvisio-w39}
FILES="build/nr-softmodem build/liboran_fhlib_5g.so build/libcoding.so build/libparams_libconfig.so build/libdfts.so build/libldpc.so build/libldpc_aal.so build/libldpc_orig.so build/libparams_yaml.so"
COMPOSE="-f docker-compose.yaml -f /build/dev/du-options.override.yaml"
t0=$(date +%s); say() { echo "[$(( $(date +%s) - t0 ))s] $*"; }
bssh() {  # board ssh with retries: a running DU can starve sshd for a while
  local i; for i in 1 2 3 4 5; do
    ssh -o BatchMode=yes -o ConnectTimeout=25 -o ServerAliveInterval=10 $BOARD "$@" && return 0
    [ $? -ne 255 ] && return 1; echo "  (board ssh retry $i)" >&2; sleep 5
  done; return 1; }

restart_du() {  # $1 = extra compose file or empty
  bssh "cd /build/docker-compose-ran && \
    docker compose $COMPOSE ${1:+-f $1} down >/dev/null 2>&1; /build/gnb-prerun.sh >/dev/null 2>&1; \
    docker compose $COMPOSE ${1:+-f $1} up -d 2>&1 | tail -1; \
    i=0; until docker logs oai-du 2>&1 | grep -q 'received F1 Setup Response' || [ \$i -ge 30 ]; do sleep 2; i=\$((i+1)); done; \
    docker logs oai-du 2>&1 | grep -c 'received F1 Setup Response' | sed 's/^/F1 up: /'"
}

if [ "$1" = "--off" ]; then say "restarting DU on the plain image (no overlay)"; restart_du ""; exit 0; fi
if [ "$1" = "--restart" ]; then say "restarting DU with the current overlay"; restart_du /build/dev/overlay.override.yaml; exit 0; fi

# 1. sync source: bulbul branch -> GH200 oai-dev
base=$(ssh -o BatchMode=yes $GH "docker exec oai-dev git -C /oai-ran rev-parse HEAD")
tip=$(ssh -o BatchMode=yes $BULBUL "git -C $SRC_REPO rev-parse $BRANCH")
if [ "$base" != "$tip" ]; then
  say "sync ${base:0:10} -> ${tip:0:10}"
  ssh -o BatchMode=yes $BULBUL "cd $SRC_REPO && git bundle create - $base..$BRANCH 2>/dev/null" \
    | ssh -o BatchMode=yes $GH "docker exec -i oai-dev sh -c 'cat > /tmp/sync.bundle && cd /oai-ran && git fetch -q /tmp/sync.bundle $BRANCH && git checkout -q -f FETCH_HEAD'"
else say "source already at ${tip:0:10}"; fi

# 2. incremental build + A72 ISA check on just the shipped files
say "building"
ssh -o BatchMode=yes $GH "docker exec oai-dev bash -c 'cd /oai-ran && ninja -C build nr-softmodem oran_fhlib_5g ldpc_aal ldpc coding params_libconfig params_yaml dfts 2>&1 | grep -E \"error|FAILED|warning: implicit|^\[[0-9]+/[0-9]+\] Linking\" | tail -15; test \${PIPESTATUS[0]} -eq 0' && \
  docker exec oai-dev /usr/local/bin/check-a72-isa.sh $FILES | tail -1"

# 3. stage stripped copies (keep unstripped for gdb under /tmp/dbg/<commit>) and ship
say "shipping"
ssh -o BatchMode=yes $GH "docker exec oai-dev bash -c 'rm -rf /tmp/ov && mkdir -p /tmp/ov/bin /tmp/ov/lib /tmp/dbg/${tip:0:10} && cd /oai-ran && cp $FILES /tmp/dbg/${tip:0:10}/ && cp build/nr-softmodem /tmp/ov/bin/ && cp build/*.so /tmp/ov/lib/ 2>/dev/null; cd /tmp/ov && for f in bin/* lib/*; do case \" $FILES \" in *\" build/\$(basename \$f) \"*) strip --strip-debug \$f;; *) rm -f \$f;; esac; done; echo ${tip} > COMMIT; tar czf - .'" \
  | bssh "rm -rf /build/dev/bin /build/dev/lib && mkdir -p /build/dev && tar xmzf - -C /build/dev && du -sh /build/dev | sed 's/^/overlay size: /'; echo commit \$(cat /build/dev/COMMIT | cut -c1-10)"

# 4. restart with the overlay
if [ "$1" != "--no-restart" ]; then say "restarting DU with overlay"; restart_du /build/dev/overlay.override.yaml; fi
say "done"
