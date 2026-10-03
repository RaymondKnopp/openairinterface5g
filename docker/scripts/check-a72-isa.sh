#!/bin/bash
# Fail if any given ELF contains an instruction the Cortex-A72 cannot execute.
#
# The LX2160A is ARMv8.0-A (+simd+crypto). Anything from a later revision traps
# with SIGILL at runtime. This has bitten twice:
#   - LSE atomics (FEAT_LSE, v8.1) from DPDK's arm meson config
#   - FEAT_CSSC (v8.9) `smin w4, w3, #1` in OAI, because /proc/cpuinfo in an
#     emulated aarch64 container has no "CPU part" line, so OAI's CMakeLists
#     falls through to -mcpu=native -march=native and GCC targets the emulated
#     core rather than the A72.
#
# Usage: check-a72-isa.sh <file|dir> [...]
set -uo pipefail

# Scalar GPR forms only for the CSSC arithmetic ops: the same mnemonics exist as
# ARMv8.0 NEON instructions on vector registers (smin v0.4s, ...), which are fine.
CSSC='[[:space:]](smin|smax|umin|umax|abs|ctz|cnt)[[:space:]]+[wx][0-9]'
# LSE atomics. Excluded inside the __aarch64_* outline stubs, which are
# HWCAP-guarded and therefore safe.
LSE='[[:space:]](cas|casa|casl|casal|casb|cash|swp|swpa|swpl|swpal|ldadd|ldadda|ldaddl|ldaddal|ldclr|ldclra|ldset|ldseta|ldeor|ldeora)[[:space:]]'
# Assorted post-v8.0 families.
MISC='[[:space:]](ldapr|ldaprb|ldaprh|fjcvtzs|sdot|udot|bfdot|bfmmla|smmla|ummla|sqrdmlah|sqrdmlsh|ptrue|whilelo|fmlal|fmlal2|irg|gmi|st2g|stzg)[[:space:]]'

command -v objdump >/dev/null 2>&1 || {
    echo "ERROR: objdump not found - cannot verify ISA. Refusing to report a pass." >&2
    exit 2
}
# The disassembler must be new enough to decode the encodings we are hunting.
# binutils < 2.39 does not know FEAT_CSSC and will silently print something else,
# which is exactly how a v8.9 smin reached the board unnoticed.
objdump --version | head -1

rc=0
scan_one() {
    local f="$1" out
    head -c4 "$f" 2>/dev/null | grep -q ELF || return 0
    out=$(objdump -d "$f" 2>/dev/null | awk -v cssc="$CSSC" -v lse="$LSE" -v misc="$MISC" '
        /^[[:xdigit:]]+ <__aarch64_(cas|ldadd|swp|ldclr|ldeor|ldset)[^>]*>:/ { stub=1; next }
        /^[[:xdigit:]]+ <[^>]+>:/ { stub=0; sym=$2 }
        {
          if ($0 ~ cssc) { print "  CSSC(v8.9) " sym " " $0; next }
          if (!stub && $0 ~ lse) { print "  LSE(v8.1)  " sym " " $0; next }
          if ($0 ~ misc) { print "  post-v8.0  " sym " " $0 }
        }' | head -5)
    if [ -n "$out" ]; then
        echo "FAIL $(basename "$f")"; echo "$out"; rc=1
    fi
}

for target in "$@"; do
    if [ -d "$target" ]; then
        while IFS= read -r f; do scan_one "$f"; done \
            < <(find "$target" -type f \( -name '*.so' -o -name '*.so.*' -o -perm -u+x \) 2>/dev/null)
    elif [ -e "$target" ]; then
        scan_one "$target"
    fi
done

if [ "$rc" -ne 0 ]; then
    echo "ERROR: instructions above are not available on Cortex-A72 (ARMv8.0-A) - would SIGILL on the board" >&2
    exit 1
fi
echo "OK: no post-ARMv8.0 instructions found in: $*"
