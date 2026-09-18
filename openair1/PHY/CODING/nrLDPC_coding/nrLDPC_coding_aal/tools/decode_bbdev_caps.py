#!/usr/bin/env python3
"""Decode bbdev capability flag words against THIS machine's rte_bbdev_op.h.

Bit positions are read from the header you pass, never assumed, because a patched
DPDK (e.g. the T2 tree) may renumber them. Names that share a bit position are all
listed -- the encode and decode flag enums overlap numerically.

Usage: decode_bbdev_caps.py <rte_bbdev_op.h> LDPC_ENC 0x2b LDPC_DEC 0xf87f
"""
import re, sys

def bit_names(hdr):
    src = open(hdr).read()
    by_bit = {}
    for m in re.finditer(r'\b(RTE_BBDEV_LDPC_\w+)\s*=\s*\(?\s*1(?:ULL|UL|U)?\s*<<\s*(\d+)\s*\)?', src):
        by_bit.setdefault(int(m.group(2)), []).append(m.group(1))
    return by_bit

def main():
    if len(sys.argv) < 4 or len(sys.argv) % 2 != 0:
        print(__doc__); sys.exit(1)
    by_bit = bit_names(sys.argv[1])
    if not by_bit:
        print("!! no RTE_BBDEV_LDPC_* bit enums found -- check the header path"); sys.exit(2)
    args = sys.argv[2:]
    for i in range(0, len(args), 2):
        kind, val = args[i], int(args[i + 1], 0)
        print(f"\n{kind} = 0x{val:x}")
        print("  bits SET:")
        for b in sorted(by_bit):
            if (val >> b) & 1:
                print(f"    bit {b:2d}  {' | '.join(by_bit[b])}")
        unnamed = [b for b in range(64) if (val >> b) & 1 and b not in by_bit]
        if unnamed:
            print(f"    !! set but not named in this header: bits {unnamed}")
        print("  bits CLEAR:")
        for b in sorted(by_bit):
            if not (val >> b) & 1:
                print(f"    bit {b:2d}  {' | '.join(by_bit[b])}")
    print(f"\n(bit positions read from {sys.argv[1]}, not assumed)")

main()
