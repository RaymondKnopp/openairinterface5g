#!/usr/bin/env python3
"""Strip LSE atomic config from DPDK arm/meson.build for Cortex-A72 (ARMv8.0-A)."""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else 'config/arm/meson.build'
with open(path) as f:
    text = f.read()

patched = text.replace('+lse', '').replace('+atomics', '')
patched = re.sub(r"\['RTE_ARM_FEATURE_ATOMICS',\s*1\],?\s*\n?", '', patched)

with open(path, 'w') as f:
    f.write(patched)

changes = []
if '+lse' in text or '+atomics' in text:
    changes.append('+lse/+atomics arch extensions')
if re.search(r"\['RTE_ARM_FEATURE_ATOMICS',\s*1\]", text):
    changes.append('RTE_ARM_FEATURE_ATOMICS=1')
print('{}: removed {}'.format(path, ', '.join(changes) if changes else 'nothing (already clean)'))
