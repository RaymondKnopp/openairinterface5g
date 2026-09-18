<!-- SPDX-License-Identifier: CC-BY-4.0 -->

# bbdev capability probe

Dumps what a bbdev device actually advertises, so the AAL backend's
capability-driven decisions can be checked against real hardware without building
or running OAI on the target.

Flag bits are decoded against the **target machine's own** `rte_bbdev_op.h`, never
against hardcoded values: some vendors ship DPDK patches, and assuming stock bit
positions would silently misreport. (Checked for the AccelerComm T2 tree: its bit
enums are in fact identical to stock 22.11, but that is a result, not an
assumption.)

## Build on the target

    gcc -O2 bbdev_caps.c -o bbdev_caps \
        $(PKG_CONFIG_PATH=<dpdk>/lib64/pkgconfig pkg-config --cflags --libs libdpdk)

## Run

The EAL argument form matters:
* the VFIO token is a separate `--vfio-vf-token` option, not an `-a` suffix;
* shared-PMD builds need `-d <dpdk>/lib64/dpdk/pmds-*/` or `rte_bbdev_count()`
  returns 0;
* `--no-huge` does not work with `igb_uio`, which needs IOVA=PA;
* pin an lcore on the device's NUMA node.

    sudo LD_LIBRARY_PATH=<dpdk>/lib64 ./bbdev_caps bbdev \
        -l <core> -a <PCI_BDF> --file-prefix probe \
        [--vfio-vf-token <uuid>] \
        [-d <dpdk>/lib64/dpdk/pmds-*/]

Note the device name reported by bbdev follows whatever form is passed to `-a`.
OAI normalises to the full `0000:xx:yy.z` form and `strcmp`s against
`info.dev_name`, so pass the full BDF.

## Decode

    python3 decode_bbdev_caps.py <dpdk>/include/rte_bbdev_op.h \
        LDPC_ENC 0x<enc> LDPC_DEC 0x<dec>

The encode and decode flag enums overlap numerically, so every name sharing a bit
is printed; read the ones belonging to the op type in question.

## Measured reference values

Intel ACC200/VRB1 (`intel_acc200_vf`) and AccelerComm T2 (`baseband_accl_ldpc`):

| | ACC200 | T2 |
|---|---|---|
| `LDPC_ENC` flags | `0x2b` | `0x1f` |
| `LDPC_DEC` flags | `0xf87f` | `0x3a876` |
| `min_alignment` | 1 | **64** |
| `harq_buffer_size` | 0 | 524288 |
| `llr_size` / `llr_decimals` | 8 / 1 | 6 / 2 |
| `max_num_queues` | 256 | 32 |

Encode capabilities that differ:

| flag | ACC200 | T2 |
|---|---|---|
| `CRC_24A_ATTACH` (TB CRC) | no | **yes** |
| `CRC_24B_ATTACH` (per-CB CRC) | yes | yes |
| `CRC_16_ATTACH` | no | yes |
| `ENC_INTERRUPTS` | yes | no |
| `ENC_SCATTER_GATHER` | no | no |
| `ENC_CONCATENATION` | no | no |

Neither device advertises `ENC_SCATTER_GATHER` or `ENC_CONCATENATION`, so
device-side segmentation (bbdev TB mode, `code_block_mode = 0`) is unavailable for
C > 1 on both: the transport block would have to sit in a single mbuf, and DPDK
caps one mbuf at 64 KiB (`rte_pktmbuf_pool_create` takes a `uint16_t
data_room_size`; `RTE_BBDEV_LDPC_E_MAX_MBUF` is 64000). Per-code-block mode is
therefore forced by the hardware, not an OAI design choice.

Decode capabilities that differ:

| flag | ACC200 | T2 |
|---|---|---|
| `CRC_TYPE_24A_CHECK` | yes | no |
| `CRC_TYPE_16_CHECK` | yes | no |
| `DEC_INTERRUPTS` | yes | no |
| `HARQ_6BIT_COMPRESSION` | yes | no |
| `INTERNAL_HARQ_MEMORY_IN/OUT` | **no** | **yes** |

`LLR_COMPRESSION` and `DEC_SCATTER_GATHER` are advertised by both and unused by
OAI. The ACC200 has no internal HARQ memory, so its HARQ combined buffers cross
PCIe every round, while the T2 keeps them on card.
