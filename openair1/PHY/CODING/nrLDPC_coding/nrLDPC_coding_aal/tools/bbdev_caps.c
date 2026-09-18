/* Standalone bbdev capability dump.
 * Prints RAW capability flag words; decode them with decode_bbdev_caps.py against
 * the SAME machine's rte_bbdev_op.h, because a patched DPDK may renumber the bits.
 *
 * Build:  gcc -O2 bbdev_caps.c -o bbdev_caps $(pkg-config --cflags --libs libdpdk)
 * Run:    sudo ./bbdev_caps -a <PCI_BDF> --file-prefix=caps [EAL args...]
 */
#include <stdio.h>
#include <stdint.h>
#include <rte_eal.h>
#include <rte_bbdev.h>
#include <rte_bbdev_op.h>

int main(int argc, char **argv)
{
  int ret = rte_eal_init(argc, argv);
  if (ret < 0) { fprintf(stderr, "rte_eal_init failed\n"); return 1; }

  uint16_t n = rte_bbdev_count();
  printf("bbdev_count=%u\n", n);
  if (n == 0) { printf("NO BBDEV DEVICES FOUND\n"); return 0; }

  for (uint16_t id = 0; id < n; id++) {
    struct rte_bbdev_info info;
    if (rte_bbdev_info_get(id, &info) != 0) { printf("dev %u: info_get failed\n", id); continue; }

    printf("\n=== dev %u: name=%s driver=%s socket=%d ===\n",
           id, info.dev_name ? info.dev_name : "?",
           info.drv.driver_name ? info.drv.driver_name : "?", info.socket_id);
    printf("  num_queues=%u max_num_queues=%u queue_size_lim=%u\n",
           info.num_queues, info.drv.max_num_queues, info.drv.queue_size_lim);
    printf("  min_alignment=%u harq_buffer_size=%u data_endianness=%u\n",
           info.drv.min_alignment, info.drv.harq_buffer_size, info.drv.data_endianness);

    const struct rte_bbdev_op_cap *c = info.drv.capabilities;
    for (; c && c->type != RTE_BBDEV_OP_NONE; c++) {
      if (c->type == RTE_BBDEV_OP_LDPC_ENC) {
        printf("  LDPC_ENC  flags=0x%016llx  num_buffers_src=%u num_buffers_dst=%u\n",
               (unsigned long long)c->cap.ldpc_enc.capability_flags,
               c->cap.ldpc_enc.num_buffers_src, c->cap.ldpc_enc.num_buffers_dst);
      } else if (c->type == RTE_BBDEV_OP_LDPC_DEC) {
        printf("  LDPC_DEC  flags=0x%016llx  llr_size=%u llr_decimals=%u"
               " num_buffers_src=%u num_buffers_hard_out=%u num_buffers_soft_out=%u\n",
               (unsigned long long)c->cap.ldpc_dec.capability_flags,
               c->cap.ldpc_dec.llr_size, c->cap.ldpc_dec.llr_decimals,
               c->cap.ldpc_dec.num_buffers_src,
               c->cap.ldpc_dec.num_buffers_hard_out,
               c->cap.ldpc_dec.num_buffers_soft_out);
      } else {
        printf("  op_type=%d (not LDPC)\n", (int)c->type);
      }
    }
  }
  return 0;
}
