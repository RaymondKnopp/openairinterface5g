/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "rt_deferred_log.h"
#include "common/utils/LOG/log.h"
#include <stdatomic.h>
#include <stdio.h>

/* Power of two: the index is masked, not divided. */
#define RT_LOG_RING 64u

typedef struct {
  _Atomic uint32_t ready;
  char line[RT_LOG_LINE];
} rt_log_slot_t;

static rt_log_slot_t g_slot[RT_LOG_RING];
static _Atomic uint64_t g_claim; /* producers: next slot to hand out */
static uint64_t g_tail; /* consumer only */
static _Atomic uint64_t g_dropped;

void rt_log_defer(const char *fmt, ...)
{
  const uint64_t s = atomic_fetch_add_explicit(&g_claim, 1, memory_order_relaxed);
  rt_log_slot_t *sl = &g_slot[s & (RT_LOG_RING - 1)];
  /* Overwriting a slot the consumer has not taken yet loses that line.  Counted, not
     prevented: blocking a real-time thread to preserve a diagnostic is the wrong trade. */
  if (atomic_exchange_explicit(&sl->ready, 0, memory_order_relaxed) != 0)
    atomic_fetch_add_explicit(&g_dropped, 1, memory_order_relaxed);
  va_list args;
  va_start(args, fmt);
  vsnprintf(sl->line, sizeof(sl->line), fmt, args);
  va_end(args);
  atomic_store_explicit(&sl->ready, 1, memory_order_release);
}

int rt_log_drain(void)
{
  const uint64_t head = atomic_load_explicit(&g_claim, memory_order_acquire);
  int n = 0;
  while (g_tail < head) {
    rt_log_slot_t *sl = &g_slot[g_tail & (RT_LOG_RING - 1)];
    if (atomic_load_explicit(&sl->ready, memory_order_acquire) == 0) {
      /* still being written, or already consumed after an overwrite: skip it */
      g_tail++;
      continue;
    }
    LOG_W(HW, "%s\n", sl->line);
    atomic_store_explicit(&sl->ready, 0, memory_order_relaxed);
    g_tail++;
    n++;
  }
  const uint64_t dropped = atomic_exchange_explicit(&g_dropped, 0, memory_order_relaxed);
  if (dropped)
    LOG_W(HW, "rt_log: %llu deferred line(s) dropped, ring holds %u\n", (unsigned long long)dropped, RT_LOG_RING);
  return n;
}

uint64_t rt_log_dropped(void)
{
  return atomic_load_explicit(&g_dropped, memory_order_relaxed);
}
