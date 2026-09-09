/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#include "rt_deferred_log.h"
#include "common/utils/LOG/log.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

/* Power of two: the index is masked, not divided. */
#define RT_LOG_RING 64u

/* seq == 0 means "being written". Otherwise it is claim_index + 1, so a slot reused by a
   later producer gets a different value and the consumer can tell it was overwritten. */
typedef struct {
  _Atomic uint64_t seq;
  char line[RT_LOG_LINE];
} rt_log_slot_t;

static rt_log_slot_t g_slot[RT_LOG_RING];
static _Atomic uint64_t g_claim; /* producers: next slot to hand out */
static uint64_t g_tail; /* consumer only */
static _Atomic uint64_t g_dropped; /* monotonic total, never reset */
static uint64_t g_dropped_reported; /* consumer only: how much of it we have printed */

void rt_log_defer(const char *fmt, ...)
{
  const uint64_t s = atomic_fetch_add_explicit(&g_claim, 1, memory_order_relaxed);
  rt_log_slot_t *sl = &g_slot[s & (RT_LOG_RING - 1)];
  /* Overwriting a slot the consumer has not taken yet loses that line. Counted, not
     prevented: blocking a real-time thread to preserve a diagnostic is the wrong trade. */
  if (atomic_exchange_explicit(&sl->seq, 0, memory_order_relaxed) != 0)
    atomic_fetch_add_explicit(&g_dropped, 1, memory_order_relaxed);
  va_list args;
  va_start(args, fmt);
  vsnprintf(sl->line, sizeof(sl->line), fmt, args);
  va_end(args);
  atomic_store_explicit(&sl->seq, s + 1, memory_order_release);
}

int rt_log_drain(void)
{
  const uint64_t head = atomic_load_explicit(&g_claim, memory_order_acquire);
  int n = 0;
  while (g_tail < head) {
    rt_log_slot_t *sl = &g_slot[g_tail & (RT_LOG_RING - 1)];
    const uint64_t want = g_tail + 1;
    char tmp[RT_LOG_LINE];
    /* Copy out, then confirm the slot still holds the line we started copying. A producer
       that laps us zeroes seq before writing and stores its own claim index after, so any
       interference makes this second load differ and we discard rather than print a torn
       line. Reading into a local is what makes that check meaningful. */
    if (atomic_load_explicit(&sl->seq, memory_order_acquire) != want) {
      g_tail++;
      continue;
    }
    memcpy(tmp, sl->line, sizeof(tmp));
    if (atomic_load_explicit(&sl->seq, memory_order_acquire) != want) {
      g_tail++;
      continue;
    }
    tmp[RT_LOG_LINE - 1] = '\0';
    /* Mark the slot consumed BEFORE emitting, so a producer reusing it does not count an
       already-emitted line as dropped. Compare-exchange, not a plain store: if a producer
       has claimed the slot since we copied, it owns the stamp and we must not clear it.
       A producer that laps between the check above and this exchange still counts a drop
       for a line we go on to emit, so the drop count is an upper bound on losses under
       contention, never an undercount. */
    uint64_t expect = want;
    atomic_compare_exchange_strong_explicit(&sl->seq, &expect, 0, memory_order_relaxed, memory_order_relaxed);
    LOG_W(HW, "%s\n", tmp);
    g_tail++;
    n++;
  }
  /* Report the delta but keep the total monotonic: a counter that resets itself cannot be
     sampled by anything else, including a test. */
  const uint64_t dropped = atomic_load_explicit(&g_dropped, memory_order_relaxed);
  if (dropped > g_dropped_reported) {
    LOG_W(HW,
          "rt_log: %llu deferred line(s) dropped, ring holds %u\n",
          (unsigned long long)(dropped - g_dropped_reported),
          RT_LOG_RING);
    g_dropped_reported = dropped;
  }
  return n;
}

uint64_t rt_log_dropped(void)
{
  return atomic_load_explicit(&g_dropped, memory_order_relaxed);
}
