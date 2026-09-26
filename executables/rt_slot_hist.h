/* Threshold histograms for RT slot timing.
 *
 * Averages over seconds cannot see a stall that happens once a minute: 30 ms in
 * a 5 s window is 0.6% and vanishes into the mean.  So keep THRESHOLD COUNTERS
 * and the identity of the worst slot, never an average alone.
 *
 * Updated on RT threads: no locks, no LOG_*, no allocation.  Formatted only by
 * the 1 Hz stats thread. */
#ifndef RT_SLOT_HIST_H
#define RT_SLOT_HIST_H

#include <stdint.h>
#include <stdio.h>
#include "time_meas.h"

typedef struct {
  uint64_t n;
  uint64_t sum_ticks;
  uint64_t max_ticks;
  uint32_t max_frame, max_slot;
  uint64_t over[4];          /* > 0.5 / 1 / 5 / 10 ms */
} rt_hist_t;

static inline void rt_hist_add(rt_hist_t *h, uint64_t ticks, int frame, int slot)
{
  h->n++;
  h->sum_ticks += ticks;
  if (ticks > h->max_ticks) { h->max_ticks = ticks; h->max_frame = frame; h->max_slot = slot; }
  static uint64_t th[4];
  if (th[0] == 0) {
    const double tps_us = cpu_freq_GHz * 1000.0;
    th[0] = (uint64_t)(500.0 * tps_us);   th[1] = (uint64_t)(1000.0 * tps_us);
    th[2] = (uint64_t)(5000.0 * tps_us);  th[3] = (uint64_t)(10000.0 * tps_us);
  }
  for (int i = 0; i < 4; i++)
    if (ticks > th[i]) h->over[i]++;
}

static inline size_t rt_hist_one(char *o, size_t n, const char *tag, const rt_hist_t *h)
{
  const double tps_us = cpu_freq_GHz * 1000.0;
  if (h->n == 0 || tps_us <= 0.0)
    return snprintf(o, n, "  %-10s : no samples\n", tag);
  return snprintf(o, n,
                  "  %-10s : n=%-8llu avg=%7.1f us  max=%9.1f us @%u.%u"
                  "  >0.5ms=%llu >1ms=%llu >5ms=%llu >10ms=%llu\n",
                  tag, (unsigned long long)h->n,
                  (double)h->sum_ticks / h->n / tps_us,
                  (double)h->max_ticks / tps_us, h->max_frame, h->max_slot,
                  (unsigned long long)h->over[0], (unsigned long long)h->over[1],
                  (unsigned long long)h->over[2], (unsigned long long)h->over[3]);
}

/* Implemented in nr-ru.c, printed by the stats thread in nr-gnb.c. */
size_t ru_hist_dump(char *o, size_t n);

#endif /* RT_SLOT_HIST_H */
