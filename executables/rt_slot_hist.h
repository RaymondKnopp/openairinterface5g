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
  uint64_t sum_ns;
  uint64_t max_ns;
  uint32_t max_frame, max_slot;
  uint64_t over[4];          /* > 0.5 / 1 / 5 / 10 ms */
} rt_hist_t;

/* Nanoseconds from clock_gettime_oai(), not TSC ticks: upstream dropped the
 * calibrated-frequency helper, and a wall clock also removes the assumption that
 * the counter ticks at one rate across the A78 and A55 clusters. */
static inline void rt_hist_add(rt_hist_t *h, uint64_t ns, int frame, int slot)
{
  h->n++;
  h->sum_ns += ns;
  if (ns > h->max_ns) { h->max_ns = ns; h->max_frame = frame; h->max_slot = slot; }
  static const uint64_t th[4] = {500000ull, 1000000ull, 5000000ull, 10000000ull};
  for (int i = 0; i < 4; i++)
    if (ns > th[i]) h->over[i]++;
}

static inline size_t rt_hist_one(char *o, size_t n, const char *tag, const rt_hist_t *h)
{
  if (h->n == 0)
    return snprintf(o, n, "  %-10s : no samples\n", tag);
  return snprintf(o, n,
                  "  %-10s : n=%-8llu avg=%7.1f us  max=%9.1f us @%u.%u"
                  "  >0.5ms=%llu >1ms=%llu >5ms=%llu >10ms=%llu\n",
                  tag, (unsigned long long)h->n,
                  (double)h->sum_ns / h->n / 1000.0,
                  (double)h->max_ns / 1000.0, h->max_frame, h->max_slot,
                  (unsigned long long)h->over[0], (unsigned long long)h->over[1],
                  (unsigned long long)h->over[2], (unsigned long long)h->over[3]);
}

/* Implemented in nr-ru.c, printed by the stats thread in nr-gnb.c. */
size_t ru_hist_dump(char *o, size_t n);

#endif /* RT_SLOT_HIST_H */
