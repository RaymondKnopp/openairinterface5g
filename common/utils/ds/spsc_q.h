/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef SPSC_Q_H_
#define SPSC_Q_H_

#include <stdint.h>
#include <stdbool.h>

#if defined(__cplusplus)
#include <atomic>
// use atomic_size_t to help interworking with C++
using std::atomic_size_t;
#else
#include <stdatomic.h>
#endif

typedef struct spsc_q {
  uint8_t *buf;
  size_t elsiz;
  size_t cnt;

  atomic_size_t write_idx;
  atomic_size_t read_idx;
} spsc_q_t;

bool spsc_q_alloc(spsc_q_t *rbn, size_t cnt, size_t elsiz);
void spsc_q_free(spsc_q_t *rb);

bool spsc_q_put(spsc_q_t *rb, const void *src, size_t elsiz);

/* Current occupancy, for instrumentation only.  Racy by construction (a single
 * producer and consumer move the indices independently), which is fine for a
 * high-water mark: it can under-report, never invent depth that was not there. */
static inline size_t spsc_q_count(const spsc_q_t *rb)
{
  const size_t w = atomic_load_explicit((atomic_size_t *)&rb->write_idx, memory_order_relaxed);
  const size_t r = atomic_load_explicit((atomic_size_t *)&rb->read_idx, memory_order_relaxed);
  /* indices are modulo rb->cnt, so this must wrap too -- a plain w-r underflows */
  return (w + rb->cnt - r) % rb->cnt;
}
/* alloc() adds one slot so full is distinguishable from empty, so usable = cnt-1 */
static inline size_t spsc_q_capacity(const spsc_q_t *rb) { return rb->cnt ? rb->cnt - 1 : 0; }
bool spsc_q_get(spsc_q_t *rb, void *dest, size_t elsiz);

typedef bool (*pred)(const void *data, void *user);
bool spsc_q_get_if(spsc_q_t *rb, pred p, void *user, void *dest, size_t elsiz);
int spsc_q_drop_while(spsc_q_t *rb, pred p, void *user);
int spsc_q_get_while(spsc_q_t *rb, pred p, void *user, void *dest, size_t elsiz, size_t max_len);

#endif /* SPSC_Q_H_ */
