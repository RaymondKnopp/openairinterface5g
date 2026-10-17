/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE // RUSAGE_THREAD, sched_getcpu()
#endif
#include "nr_rlc_ue_manager.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/resource.h>

#include "LOG/log.h"

typedef struct {
  pthread_mutex_t lock;
  nr_rlc_ue_t     **ue_list;
  int             ue_count;
  nr_rlc_op_mode_t mode;
} nr_rlc_ue_manager_internal_t;

nr_rlc_ue_manager_t *new_nr_rlc_ue_manager(nr_rlc_op_mode_t mode)
{
  nr_rlc_ue_manager_internal_t *ret;

  ret = calloc(1, sizeof(nr_rlc_ue_manager_internal_t));
  if (ret == NULL) {
    LOG_E(RLC, "%s:%d:%s: out of memory\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  if (pthread_mutex_init(&ret->lock, NULL)) abort();
  ret->mode = mode;

  return ret;
}

bool nr_rlc_manager_rlc_is_split(nr_rlc_ue_manager_t *_m)
{
  nr_rlc_ue_manager_internal_t *m = _m;
  return m->mode == NR_RLC_OP_MODE_SPLIT_GNB;
}

int nr_rlc_manager_get_gnb_flag(nr_rlc_ue_manager_t *_m)
{
  nr_rlc_ue_manager_internal_t *m = _m;
  return m->mode != NR_RLC_OP_MODE_UE;
}

/* manager lock instrumentation, see nr_rlc_manager_lock() in nr_rlc_ue_manager.h */
#define RLC_LOCK_MAX_SITES 64
typedef struct {
  const char *file;
  const char *func;
  int line;
  bool watch; // stage-A site: getrusage() around the hold, see rl_slow[]
  _Atomic uint64_t n;
  _Atomic uint64_t wait_ns, wait_max_ns, wait_win_ns; // win: max since the last dump
  _Atomic uint64_t hold_ns, hold_max_ns, hold_win_ns;
} rlc_lock_site_t;
static rlc_lock_site_t rl_sites[RLC_LOCK_MAX_SITES];
static _Atomic int rl_nsites;
static pthread_mutex_t rl_reg_lock = PTHREAD_MUTEX_INITIALIZER;
static int rl_owner = -1; // written only by the lock holder
static uint64_t rl_t_acq;

/* Holds over RL_SLOW_NS at a watched site, with what the holding thread did meanwhile.
 * cpu_us close to hold_us: it was running (user or kernel code). cpu_us far below: it was
 * off the CPU, and nivcsw (preempted) or nvcsw / faults (blocked in the kernel) say why. */
#define RL_SLOW_NS 1000000ULL
#define RL_SLOW_N 16
typedef struct {
  int site;
  int cpu;
  uint32_t hold_us, cpu_us;
  long minflt, majflt, nvcsw, nivcsw;
} rl_slow_t;
static rl_slow_t rl_slow[RL_SLOW_N];
static _Atomic uint32_t rl_slow_n; // total recorded, ring index = n % RL_SLOW_N
static struct rusage rl_ru_acq; // written only by the lock holder

static const char *const rl_watch_funcs[] = {"nr_mac_rlc_status_ind", "nr_mac_rlc_multi_data_req"};

static inline uint64_t rl_now(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static void rl_max(_Atomic uint64_t *m, uint64_t v)
{
  uint64_t cur = *m;
  while (v > cur && !atomic_compare_exchange_weak(m, &cur, v))
    ;
}

static int rl_register(const char *file, int line, const char *func)
{
  pthread_mutex_lock(&rl_reg_lock);
  int id = -1;
  for (int i = 0; i < rl_nsites; i++)
    if (rl_sites[i].line == line && strcmp(rl_sites[i].file, file) == 0)
      id = i;
  if (id < 0 && rl_nsites < RLC_LOCK_MAX_SITES) {
    id = rl_nsites;
    const char *b = strrchr(file, '/');
    rl_sites[id].file = b ? b + 1 : file;
    rl_sites[id].func = func;
    rl_sites[id].line = line;
    for (size_t i = 0; i < sizeof(rl_watch_funcs) / sizeof(rl_watch_funcs[0]); i++)
      if (strcmp(func, rl_watch_funcs[i]) == 0)
        rl_sites[id].watch = true;
    rl_nsites = id + 1;
  }
  pthread_mutex_unlock(&rl_reg_lock);
  return id < 0 ? RLC_LOCK_MAX_SITES : id; // overflow: never counted
}

void nr_rlc_manager_lock_at(nr_rlc_ue_manager_t *_m, int *site, const char *file, int line, const char *func)
{
  nr_rlc_ue_manager_internal_t *m = _m;
  if (*site < 0)
    *site = rl_register(file, line, func);
  const uint64_t t_req = rl_now();
  if (pthread_mutex_lock(&m->lock)) {
    LOG_E(RLC, "%s:%d:%s: fatal\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }
  const uint64_t now = rl_now();
  rl_owner = *site;
  rl_t_acq = now;
  if (*site >= RLC_LOCK_MAX_SITES)
    return;
  rlc_lock_site_t *s = &rl_sites[*site];
  if (s->watch)
    getrusage(RUSAGE_THREAD, &rl_ru_acq);
  const uint64_t w = now - t_req;
  s->n++;
  s->wait_ns += w;
  rl_max(&s->wait_max_ns, w);
  rl_max(&s->wait_win_ns, w);
}

void nr_rlc_manager_unlock_at(nr_rlc_ue_manager_t *_m)
{
  nr_rlc_ue_manager_internal_t *m = _m;
  const int site = rl_owner;
  if (site >= 0 && site < RLC_LOCK_MAX_SITES) {
    const uint64_t h = rl_now() - rl_t_acq;
    rlc_lock_site_t *s = &rl_sites[site];
    s->hold_ns += h;
    rl_max(&s->hold_max_ns, h);
    rl_max(&s->hold_win_ns, h);
    if (s->watch && h > RL_SLOW_NS) {
      struct rusage ru;
      getrusage(RUSAGE_THREAD, &ru);
      const uint64_t cpu0 = rl_ru_acq.ru_utime.tv_sec * 1000000ULL + rl_ru_acq.ru_utime.tv_usec + rl_ru_acq.ru_stime.tv_sec * 1000000ULL + rl_ru_acq.ru_stime.tv_usec;
      const uint64_t cpu1 = ru.ru_utime.tv_sec * 1000000ULL + ru.ru_utime.tv_usec + ru.ru_stime.tv_sec * 1000000ULL + ru.ru_stime.tv_usec;
      rl_slow[atomic_fetch_add(&rl_slow_n, 1) % RL_SLOW_N] = (rl_slow_t){.site = site,
                                                                         .cpu = sched_getcpu(),
                                                                         .hold_us = h / 1000,
                                                                         .cpu_us = cpu1 - cpu0,
                                                                         .minflt = ru.ru_minflt - rl_ru_acq.ru_minflt,
                                                                         .majflt = ru.ru_majflt - rl_ru_acq.ru_majflt,
                                                                         .nvcsw = ru.ru_nvcsw - rl_ru_acq.ru_nvcsw,
                                                                         .nivcsw = ru.ru_nivcsw - rl_ru_acq.ru_nivcsw};
    }
  }
  rl_owner = -1;
  if (pthread_mutex_unlock(&m->lock)) {
    LOG_E(RLC, "%s:%d:%s: fatal\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }
}

/* One line per call site that took the lock; "win" maxima are since the previous call and
 * reset here, so with the 1 s MAC stats dump they cover the last second. */
size_t nr_rlc_lock_stats(char *buf, size_t len)
{
  char *p = buf, *end = buf + len;
  p += snprintf(p, end - p, "\nRLC manager lock per call site [us]: n | wait avg max win | hold avg max win\n");
  const int ns = rl_nsites;
  for (int i = 0; i < ns && p < end; i++) {
    rlc_lock_site_t *s = &rl_sites[i];
    const uint64_t n = s->n;
    if (n == 0)
      continue;
    const uint64_t ww = atomic_exchange(&s->wait_win_ns, 0);
    const uint64_t hw = atomic_exchange(&s->hold_win_ns, 0);
    p += snprintf(p, end - p, "  %-24s %5d %-30s %9llu | %7.1f %8.1f %8.1f | %7.1f %8.1f %8.1f\n",
                  s->file, s->line, s->func, (unsigned long long)n,
                  s->wait_ns / 1e3 / n, s->wait_max_ns / 1e3, ww / 1e3,
                  s->hold_ns / 1e3 / n, s->hold_max_ns / 1e3, hw / 1e3);
  }
  const uint32_t ns_slow = rl_slow_n;
  if (ns_slow && p < end) {
    p += snprintf(p, end - p, "RLC lock holds > 1 ms at stage-A sites: %u (last %d: func hold_us cpu_us minflt majflt nvcsw nivcsw core)\n", ns_slow, ns_slow < RL_SLOW_N ? (int)ns_slow : RL_SLOW_N);
    for (uint32_t k = ns_slow > RL_SLOW_N ? ns_slow - RL_SLOW_N : 0; k < ns_slow && p < end; k++) {
      const rl_slow_t *e = &rl_slow[k % RL_SLOW_N];
      p += snprintf(p, end - p, "  %-28s %6u %6u %5ld %3ld %4ld %4ld %3d\n", rl_sites[e->site].func, e->hold_us, e->cpu_us, e->minflt, e->majflt, e->nvcsw, e->nivcsw, e->cpu);
    }
  }
  return p - buf;
}

/* must be called with lock acquired */
nr_rlc_ue_t *nr_rlc_manager_get_ue(nr_rlc_ue_manager_t *_m, int ue_id)
{
  /* TODO: optimze */
  nr_rlc_ue_manager_internal_t *m = _m;
  int i;

  for (i = 0; i < m->ue_count; i++)
    if (m->ue_list[i]->ue_id == ue_id)
      return m->ue_list[i];

  LOG_D(RLC, "New UE with ID %d\n", ue_id);

  m->ue_count++;
  m->ue_list = realloc(m->ue_list, sizeof(nr_rlc_ue_t *) * m->ue_count);
  if (m->ue_list == NULL) {
    LOG_E(RLC, "%s:%d:%s: out of memory\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }
  m->ue_list[m->ue_count-1] = calloc(1, sizeof(nr_rlc_ue_t));
  if (m->ue_list[m->ue_count-1] == NULL) {
    LOG_E(RLC, "%s:%d:%s: out of memory\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  m->ue_list[m->ue_count-1]->ue_id = ue_id;

  return m->ue_list[m->ue_count-1];
}

/* must be called with lock acquired */
void nr_rlc_manager_remove_ue(nr_rlc_ue_manager_t *_m, int ue_id)
{
  nr_rlc_ue_manager_internal_t *m = _m;
  nr_rlc_ue_t *ue;
  int i;
  int j;

  for (i = 0; i < m->ue_count; i++)
    if (m->ue_list[i]->ue_id == ue_id)
      break;

  if (i == m->ue_count) {
    LOG_W(RLC, "Warning: ue %d not found\n", ue_id);
    return;
  }

  ue = m->ue_list[i];

  if (ue->srb0 != NULL) {
    /* deliver_sdu_data for srb0 is allocated, needs a free() */
    free(ue->srb0->deliver_sdu_data);
    ue->srb0->delete_entity(ue->srb0);
  }

  for (j = 0; j < 3; j++)
    if (ue->srb[j] != NULL)
      ue->srb[j]->delete_entity(ue->srb[j]);

  for (j = 0; j < MAX_DRBS_PER_UE; j++)
    if (ue->drb[j] != NULL)
      ue->drb[j]->delete_entity(ue->drb[j]);

  free(ue);

  m->ue_count--;
  if (m->ue_count == 0) {
    free(m->ue_list);
    m->ue_list = NULL;
    return;
  }

  memmove(&m->ue_list[i], &m->ue_list[i+1],
          (m->ue_count - i) * sizeof(nr_rlc_ue_t *));
  m->ue_list = realloc(m->ue_list, m->ue_count * sizeof(nr_rlc_ue_t *));
  if (m->ue_list == NULL) {
    LOG_E(RLC, "%s:%d:%s: fatal\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }
}

/* must be called with lock acquired */
void nr_rlc_ue_add_srb_rlc_entity(nr_rlc_ue_t *ue, int srb_id, nr_rlc_entity_t *entity)
{
  if (srb_id < 0 || srb_id > 3) {
    LOG_E(RLC, "%s:%d:%s: fatal, bad srb id\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  /* special case: srb0 */
  if (srb_id == 0) {
    if (ue->srb0 != NULL) {
      LOG_E(RLC, "fatal, srb0 already present\n");
      exit(1);
    }

    ue->srb0 = entity;

    return;
  }

  srb_id--;

  if (ue->srb[srb_id] != NULL) {
    LOG_E(RLC, "%s:%d:%s: fatal, srb already present\n",
          __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  ue->srb[srb_id] = entity;
}

/* must be called with lock acquired */
void nr_rlc_ue_add_drb_rlc_entity(nr_rlc_ue_t *ue, int drb_id, nr_rlc_entity_t *entity)
{
  if (drb_id < 1 || drb_id > MAX_DRBS_PER_UE) {
    LOG_E(RLC, "%s:%d:%s: fatal, bad drb id\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  drb_id--;

  if (ue->drb[drb_id] != NULL) {
    LOG_E(RLC, "%s:%d:%s: fatal, drb already present\n",
          __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  ue->drb[drb_id] = entity;
}
