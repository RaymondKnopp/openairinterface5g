/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

#ifndef _NR_RLC_UE_MANAGER_H_
#define _NR_RLC_UE_MANAGER_H_
#include <stddef.h>
#include "common/platform_types.h"
#include "nr_rlc_entity.h"
#include "common/platform_constants.h"
#include "common/ngran_types.h"

typedef void nr_rlc_ue_manager_t;

typedef enum {
  NR_RLC_OP_MODE_SPLIT_GNB,
  NR_RLC_OP_MODE_MONO_GNB,
  NR_RLC_OP_MODE_UE,
} nr_rlc_op_mode_t;

typedef void (*rlf_handler_t)(int rnti);

typedef struct nr_rlc_ue_t {
  int ue_id;
  nr_rlc_entity_t *srb0;
  nr_rlc_entity_t *srb[3];
  nr_rlc_entity_t *drb[MAX_DRBS_PER_UE];
  nr_lcid_rb_t lcid2rb[32];
  rlf_handler_t rlf_handler;
} nr_rlc_ue_t;

/***********************************************************************/
/* manager functions                                                   */
/***********************************************************************/

nr_rlc_ue_manager_t *new_nr_rlc_ue_manager(nr_rlc_op_mode_t mode);

bool nr_rlc_manager_rlc_is_split(nr_rlc_ue_manager_t *_m);
int nr_rlc_manager_get_gnb_flag(nr_rlc_ue_manager_t *m);

/* The manager lock is instrumented per call site, like NR_SCHED_LOCK(): each site
 * registers once (file:line) and accumulates its wait for the lock and how long it then
 * held it; nr_rlc_lock_stats() formats the table (the gNB MAC stats thread appends it to
 * nrMAC_stats.log).  One manager lock per process is assumed, as gNB and UE have. */
void nr_rlc_manager_lock_at(nr_rlc_ue_manager_t *m, int *site, const char *file, int line, const char *func);
void nr_rlc_manager_unlock_at(nr_rlc_ue_manager_t *m);
size_t nr_rlc_lock_stats(char *buf, size_t len);
#define nr_rlc_manager_lock(m)                                                \
  do {                                                                        \
    static int nr_rlc_lock_site_ = -1;                                        \
    nr_rlc_manager_lock_at(m, &nr_rlc_lock_site_, __FILE__, __LINE__, __func__); \
  } while (0)
#define nr_rlc_manager_unlock(m) nr_rlc_manager_unlock_at(m)

nr_rlc_ue_t *nr_rlc_manager_get_ue(nr_rlc_ue_manager_t *m, int ue_id);
void nr_rlc_manager_remove_ue(nr_rlc_ue_manager_t *m, int ue_id);

/***********************************************************************/
/* ue functions                                                        */
/***********************************************************************/

void nr_rlc_ue_add_srb_rlc_entity(nr_rlc_ue_t *ue, int srb_id, nr_rlc_entity_t *entity);
void nr_rlc_ue_add_drb_rlc_entity(nr_rlc_ue_t *ue, int drb_id, nr_rlc_entity_t *entity);

#endif /* _NR_RLC_UE_MANAGER_H_ */
