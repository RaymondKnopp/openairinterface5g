/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/*! \file rt_deferred_log.h
 * \brief Deferred logging for real-time threads.
 *
 * OAI's log path ends in a raw write(2) on the CALLING thread:
 *
 *     // log_output_memory(), taken unless --log-mem is set
 *     if (write(fileno(c->stream), log_buffer, len)) {};
 *
 * From a SCHED_RR prio-97 thread pinned to an isolated core that is an unbounded blocking
 * syscall in the slot path.  Cost depends entirely on how fast the consumer drains stdout:
 * microseconds to a file, but measured at mean 3 ms and worst 400 ms for one 200-byte
 * write with a slow reader.  That is the whole argument -- a bounded vsnprintf on the RT
 * thread in exchange for an unbounded write.
 *
 * The sites converted here are in ru_thread, which drives fronthaul timing, and unlike the
 * L1 overrun alarms they are NOT cpu_meas_enabled gated: print_fhi_counters() emits five
 * lines every 128 frames on every run, with or without -q.
 *
 * RT_LOG_DEFER() formats into a fixed slot in a lock-free ring and returns.  vsnprintf of
 * a couple of hundred bytes is bounded work (~1-2 us) and makes no syscall.  A non-RT
 * thread later calls rt_log_drain() to emit the lines.
 *
 * Multi-producer (several RT threads), single-consumer.  A slot is claimed with an atomic
 * fetch_add and stamped with that claim index only once written; the consumer copies the
 * line out and re-checks the stamp, so it never prints a line that a producer overwrote
 * while it was reading.  If producers lap the consumer the oldest lines are dropped and the
 * count is reported -- losing diagnostics is always preferable to stalling the DU.
 */

#ifndef RT_DEFERRED_LOG_H
#define RT_DEFERRED_LOG_H

#include <stdarg.h>
#include <stdint.h>

#define RT_LOG_LINE 240

/// Format a line into the deferred ring. Safe from a real-time thread: no syscall, no lock,
/// no allocation. Do NOT include a trailing newline; the drain adds one.
void rt_log_defer(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/// Emit everything queued. MUST be called only from a non-real-time thread, because this
/// is where the blocking write(2) happens. Returns the number of lines emitted.
int rt_log_drain(void);

/// Total lines dropped because producers outran the drain, since start. Monotonic, and an
/// UPPER bound: a producer that laps the consumer mid-drain counts a drop for a line that
/// still gets emitted, so this can exceed the true loss slightly under heavy contention.
/// It never undercounts, which is the property that matters for a diagnostic.
uint64_t rt_log_dropped(void);

#define RT_LOG_DEFER(fmt, ...) rt_log_defer(fmt, ##__VA_ARGS__)

#endif /* RT_DEFERRED_LOG_H */
