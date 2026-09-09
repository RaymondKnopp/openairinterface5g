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
 * write with a slow reader.  With logging on the RT threads, a DU behind a laggy terminal
 * showed L1 Tx job maxima of 22.9 s and got through 6x fewer slots than the same build
 * with the printing moved off those threads.
 *
 * RT_LOG_DEFER() formats into a fixed slot in a lock-free ring and returns.  vsnprintf of
 * a couple of hundred bytes is bounded work (~1-2 us) and makes no syscall.  A non-RT
 * thread later calls rt_log_drain() to emit the lines.
 *
 * Multi-producer (several RT threads), single-consumer.  A slot is claimed with an atomic
 * fetch_add and marked ready only once written, so the consumer never prints a partially
 * formatted line.  If producers lap the consumer the oldest lines are dropped and the
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

/// How many lines were dropped because producers outran the drain.
uint64_t rt_log_dropped(void);

#define RT_LOG_DEFER(fmt, ...) rt_log_defer(fmt, ##__VA_ARGS__)

#endif /* RT_DEFERRED_LOG_H */
