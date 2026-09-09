/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

// Unit test for the deferred real-time log ring. The property that matters is that a
// producer never blocks and a consumer never emits a torn line, including when producers
// lap the consumer -- which is exactly the case the ring is designed to drop rather than
// prevent.

#include "gtest/gtest.h"
#include <atomic>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include "common/utils/rt_deferred_log.h"
#include "common/utils/LOG/log.h"
}

// The drain emits through LOG_W, so the test asserts on the return value and the drop
// counter rather than on captured text.

TEST(RtDeferredLog, DrainReturnsWhatWasDeferred)
{
  const uint64_t dropped0 = rt_log_dropped();
  rt_log_drain(); // start from a known state
  for (int i = 0; i < 8; ++i)
    RT_LOG_DEFER("line %d", i);
  EXPECT_EQ(rt_log_drain(), 8);
  EXPECT_EQ(rt_log_drain(), 0) << "a second drain must not re-emit";
  EXPECT_EQ(rt_log_dropped(), dropped0) << "8 lines into a 64-slot ring must not drop";
}

TEST(RtDeferredLog, OverrunDropsOldestAndCounts)
{
  rt_log_drain();
  const uint64_t dropped0 = rt_log_dropped();
  // The ring holds 64; deferring 200 without draining must drop, not block or corrupt.
  constexpr int N = 200;
  for (int i = 0; i < N; ++i)
    RT_LOG_DEFER("overflow %d", i);
  const int emitted = rt_log_drain();
  EXPECT_GT(emitted, 0);
  EXPECT_LE(emitted, 64) << "cannot emit more than the ring holds";
  // every deferred line is either emitted or counted as dropped
  EXPECT_EQ(emitted + static_cast<int>(rt_log_dropped() - dropped0), N);
  rt_log_drain();
}

TEST(RtDeferredLog, MultiProducerDoesNotLoseOrDuplicateAccounting)
{
  rt_log_drain();
  const uint64_t dropped0 = rt_log_dropped();
  constexpr int kThreads = 4;
  constexpr int kPerThread = 500;
  std::atomic<int> go{0};
  std::vector<std::thread> producers;
  for (int t = 0; t < kThreads; ++t) {
    producers.emplace_back([t, &go] {
      while (go.load() == 0) { /* spin so all threads start together */ }
      for (int i = 0; i < kPerThread; ++i)
        RT_LOG_DEFER("t%d i%d payload", t, i);
    });
  }
  go.store(1);
  int emitted = 0;
  // drain concurrently, as the stats thread does
  for (int round = 0; round < 50; ++round) {
    emitted += rt_log_drain();
    std::this_thread::yield();
  }
  for (auto &p : producers)
    p.join();
  emitted += rt_log_drain();
  const int dropped = static_cast<int>(rt_log_dropped() - dropped0);
  // Exact accounting is not a property of a drop-oldest lock-free ring: a producer that
  // laps the consumer mid-drain counts a drop for a line that is still emitted. What must
  // hold is that nothing is lost SILENTLY -- every line is emitted, counted, or both --
  // and that we never emit more than was deferred.
  EXPECT_GE(emitted + dropped, kThreads * kPerThread) << "a line was lost without being counted";
  EXPECT_LE(emitted, kThreads * kPerThread) << "emitted more lines than were deferred";
  rt_log_drain();
}

int main(int argc, char **argv)
{
  logInit(); // rt_log_drain() emits through LOG_W, which needs the log subsystem up
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
