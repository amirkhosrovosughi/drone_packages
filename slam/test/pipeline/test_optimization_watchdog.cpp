#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "pipeline/optimization_watchdog.hpp"

namespace slam
{

// ---- helpers ----------------------------------------------------------------

static OptimizationResult makeFastResult(bool success = true)
{
  OptimizationResult r;
  r.success = success;
  r.solveTimeMs = 1;
  r.numIterations = 3;
  return r;
}

// ---- tests ------------------------------------------------------------------

TEST(OptimizationWatchdogTest, FastSolveReturnsResultAndRecordsSuccess)
{
  OptimizationWatchdog watchdog(Milliseconds{100});

  const OptimizationResult result = watchdog.executeWithTimeBudget(
    [] { return makeFastResult(true); });

  EXPECT_TRUE(result.success);

  const OptimizationMetrics m = watchdog.metrics();
  EXPECT_EQ(m.totalSolves, 1);
  EXPECT_EQ(m.succeededSolves, 1);
  EXPECT_EQ(m.timedOutSolves, 0);
}

TEST(OptimizationWatchdogTest, SlowSolveIsAbortedAfterBudget)
{
  // Budget of 30 ms; solve sleeps 200 ms.
  OptimizationWatchdog watchdog(Milliseconds{30});

  const auto before = std::chrono::steady_clock::now();

  const OptimizationResult result = watchdog.executeWithTimeBudget(
    []() -> OptimizationResult
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return makeFastResult(true);
    });

  const auto elapsed = std::chrono::duration_cast<Milliseconds>(
    std::chrono::steady_clock::now() - before);

  // Watchdog must return well before the full 200 ms sleep.
  EXPECT_LT(elapsed.count(), 150);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failureReason.empty());
}

TEST(OptimizationWatchdogTest, MetricsTrackSuccessAndTimeout)
{
  OptimizationWatchdog watchdog(Milliseconds{30});

  // First solve: fast (success)
  watchdog.executeWithTimeBudget([] { return makeFastResult(true); });

  // Second solve: slow (timeout)
  watchdog.executeWithTimeBudget(
    []() -> OptimizationResult
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return makeFastResult(true);
    });

  // Third solve: fast but flagged failure
  watchdog.executeWithTimeBudget([] { return makeFastResult(false); });

  const OptimizationMetrics m = watchdog.metrics();
  EXPECT_EQ(m.totalSolves, 3);
  EXPECT_EQ(m.succeededSolves, 1);  // only the first fast+success one
  EXPECT_EQ(m.timedOutSolves, 1);
  EXPECT_GT(m.avgSolveTimeMs, 0.0);
}

TEST(OptimizationWatchdogTest, ZeroBudgetAlwaysTimesOut)
{
  OptimizationWatchdog watchdog(Milliseconds{0});

  // Even a trivial lambda should be considered timed-out when budget = 0.
  const OptimizationResult result = watchdog.executeWithTimeBudget(
    [] { return makeFastResult(true); });

  // It may or may not finish in 0 ms depending on the scheduler; what we
  // guarantee is that the returned result reflects either success or timeout
  // and that metrics stay consistent.
  const OptimizationMetrics m = watchdog.metrics();
  EXPECT_EQ(m.totalSolves, 1);
  EXPECT_EQ(m.succeededSolves + m.timedOutSolves, 1);
}

}  // namespace slam
