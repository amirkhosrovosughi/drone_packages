#ifndef SLAM__PIPELINE__OPTIMIZATION_WATCHDOG_HPP_
#define SLAM__PIPELINE__OPTIMIZATION_WATCHDOG_HPP_

#include <functional>
#include <mutex>

#include "common/def_slam_optimization.hpp"
#include "common/slam_logger.hpp"
#include "pipeline/optimization_scheduler.hpp"  // for Milliseconds alias

namespace slam
{

constexpr Milliseconds kDefaultWatchdogTimeBudgetMs{200};

/**
 * @brief Tracks aggregate metrics for all optimization solves.
 */
struct OptimizationMetrics
{
  int totalSolves = 0;
  int succeededSolves = 0;
  int timedOutSolves = 0;
  double avgSolveTimeMs = 0.0;
};

/**
 * @brief Wraps an optimization call with a time budget.
 *
 * Launches the solve on a detached thread and waits up to `timeBudget`.
 * If the solve exceeds the budget the watchdog returns a timed-out
 * OptimizationResult and records the event in its metrics.
 * The detached thread may still complete after the timeout; its result is
 * silently discarded once the watchdog has returned.
 */
class OptimizationWatchdog
{
public:
  explicit OptimizationWatchdog(
    Milliseconds timeBudget = kDefaultWatchdogTimeBudgetMs);

  /**
   * @brief Execute solveFn under a time budget.
   *
   * @param solveFn  Callable that returns OptimizationResult. Must be
   *                 thread-safe with respect to any shared state it uses.
   * @return         The result from solveFn, or a timed-out result if the
   *                 budget is exceeded.
   */
  OptimizationResult executeWithTimeBudget(
    std::function<OptimizationResult()> solveFn);

  /**
   * @brief Snapshot of accumulated solve metrics.
   */
  OptimizationMetrics metrics() const;

  void setLogger(LoggerPtr logger);

private:
  void updateMetrics(bool timedOut, bool solveSucceeded, int elapsedMs);

  Milliseconds _timeBudget;
  mutable std::mutex _metricsMutex;
  OptimizationMetrics _metrics;
  LoggerPtr _logger;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__OPTIMIZATION_WATCHDOG_HPP_
