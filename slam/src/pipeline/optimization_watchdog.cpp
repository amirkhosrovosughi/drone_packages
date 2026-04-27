#include "pipeline/optimization_watchdog.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <thread>

namespace slam
{

OptimizationWatchdog::OptimizationWatchdog(Milliseconds timeBudget)
  : _timeBudget(timeBudget)
{
}

OptimizationResult OptimizationWatchdog::executeWithTimeBudget(
  std::function<OptimizationResult()> solveFn)
{
  // Use a shared promise so the detached thread can write a result without
  // accessing any stack frame of this function after it may have returned.
  auto promise = std::make_shared<std::promise<OptimizationResult>>();
  std::future<OptimizationResult> future = promise->get_future();

  const auto startTime = std::chrono::steady_clock::now();

  std::thread(
    [promise, solveFn = std::move(solveFn)]() mutable
    {
      try
      {
        promise->set_value(solveFn());
      }
      catch (...)
      {
        try
        {
          promise->set_exception(std::current_exception());
        }
        catch (...) {}
      }
    }).detach();

  if (future.wait_for(_timeBudget) == std::future_status::timeout)
  {
    const int elapsedMs = static_cast<int>(
      std::chrono::duration_cast<Milliseconds>(
        std::chrono::steady_clock::now() - startTime)
      .count());

    if (_logger)
    {
      _logger->logWarn(
        "Optimization exceeded time budget (", _timeBudget.count(),
        " ms); skipping update");
    }

    updateMetrics(/*timedOut=*/true, /*solveSucceeded=*/false, elapsedMs);

    OptimizationResult timedOut;
    timedOut.success = false;
    timedOut.solveTimeMs = elapsedMs;
    timedOut.failureReason = "Optimization exceeded time budget; skipping update";
    return timedOut;
  }

  OptimizationResult result;
  try
  {
    result = future.get();
  }
  catch (...)
  {
    result.success = false;
    result.failureReason = "Optimization threw an exception";
  }

  updateMetrics(/*timedOut=*/false, result.success, result.solveTimeMs);
  return result;
}

OptimizationMetrics OptimizationWatchdog::metrics() const
{
  std::lock_guard<std::mutex> lock(_metricsMutex);
  return _metrics;
}

void OptimizationWatchdog::setLogger(LoggerPtr logger)
{
  _logger = std::move(logger);
}

void OptimizationWatchdog::updateMetrics(bool timedOut, bool solveSucceeded, int elapsedMs)
{
  std::lock_guard<std::mutex> lock(_metricsMutex);
  ++_metrics.totalSolves;
  if (timedOut)
  {
    ++_metrics.timedOutSolves;
  }
  else if (solveSucceeded)
  {
    ++_metrics.succeededSolves;
  }

  // Rolling average (updated each solve regardless of outcome)
  const double prevAvg = _metrics.avgSolveTimeMs;
  const int n = _metrics.totalSolves;
  _metrics.avgSolveTimeMs = prevAvg + (static_cast<double>(elapsedMs) - prevAvg) / n;
}

}  // namespace slam
