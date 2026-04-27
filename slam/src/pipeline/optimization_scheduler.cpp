#include "pipeline/optimization_scheduler.hpp"

#include <algorithm>

namespace slam
{

OptimizationScheduler::OptimizationScheduler(
  OptimizationPolicy policy,
  int fallbackInterval,
  Milliseconds cooldown)
  : _policy(policy)
  , _fallbackInterval(std::max(1, fallbackInterval))
  , _keyframesSinceLastOptimize(0)
  , _cooldownMs(std::max(cooldown, Milliseconds::zero()))
  , _lastOptimizeTime(std::chrono::steady_clock::now() - _cooldownMs)
{
}

bool OptimizationScheduler::shouldOptimizeOnLoopClosure(bool loopClosureAccepted) const
{
  if (!loopClosureAccepted)
  {
    return false;
  }

  if (_policy != OptimizationPolicy::OnLoopClosureOnly &&
      _policy != OptimizationPolicy::Hybrid)
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  return now - _lastOptimizeTime >= _cooldownMs;
}

bool OptimizationScheduler::shouldOptimizeOnKeyframe() const
{
  if (_policy != OptimizationPolicy::FallbackEveryNKeyframes &&
      _policy != OptimizationPolicy::Hybrid)
  {
    return false;
  }

  return _keyframesSinceLastOptimize >= _fallbackInterval;
}

void OptimizationScheduler::recordOptimization()
{
  _keyframesSinceLastOptimize = 0;
  _lastOptimizeTime = std::chrono::steady_clock::now();
}

void OptimizationScheduler::recordKeyframeAccepted()
{
  ++_keyframesSinceLastOptimize;
}

int OptimizationScheduler::keyframesSinceLastOptimization() const
{
  return _keyframesSinceLastOptimize;
}

}  // namespace slam
