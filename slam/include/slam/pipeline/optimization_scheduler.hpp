#ifndef SLAM__PIPELINE__OPTIMIZATION_SCHEDULER_HPP_
#define SLAM__PIPELINE__OPTIMIZATION_SCHEDULER_HPP_

#include <chrono>

namespace slam
{

using Milliseconds = std::chrono::milliseconds;
constexpr Milliseconds kDefaultOptimizationCooldownMs{500};

enum class OptimizationPolicy
{
  OnLoopClosureOnly,
  FallbackEveryNKeyframes,
  Hybrid,
};

/**
 * @brief Scheduler for deciding when to run global optimization.
 */
class OptimizationScheduler
{
public:
  explicit OptimizationScheduler(
    OptimizationPolicy policy = OptimizationPolicy::Hybrid,
    int fallbackInterval = 5,
    Milliseconds cooldown = kDefaultOptimizationCooldownMs);

  bool shouldOptimizeOnLoopClosure(bool loopClosureAccepted) const;
  bool shouldOptimizeOnKeyframe() const;

  void recordOptimization();
  void recordKeyframeAccepted();

  int keyframesSinceLastOptimization() const;

private:
  OptimizationPolicy _policy;
  int _fallbackInterval;
  int _keyframesSinceLastOptimize;
  Milliseconds _cooldownMs;
  std::chrono::steady_clock::time_point _lastOptimizeTime;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__OPTIMIZATION_SCHEDULER_HPP_
