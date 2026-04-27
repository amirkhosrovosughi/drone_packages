#include <gtest/gtest.h>

#include "pipeline/optimization_scheduler.hpp"

namespace slam
{

TEST(OptimizationSchedulerTest, HybridPolicyTriggersFallbackAndLoopClosure)
{
  OptimizationScheduler scheduler(
    OptimizationPolicy::Hybrid,
    3,
    std::chrono::milliseconds(0));

  // Before threshold, keyframe trigger should remain false.
  scheduler.recordKeyframeAccepted();
  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());

  scheduler.recordKeyframeAccepted();
  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());

  // At fallback interval, keyframe trigger should fire.
  scheduler.recordKeyframeAccepted();
  EXPECT_TRUE(scheduler.shouldOptimizeOnKeyframe());

  scheduler.recordOptimization();
  EXPECT_EQ(scheduler.keyframesSinceLastOptimization(), 0);
  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());

  // Loop-closure trigger should fire when accepted.
  EXPECT_TRUE(scheduler.shouldOptimizeOnLoopClosure(true));
  EXPECT_FALSE(scheduler.shouldOptimizeOnLoopClosure(false));
}

TEST(OptimizationSchedulerTest, CounterResetsAfterOptimization)
{
  OptimizationScheduler scheduler(OptimizationPolicy::FallbackEveryNKeyframes, 2);

  scheduler.recordKeyframeAccepted();
  EXPECT_EQ(scheduler.keyframesSinceLastOptimization(), 1);
  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());

  scheduler.recordKeyframeAccepted();
  EXPECT_EQ(scheduler.keyframesSinceLastOptimization(), 2);
  EXPECT_TRUE(scheduler.shouldOptimizeOnKeyframe());

  scheduler.recordOptimization();
  EXPECT_EQ(scheduler.keyframesSinceLastOptimization(), 0);
  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());
}

TEST(OptimizationSchedulerTest, LoopClosureOnlyPolicyIgnoresKeyframeFallback)
{
  OptimizationScheduler scheduler(OptimizationPolicy::OnLoopClosureOnly, 1);

  scheduler.recordKeyframeAccepted();
  scheduler.recordKeyframeAccepted();

  EXPECT_FALSE(scheduler.shouldOptimizeOnKeyframe());
  EXPECT_TRUE(scheduler.shouldOptimizeOnLoopClosure(true));
}

TEST(OptimizationSchedulerTest, LoopClosureCooldownSuppressesImmediateSecondTrigger)
{
  OptimizationScheduler scheduler(
    OptimizationPolicy::OnLoopClosureOnly,
    1,
    std::chrono::milliseconds(500));

  EXPECT_TRUE(scheduler.shouldOptimizeOnLoopClosure(true));

  scheduler.recordOptimization();

  // Second accepted loop closure lands inside cooldown window and must be deferred.
  EXPECT_FALSE(scheduler.shouldOptimizeOnLoopClosure(true));
  EXPECT_FALSE(scheduler.shouldOptimizeOnLoopClosure(false));
}

}  // namespace slam
