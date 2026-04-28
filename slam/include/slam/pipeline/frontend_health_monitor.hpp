#ifndef SLAM__PIPELINE__FRONTEND_HEALTH_MONITOR_HPP_
#define SLAM__PIPELINE__FRONTEND_HEALTH_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "common/slam_logger.hpp"

namespace slam
{

struct FrontendHealthMetrics
{
  std::uint64_t totalMeasurementAttempts = 0;
  std::uint64_t totalAssociatedMeasurements = 0;
  std::uint64_t totalDroppedMeasurements = 0;
  double associationAcceptanceRate = 0.0;
  double associationDropRate = 0.0;

  std::uint64_t totalLoopCandidates = 0;
  std::uint64_t totalRejectedLoopCandidates = 0;
  double loopConstraintRejectRate = 0.0;

  std::uint64_t rejectionSpikeCount = 0;
  bool rejectionSpikeInLastCycle = false;

  int loopClosureCooldownRemaining = 0;
};

/**
 * @brief Tracks frontend association quality and loop-closure rejection health.
 *
 * Responsibilities:
 * - Accumulates measurement attempt / acceptance counts and derives rates.
 * - Warns when the association drop rate exceeds a configured threshold.
 * - Maintains a rolling window of per-cycle loop-rejection ratios and detects
 *   sudden spikes that may indicate sensor degradation or map corruption.
 */
class FrontendHealthMonitor
{
public:
  FrontendHealthMonitor();

  void setLogger(LoggerPtr logger);

  /**
   * @brief Record one association cycle.
   * @param attempted Number of measurements forwarded to association.
   * @param accepted  Number of measurements that passed association.
   */
  void recordMeasurementBatch(std::size_t attempted, std::size_t accepted);

  /**
   * @brief Record one loop-closure processing cycle.
   * @param totalCandidates         Total candidates evaluated this cycle.
   * @param rejectedByValidation    Candidates rejected by the validator.
   */
  void recordLoopClosureCycle(std::size_t totalCandidates, std::size_t rejectedByValidation);

  /** @brief Decrement loop-closure cooldown after a keyframe is accepted. */
  void onKeyframeAccepted();

  /** @brief Returns false while the loop-closure cooldown is active. */
  bool isLoopClosureEnabled() const;

  FrontendHealthMetrics metrics() const;
  void reset();

private:
  FrontendHealthMetrics _metrics;
  std::deque<double> _recentLoopRejectRatios;
  int _loopClosureCooldownRemaining = 0;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__FRONTEND_HEALTH_MONITOR_HPP_
