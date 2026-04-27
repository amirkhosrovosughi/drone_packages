#ifndef SLAM__PIPELINE__GRAPH_SLAM_BACKEND_HPP_
#define SLAM__PIPELINE__GRAPH_SLAM_BACKEND_HPP_

#include <memory>
#include <mutex>

#include "common/def_slam.hpp"
#include "common/slam_logger.hpp"
#include "graph/graph_optimizer.hpp"
#include "measurement/measurement.hpp"

namespace slam
{

/**
 * @brief Graph SLAM backend core owning only optimizer-facing operations.
 */
class GraphSlamBackend
{
public:
  explicit GraphSlamBackend(std::shared_ptr<GraphOptimizer> optimizer);

  void initialize();
  void reset();
  void applyMotionConstraint(const MotionConstraint& motion);
  void applyObservationConstraint(const AssignedMeasurements& measurements);
  std::vector<LoopClosureCandidate> findSpatialLoopClosureCandidates(
    double maxDistanceMeters,
    int minKeyframeSeparation) const;
  LoopClosureValidationResult validateLoopClosureCandidate(
    const LoopClosureCandidate& candidate) const;
  bool validateAndCommitLoopClosure(
    const LoopClosureCandidate& candidate,
    LoopClosureValidationResult* validationOut = nullptr);
  MapSummary getMap() const;
  void setLogger(LoggerPtr logger);

    /**
     * @brief Lightweight per-keyframe pose refinement.
     * Called after each keyframe observation to incrementally improve estimates.
     */
    void refineActiveKeyframe(const OptimizationConfig& config = OptimizationConfig());

    /**
     * @brief Global graph optimization/solve.
     * Triggered on loop closure acceptance or fallback after N keyframes.
     */
    bool optimizeGraph(
      const OptimizationConfig& config,
      OptimizationResult* resultOut = nullptr);

private:
  std::shared_ptr<GraphOptimizer> _optimizer;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__GRAPH_SLAM_BACKEND_HPP_
