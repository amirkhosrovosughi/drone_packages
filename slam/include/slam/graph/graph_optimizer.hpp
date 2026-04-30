#ifndef SLAM__GRAPH__GRAPH_OPTIMIZER_HPP_
#define SLAM__GRAPH__GRAPH_OPTIMIZER_HPP_

#include "common/def_slam_graph.hpp"
#include "common/slam_logger.hpp"
#include "measurement/measurement.hpp"

namespace slam
{

/**
 * @brief Optimizer abstraction used by GraphSlamPipeline.
 *
 * This interface allows swapping internal and external graph optimizers
 * without changing pipeline orchestration logic.
 */
class GraphOptimizer
{
public:
  virtual ~GraphOptimizer() = default;

  virtual void initialize() = 0;
  virtual void reset() = 0;
  virtual void applyMotion(const MotionConstraint& motion) = 0;
  virtual void applyObservation(const AssignedMeasurements& measurements) = 0;
  virtual std::vector<LoopClosureCandidate> findSpatialLoopClosureCandidates(
    double maxDistanceMeters,
    int minKeyframeSeparation) const = 0;
  virtual LoopClosureValidationResult validateLoopClosureCandidate(
    const LoopClosureCandidate& candidate) const = 0;
  virtual bool commitLoopClosure(
    const LoopClosureCandidate& candidate,
    const LoopClosureValidationResult& validation) = 0;
  virtual GraphState getGraphState() const = 0;

  /**
   * @brief Lightweight per-keyframe pose refinement using recent constraints.
   * Called after each keyframe is accepted to incrementally improve pose estimate.
   * Uses strategy specified in config (Gauss-Newton or closed-form).
   */
  virtual void refineActiveKeyframe(const OptimizationConfig& config = OptimizationConfig()) = 0;

  /**
   * @brief Global graph optimization/solve.
   * Triggered on loop closure acceptance or fallback after N keyframes.
   * Returns false if solve was skipped or failed (e.g., timeout).
   */
  virtual bool optimizeGraph(
    const OptimizationConfig& config,
    OptimizationResult* resultOut = nullptr) = 0;

  MapSummary getMap() const
  {
    return graphStateToMapSummary(getGraphState());
  }

  virtual void setLogger(LoggerPtr logger) = 0;
};

}  // namespace slam

#endif  // SLAM__GRAPH__GRAPH_OPTIMIZER_HPP_
