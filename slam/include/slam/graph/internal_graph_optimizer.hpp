#ifndef SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
#define SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_

#include <chrono>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "graph/graph_optimizer.hpp"

namespace slam
{

/**
 * @brief Lightweight in-tree graph optimizer used as initial implementation.
 *
 * This class intentionally keeps behavior simple while the graph pipeline
 * pipeline is being integrated.
 *
 * Features:
 * - Spatial loop closure detection via keyframe proximity
 * - Geometric loop closure validation via landmark correspondences
 * - Per-keyframe incremental refinement (Gauss-Newton strategy)
 * - Global optimization interface (stub; real solver tbd)
 */
class InternalGraphOptimizer : public GraphOptimizer
{
public:
  void initialize() override;
  void reset() override;
  void applyMotion(const MotionConstraint& motion) override;
  void applyObservation(const AssignedMeasurements& measurements) override;
  std::vector<LoopClosureCandidate> findSpatialLoopClosureCandidates(
    double maxDistanceMeters,
    int minKeyframeSeparation) const override;
  LoopClosureValidationResult validateLoopClosureCandidate(
    const LoopClosureCandidate& candidate) const override;
  bool commitLoopClosure(
    const LoopClosureCandidate& candidate,
    const LoopClosureValidationResult& validation) override;
  GraphState getGraphState() const override;
  void refineActiveKeyframe(const OptimizationConfig& config = OptimizationConfig()) override;
  bool optimizeGraph(
    const OptimizationConfig& config,
    OptimizationResult* resultOut = nullptr) override;
  void setLogger(LoggerPtr logger) override;

private:
  // Refinement strategy implementations
  void refineActiveKeyframeGaussNewton(const OptimizationConfig& config);
  void refineActiveKeyframeClosedForm(const OptimizationConfig& config);

  // Returns a 6D local pose delta [dx, dy, dz, droll, dpitch, dyaw].
  Eigen::VectorXd computeLocalPoseRefinement(int keyframeId);

  bool keyframeIdExists(int keyframeId) const;
  const GraphKeyframeNode* findKeyframeById(int keyframeId) const;
  GraphKeyframeNode* findKeyframeByIdMutable(int keyframeId);
  bool loopClosureExists(int keyframeAId, int keyframeBId) const;
  bool accumulateSequentialOdometryDelta(
    int fromKeyframeId,
    int toKeyframeId,
    Eigen::Vector3d* deltaOut,
    int* edgeCountOut) const;
  const GraphLandmarkNode* findLandmarkById(int landmarkId) const;
  std::vector<int> collectCommonObservedLandmarkIds(
    int keyframeAId,
    int keyframeBId) const;

  // --- Global optimization helpers (called only while _mutex is held) ---

  // Build a keyframe-id → vector-index lookup for the current graph.
  std::unordered_map<int, int> buildKeyframeIdToIndex() const;

  // Returns the largest per-node step norm in a packed 3N delta vector.
  static double computeMaxNodeStep(const Eigen::VectorXd& delta, int numKeyframes);

  // Assemble the Gauss-Newton Hessian and gradient vectors from anchor,
  // odometry, and loop-closure constraints.
  void buildSystemMatrices(
    const std::unordered_map<int, int>& idToIndex,
    const Eigen::Vector3d& anchorPosition,
    Eigen::MatrixXd& hessian,
    Eigen::VectorXd& gradient,
    int& constraintCount) const;

  // Apply a packed 3N-DOF pose-delta vector to all keyframes in-place.
  void applyKeyframeDelta(const Eigen::VectorXd& delta);

  // Compute total weighted edge residual squared-norm for convergence checks.
  double computeWeightedResidual(
    const std::unordered_map<int, int>& idToIndex) const;

  GraphState _graph;
  std::chrono::steady_clock::time_point _lastRefinementTime;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
