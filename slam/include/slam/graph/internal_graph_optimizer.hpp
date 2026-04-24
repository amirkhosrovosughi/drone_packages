#ifndef SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
#define SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_

#include <mutex>
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
  void setLogger(LoggerPtr logger) override;

private:
  bool keyframeIdExists(int keyframeId) const;
  const GraphKeyframeNode* findKeyframeById(int keyframeId) const;
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

  GraphState _graph;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
