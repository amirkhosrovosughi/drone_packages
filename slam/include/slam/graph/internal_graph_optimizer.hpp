#ifndef SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
#define SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_

#include <mutex>

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
  GraphState getGraphState() const override;
  void setLogger(LoggerPtr logger) override;

private:
  GraphState _graph;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__GRAPH__INTERNAL_GRAPH_OPTIMIZER_HPP_
