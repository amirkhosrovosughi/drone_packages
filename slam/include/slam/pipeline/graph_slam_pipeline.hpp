#ifndef SLAM__GRAPH_SLAM_BACKEND_HPP_
#define SLAM__GRAPH_SLAM_BACKEND_HPP_

#include <memory>
#include <mutex>

#include "pipeline/graph_slam_backend.hpp"
#include "pipeline/graph_slam_frontend.hpp"
#include "pipeline/slam_pipeline.hpp"

namespace slam
{

/**
 * @brief Graph-based SLAM pipeline skeleton.
 *
 * Owns a graph frontend (keyframe gating + association) and a graph backend
 * (optimizer-facing core only).
 */
class GraphSlamPipeline : public SlamPipeline
{
public:
  GraphSlamPipeline(
    std::shared_ptr<GraphSlamFrontend> frontend,
    std::shared_ptr<GraphSlamBackend> backend);

  void initialize() override;
  void reset() override;
  void processMotion(const MotionConstraint& m) override;
  void processObservation(const Observations& o) override;
  MapSummary getMap() const override;
  void setLogger(LoggerPtr logger) override;

private:
  void onFrontendMotionConstraint(const MotionConstraint& motion);
  void onFrontendAssociatedMeasurements(const AssignedMeasurements& meas);

  std::shared_ptr<GraphSlamFrontend> _frontend;
  std::shared_ptr<GraphSlamBackend> _backend;
  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__GRAPH_SLAM_BACKEND_HPP_
