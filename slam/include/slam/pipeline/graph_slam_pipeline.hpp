#ifndef SLAM__PIPELINE__GRAPH_SLAM_PIPELINE_HPP_
#define SLAM__PIPELINE__GRAPH_SLAM_PIPELINE_HPP_

#include <memory>

#include "pipeline/graph_slam_backend.hpp"
#include "pipeline/graph_slam_frontend.hpp"
#include "pipeline/optimization_scheduler.hpp"
#include "pipeline/optimization_watchdog.hpp"
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
  void applyStartupAnchor(const LocalFrameAnchor& anchor) override;
  void processGpsMeasurement(const GpsConstraint& constraint) override;
  void setScheduler(std::shared_ptr<OptimizationScheduler> scheduler);
  void setWatchdog(std::shared_ptr<OptimizationWatchdog> watchdog);
  OptimizationMetrics watchdogMetrics() const;
  FrontendHealthMetrics frontendHealthMetrics() const;

private:
  bool processLoopClosureCandidates();
  void checkAndExecuteOptimization(bool loopClosureAccepted);
  void onFrontendMotionConstraint(const MotionConstraint& motion);
  void onFrontendAssociatedMeasurements(const AssignedMeasurements& meas);

  std::shared_ptr<GraphSlamFrontend> _frontend;
  std::shared_ptr<GraphSlamBackend> _backend;
  std::shared_ptr<OptimizationScheduler> _scheduler;
  std::shared_ptr<OptimizationWatchdog> _watchdog;
  int _keyframeCount = 0;
  LoggerPtr _logger;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__GRAPH_SLAM_PIPELINE_HPP_
