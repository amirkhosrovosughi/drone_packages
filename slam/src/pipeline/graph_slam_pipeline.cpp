#include "pipeline/graph_slam_pipeline.hpp"

#include <stdexcept>

namespace slam
{

constexpr double kLoopClosureCandidateDistanceMeters = 0.5;
constexpr int kLoopClosureMinKeyframeSeparation = 2;
constexpr int kOptimizationFallbackEveryNKeyframes = 5;
constexpr Milliseconds kPipelineWatchdogTimeBudgetMs{150};

GraphSlamPipeline::GraphSlamPipeline(
  std::shared_ptr<GraphSlamFrontend> frontend,
  std::shared_ptr<GraphSlamBackend> backend)
  : _frontend(std::move(frontend))
  , _backend(std::move(backend))
  , _scheduler(std::make_shared<OptimizationScheduler>(
      OptimizationPolicy::Hybrid,
      kOptimizationFallbackEveryNKeyframes))
  , _watchdog(std::make_shared<OptimizationWatchdog>(kPipelineWatchdogTimeBudgetMs))
{
}

void GraphSlamPipeline::initialize()
{
  if (!_frontend || !_backend)
  {
    throw std::runtime_error("Graph pipeline not properly constructed");
  }

  _backend->initialize();
  _frontend->initialize();

  _frontend->setMotionConstraintCallback(
    [this](const MotionConstraint& motion)
    {
      this->onFrontendMotionConstraint(motion);
    });

  _frontend->setGpsPriorCallback(
    [this](const AbsolutePositionConstraint& constraint)
    {
      _backend->applyGpsPrior(constraint);
    });

  _frontend->setAssignedMeasurementsCallback(
    [this](const AssignedMeasurements& measurements)
    {
      this->onFrontendAssociatedMeasurements(measurements);
    });

  _frontend->updateMap(_backend->getMap());
}

void GraphSlamPipeline::reset()
{
  _backend->reset();
  _frontend->reset();
  _gpsMeasurementGate.reset();
  _keyframeCount = 0;
  if (_scheduler)
  {
    _scheduler->recordOptimization();
  }
  _frontend->updateMap(_backend->getMap());
}

void GraphSlamPipeline::processMotion(const MotionConstraint& m)
{
  _frontend->onMotion(m);
}

void GraphSlamPipeline::processObservation(const Observations& o)
{
  _frontend->onObservation(o);
}

MapSummary GraphSlamPipeline::getMap() const
{
  return _backend->getMap();
}

void GraphSlamPipeline::setLogger(LoggerPtr logger)
{
  _logger = logger;
  _gpsMeasurementGate.setLogger(logger);
  _frontend->setLogger(logger);
  _backend->setLogger(logger);
  if (_watchdog)
  {
    _watchdog->setLogger(logger);
  }
}

void GraphSlamPipeline::applyStartupAnchor(const LocalFrameAnchor& anchor)
{
  if (_logger)
  {
    _logger->logInfo(
      "Graph startup anchor aligned (no-op for now): lat=",
      anchor.anchorReference.latitudeDeg,
      ", lon=",
      anchor.anchorReference.longitudeDeg,
      ", alt=",
      anchor.anchorReference.altitudeM,
      ", initial_enu=[",
      anchor.initialEnuPosition.x(),
      ", ",
      anchor.initialEnuPosition.y(),
      ", ",
      anchor.initialEnuPosition.z(),
      "]");
  }
}

void GraphSlamPipeline::processGpsMeasurement(const GpsConstraint& constraint)
{
  const MapSummary mapState = _backend->getMap();
  const Eigen::Vector3d predictedEnuPosition = mapState.robot.pose.position.getPositionVector();
  if (_gpsMeasurementGate.shouldAccept(constraint, predictedEnuPosition))
  {
    _frontend->onGpsMeasurement(constraint);
  }
}

void GraphSlamPipeline::setScheduler(std::shared_ptr<OptimizationScheduler> scheduler)
{
  if (scheduler)
  {
    _scheduler = std::move(scheduler);
  }
}

void GraphSlamPipeline::setWatchdog(std::shared_ptr<OptimizationWatchdog> watchdog)
{
  if (watchdog)
  {
    _watchdog = std::move(watchdog);
    if (_logger)
    {
      _watchdog->setLogger(_logger);
    }
  }
}

void GraphSlamPipeline::onFrontendMotionConstraint(const MotionConstraint& motion)
{
  _backend->applyMotionConstraint(motion);
  _frontend->onKeyframeAccepted();

  ++_keyframeCount;
  if (_scheduler)
  {
    _scheduler->recordKeyframeAccepted();
    checkAndExecuteOptimization(false);
  }

  _frontend->updateMap(_backend->getMap());
}

void GraphSlamPipeline::onFrontendAssociatedMeasurements(const AssignedMeasurements& measurements)
{
  _backend->applyObservationConstraint(measurements);
  _backend->refineActiveKeyframe();

  const bool loopClosureAccepted = processLoopClosureCandidates();
  checkAndExecuteOptimization(loopClosureAccepted);

  _frontend->updateMap(_backend->getMap());
}

bool GraphSlamPipeline::processLoopClosureCandidates()
{
  if (!_frontend->isLoopClosureEnabled())
  {
    return false;
  }

  const std::vector<LoopClosureCandidate> candidates =
    _backend->findSpatialLoopClosureCandidates(
      kLoopClosureCandidateDistanceMeters,
      kLoopClosureMinKeyframeSeparation);

  bool loopClosureAccepted = false;
  std::size_t rejectedByValidationCount = 0;

  // TODO(Step 5): Merge appearance-cue candidates as an optional secondary source.
  // Query an appearance index (e.g. DBoW / NetVLAD descriptor store) for the active
  // keyframe, produce additional LoopClosureCandidates with hasAppearanceScore=true,
  // and append them here (deduplicating against spatially-found candidates by
  // {sourceKeyframeId, targetKeyframeId} pair) before the validation loop below.

  for (const LoopClosureCandidate& candidate : candidates)
  {
    LoopClosureValidationResult validation;
    if (_backend->validateAndCommitLoopClosure(candidate, &validation))
    {
      loopClosureAccepted = true;
      if (_logger)
      {
        _logger->logInfo(
          "Loop closure committed source=",
          candidate.sourceKeyframeId,
          ", target=",
          candidate.targetKeyframeId,
          ", supportCount=",
          validation.supportCount,
          ", inlierCount=",
          validation.inlierCount,
          ", inlierRatio=",
          validation.inlierRatio);
      }
    }
    else
    {
      if (!validation.accepted)
      {
        ++rejectedByValidationCount;
      }

      if (_logger)
      {
        _logger->logDebug(
          "Loop closure rejected source=",
          candidate.sourceKeyframeId,
          ", target=",
          candidate.targetKeyframeId,
          ", reason=",
          validation.reason.empty() ? "unknown" : validation.reason,
          ", supportCount=",
          validation.supportCount,
          ", inlierCount=",
          validation.inlierCount,
          ", inlierRatio=",
          validation.inlierRatio);
      }
    }
  }

  _frontend->recordLoopClosureCycle(candidates.size(), rejectedByValidationCount);

  return loopClosureAccepted;
}

OptimizationMetrics GraphSlamPipeline::watchdogMetrics() const
{
  if (_watchdog)
  {
    return _watchdog->metrics();
  }
  return OptimizationMetrics{};
}

FrontendHealthMetrics GraphSlamPipeline::frontendHealthMetrics() const
{
  return _frontend->healthMetrics();
}

GpsMeasurementGateHealth GraphSlamPipeline::gpsMeasurementGateHealth() const
{
  return _gpsMeasurementGate.health();
}

void GraphSlamPipeline::checkAndExecuteOptimization(bool loopClosureAccepted)
{
  if (!_scheduler)
  {
    return;
  }

  bool shouldOptimize = false;

  if (_scheduler->shouldOptimizeOnLoopClosure(loopClosureAccepted))
  {
    shouldOptimize = true;
  }
  else if (_scheduler->shouldOptimizeOnKeyframe())
  {
    shouldOptimize = true;
  }

  if (!shouldOptimize)
  {
    return;
  }

  const OptimizationResult result = _watchdog
    ? _watchdog->executeWithTimeBudget(
        [this]() -> OptimizationResult
        {
          OptimizationResult r;
          _backend->optimizeGraph(OptimizationConfig(), &r);
          return r;
        })
    : [this]() -> OptimizationResult
      {
        OptimizationResult r;
        _backend->optimizeGraph(OptimizationConfig(), &r);
        return r;
      }();

  if (!result.success && !result.failureReason.empty() && _logger)
  {
    _logger->logWarn("Optimization did not converge: ", result.failureReason);
  }

  _scheduler->recordOptimization();
}

}  // namespace slam
