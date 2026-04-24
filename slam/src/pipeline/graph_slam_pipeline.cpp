#include "pipeline/graph_slam_pipeline.hpp"

#include <stdexcept>

namespace slam
{

constexpr double kLoopClosureCandidateDistanceMeters = 0.5;
constexpr int kLoopClosureMinKeyframeSeparation = 2;

GraphSlamPipeline::GraphSlamPipeline(
  std::shared_ptr<GraphSlamFrontend> frontend,
  std::shared_ptr<GraphSlamBackend> backend)
  : _frontend(std::move(frontend))
  , _backend(std::move(backend))
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
  _frontend->setLogger(logger);
  _backend->setLogger(logger);
}

void GraphSlamPipeline::onFrontendMotionConstraint(const MotionConstraint& motion)
{
  _backend->applyMotionConstraint(motion);
  _frontend->updateMap(_backend->getMap());
}

void GraphSlamPipeline::onFrontendAssociatedMeasurements(const AssignedMeasurements& measurements)
{
  _backend->applyObservationConstraint(measurements);
  processLoopClosureCandidates();
  _frontend->updateMap(_backend->getMap());
}

void GraphSlamPipeline::processLoopClosureCandidates()
{
  const std::vector<LoopClosureCandidate> candidates =
    _backend->findSpatialLoopClosureCandidates(
      kLoopClosureCandidateDistanceMeters,
      kLoopClosureMinKeyframeSeparation);

  // TODO(Step 5): Merge appearance-cue candidates as an optional secondary source.
  // Query an appearance index (e.g. DBoW / NetVLAD descriptor store) for the active
  // keyframe, produce additional LoopClosureCandidates with hasAppearanceScore=true,
  // and append them here (deduplicating against spatially-found candidates by
  // {sourceKeyframeId, targetKeyframeId} pair) before the validation loop below.

  for (const LoopClosureCandidate& candidate : candidates)
  {
    _backend->validateAndCommitLoopClosure(candidate);
  }
}

}  // namespace slam
