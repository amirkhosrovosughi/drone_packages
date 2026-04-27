#include "pipeline/graph_slam_backend.hpp"

#include <stdexcept>

namespace slam
{

GraphSlamBackend::GraphSlamBackend(std::shared_ptr<GraphOptimizer> optimizer)
  : _optimizer(std::move(optimizer))
{
}

void GraphSlamBackend::initialize()
{
  if (!_optimizer)
  {
    throw std::runtime_error("Graph backend not properly constructed");
  }
  _optimizer->initialize();
}

void GraphSlamBackend::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _optimizer->reset();
}

void GraphSlamBackend::applyMotionConstraint(const MotionConstraint& motion)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _optimizer->applyMotion(motion);
}

void GraphSlamBackend::applyObservationConstraint(const AssignedMeasurements& measurements)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _optimizer->applyObservation(measurements);
}

std::vector<LoopClosureCandidate> GraphSlamBackend::findSpatialLoopClosureCandidates(
  double maxDistanceMeters,
  int minKeyframeSeparation) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _optimizer->findSpatialLoopClosureCandidates(maxDistanceMeters, minKeyframeSeparation);
}

LoopClosureValidationResult GraphSlamBackend::validateLoopClosureCandidate(
  const LoopClosureCandidate& candidate) const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _optimizer->validateLoopClosureCandidate(candidate);
}

bool GraphSlamBackend::validateAndCommitLoopClosure(
  const LoopClosureCandidate& candidate,
  LoopClosureValidationResult* validationOut)
{
  std::lock_guard<std::mutex> lock(_mutex);

  const LoopClosureValidationResult validation =
    _optimizer->validateLoopClosureCandidate(candidate);
  if (validationOut)
  {
    *validationOut = validation;
  }

  if (!validation.accepted)
  {
    return false;
  }

  return _optimizer->commitLoopClosure(candidate, validation);
}

MapSummary GraphSlamBackend::getMap() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _optimizer->getMap();
}

void GraphSlamBackend::setLogger(LoggerPtr logger)
{
  _logger = logger;
  _optimizer->setLogger(logger);
}

void GraphSlamBackend::refineActiveKeyframe(const OptimizationConfig& config)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _optimizer->refineActiveKeyframe(config);
}

bool GraphSlamBackend::optimizeGraph(
  const OptimizationConfig& config,
  OptimizationResult* resultOut)
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _optimizer->optimizeGraph(config, resultOut);
}

}  // namespace slam
