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

}  // namespace slam
