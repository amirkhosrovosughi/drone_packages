#include "pipeline/graph_slam_pipeline.hpp"

#include <stdexcept>

namespace slam
{

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
  _frontend->updateMap(_backend->getMap());
}

}  // namespace slam
