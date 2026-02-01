#include "backend/ekf_slam_backend.hpp"

#include <mutex>

namespace slam
{

/**
 * @brief Construct EKF SLAM backend.
 */
EkfSlamBackend::EkfSlamBackend(
  std::shared_ptr<ExtendedKalmanFilter> ekf,
  std::shared_ptr<BaseAssociation> association,
  std::shared_ptr<MeasurementFactory> measurementFactory)
  : _ekf(std::move(ekf))
  , _association(std::move(association))
  , _measurementFactory(std::move(measurementFactory))
{
}

/**
 * @brief Initialize EKF backend and wire internal callbacks.
 */
void EkfSlamBackend::initialize()
{
  if (!_ekf || !_association || !_measurementFactory)
    throw std::runtime_error("EKF backend not properly constructed");

  // EKF → backend → SlamManager
  _ekf->registerCallback(
    [this](const MapSummary& map)
    {
      this->onBackendUpdate(map);
    });

  // Association → backend → EKF
  _association->registerCallback(
    [this](const AssignedMeasurements& meas)
    {
      this->correctFilter(meas);
    });
}

/**
 * @brief Handle incoming motion update.
 */
void EkfSlamBackend::processMotion(const MotionConstraint& m)
{
  std::lock_guard<std::mutex> lock(_mutex);
  updateFilter(m);
}

void EkfSlamBackend::processObservation(const Observations& o)
{
  std::lock_guard<std::mutex> lock(_mutex);
  Measurements measurements = _measurementFactory->build(o);
  _association->onReceiveMeasurement(measurements);

}

/**
 * @brief Set logger instance.
 */
void EkfSlamBackend::setLogger(LoggerPtr logger)
{
  _logger = logger;
  _ekf->setLogger(logger);
  _association->setLogger(logger);
}

/**
 * @brief Internal EKF prediction step.
 */
void EkfSlamBackend::updateFilter(const MotionConstraint& m)
{
  PredictionInput odom = PredictionInput(m);
  _ekf->prediction(odom);
}

void EkfSlamBackend::correctFilter(const AssignedMeasurements& meas)
{
  _ekf->correction(meas);
}

/**
 * @brief return map .
 */
MapSummary EkfSlamBackend::getMap() const
{
  return _ekf->getMap();
}

void EkfSlamBackend::onBackendUpdate(const MapSummary& map)
{
    _association->handleUpdate(map);
}

// Implement required interface methods
void EkfSlamBackend::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _ekf->reset();
}

} // namespace slam
