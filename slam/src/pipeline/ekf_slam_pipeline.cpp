#include "pipeline/ekf_slam_pipeline.hpp"

#include <mutex>

namespace slam
{

/**
 * @brief Construct EKF SLAM pipeline.
 */
EkfSlamPipeline::EkfSlamPipeline(
  std::shared_ptr<ExtendedKalmanFilter> ekf,
  std::shared_ptr<BaseAssociation> association,
  std::shared_ptr<MeasurementFactory> measurementFactory)
  : _ekf(std::move(ekf))
  , _association(std::move(association))
  , _measurementFactory(std::move(measurementFactory))
{
}

/**
 * @brief Initialize EKF pipeline and wire internal callbacks.
 */
void EkfSlamPipeline::initialize()
{
  if (!_ekf || !_association || !_measurementFactory)
    throw std::runtime_error("EKF pipeline not properly constructed");

  // EKF → pipeline → SlamManager
  _ekf->registerCallback(
    [this](const MapSummary& map)
    {
      this->onBackendUpdate(map);
    });

  // Association → pipeline → EKF
  _association->registerCallback(
    [this](const AssignedMeasurements& meas)
    {
      this->correctFilter(meas);
    });
}

/**
 * @brief Handle incoming motion update.
 */
void EkfSlamPipeline::processMotion(const MotionConstraint& m)
{
  std::lock_guard<std::mutex> lock(_mutex);
  updateFilter(m);
}

void EkfSlamPipeline::processObservation(const Observations& o)
{
  std::lock_guard<std::mutex> lock(_mutex);
  Measurements measurements = _measurementFactory->build(o);
  _association->onReceiveMeasurement(measurements);

}

/**
 * @brief Set logger instance.
 */
void EkfSlamPipeline::setLogger(LoggerPtr logger)
{
  _logger = logger;
  _ekf->setLogger(logger);
  _association->setLogger(logger);
}

void EkfSlamPipeline::applyStartupAnchor(const LocalFrameAnchor& anchor)
{
  std::lock_guard<std::mutex> lock(_mutex);
  if (_logger)
  {
    _logger->logInfo(
      "EKF startup anchor aligned (no-op for now): lat=",
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

void EkfSlamPipeline::processGpsMeasurement(const GpsConstraint& constraint)
{
  std::lock_guard<std::mutex> lock(_mutex);
  AbsolutePositionConstraint c;
  c.enuPosition = constraint.enuPosition;
  c.sigmaXyM    = constraint.sigmaXyM;
  c.sigmaZM     = constraint.sigmaZM;
  _ekf->applyAbsolutePositionCorrection(c);
}

/**
 * @brief Internal EKF prediction step.
 */
void EkfSlamPipeline::updateFilter(const MotionConstraint& m)
{
  PredictionInput odom = PredictionInput(m);
  _ekf->prediction(odom);
}

void EkfSlamPipeline::correctFilter(const AssignedMeasurements& meas)
{
  _ekf->correction(meas);
}

/**
 * @brief return map .
 */
MapSummary EkfSlamPipeline::getMap() const
{
  return _ekf->getMap();
}

void EkfSlamPipeline::onBackendUpdate(const MapSummary& map)
{
    _association->handleUpdate(map);
}

// Implement required interface methods
void EkfSlamPipeline::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _ekf->reset();
}

} // namespace slam
