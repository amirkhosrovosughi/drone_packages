#include "pipeline/graph_slam_frontend.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double KEYFRAME_TRANSLATION_THRESHOLD_M = 0.5;
constexpr double KEYFRAME_ROTATION_THRESHOLD_RAD = 10.0 * M_PI / 180.0;
constexpr auto OBSERVATION_MERGE_WINDOW = std::chrono::milliseconds(80);
}

namespace slam
{

GraphSlamFrontend::GraphSlamFrontend(
  std::shared_ptr<BaseAssociation> association,
  std::shared_ptr<MeasurementFactory> measurementFactory)
  : _association(std::move(association))
  , _measurementFactory(std::move(measurementFactory))
  , _healthMonitor(std::make_unique<FrontendHealthMonitor>())
{
}

void GraphSlamFrontend::initialize()
{
  if (!_association || !_measurementFactory)
  {
    throw std::runtime_error("Graph frontend not properly constructed");
  }

  _association->registerCallback(
    [this](AssignedMeasurements measurements)
    {
      const std::size_t attempted = _pendingMeasurementAttemptCount.exchange(0);
      _healthMonitor->recordMeasurementBatch(
        std::max(attempted, measurements.size()), measurements.size());

      auto callback = _assignedMeasurementsCallback;
      if (callback)
      {
        callback(measurements);
      }
    });
}

void GraphSlamFrontend::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _accumulatedTranslation = Eigen::Vector3d::Zero();
  _lastKeyframeOrientation = Eigen::Quaterniond::Identity();
  _hasOrientationReference = false;
  _pendingMeasurementAttemptCount.store(0);
  closeObservationWindow();
  _healthMonitor->reset();
}

void GraphSlamFrontend::onMotion(const MotionConstraint& motion)
{
  MotionConstraint keyframeMotion;
  bool shouldCommitKeyframe = false;

  std::lock_guard<std::mutex> lock(_mutex);

  if (_observationWindowOpen && std::chrono::steady_clock::now() > _observationWindowCloseTime)
  {
    closeObservationWindow();
  }

  Eigen::Quaterniond currentOrientation = motion.orientation;
  currentOrientation.normalize();

  if (!_hasOrientationReference)
  {
    _lastKeyframeOrientation = currentOrientation;
    _hasOrientationReference = true;
  }

  _accumulatedTranslation += motion.delta_position;

  const double translationDelta = _accumulatedTranslation.norm();
  const double rotationDelta = computeRotationDeltaRad(_lastKeyframeOrientation, currentOrientation);

  if (translationDelta > KEYFRAME_TRANSLATION_THRESHOLD_M ||
      rotationDelta > KEYFRAME_ROTATION_THRESHOLD_RAD)
  {
    keyframeMotion.delta_position = _accumulatedTranslation;
    keyframeMotion.orientation = currentOrientation;
    shouldCommitKeyframe = true;

    _accumulatedTranslation = Eigen::Vector3d::Zero();
    _lastKeyframeOrientation = currentOrientation;

    _observationWindowOpen = true;
    _observationWindowCloseTime = std::chrono::steady_clock::now() + OBSERVATION_MERGE_WINDOW;
    _receivedObservationTypes.clear();
    _receivedObservationGroupIds.clear();
  }

  if (shouldCommitKeyframe && _motionConstraintCallback)
  {
    _motionConstraintCallback(keyframeMotion);
  }
}

void GraphSlamFrontend::onObservation(const Observations& observations)
{
  if (observations.empty())
  {
    return;
  }

  ObservationStreamType streamType = inferObservationType(observations.front());
  if (streamType == ObservationStreamType::Unknown)
  {
    return;
  }

  std::vector<Observation> filtered;
  filtered.reserve(observations.size());

  bool shouldProcess = false;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_observationWindowOpen)
    {
      return;
    }

    if (std::chrono::steady_clock::now() > _observationWindowCloseTime)
    {
      closeObservationWindow();
      return;
    }

    const int streamTypeKey = static_cast<int>(streamType);
    const long long groupId = extractObservationGroupId(observations);

    // Keep one processing pass per observation type inside the merge window.
    if (_receivedObservationTypes.find(streamTypeKey) != _receivedObservationTypes.end())
    {
      return;
    }

    _receivedObservationTypes.insert(streamTypeKey);
    if (groupId >= 0)
    {
      _receivedObservationGroupIds.insert(groupId);
    }

    for (const auto& obs : observations)
    {
      if (inferObservationType(obs) == streamType)
      {
        filtered.push_back(obs);
      }
    }

    shouldProcess = !filtered.empty();
  }

  if (!shouldProcess)
  {
    return;
  }

  Measurements measurements = _measurementFactory->build(filtered);
  _pendingMeasurementAttemptCount.store(measurements.size());
  _association->onReceiveMeasurement(measurements);
}

void GraphSlamFrontend::updateMap(const MapSummary& map)
{
  _association->handleUpdate(map);
}

void GraphSlamFrontend::setLogger(LoggerPtr logger)
{
  _logger = logger;
  _association->setLogger(logger);
  _healthMonitor->setLogger(logger);
}

void GraphSlamFrontend::setMotionConstraintCallback(
  std::function<void(const MotionConstraint&)> callback)
{
  _motionConstraintCallback = std::move(callback);
}

void GraphSlamFrontend::setAssignedMeasurementsCallback(
  std::function<void(const AssignedMeasurements&)> callback)
{
  _assignedMeasurementsCallback = std::move(callback);
}

FrontendHealthMetrics GraphSlamFrontend::healthMetrics() const
{
  return _healthMonitor->metrics();
}

void GraphSlamFrontend::recordLoopClosureCycle(
  std::size_t totalCandidates, std::size_t rejectedByValidation)
{
  _healthMonitor->recordLoopClosureCycle(totalCandidates, rejectedByValidation);
}

double GraphSlamFrontend::computeRotationDeltaRad(
  const Eigen::Quaterniond& a,
  const Eigen::Quaterniond& b) const
{
  const double dot = std::abs(a.dot(b));
  const double clampedDot = std::min(1.0, std::max(-1.0, dot));
  return 2.0 * std::acos(clampedDot);
}

GraphSlamFrontend::ObservationStreamType GraphSlamFrontend::inferObservationType(
  const Observation& observation) const
{
  if (std::holds_alternative<Point3D>(observation.payload))
  {
    return ObservationStreamType::Point3D;
  }
  if (std::holds_alternative<Bearing>(observation.payload))
  {
    return ObservationStreamType::Bearing;
  }
  if (std::holds_alternative<BoundingBox>(observation.payload))
  {
    return ObservationStreamType::BoundingBox;
  }
  return ObservationStreamType::Unknown;
}

long long GraphSlamFrontend::extractObservationGroupId(const Observations& observations) const
{
  if (observations.empty())
  {
    return -1;
  }

  const double t = observations.front().timeTag;
  if (!std::isfinite(t) || t <= 0.0)
  {
    return -1;
  }

  const double millis = t * 1000.0;
  if (millis > static_cast<double>(std::numeric_limits<long long>::max()))
  {
    return -1;
  }
  return static_cast<long long>(millis);
}

void GraphSlamFrontend::closeObservationWindow()
{
  _observationWindowOpen = false;
  _receivedObservationTypes.clear();
  _receivedObservationGroupIds.clear();
}

}  // namespace slam
