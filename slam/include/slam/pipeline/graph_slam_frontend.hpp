#ifndef SLAM__PIPELINE__GRAPH_SLAM_FRONTEND_HPP_
#define SLAM__PIPELINE__GRAPH_SLAM_FRONTEND_HPP_

#include <functional>
#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_set>

#include "association/base_association.hpp"
#include "common/def_slam.hpp"
#include "common/slam_logger.hpp"
#include "measurement/measurement_factory.hpp"
#include "observation/observation.hpp"

namespace slam
{

/**
 * @brief Graph SLAM frontend handling keyframe gating and data association.
 *
 * Responsibilities:
 * - Keyframe selection using odometry thresholds.
 * - Odometry accumulation between keyframes.
 * - Measurement building + association for accepted keyframes.
 */
class GraphSlamFrontend
{
public:
  GraphSlamFrontend(
    std::shared_ptr<BaseAssociation> association,
    std::shared_ptr<MeasurementFactory> measurementFactory);

  void initialize();
  void reset();

  void onMotion(const MotionConstraint& motion);
  void onObservation(const Observations& observations);

  void updateMap(const MapSummary& map);
  void setLogger(LoggerPtr logger);

  void setMotionConstraintCallback(
    std::function<void(const MotionConstraint&)> callback);

  void setAssignedMeasurementsCallback(
    std::function<void(const AssignedMeasurements&)> callback);

private:
  enum class ObservationStreamType
  {
    Point3D,
    Bearing,
    BoundingBox,
    Unknown
  };

  double computeRotationDeltaRad(
    const Eigen::Quaterniond& a,
    const Eigen::Quaterniond& b) const;

  ObservationStreamType inferObservationType(const Observation& observation) const;
  long long extractObservationGroupId(const Observations& observations) const;
  void closeObservationWindow();

  std::shared_ptr<BaseAssociation> _association;
  std::shared_ptr<MeasurementFactory> _measurementFactory;

  std::function<void(const MotionConstraint&)> _motionConstraintCallback;
  std::function<void(const AssignedMeasurements&)> _assignedMeasurementsCallback;

  Eigen::Vector3d _accumulatedTranslation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond _lastKeyframeOrientation = Eigen::Quaterniond::Identity();
  bool _hasOrientationReference = false;

  bool _observationWindowOpen = false;
  std::chrono::steady_clock::time_point _observationWindowCloseTime;
  std::unordered_set<int> _receivedObservationTypes;
  std::unordered_set<long long> _receivedObservationGroupIds;

  LoggerPtr _logger;
  mutable std::mutex _mutex;
};

}  // namespace slam

#endif  // SLAM__PIPELINE__GRAPH_SLAM_FRONTEND_HPP_
