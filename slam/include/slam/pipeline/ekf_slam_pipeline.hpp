#ifndef SLAM__PIPELINE__EKF_SLAM_PIPELINE_HPP_
#define SLAM__PIPELINE__EKF_SLAM_PIPELINE_HPP_

#include <memory>
#include <mutex>

#include "pipeline/slam_pipeline.hpp"
#include "common/slam_logger.hpp"

#include "filter/extended_kalman_filter.hpp"
#include "association/base_association.hpp"
#include "measurement/measurement_factory.hpp"

namespace slam
{

/**
 * @class EkfSlamPipeline
 * @brief EKF-based SLAM pipeline implementation.
 *
 * Owns and coordinates:
 *  - Extended Kalman Filter
 *  - Motion / measurement model
 *  - Data association strategy
 *
 * Exposes only algorithm-agnostic interfaces to SlamManager.
 */
class EkfSlamPipeline : public SlamPipeline
{
public:
  /**
   * @brief Construct EKF SLAM pipeline.
   */
  EkfSlamPipeline(
    std::shared_ptr<ExtendedKalmanFilter> filter,
    std::shared_ptr<BaseAssociation> association,
    std::shared_ptr<MeasurementFactory> measurementFactory);

  /**
   * @brief Initialize pipeline internal state.
   */
  void initialize() override;

  void reset() override;
  
  void processMotion(const MotionConstraint& m) override;

  void processObservation(const Observations& o) override;

  MapSummary getMap() const override;

  void setLogger(LoggerPtr logger) override;

  void applyStartupAnchor(const LocalFrameAnchor& anchor) override;

private:
  void onBackendUpdate(const MapSummary& map);
  void updateFilter(const MotionConstraint& m);
  void correctFilter(const AssignedMeasurements& meas);

  std::shared_ptr<ExtendedKalmanFilter> _ekf; ///< EKF implementation
  std::shared_ptr<BaseAssociation> _association; ///< Data association
  std::shared_ptr<MeasurementFactory> _measurementFactory; ///< Measurement factory
  bool _hasMotion = false; ///< Motion received flag
  bool _hasObservation = false; ///< Observation received flag
  std::mutex _mutex; ///< Synchronization
  LoggerPtr _logger; ///< Logger instance
};

} // namespace slam

#endif  // SLAM__PIPELINE__EKF_SLAM_PIPELINE_HPP_
