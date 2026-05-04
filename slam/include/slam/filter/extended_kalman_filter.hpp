#ifndef SLAM__FILTER__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__FILTER__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"
#include "motion/motion_model.hpp"
#include "measurement/measurement_model.hpp"
#include "map/slam_map.hpp"

#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <memory>

/**
 * @brief Extended Kalman Filter (EKF) implementation for SLAM.
 *
 * Handles prediction and correction steps using odometry and landmark measurements.
 * Maintains robot state, landmark map, and covariances in a SlamMap structure.
 */
class ExtendedKalmanFilter : public KalmanFilter {
public:
    /**
     * @brief Construct a new ExtendedKalmanFilter object.
     */
    ExtendedKalmanFilter(
        std::shared_ptr<MotionModel> motionModel);

    /**
     * @brief EKF prediction step.
     */
    void prediction(const PredictionInput& predictionInput) override;

    /**
     * @brief EKF correction step using measurements.
     */
    void correction(const AssignedMeasurements& meas) override;

    /**
     * @brief Register callback to return map updates.
     */
    void registerCallback(std::function<void(const MapSummary& map)> callback) override;

    /**
     * @brief Set sensor information (e.g., camera extrinsics).
     */
    void setSensorInfo(const Eigen::Matrix4d& transform) override;

    /**
     * @brief Set logger instance for debug/info messages.
     */
    void setLogger(LoggerPtr logger) override;

    MapSummary getMap();

    void reset();

    /**
     * @brief Fuse a GPS absolute-position fix into the full joint state.
     *
     * Applies a standard EKF update with H = [I₃ | 0] and
     * R = diag(sigma²_XY, sigma²_XY, sigma²_Z) over the full (robot + landmark)
     * covariance in one shot.  Must not be called before the first prediction.
     */
    void applyAbsolutePositionCorrection(const AbsolutePositionConstraint& constraint);

private:
    /**
     * @brief Internal EKF prediction step.
     */
    void processPrediction(const PredictionInput& odom);

    /**
     * @brief Internal EKF correction step.
     */
    void processCorrection(const AssignedMeasurements& meas);

    /**
     * @brief EKF update for an existing landmark.
     */
    void updateLandmark(const AssignedMeasurement& meas);

    /**
     * @brief Add a new landmark to the map.
     */
    void addLandmark(const AssignedMeasurement& meas);

    /**
     * @brief Build a summary of the current map state.
     */
    MapSummary summarizeMap();

    /**
     * @brief Get current system time in seconds.
     */
    double getCurrentTimeInSeconds();

private:
    std::function<void(const MapSummary& map)> _callback; ///< Map update callback
    std::mutex _mutex; ///< Synchronization for prediction/correction
    std::shared_ptr<MotionModel> _motionModel;          ///< Robot motion model
    // measurement model is provided per-measurement via MeasurementFactory
    // std::shared_ptr<MeasurementModel> _measurementModel;///< Landmark measurement model
    std::shared_ptr<SlamMap> _slamMap; ///< SLAM state map
    std::map<int, int> _landmarkObservationCount; ///< Landmark observation counts
    Quaternion _robotQuaternion; ///< Current robot orientation
    LoggerPtr _logger; ///< Logger instance
};

#endif  // SLAM__FILTER__EXTENDED_KALMAN_FILTER_HPP_