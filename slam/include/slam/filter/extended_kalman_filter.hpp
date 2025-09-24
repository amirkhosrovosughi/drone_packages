#ifndef SLAM__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"
#include "filter/models/motion_measurement_model.hpp"
#include "filter/models/position_position_motion_measurement_model.hpp"
#include "filter/slam_map.hpp"

#include <functional>
#include <iostream>
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
    ExtendedKalmanFilter();

    /**
     * @brief EKF prediction step.
     * @param odom Odometry information
     */
    void prediction(const OdometryInfo& odom) override;

    /**
     * @brief EKF correction step using measurements.
     * @param meas Landmark measurements
     */
    void correction(const Measurements& meas) override;

    /**
     * @brief Register callback to return map updates.
     * @param callback Function to receive MapSummary
     */
    void registerCallback(std::function<void(const MapSummary& map)> callback) override;

    /**
     * @brief Set sensor information (e.g., camera extrinsics).
     * @param transform 4x4 homogeneous transform
     */
    void setSensorInfo(const Eigen::Matrix4d& transform) override;

    /**
     * @brief Set logger instance for debug/info messages.
     * @param logger Shared pointer to logger
     */
    void setLogger(LoggerPtr logger) override;

private:
/**
     * @brief Internal EKF prediction step.
     * @param odom Odometry information
     */
    void processPrediction(const OdometryInfo& odom);

    /**
     * @brief Internal EKF correction step.
     * @param meas Landmark measurements
     */
    void processCorrection(const Measurements& meas);

    /**
     * @brief EKF update for an existing landmark.
     * @param meas Measurement of the landmark
     */
    void updateLandmark(const Measurement& meas);

    /**
     * @brief Add a new landmark to the map.
     * @param meas Measurement of the landmark
     */
    void addLandmark(const Measurement& meas);

    /**
     * @brief Build a summary of the current map state.
     * @return MapSummary Object containing current state
     */
    MapSummary summarizeMap();

    /**
     * @brief Get current system time in seconds.
     * @return double Time in seconds
     */
    double getCurrentTimeInSeconds();

private:
    std::function<void(const MapSummary& map)> _callback; ///< Map update callback
    std::mutex _mutex; ///< Synchronization for prediction/correction
    std::shared_ptr<MotionMeasurementModel> _model; ///< Motion/measurement model
    std::shared_ptr<SlamMap> _slamMap; ///< SLAM state map
    std::map<int, int> _landmarkObservationCount; ///< Landmark observation counts
    Quaternion _robotQuaternion; ///< Current robot orientation
    MotionMeasurementModel::OdometryType _odometryType; ///< Type of odometry used
    double _lastUpdateTime; ///< Last update timestamp [s]
    LoggerPtr _logger; ///< Logger instance
};

#endif  // SLAM__EXTENDED_KALMAN_FILTER_HPP_