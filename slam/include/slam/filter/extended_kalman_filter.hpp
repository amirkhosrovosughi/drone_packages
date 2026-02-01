#ifndef SLAM__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"
#include "motion/motion_model.hpp"
#include "measurement/measurement_model.hpp"
#include "map/slam_map.hpp"

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
    ExtendedKalmanFilter(
        std::shared_ptr<MotionModel> motionModel);

    /**
     * @brief EKF prediction step.
     * @param predictionInput prediction input
     */
    void prediction(const PredictionInput& predictionInput) override;

    /**
     * @brief EKF correction step using measurements.
     * @param meas Landmark measurements
     */
    void correction(const AssignedMeasurements& meas) override;

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

    MapSummary getMap();

    void reset();

private:
/**
     * @brief Internal EKF prediction step.
     * @param odom Odometry information
     */
    void processPrediction(const PredictionInput& odom);

    /**
     * @brief Internal EKF correction step.
     * @param meas Landmark measurements
     */
    void processCorrection(const AssignedMeasurements& meas);

    /**
     * @brief EKF update for an existing landmark.
     * @param meas Measurement of the landmark
     */
    void updateLandmark(const AssignedMeasurement& meas);

    /**
     * @brief Add a new landmark to the map.
     * @param meas Measurement of the landmark
     */
    void addLandmark(const AssignedMeasurement& meas);

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
    std::shared_ptr<MotionModel> _motionModel;          ///< Robot motion model
    // measurement model is provided per-measurement via MeasurementFactory
    // std::shared_ptr<MeasurementModel> _measurementModel;///< Landmark measurement model
    std::shared_ptr<SlamMap> _slamMap; ///< SLAM state map
    std::map<int, int> _landmarkObservationCount; ///< Landmark observation counts
    Quaternion _robotQuaternion; ///< Current robot orientation
    LoggerPtr _logger; ///< Logger instance
};

#endif  // SLAM__EXTENDED_KALMAN_FILTER_HPP_