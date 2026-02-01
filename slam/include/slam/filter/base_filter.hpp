#ifndef SLAM__FILTER_HPP_
#define SLAM__FILTER_HPP_

#include <functional>
#include <Eigen/Dense>
#include "common/def_slam.hpp"
#include "common/slam_logger.hpp"
#include "measurement/measurement.hpp"

/**
 * @brief Abstract base class for all SLAM filters.
 *
 * Defines the core interface for prediction, correction, and map callbacks.
 * Specific implementations (e.g., EKF, UKF, FastSLAM) must implement these methods.
 */
class BaseFilter {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~BaseFilter() = default;

    /**
     * @brief Perform the filter prediction step.
     * @param odom Odometry information used for motion update
     */
    virtual void prediction(const PredictionInput& predictionInpu) = 0;

    /**
     * @brief Perform the filter correction step.
     * @param meas Measurements used for observation update
     */
    virtual void correction(const AssignedMeasurements& meas) = 0;

    /**
     * @brief Register callback to return map updates.
     * @param callback Function receiving a MapSummary
     */
    virtual void registerCallback(std::function<void(const MapSummary& map)> callback) = 0;

    /**
     * @brief Set sensor information (e.g., extrinsic parameters).
     * @param transform 4x4 homogeneous transform from sensor to robot frame
     */
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;

    /**
     * @brief Set logger instance for debug/info messages.
     * @param logger Shared pointer to logger
     */
    virtual void setLogger(LoggerPtr logger) = 0;
};

#endif  // SLAM__FILTER_HPP_