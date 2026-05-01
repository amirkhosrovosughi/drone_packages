#ifndef SLAM__FILTER__BASE_FILTER_HPP_
#define SLAM__FILTER__BASE_FILTER_HPP_

#include <functional>
#include <Eigen/Dense>
#include "common/def_slam_core.hpp"
#include "common/def_slam_ekf.hpp"
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
     */
    virtual void prediction(const PredictionInput& predictionInput) = 0;

    /**
     * @brief Perform the filter correction step.
     */
    virtual void correction(const AssignedMeasurements& meas) = 0;

    /**
     * @brief Register callback to return map updates.
     */
    virtual void registerCallback(std::function<void(const MapSummary& map)> callback) = 0;

    /**
     * @brief Set sensor information (e.g., extrinsic parameters).
     */
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;

    /**
     * @brief Set logger instance for debug/info messages.
     */
    virtual void setLogger(LoggerPtr logger) = 0;
};

#endif  // SLAM__FILTER__BASE_FILTER_HPP_