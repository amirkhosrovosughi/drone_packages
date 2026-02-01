#ifndef SLAM__ASSOCIATION_HPP_
#define SLAM__ASSOCIATION_HPP_

#include <functional>
#include "common/def_slam.hpp"
#include "measurement/measurement.hpp"
#include "common/slam_logger.hpp"

/**
 * @brief Abstract base class for data association in SLAM.
 *
 * Defines the interface for associating sensor measurements with map features.
 */
class BaseAssociation {
public:
    virtual ~BaseAssociation() = default;

    /**
     * @brief Callback for new sensor measurements.
     * @param meas New measurements.
     */
    virtual void onReceiveMeasurement(const Measurements& meas) = 0;

    /**
     * @brief Callback for map updates.
     * @param map Current map summary.
     */
    virtual void handleUpdate(const MapSummary& map) = 0;

    /**
     * @brief Register callback for publishing associated measurements.
     * @param callback Function to be called with processed measurements.
     */
    virtual void registerCallback(std::function<void(AssignedMeasurements)> callback) = 0;

    /**
     * @brief Set logger instance.
     * @param logger Shared logger pointer.
     */
    virtual void setLogger(LoggerPtr logger) = 0;
private:
    /**
     * @brief Internal measurement processing.
     * @param meas Input measurements.
     */
    virtual void processMeasurement(const Measurements& meas) = 0;
};

#endif  // SLAM__ASSOCIATION_HPP_