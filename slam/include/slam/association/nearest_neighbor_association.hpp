#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"
#include "motion/motion_model.hpp"
#include "measurement/measurement_model.hpp"
#include "measurement/measurement.hpp"
#include <cmath>
#include <mutex>
#include "common/slam_logger.hpp"

#ifdef STORE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif

/**
 * @brief Nearest Neighbor Association implementation.
 *
 * Compares incoming measurements to stored landmarks and assigns each measurement
 * to the nearest landmark (if within a threshold) or creates a new landmark.
 */
class NearestNeighborAssociation : public BaseAssociation {
public:
    /**
     * @brief Construct a new NearestNeighborAssociation object.
     */
    NearestNeighborAssociation();

    /**
     * @brief Receive new raw measurements for association.
     *
     * This function typically schedules processing (e.g. on a background thread)
     * and returns quickly.
     *
     * @param meas Vector of incoming measurements.
     */
    void onReceiveMeasurement(const Measurements& meas) override;

    /**
     * @brief Update internal map summary (landmarks + robot pose).
     *
     * Synchronizes internal landmark list and robot pose with the SLAM map.
     *
     * @param map Current map summary.
     */
    void handleUpdate(const MapSummary& map) override;

    /**
     * @brief Register callback to receive associated measurements.
     *
     * @param callback Function to call after association completes.
     */
    void registerCallback(std::function<void(AssignedMeasurements)> callback) override
    {
        _callback = callback;
    }

    /**
     * @brief Set logger instance used by this component.
     *
     * @param logger Shared logger pointer.
     */
    void setLogger(LoggerPtr logger) override
    {
        _logger = logger;
    }

private:
    /**
     * @brief Core measurement-processing routine (does the actual association).
     *
     * @param meas Measurements to process/associate.
     */
    void processMeasurement(const Measurements& meas) override;

    /**
     * @brief Compute Euclidean distance between two landmarks (positions).
     *
     * @param meas Landmark derived from measurement (candidate).
     * @param feature Existing map feature to compare against.
     * @return Euclidean distance.
     */

protected:
    double euclideanDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Compute Mahalanobis distance between two landmarks (placeholder).
     *
     * @param meas Landmark derived from measurement.
     * @param feature Existing map feature to compare against.
     * @return Mahalanobis distance.
     */
    double mahalanobisDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Convert a distance value into a matching score (0..1).
     *
     * @param distance Euclidean distance value.
     * @return Matching score in (0,1].
     */
    double matchingScore(double distance);

private:
    std::function<void(AssignedMeasurements)> _callback;       ///< Callback invoked with associated measurements
    std::mutex _mutex;                                         ///< Mutex to protect internal state
    Landmarks _landmarks;                                      ///< Current list of landmarks
    int _numberLandmarks = 0;                                  ///< Counter used to assign new landmark ids
    Pose _robotPose;                                           ///< Latest robot pose
    LoggerPtr _logger;                                         ///< Logger instance
    double _quaternionRate = 0.0;                              ///< Rate of quaternion change (used for skipping)
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_