#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"
#include "filter/models/motion_measurement_model.hpp"
#include "filter/models/position_position_motion_measurement_model.hpp"
#include <cmath>
#include <mutex>
#include "slam_logger.hpp"
#include "data_logging_utils/data_logger.hpp"

class NearestNeighborAssociation : public BaseAssociation {
public:
    NearestNeighborAssociation();
    void onReceiveMeasurement(const Measurements& meas) override;
    void handleUpdate(const MapSummary& map) override;
    void registerCallback(std::function<void(Measurements)> callback) override
    {
        _callback = callback;
    }
    void setLogger(LoggerPtr logger) override
    {
        _logger = logger;
    }

private:
    void processMeasurement(const Measurements& meas) override;
    double euclideanDistance(const Landmark& meas, Landmark feature);
    double mahalanobisDistance(const Landmark& meas, Landmark feature);
    double matchingScore(double distance);

private:
    std::function<void(Measurements)> _callback;
    std::mutex _mutex;
    Landmarks _landmarks;
    int numberLandmarks = 0;
    Pose _robotPose;
    std::shared_ptr<MotionMeasurementModel> _model;
    LoggerPtr _logger;
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_