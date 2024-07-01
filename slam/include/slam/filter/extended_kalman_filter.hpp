#ifndef SLAM__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"
#include "filter/models/motion_model.hpp"
#include "filter/models/pose_odometry_motion_model.hpp"
#include "filter/models/position_odometry_motion_model.hpp"
#include "filter/models/measurement_model.hpp"
#include "filter/models/position_measurement_model.hpp"
#include "filter/slam_map.hpp"

#include <functional>
#include <iostream>
#include <mutex>
#include <memory>

class ExtendedKalmanFilter : public KalmanFilter {
public:
    ExtendedKalmanFilter();
    void prediction(const OdometryInfo& odom) override;
    void correction(const Measurements& meas) override;
    void registerCallback(std::function<void(const Map& map)> callback) override;

private:
    void processPrediction(const OdometryInfo& odom);
    void processCorrection(const Measurements& meas);

private:
    std::function<void(const Map& map)> _callback;
    std::mutex _mutex;

    std::unique_ptr<MotionModel> _motionModel;
    std::unique_ptr<MeasurementModel> _measurementModel;
    std::shared_ptr<SlamMap> _slamMap;
};

#endif  // SLAM__EXTENDED_KALMAN_FILTER_HPP_