#ifndef SLAM__UNSCENTED_KALMAN_FILTER_HPP_
#define SLAM__UNSCENTED_KALMAN_FILTER_HPP_

#include "filter/kalman_filter.hpp"

class UnscentedKalmanFilter : public KalmanFilter {
public:
    UnscentedKalmanFilter();
    void prediction(const OdometryInfo& odom) override;
    void correction(const Measurements& meas) override;
    void registerCallback(std::function<void(const MapSummary& map)> callback) override;
    void setSensorInfo(const Eigen::Matrix4d& transform) override {}

private:
    std::function<void(const MapSummary& map)> _callback;
};

#endif  // SLAM__UNSCENTED_KALMAN_FILTER_HPP_
