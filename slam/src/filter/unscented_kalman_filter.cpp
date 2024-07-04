#include "filter/unscented_kalman_filter.hpp"
#include <iostream>

UnscentedKalmanFilter::UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::prediction(const OdometryInfo& odom) {
    std::cout << "Unscented Kalman Filter prediction step" << std::endl;
    MapSummary map;
    if (_callback) _callback(map);
}

void UnscentedKalmanFilter::correction(const Measurements& meas) {
    std::cout << "Unscented Kalman Filter correction step" << std::endl;
}

void UnscentedKalmanFilter::registerCallback(std::function<void(const MapSummary& map)> callback) {
    _callback = callback;
}
