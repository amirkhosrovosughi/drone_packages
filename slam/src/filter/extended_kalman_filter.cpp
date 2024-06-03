#include "filter/extended_kalman_filter.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::prediction() {
    std::cout << "Extended Kalman Filter prediction step" << std::endl;
    if (_callback) _callback();
}

void ExtendedKalmanFilter::correction() {
    std::cout << "Extended Kalman Filter correction step" << std::endl;
}

void ExtendedKalmanFilter::registerCallback(std::function<void()> callback) {
    _callback = callback;
}