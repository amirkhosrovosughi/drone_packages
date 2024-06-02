#include "filter/extended_kalman_filter.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::prediction() {
    std::cout << "Extended Kalman Filter prediction step" << std::endl;
    if (callback_) callback_();
}

void ExtendedKalmanFilter::correction() {
    std::cout << "Extended Kalman Filter correction step" << std::endl;
}

void ExtendedKalmanFilter::registerCallback(std::function<void()> callback) {
    callback_ = callback;
}