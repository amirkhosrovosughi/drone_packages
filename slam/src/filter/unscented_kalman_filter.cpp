#include "filter/unscented_kalman_filter.hpp"
#include <iostream>

UnscentedKalmanFilter::UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::prediction() {
    std::cout << "Unscented Kalman Filter prediction step" << std::endl;
    if (_callback) _callback();
}

void UnscentedKalmanFilter::correction() {
    std::cout << "Unscented Kalman Filter correction step" << std::endl;
}

void UnscentedKalmanFilter::registerCallback(std::function<void()> callback) {
    _callback = callback;
}
