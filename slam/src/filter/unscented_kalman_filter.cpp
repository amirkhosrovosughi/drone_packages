#include "filter/unscented_kalman_filter.hpp"
#include <iostream>

UnscentedKalmanFilter::UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::prediction() {
    std::cout << "Unscented Kalman Filter prediction step" << std::endl;
    if (callback_) callback_();
}

void UnscentedKalmanFilter::correction() {
    std::cout << "Unscented Kalman Filter correction step" << std::endl;
}

void UnscentedKalmanFilter::registerCallback(std::function<void()> callback) {
    callback_ = callback;
}
