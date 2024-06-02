#ifndef SLAM__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"

#include <functional>
#include <iostream>

class ExtendedKalmanFilter : public KalmanFilter {
public:
    ExtendedKalmanFilter();
    void prediction() override;
    void correction() override;
    void registerCallback(std::function<void()> callback) override;

private:
    std::function<void()> callback_;
};

#endif  // SLAM__EXTENDED_KALMAN_FILTER_HPP_