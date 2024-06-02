#ifndef SLAM__UNSCENTED_KALMAN_FILTER_HPP_
#define SLAM__UNSCENTED_KALMAN_FILTER_HPP_

#include "filter/kalman_filter.hpp"

class UnscentedKalmanFilter : public KalmanFilter {
public:
    UnscentedKalmanFilter();
    void prediction() override;
    void correction() override;
    void registerCallback(std::function<void()> callback) override;

private:
    std::function<void()> callback_;
};

#endif  // SLAM__UNSCENTED_KALMAN_FILTER_HPP_
