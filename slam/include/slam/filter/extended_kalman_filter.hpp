#ifndef SLAM__EXTENDED_KALMAN_FILTER_HPP_
#define SLAM__EXTENDED_KALMAN_FILTER_HPP_

#include "kalman_filter.hpp"

#include <functional>
#include <iostream>
#include <mutex>

class ExtendedKalmanFilter : public KalmanFilter {
public:
    ExtendedKalmanFilter();
    void prediction(const Velocity& velocity) override;
    void correction(const Measurements& meas) override;
    void registerCallback(std::function<void(const Map& map)> callback) override;

private:
    void processPrediction(const Velocity& velocity);
    void processCorrection(const Measurements& meas);

private:
    std::function<void(const Map& map)> _callback;
    std::mutex _mutex;
};

#endif  // SLAM__EXTENDED_KALMAN_FILTER_HPP_