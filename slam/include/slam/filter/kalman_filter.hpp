#ifndef SLAM__KALMAN_FILTER_HPP_
#define SLAM__KALMAN_FILTER_HPP_

#include "base_filter.hpp"

class KalmanFilter : public BaseFilter {
public:
    virtual ~KalmanFilter() = default;
};

#endif  // SLAM__KALMAN_FILTER_HPP_