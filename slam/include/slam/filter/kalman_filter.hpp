#ifndef SLAM__KALMAN_FILTER_HPP_
#define SLAM__KALMAN_FILTER_HPP_

#include "base_filter.hpp"

/**
 * @brief Base class for all Kalman filter variants.
 */
class KalmanFilter : public BaseFilter {
public:
    virtual ~KalmanFilter() = default;

};

#endif  // SLAM__KALMAN_FILTER_HPP_