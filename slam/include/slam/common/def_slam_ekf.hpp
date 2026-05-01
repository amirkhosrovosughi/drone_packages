#ifndef SLAM__COMMON__DEF_SLAM_EKF_HPP_
#define SLAM__COMMON__DEF_SLAM_EKF_HPP_

#include "common/def_slam_core.hpp"

struct PredictionInput
{

    Eigen::Vector3d deltaPosition;  // or velocity * dt

    // External attitude reference (not estimated)
    Eigen::Quaterniond orientation;

    PredictionInput() = default;
    PredictionInput(MotionConstraint m) {
        this->deltaPosition = m.deltaPosition;
        this->orientation = m.orientation;
    }
};

#endif  // SLAM__COMMON__DEF_SLAM_EKF_HPP_
