#ifndef DEF_SLAM_EKF_HPP_
#define DEF_SLAM_EKF_HPP_

#include "common/def_slam_core.hpp"

struct PredictionInput
{

    Eigen::Vector3d delta_position;  // or velocity * dt

    // External attitude reference (not estimated)
    Eigen::Quaterniond orientation;

    PredictionInput() = default;
    PredictionInput(MotionConstraint m) {
        this->delta_position = m.delta_position;
        this->orientation = m.orientation;
    }
};

#endif // DEF_SLAM_EKF_HPP_
