#ifndef SLAM__MOTION__POSE_MOTION_MODEL_HPP_
#define SLAM__MOTION__POSE_MOTION_MODEL_HPP_

#include "motion/motion_model.hpp"

/**
 * @brief Motion model for full pose odometry.
 *
 * State: [x, y, z, roll, pitch, yaw] or SE(3)
 * Control: linear + angular velocity
 */
class PoseMotionModel : public MotionModel {
public:
    /**
     * @brief Get pose state dimension.
     */
    int getStateDimension() const override;

    /**
     * @brief Propagate pose using velocity-based odometry.
     */
    State propagate(
        const State& currentState,
        const Eigen::VectorXd& motionDisplacement) const override;

    /**
     * @brief Compute motion Jacobian.
     */
    Eigen::MatrixXd computeStateJacobian(
        const State& state,
        const Eigen::VectorXd& motionDisplacement) const override;

    /**
     * @brief Get motion noise covariance.
     */
    Eigen::MatrixXd getProcessNoise() const override;
};

#endif  // SLAM__MOTION__POSE_MOTION_MODEL_HPP_
