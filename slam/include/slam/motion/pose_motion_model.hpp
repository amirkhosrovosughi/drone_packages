#ifndef POSE_MOTION_MODEL_HPP_
#define POSE_MOTION_MODEL_HPP_

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
        const State& current_state,
        const Eigen::VectorXd& motionDisplacement) const override;

    /**
     * @brief Compute motion Jacobian.
     */
    Eigen::MatrixXd computeStateJacobian(
        const State& state,
        const Eigen::VectorXd& delta_position) const override;

    /**
     * @brief Get motion noise covariance.
     */
    Eigen::MatrixXd getProcessNoise() const override;
};

#endif  // POSE_MOTION_MODEL_HPP_
