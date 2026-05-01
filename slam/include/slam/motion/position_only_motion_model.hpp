#ifndef SLAM__MOTION__POSITION_ONLY_MOTION_MODEL_HPP_
#define SLAM__MOTION__POSITION_ONLY_MOTION_MODEL_HPP_

#include "motion/motion_model.hpp"

/**
 * @brief Motion model for position-only odometry.
 *
 * State: [x, y, z]
 * Control: [dx, dy, dz]
 */
class PositionOnlyMotionModel : public MotionModel {
public:
    /**
     * @brief Get state dimension.
     */
    int getStateDimension() const override;

    /**
     * @brief Propagate position using delta position.
     */
    State propagate(
        const State& currentState,
        const Eigen::VectorXd& motionDisplacement) const override;

    /**
     * @brief Compute Jacobian of motion model.
     */
    Eigen::MatrixXd computeStateJacobian(
        const State& state,
        const Eigen::VectorXd& motionDisplacement) const override;

    /**
     * @brief Get process noise covariance.
     */
    Eigen::MatrixXd getProcessNoise() const override;
};

#endif  // SLAM__MOTION__POSITION_ONLY_MOTION_MODEL_HPP_
