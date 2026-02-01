#include "motion/position_only_motion_model.hpp"

/**
 * @brief Position-only motion model.
 *
 * State: (x, y, z)
 * Control / odometry: Δx, Δy, Δz (already computed upstream)
 *
 * No orientation is modeled.
 */

int PositionOnlyMotionModel::getStateDimension() const
{
    return 3;  // x, y, z
}

MotionModel::State PositionOnlyMotionModel::propagate(
    const State& current_state,
    const Eigen::VectorXd& motionDisplacement) const
{
    // Simple additive model: x_{k+1} = x_k + delta
    MotionModel::State next = current_state;
    if (next.size() == 0) {
        next = MotionModel::State::Zero(3);
    }
    if (motionDisplacement.size() == 3) {
        next.segment(0, 3) = next.segment(0, 3) + motionDisplacement;
    }
    return next;
}

Eigen::MatrixXd PositionOnlyMotionModel::computeStateJacobian(
    const State& /*state*/,
    const Eigen::VectorXd& /*motionDisplacement*/) const
{
    // ∂x_{k+1}/∂x_k = I
    return Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd PositionOnlyMotionModel::getProcessNoise() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(3, 3);
    Q(0, 0) = 0.05;
    Q(1, 1) = 0.05;
    Q(2, 2) = 0.05;
    return Q;
}
