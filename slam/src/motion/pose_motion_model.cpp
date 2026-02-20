#include "motion/pose_motion_model.hpp"

/**
 * @brief Full pose motion model.
 *
 * State: (x, y, z, roll, pitch, yaw)
 * Control / odometry: Δpose (already computed upstream)
 *
 * Orientation is explicitly part of the state.
 */

int PoseMotionModel::getStateDimension() const
{
    return 6;  // x, y, z, roll, pitch, yaw
}

MotionModel::State PoseMotionModel::propagate(
    const State& current_state,
    const Eigen::VectorXd& motionDisplacement) const
{
    MotionModel::State next = current_state;
    if (next.size() == 0) {
        next = MotionModel::State::Zero(6);
    }
    if (motionDisplacement.size() == 6) {
        next.segment(0, 6) = next.segment(0, 6) + motionDisplacement;
    }
    return next;
}

Eigen::MatrixXd PoseMotionModel::computeStateJacobian(
    const State& /*state*/,
    const Eigen::VectorXd& /*delta_position*/) const
{
    // ∂x_{k+1}/∂x_k = I
    return Eigen::MatrixXd::Identity(6, 6);
}

Eigen::MatrixXd PoseMotionModel::getProcessNoise() const
{
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);

    // Position noise
    Q(0, 0) = 0.05;
    Q(1, 1) = 0.05;
    Q(2, 2) = 0.05;

    // Orientation noise
    Q(3, 3) = 0.01;
    Q(4, 4) = 0.01;
    Q(5, 5) = 0.01;

    return Q;
}
