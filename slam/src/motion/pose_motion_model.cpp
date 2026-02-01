#include "motion/pose_motion_model.hpp"

/**
 * @brief Full pose motion model.
 *
 * State: (x, y, z, roll, pitch, yaw)
 * Control / odometry: Δpose (already computed upstream)
 *
 * Orientation is explicitly part of the state.
 */

int PoseMotionModel::getMotionDimension()
{
    return 6;  // x, y, z, roll, pitch, yaw
}

Eigen::MatrixXd PoseMotionModel::getRobotToRobotJacobian()
{
    // First-order EKF approximation
    // x_k+1 = x_k + u
    return Eigen::MatrixXd::Identity(6, 6);
}

Eigen::MatrixXd PoseMotionModel::getMotionNoise()
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
