#include "filter/models/position_odometry_motion_model.hpp"

static int STATE_DIMENSION = 3;

int PositionOdometryMotionModel::getDimension()
{
    return STATE_DIMENSION;
}

Eigen::VectorXd PositionOdometryMotionModel::stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt)
{
    Velocity velocity = odom.NedVelocity;

    Eigen::Vector3d linearVel{velocity.linear.x, velocity.linear.y, velocity.linear.z};
    Eigen::Vector3d position{state[0], state[1], state[2]};

    return position + linearVel * dt;
}

Eigen::MatrixXd PositionOdometryMotionModel::corrolationUpdate(const Eigen::MatrixXd& covariance)
{
    // Covariance prediction =
    Eigen::Matrix<double, 3, 3> F = Eigen::Matrix<double, 3, 3>::Identity(); 


    Eigen::Matrix<double, 3, 3> Q = Eigen::Matrix<double, 3, 3>::Identity(); // Process noise covariance (tune this)
    Q.block<3, 3>(0, 0) *= 0.1f; // Position noise

    // Update covariance
    return F * covariance * F.transpose() + Q;
}
