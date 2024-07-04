// deprecated file, to be removed

#include "filter/models/pose_odometry_motion_model.hpp"

static int STATE_DIMENSION = 7;

int PoseOdometryMotionModel::getDimension()
{
    return STATE_DIMENSION;
}

Eigen::VectorXd stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt) override;
{
     Velocity velocity = odom.EnuVelocity;
    // Transform linear velocity to inertial frame
    Eigen::Vector3f linearVel{velocity.linear.x, velocity.linear.y, velocity.linear.z};
    Eigen::Vector3f angular_velocity_body{velocity.angular.roll, velocity.angular.pitch, velocity.angular.yaw};

    Eigen::Quaternionf orientation{state[3], state[4], state[5], state[6]};
    Eigen::Vector3f position{state[0], state[1], state[2]};

    Eigen::Vector3f linear_velocity_inertial = bodyToInertial(linearVel, orientation);



    // Update position
    position += linear_velocity_inertial * dt;

    // Update orientation
    orientation = updateOrientation(orientation, angular_velocity_body, dt);

    Eigen::VectorXd result = position;
    return result;
}

Eigen::MatrixXd PoseOdometryMotionModel::corrolationUpdate(const Eigen::MatrixXd& covariance)
{
    // Covariance prediction (simplified)
    Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity(); // State transition matrix
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 7, 7> Q = Eigen::Matrix<double, 7, 7>::Identity(); // Process noise covariance (tune this)
    Q.block<3, 3>(0, 0) *= 0.1f; // Position noise
    Q.block<4, 4>(3, 3) *= 0.1f; // Orientation noise

    // Update covariance
    return F * covariance * F.transpose() + Q;
}

// Method to convert body frame velocity to inertial frame velocity using quaternion
Eigen::Vector3f PoseOdometryMotionModel::bodyToInertial(const Eigen::Vector3f& linear_velocity_body, const Eigen::Quaternionf& orientation)
{
    return orientation * linear_velocity_body;
}

// Metohd to update the orientation quaternion given 3D angular velocity and time step
Eigen::Quaternionf PoseOdometryMotionModel::updateOrientation(const Eigen::Quaternionf& orientation, const Eigen::Vector3f& angular_velocity, float dt)
{
    // Calculate the change in orientation over the time step
    Eigen::Vector3f theta = angular_velocity * dt;
    float angle = theta.norm();
    if (angle > 0) {
        Eigen::Vector3f axis = theta.normalized();
        Eigen::Quaternionf delta_q(Eigen::AngleAxisf(angle, axis));
        return (orientation * delta_q).normalized();
    } else {
        return orientation;
    }
}
