#include "measurement/point3d_measurement_model.hpp"

Point3DMeasurementModel::Point3DMeasurementModel(
    const Eigen::Matrix4d& sensor_extrinsic)
    : _T_sensor_robot(sensor_extrinsic) {}

int Point3DMeasurementModel::measurementDimension() const
{
    return 3;
}

Measurement Point3DMeasurementModel::predict(
    const Pose& robot_pose,
    const Position& landmark_position)
{
    Measurement z_hat;

    Eigen::Vector3d p_r(robot_pose.position.x,
                        robot_pose.position.y,
                        robot_pose.position.z);

    Eigen::Vector3d p_l(landmark_position.x,
                        landmark_position.y,
                        landmark_position.z);

    Eigen::Quaterniond q_wr = normalizedRobotQuaternion(robot_pose);

    Eigen::Matrix3d R_wr = q_wr.toRotationMatrix();

    // Relative position expressed in robot frame.
    Eigen::Vector3d rel = R_wr.transpose() * (p_l - p_r);

    z_hat.payload = Eigen::VectorXd(3);
    z_hat.payload << rel.x(), rel.y(), rel.z();
    z_hat.model = shared_from_this();
    return z_hat;
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtRobot(
    const Pose& robot_pose,
    const Position&) const
{
    Eigen::Quaterniond q_wr = normalizedRobotQuaternion(robot_pose);

    // z = R_wr^T (p_l - p_r)  => ∂z/∂p_r = -R_wr^T
    return -q_wr.toRotationMatrix().transpose();
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtLandmark(
    const Pose& robot_pose,
    const Position&) const
{
    Eigen::Quaterniond q_wr = normalizedRobotQuaternion(robot_pose);

    // z = R_wr^T (p_l - p_r)  => ∂z/∂p_l = R_wr^T
    return q_wr.toRotationMatrix().transpose();
}

Eigen::MatrixXd Point3DMeasurementModel::measurementNoise() const
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);
    R(0, 0) = 0.05;
    R(1, 1) = 0.05;
    R(2, 2) = 0.05;
    return R;
}

std::optional<Position> Point3DMeasurementModel::inverse(
    const Pose& robot_pose, const Measurement& m) const
{
    Eigen::Vector3d p_r(robot_pose.position.x,
                        robot_pose.position.y,
                        robot_pose.position.z);

    Eigen::Quaterniond q_wr = normalizedRobotQuaternion(robot_pose);

    Eigen::Vector3d z = m.payload;
    Eigen::Vector3d p_world = q_wr.toRotationMatrix() * z + p_r;

    return Position(p_world);
}

Eigen::Quaterniond Point3DMeasurementModel::normalizedRobotQuaternion(const Pose& robot_pose)
{
    Eigen::Quaterniond q_wr(robot_pose.quaternion.w,
                            robot_pose.quaternion.x,
                            robot_pose.quaternion.y,
                            robot_pose.quaternion.z);
    q_wr.normalize();
    return q_wr;
}
