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

    // Direct relative position (sensor frame simplification)
    Eigen::Vector3d rel = p_l - p_r;

    z_hat.payload = Eigen::VectorXd(3);
    z_hat.payload << rel.x(), rel.y(), rel.z();
    z_hat.model = shared_from_this();
    return z_hat;
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtRobot(
    const Pose&,
    const Position&) const
{
    // ∂(p_l - p_r)/∂p_r = -I
    return -Eigen::MatrixXd::Identity(3, 3);
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtLandmark(
    const Pose&,
    const Position&) const
{
    // ∂(p_l - p_r)/∂p_l = I
    return Eigen::MatrixXd::Identity(3, 3);
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
    Eigen::Vector4d p_cam;
    p_cam.head<3>() = m.payload;
    p_cam(3) = 1.0;

    Eigen::Vector4d p_world =
        robot_pose.getTransformationMatrix() * p_cam;

    return Position(p_world.head<3>());
}
