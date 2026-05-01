#include "measurement/point3d_measurement_model.hpp"

Point3DMeasurementModel::Point3DMeasurementModel(
    const Eigen::Matrix4d& sensorExtrinsic)
    : _tSensorRobot(sensorExtrinsic) {}

int Point3DMeasurementModel::measurementDimension() const
{
    return 3;
}

Measurement Point3DMeasurementModel::predict(
    const Pose& robotPose,
    const Position& landmarkPosition)
{
    Measurement z_hat;

    Eigen::Vector3d pRobot(robotPose.position.x,
                           robotPose.position.y,
                           robotPose.position.z);

    Eigen::Vector3d pLandmark(landmarkPosition.x,
                              landmarkPosition.y,
                              landmarkPosition.z);

    Eigen::Quaterniond qWorldRobot = normalizedRobotQuaternion(robotPose);

    Eigen::Matrix3d rotWorldRobot = qWorldRobot.toRotationMatrix();

    // Relative position expressed in robot frame.
    Eigen::Vector3d rel = rotWorldRobot.transpose() * (pLandmark - pRobot);

    z_hat.payload = Eigen::VectorXd(3);
    z_hat.payload << rel.x(), rel.y(), rel.z();
    z_hat.model = shared_from_this();
    return z_hat;
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtRobot(
    const Pose& robotPose,
    const Position&) const
{
    Eigen::Quaterniond qWorldRobot = normalizedRobotQuaternion(robotPose);

    // z = R_wr^T (p_l - p_r)  => ∂z/∂p_r = -R_wr^T
    return -qWorldRobot.toRotationMatrix().transpose();
}

Eigen::MatrixXd Point3DMeasurementModel::jacobianWrtLandmark(
    const Pose& robotPose,
    const Position&) const
{
    Eigen::Quaterniond qWorldRobot = normalizedRobotQuaternion(robotPose);

    // z = R_wr^T (p_l - p_r)  => ∂z/∂p_l = R_wr^T
    return qWorldRobot.toRotationMatrix().transpose();
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
    const Pose& robotPose, const Measurement& m) const
{
    Eigen::Vector3d pRobot(robotPose.position.x,
                           robotPose.position.y,
                           robotPose.position.z);

    Eigen::Quaterniond qWorldRobot = normalizedRobotQuaternion(robotPose);

    Eigen::Vector3d z = m.payload;
    Eigen::Vector3d pWorld = qWorldRobot.toRotationMatrix() * z + pRobot;

    return Position(pWorld);
}

Eigen::Quaterniond Point3DMeasurementModel::normalizedRobotQuaternion(const Pose& robotPose)
{
    Eigen::Quaterniond qWorldRobot(robotPose.quaternion.w,
                                   robotPose.quaternion.x,
                                   robotPose.quaternion.y,
                                   robotPose.quaternion.z);
    qWorldRobot.normalize();
    return qWorldRobot;
}
