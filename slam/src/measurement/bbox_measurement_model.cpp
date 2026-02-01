#include "measurement/bbox_measurement_model.hpp"
#include <stdexcept>
#include <cmath>

BBoxMeasurementModel::BBoxMeasurementModel() = default;

void BBoxMeasurementModel::setCameraIntrinsics(const Eigen::Matrix3d& K)
{
    _K = K;
}

bool BBoxMeasurementModel::hasCameraIntrinsics() const
{
    return _K.has_value();
}

int BBoxMeasurementModel::measurementDimension() const
{
    // bearing: [yaw, pitch]
    return 2;
}

void BBoxMeasurementModel::assertIntrinsicsAvailable() const
{
    if (!_K.has_value()) {
        throw std::runtime_error(
            "BBoxMeasurementModel: Camera intrinsics not set");
    }
}

Measurement BBoxMeasurementModel::predict(
    const Pose& robot_pose,
    const Position& landmark_position) const
{
    assertIntrinsicsAvailable();

    Measurement z_hat;

    Eigen::Vector3d p_r(robot_pose.position.x,
                        robot_pose.position.y,
                        robot_pose.position.z);

    Eigen::Vector3d p_l(landmark_position.x,
                        landmark_position.y,
                        landmark_position.z);

    Eigen::Vector3d rel = p_l - p_r;

    double x = rel.x();
    double y = rel.y();
    double z = rel.z();

    // Bearing-only measurement
    double yaw   = std::atan2(y, x);
    double pitch = std::atan2(z, std::sqrt(x * x + y * y));

    z_hat.payload = Eigen::VectorXd(2);
    z_hat.payload << yaw, pitch;
    z_hat.model = std::make_shared<BBoxMeasurementModel>();

    return z_hat;
}

Eigen::MatrixXd BBoxMeasurementModel::jacobianWrtRobot(
    const Pose&,
    const Position&) const
{
    // Placeholder: proper Jacobian will be implemented later
    return Eigen::MatrixXd::Zero(2, 3);
}

Eigen::MatrixXd BBoxMeasurementModel::jacobianWrtLandmark(
    const Pose&,
    const Position&) const
{
    // Placeholder: proper Jacobian will be implemented later
    return Eigen::MatrixXd::Zero(2, 3);
}

Eigen::MatrixXd BBoxMeasurementModel::measurementNoise() const
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = 0.01;  // yaw noise
    R(1, 1) = 0.01;  // pitch noise
    return R;
}

std::optional<Position> BBoxMeasurementModel::inverse(
    const Pose& robot_pose,
    const Measurement& m
) const
{
    // cannot initialize a 3D landmark from a single bearing
    return std::nullopt;
}
