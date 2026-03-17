#include "measurement/bearing_measurement_model.hpp"
#include <stdexcept>
#include <cmath>

BearingMeasurementModel::BearingMeasurementModel() = default;

void BearingMeasurementModel::setCameraInfo(const CameraInfo& cameraInfo)
{
    _cameraInfo = cameraInfo;
}

int BearingMeasurementModel::measurementDimension() const
{
    return 2; // yaw, pitch
}

void BearingMeasurementModel::assertCameraInfoAvailable() const
{
    if (!_cameraInfo.has_value())
    {
        throw std::runtime_error("Camera info not set in BearingMeasurementModel");
    }
}

Measurement BearingMeasurementModel::predict(
    const Pose& robot_pose,
    const Position& landmark_position)
{
    assertCameraInfoAvailable();

    const auto& cam = _cameraInfo.value();

    Measurement z_hat;

    /* -------------------------------
     * World -> Robot -> Camera
     * ------------------------------- */

    Eigen::Vector4d p_w;
    p_w << landmark_position.x,
           landmark_position.y,
           landmark_position.z,
           1.0;

    // World -> Robot
    Eigen::Matrix4d T_wr = robot_pose.getTransformationMatrix();

    // Robot -> Camera (extrinsics)
    const Eigen::Matrix4d& T_rc = cam.extrinsics;

    // World -> Camera
    Eigen::Vector4d p_c = T_rc.inverse() * T_wr.inverse() * p_w;

    double X = p_c.x();
    double Y = p_c.y();
    double Z = p_c.z();

    if (Z <= 0.0)
    {
        throw std::runtime_error("Landmark is behind the camera");
    }

    /* -------------------------------
     * Camera projection -> bearings
     * ------------------------------- */

    double u_n = X / Z;
    double v_n = Y / Z;

    double yaw   = std::atan(u_n);
    double pitch = std::atan(v_n);

    z_hat.payload = Eigen::VectorXd(2);
    z_hat.payload << yaw, pitch;

    // Attach this model instance rather than creating a new copy
    z_hat.model = shared_from_this();

    return z_hat;
}

Eigen::MatrixXd BearingMeasurementModel::jacobianWrtRobot(
    const Pose&,
    const Position&) const
{
    return Eigen::MatrixXd::Zero(2, 3);
}

Eigen::MatrixXd BearingMeasurementModel::jacobianWrtLandmark(
    const Pose&,
    const Position&) const
{
    return Eigen::MatrixXd::Zero(2, 3);
}

Eigen::MatrixXd BearingMeasurementModel::measurementNoise() const
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = 0.01;  // yaw noise
    R(1, 1) = 0.01;  // pitch noise
    return R;
}

std::optional<Position> BearingMeasurementModel::inverse(
    const Pose& robot_pose, const Measurement& m) const
{
    assertCameraInfoAvailable();

    if (m.payload.size() < 2)
    {
        return std::nullopt;
    }

    // Use the same default depth prior as EKF bearing initialization.
    constexpr double kDefaultDepthMeters = 6.0;

    const double yaw = m.payload(0);
    const double pitch = m.payload(1);

    Eigen::Vector3d dir_camera(std::tan(yaw), std::tan(pitch), 1.0);
    const double norm = dir_camera.norm();
    if (norm <= 1e-9)
    {
        return std::nullopt;
    }
    dir_camera /= norm;

    Eigen::Vector4d p_c;
    p_c << kDefaultDepthMeters * dir_camera.x(),
           kDefaultDepthMeters * dir_camera.y(),
           kDefaultDepthMeters * dir_camera.z(),
           1.0;

    const auto& cam = _cameraInfo.value();
    const Eigen::Matrix4d T_wr = robot_pose.getTransformationMatrix();
    const Eigen::Matrix4d& T_rc = cam.extrinsics;

    const Eigen::Vector4d p_w = T_wr * T_rc * p_c;
    return Position(p_w.head<3>());
}
