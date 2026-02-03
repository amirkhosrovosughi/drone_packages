#include "measurement/bbox_measurement_model.hpp"
#include <stdexcept>
#include <cmath>

BBoxMeasurementModel::BBoxMeasurementModel() = default;

void BBoxMeasurementModel::setCameraInfo(const CameraInfo& cameraInfo)
{
    _cameraInfo = cameraInfo;
}

int BBoxMeasurementModel::measurementDimension() const
{
    // bearing: [yaw, pitch]
    return 2;
}

void BBoxMeasurementModel::assertCameraInfoAvailable() const
{
    if (!_cameraInfo.has_value())
    {
        throw std::runtime_error("Camera info not set in BBoxMeasurementModel");
    }
}

Measurement BBoxMeasurementModel::predict(
    const Pose& robot_pose,
    const Position& landmark_position) const
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
     * Camera projection
     * ------------------------------- */

    // Normalized image coordinates
    double u_n = X / Z;
    double v_n = Y / Z;

    // Bearings (camera frame)
    double yaw   = std::atan(u_n);
    double pitch = std::atan(v_n);

    z_hat.payload = Eigen::VectorXd(2);
    z_hat.payload << yaw, pitch;

    // Preserve camera info inside model copy
    auto model = std::make_shared<BBoxMeasurementModel>();
    model->setCameraInfo(cam);
    z_hat.model = model;

    return z_hat;
}

Eigen::MatrixXd BBoxMeasurementModel::jacobianWrtRobot(
    const Pose&,
    const Position&) const
{
    // TODO: derive analytically (non-trivial due to projection)
    return Eigen::MatrixXd::Zero(2, 3);
}

Eigen::MatrixXd BBoxMeasurementModel::jacobianWrtLandmark(
    const Pose&,
    const Position&) const
{
    // TODO: derive analytically (non-trivial due to projection)
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
    const Pose& robot_pose, const Measurement&) const
{
    // Bearing-only → cannot initialize landmark depth
    return std::nullopt;
}