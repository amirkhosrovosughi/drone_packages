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
    const Pose& robotPose,
    const Position& landmarkPosition)
{
    assertCameraInfoAvailable();

    const auto& cam = _cameraInfo.value();

    Measurement z_hat;

    /* -------------------------------
     * World -> Robot -> Camera
     * ------------------------------- */

    Eigen::Vector4d pWorld;
    pWorld << landmarkPosition.x,
              landmarkPosition.y,
              landmarkPosition.z,
              1.0;

    // World -> Robot
    Eigen::Matrix4d tWorldRobot = robotPose.getTransformationMatrix();

    // Robot -> Camera (extrinsics)
    const Eigen::Matrix4d& tRobotCamera = cam.extrinsics;

    // World -> Camera
    Eigen::Vector4d pCamera = tRobotCamera.inverse() * tWorldRobot.inverse() * pWorld;

    double X = pCamera.x();
    double Y = pCamera.y();
    double Z = pCamera.z();

    if (Z <= 0.0)
    {
        throw std::runtime_error("Landmark is behind the camera");
    }

    /* -------------------------------
     * Camera projection
     * ------------------------------- */

    // Normalized image coordinates
    double uNorm = X / Z;
    double vNorm = Y / Z;

    // Bearings (camera frame)
    double yaw   = std::atan(uNorm);
    double pitch = std::atan(vNorm);

    z_hat.payload = Eigen::VectorXd(2);
    z_hat.payload << yaw, pitch;

    // Attach this model instance rather than creating a new copy
    z_hat.model = shared_from_this();

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
    const Pose& /*robotPose*/, const Measurement&) const
{
    // Bearing-only → cannot initialize landmark depth
    return std::nullopt;
}