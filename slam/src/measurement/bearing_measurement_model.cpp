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
     * Camera projection -> bearings
     * ------------------------------- */

    double uNorm = X / Z;
    double vNorm = Y / Z;

    double yaw   = std::atan(uNorm);
    double pitch = std::atan(vNorm);

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
    const Pose& robotPose, const Measurement& m) const
{
    assertCameraInfoAvailable();

    if (m.payload.size() < 2)
    {
        return std::nullopt;
    }

    // Use the same default depth prior as EKF bearing initialization.
    constexpr double kDefaultDepthMeters = 6.0;

    Eigen::Vector3d rayOriginWorld;
    Eigen::Vector3d rayDirectionWorld;
    if (!worldRayFromMeasurement(robotPose, m, rayOriginWorld, rayDirectionWorld))
    {
        return std::nullopt;
    }

    const Eigen::Vector3d pWorld = rayOriginWorld + kDefaultDepthMeters * rayDirectionWorld;
    return Position(pWorld);
}

bool BearingMeasurementModel::worldRayFromMeasurement(
    const Pose& robotPose,
    const Measurement& m,
    Eigen::Vector3d& rayOriginWorld,
    Eigen::Vector3d& rayDirectionWorld) const
{
    assertCameraInfoAvailable();

    if (m.payload.size() < 2)
    {
        return false;
    }

    const double yaw = m.payload(0);
    const double pitch = m.payload(1);

    Eigen::Vector3d dirCamera(std::tan(yaw), std::tan(pitch), 1.0);
    const double norm = dirCamera.norm();
    if (norm <= 1e-9)
    {
        return false;
    }
    dirCamera /= norm;

    const auto& cam = _cameraInfo.value();
    const Eigen::Matrix4d tWorldRobot = robotPose.getTransformationMatrix();
    const Eigen::Matrix4d& tRobotCamera = cam.extrinsics;
    const Eigen::Matrix4d tWorldCamera = tWorldRobot * tRobotCamera;

    rayOriginWorld = tWorldCamera.block<3, 1>(0, 3);
    rayDirectionWorld = tWorldCamera.block<3, 3>(0, 0) * dirCamera;

    const double directionNorm = rayDirectionWorld.norm();
    if (directionNorm <= 1e-9)
    {
        return false;
    }

    rayDirectionWorld /= directionNorm;
    return true;
}
