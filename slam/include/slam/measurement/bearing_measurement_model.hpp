#ifndef BEARING_MEASUREMENT_MODEL_HPP_
#define BEARING_MEASUREMENT_MODEL_HPP_

#include <Eigen/Dense>
#include <optional>
#include "measurement/measurement_model.hpp"

/**
 * @brief Measurement model for bearing-only observations (yaw, pitch).
 *
 * Converts a landmark position into a bearing vector expressed in the
 * camera frame. Requires camera intrinsics/extrinsics to be set before use.
 */
class BearingMeasurementModel : public MeasurementModel {
public:
    BearingMeasurementModel();

    void setCameraInfo(const CameraInfo& info);

    int measurementDimension() const override;

    Measurement predict(
        const Pose& robot_pose,
        const Position& landmark_position
    ) override;

    Eigen::MatrixXd jacobianWrtRobot(
        const Pose& robot_pose,
        const Position& landmark_position
    ) const override;

    Eigen::MatrixXd jacobianWrtLandmark(
        const Pose& robot_pose,
        const Position& landmark_position
    ) const override;

    Eigen::MatrixXd measurementNoise() const override;

    std::optional<Position> inverse(const Pose &robot_pose, const Measurement &m) const override;

private:
    void assertCameraInfoAvailable() const;
    std::optional<CameraInfo> _cameraInfo;
};

#endif // BEARING_MEASUREMENT_MODEL_HPP_
