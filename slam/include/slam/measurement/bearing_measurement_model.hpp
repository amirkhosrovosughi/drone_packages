#ifndef SLAM__MEASUREMENT__BEARING_MEASUREMENT_MODEL_HPP_
#define SLAM__MEASUREMENT__BEARING_MEASUREMENT_MODEL_HPP_

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
        const Pose& robotPose,
        const Position& landmarkPosition
    ) override;

    Eigen::MatrixXd jacobianWrtRobot(
        const Pose& robotPose,
        const Position& landmarkPosition
    ) const override;

    Eigen::MatrixXd jacobianWrtLandmark(
        const Pose& robotPose,
        const Position& landmarkPosition
    ) const override;

    Eigen::MatrixXd measurementNoise() const override;

    std::optional<Position> inverse(const Pose& robotPose, const Measurement& m) const override;

    /**
     * @brief Convert a bearing measurement into a world-frame camera ray.
     */
    bool worldRayFromMeasurement(
        const Pose& robotPose,
        const Measurement& m,
        Eigen::Vector3d& rayOriginWorld,
        Eigen::Vector3d& rayDirectionWorld) const;

private:
    void assertCameraInfoAvailable() const;
    std::optional<CameraInfo> _cameraInfo;
};

#endif  // SLAM__MEASUREMENT__BEARING_MEASUREMENT_MODEL_HPP_
