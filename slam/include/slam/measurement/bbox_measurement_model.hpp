#ifndef SLAM__MEASUREMENT__BBOX_MEASUREMENT_MODEL_HPP_
#define SLAM__MEASUREMENT__BBOX_MEASUREMENT_MODEL_HPP_

#include <Eigen/Dense>
#include <optional>
#include "measurement/measurement_model.hpp"

/**
 * @brief Measurement model for camera bounding box observations.
 *
 * Produces bearing-only measurements (yaw, pitch) from a landmark.
 *
 * Requires camera intrinsics to be set before use.
 */
class BBoxMeasurementModel : public MeasurementModel {
public:
    BBoxMeasurementModel();

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

private:
    void assertCameraInfoAvailable() const;
    std::optional<CameraInfo> _cameraInfo; ///< Camera intrinsic and extrinsic information
};

#endif  // SLAM__MEASUREMENT__BBOX_MEASUREMENT_MODEL_HPP_
