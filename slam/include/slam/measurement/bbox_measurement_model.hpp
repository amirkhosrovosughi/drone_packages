#ifndef BBOX_MEASUREMENT_MODEL_HPP_
#define BBOX_MEASUREMENT_MODEL_HPP_

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

    /**
     * @brief Set camera intrinsic matrix.
     *
     * K =
     * [ fx  0  cx ]
     * [ 0  fy  cy ]
     * [ 0   0   1 ]
     */
    void setCameraIntrinsics(const Eigen::Matrix3d& K);

    /**
     * @brief Check whether camera intrinsics are available.
     */
    bool hasCameraIntrinsics() const;

    int measurementDimension() const override;

    Measurement predict(
        const Pose& robot_pose,
        const Position& landmark_position
    ) const override;

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
    void assertIntrinsicsAvailable() const;

    std::optional<Eigen::Matrix3d> _K; ///< Camera intrinsic matrix
};

#endif  // BBOX_MEASUREMENT_MODEL_HPP_
