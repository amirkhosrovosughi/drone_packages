#ifndef SLAM__MEASUREMENT__POINT3D_MEASUREMENT_MODEL_HPP_
#define SLAM__MEASUREMENT__POINT3D_MEASUREMENT_MODEL_HPP_

#include "measurement/measurement_model.hpp"

/**
 * @brief Measurement model for direct 3D point observations.
 *
 * Observation: landmark position in robot frame
 * z = Rᵀ (p_landmark - p_robot)
 */
class Point3DMeasurementModel : public MeasurementModel {
public:
    /**
     * @brief Construct model with optional sensor extrinsics.
     */
    explicit Point3DMeasurementModel(
        const Eigen::Matrix4d& sensorExtrinsic = Eigen::Matrix4d::Identity()
    );

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
    static Eigen::Quaterniond normalizedRobotQuaternion(const Pose& robotPose);
    Eigen::Matrix4d _tSensorRobot; ///< Sensor extrinsic transform
};

#endif  // SLAM__MEASUREMENT__POINT3D_MEASUREMENT_MODEL_HPP_
