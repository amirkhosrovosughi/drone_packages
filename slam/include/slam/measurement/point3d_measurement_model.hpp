#ifndef POINT3D_MEASUREMENT_MODEL_HPP_
#define POINT3D_MEASUREMENT_MODEL_HPP_

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
        const Eigen::Matrix4d& sensor_extrinsic = Eigen::Matrix4d::Identity()
    );

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
    Eigen::Matrix4d _T_sensor_robot; ///< Sensor extrinsic transform
};

#endif  // POINT3D_MEASUREMENT_MODEL_HPP_
