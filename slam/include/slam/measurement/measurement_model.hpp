#ifndef MEASUREMENT_MODEL_HPP_
#define MEASUREMENT_MODEL_HPP_

#include <Eigen/Dense>
#include <optional>
#include <memory>
#include "common/def_slam.hpp"
#include "observation/observation.hpp"
#include "measurement/measurement.hpp"

/**
 * @brief Abstract base class for measurement models in SLAM.
 *
 * A measurement model defines how an Observation relates
 * to the robot pose and a landmark.
 */
class MeasurementModel : public std::enable_shared_from_this<MeasurementModel> {
public:
    virtual ~MeasurementModel() = default;

    /**
     * @brief Dimension of the measurement vector z.
     */
    virtual int measurementDimension() const = 0;

    /**
     * @brief Predict expected measurement from state.
     *
     * z_hat = h(x, landmark)
     */
    virtual Measurement predict(
        const Pose& robot_pose,
        const Position& landmark_position
    ) = 0;

    /**
     * @brief Jacobian of measurement w.r.t robot pose.
     */
    virtual Eigen::MatrixXd jacobianWrtRobot(
        const Pose& robot_pose,
        const Position& landmark_position
    ) const = 0;

    /**
     * @brief Jacobian of measurement w.r.t landmark.
     */
    virtual Eigen::MatrixXd jacobianWrtLandmark(
        const Pose& robot_pose,
        const Position& landmark_position
    ) const = 0;

    /**
     * @brief Measurement noise covariance.
     */
    virtual Eigen::MatrixXd measurementNoise() const = 0;

    /**
     * @brief Given a robot pose and a raw observation,
     * estimate a landmark position in the world frame.
     */
    virtual std::optional<Position> inverse(const Pose &robot_pose, const Measurement &m) const = 0;
};

#endif  // MEASUREMENT_MODEL_HPP_
