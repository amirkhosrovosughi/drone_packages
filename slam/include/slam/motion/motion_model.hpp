#ifndef SLAM_MOTION_MODEL_HPP_
#define SLAM_MOTION_MODEL_HPP_

#include <Eigen/Dense>
#include "common/def_slam.hpp"

/**
 * @brief Abstract base class for robot motion models.
 *
 * A motion model defines how the robot state evolves given control or odometry.
 * It is owned and used exclusively by the SLAM backend (EKF, Graph, etc.).
 */
 
class MotionModel {
public:
    using State = Eigen::VectorXd;

    virtual ~MotionModel() = default;

    /**
     * @brief Get dimension of the motion state.
     * @return int State dimension
     */
    virtual int getStateDimension() const = 0;

    /**
     * @brief Propagate robot state using odometry/control input.
     *
     * @param current_state Current robot state
     * @param control_input Odometry or control vector
     * @param dt Time delta in seconds
     * @return State Propagated robot state
     */
    virtual State propagate(
        const State& current_state,
        const Eigen::VectorXd& motionDisplacement) const = 0;

    /**
     * @brief Compute Jacobian of motion model w.r.t robot state.
     *
     * @param state Current robot state
     * @param control_input Control vector
     * @param dt Time delta
     * @return Eigen::MatrixXd State Jacobian
     */
    virtual Eigen::MatrixXd computeStateJacobian(
        const State& state,
        const Eigen::VectorXd& delta_position) const = 0;

    /**
     * @brief Get motion noise covariance.
     * @return Eigen::MatrixXd Process noise matrix
     */
    virtual Eigen::MatrixXd getProcessNoise() const = 0;
};

#endif  // SLAM_MOTION_MODEL_HPP_
