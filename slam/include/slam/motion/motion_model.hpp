#ifndef SLAM__MOTION__MOTION_MODEL_HPP_
#define SLAM__MOTION__MOTION_MODEL_HPP_

#include <Eigen/Dense>

/**
 * @brief Abstract base class for robot motion models.
 *
 * A motion model defines how the robot state evolves given control or odometry.
 * It is owned and used exclusively by the SLAM pipeline (EKF, Graph, etc.).
 */
 
class MotionModel {
public:
    using State = Eigen::VectorXd;

    virtual ~MotionModel() = default;

    /**
     * @brief Get dimension of the motion state.
     */
    virtual int getStateDimension() const = 0;

    /**
     * @brief Propagate robot state using odometry/control input.
     */
    virtual State propagate(
        const State& currentState,
        const Eigen::VectorXd& motionDisplacement) const = 0;

    /**
     * @brief Compute Jacobian of motion model w.r.t robot state.
     */
    virtual Eigen::MatrixXd computeStateJacobian(
        const State& state,
        const Eigen::VectorXd& motionDisplacement) const = 0;

    /**
     * @brief Get motion noise covariance.
     */
    virtual Eigen::MatrixXd getProcessNoise() const = 0;
};

#endif  // SLAM__MOTION__MOTION_MODEL_HPP_
