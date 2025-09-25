#ifndef MOTION_MEASUREMENT_MODEL_HPP_
#define MOTION_MEASUREMENT_MODEL_HPP_

#include <Eigen/Dense>
#include <memory>
#include "def_slam.hpp"

/**
 * @brief Abstract base class for motion and measurement models in SLAM.
 *
 * Provides interfaces for robot motion and observation models, 
 * including Jacobians and noise matrices.
 */
class MotionMeasurementModel {
public:
    /**
     * @brief Enumeration of supported odometry types.
     */
    enum class OdometryType {
        PositionOdometry, ///< Odometry based on position only
        PoseOdometry      ///< Odometry based on full pose (position + orientation)
    };

public:
    virtual ~MotionMeasurementModel() = default;

    // motion methods
    /** @name Motion model methods */
    ///@{
    /**
     * @brief Get the dimension of the motion state.
     * @return Dimension of the motion vector.
     */
    virtual int getMotionDimension() = 0;

    /**
     * @brief Compute the Jacobian relating robot state to next robot state.
     * @return Motion Jacobian matrix.
     */
    virtual Eigen::MatrixXd getRobotToRobotJacobian() = 0;

    /**
     * @brief Get the process noise covariance for the motion model.
     * @return Motion noise covariance matrix.
     */
    virtual Eigen::MatrixXd getMotionNoise() = 0;
    ///@}

    // measurement methods
    /** @name Measurement model methods */
    ///@{
    /**
     * @brief Get the dimension of the measurement vector.
     * @return Measurement dimension.
     */
    virtual int getMeasurementDimension() = 0;

    /**
     * @brief Set sensor information (e.g., extrinsic transform).
     * @param transform Homogeneous transformation matrix from sensor to robot frame.
     */
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;

    /**
     * @brief Direct observation model.
     * @param robotPose Robot pose.
     * @param landmarkPosition Landmark position.
     * @param expectedMeasurement Output expected measurement.
     * @return True if successful.
     */
    virtual bool directObservationModel(const Pose& robotPose, const Position& landmarkPosition, Measurement& expectedMeasurement) = 0;

    /**
     * @brief Inverse observation model.
     * @param robotPose Robot pose.
     * @param measurement Measurement object.
     * @return Landmark position inferred from measurement.
     */
    virtual Position inverseObservationModel(const Pose& robotPose, const Measurement measurement) = 0;

    /**
     * @brief Compute the Jacobian of measurement w.r.t. robot pose.
     * @param robotPose Robot pose.
     * @return Measurement-to-robot Jacobian matrix.
     */
    virtual Eigen::MatrixXd getMeasurementToRobotJacobian(const Pose& robotPose) = 0;

    /**
     * @brief Compute the Jacobian of measurement w.r.t. itself (identity check).
     * @param robotPose Robot pose.
     * @return Measurement-to-measurement Jacobian matrix.
     */
    virtual Eigen::MatrixXd getMeasurementToMeasurementJacobian(const Pose& robotPose) = 0;

    /**
     * @brief Get measurement noise covariance.
     * @return Measurement noise covariance matrix.
     */
    virtual Eigen::MatrixXd getMeasurementNoise() = 0;
    ///@}

    /**
     * @brief Get odometry type used by this model.
     * @return Odometry type.
     */
    virtual OdometryType getOdometryType() = 0;

};

#endif  // MOTION_MEASUREMENT_MODEL_HPP_