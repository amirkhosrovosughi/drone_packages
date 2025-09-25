#ifndef POSITION_ODOMETRY_MOTION_MODEL_HPP_
#define POSITION_ODOMETRY_MOTION_MODEL_HPP_

#include "filter/models/motion_measurement_model.hpp"

#include <Eigen/Dense>
#include "def_slam.hpp"

/**
 * @brief Motion and measurement model for position-based odometry with position measurements.
 */
class PositionPositionMotionMeasurementModel : public MotionMeasurementModel {
public:
    /** @name Motion model methods */
    ///@{
    virtual int getMotionDimension() override;
    virtual Eigen::MatrixXd getRobotToRobotJacobian() override;
    virtual Eigen::MatrixXd getMotionNoise() override;
    ///@}

    /** @name Measurement model methods */
    ///@{
    virtual int getMeasurementDimension() override;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) override;
    virtual bool directObservationModel(const Pose& robotPose, const Position& landmarkPosition, Measurement& expectedMeasurement) override;
    virtual Position inverseObservationModel(const Pose& robotPose, const Measurement measurement) override;
    virtual Eigen::MatrixXd getMeasurementToRobotJacobian(const Pose& robotPose) override;
    virtual Eigen::MatrixXd getMeasurementToMeasurementJacobian(const Pose& robotPose) override;
    virtual Eigen::MatrixXd getMeasurementNoise() override;
    virtual OdometryType getOdometryType() {return OdometryType::PositionOdometry;}
    ///@}

private:
    Eigen::Matrix4d _sensorTransformation;          ///< Extrinsic calibration matrix for the sensor
    // bool _sensorTransformationLoaded = false;    ///< Flag indicator if sensor data is loaded

};

#endif  // POSITION_ODOMETRY_MOTION_MODEL_HPP_