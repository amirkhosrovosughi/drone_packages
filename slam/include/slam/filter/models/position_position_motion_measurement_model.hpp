#ifndef POSITION_ODOMETRY_MOTION__MODEL_HPP_
#define POSITION_ODOMETRY_MOTION__MODEL_HPP_

#include "filter/models/motion_measurement_model.hpp"

#include <Eigen/Dense>
#include "def_slam.hpp"

class PositionPositionMotionMeasurementModel : public MotionMeasurementModel {
// public:
//     virtual int getDimension() override;
//     virtual Eigen::VectorXd stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt) override;
//     virtual Eigen::MatrixXd corrolationUpdate(const Eigen::MatrixXd& state) override;
public:
    // motion methods
    virtual int getMotionDimension() override;
    virtual Eigen::MatrixXd getRobotToRobotJacobian() override;
    virtual Eigen::MatrixXd getMotionNoise() override;

    // measurement methods
    virtual int getMeasurementDimension() override;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) override;
    virtual bool directMeasurementModel(const Pose& robotPose, const Position& landmarkPosition, Measurement& expectedMeasurement) override;
    virtual Eigen::MatrixXd getMeasurementToRobotJacobian(const Pose& robotPose) override;
    virtual Eigen::MatrixXd getMeasurementToMeasurementJacobian(const Pose& robotPose) override;
    virtual Eigen::MatrixXd getMeasurementNoise() override;

private:
    Eigen::Matrix4d _sensorTranformation;
    bool _sensorTranformationLoaded = false;

};

#endif  // POSITION_ODOMETRY_MOTION__MODEL_HPP_