#ifndef MOTION_Measurement_MODEL_HPP_
#define MOTION_Measurement_MODEL_HPP_

#include <Eigen/Dense>
#include "def_slam.hpp"

class MotionMeasurementModel {
public:
    virtual ~MotionMeasurementModel() = default;
    // motion methods
    virtual int getMotionDimension() = 0;
    virtual Eigen::MatrixXd getRobotToRobotJacobian() = 0;
    virtual Eigen::MatrixXd getMotionNoise() = 0;

    // measurement methods
    virtual int getMeasurementDimension() = 0;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;
    virtual bool directMeasurementModel(const Pose& robotPose, const Position& landmarkPosition, Measurement& expectedMeasurement) = 0;
    virtual Eigen::MatrixXd getMeasurementToRobotJacobian(const Pose& robotPose) = 0;
    virtual Eigen::MatrixXd getMeasurementToMeasurementJacobian(const Pose& robotPose) = 0;
    virtual Eigen::MatrixXd getMeasurementNoise() = 0;

};

#endif  // MOTION_Measurement_MODEL_HPP_