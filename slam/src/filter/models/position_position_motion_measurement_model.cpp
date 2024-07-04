#include "filter/models/position_position_motion_measurement_model.hpp"

static int MOTION_DIMENSION = 3;
static int MEASUREMENT_DIMENSION = 3;
  
int PositionPositionMotionMeasurementModel::getMotionDimension()
{
    return MOTION_DIMENSION;
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getRobotToRobotJacobian()
{
    return Eigen::Matrix<double, 3, 3>::Identity();
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMotionNoise()
{
    return Eigen::Matrix<double, 3, 3>::Identity()*0.1f; // Process noise covariance (tune this)
}


int PositionPositionMotionMeasurementModel::getMeasurementDimension()
{
    return MEASUREMENT_DIMENSION;
}

void PositionPositionMotionMeasurementModel::setSensorInfo(const Eigen::Matrix4d& transform)
{
    _sensorTranformation = transform;
    _sensorTranformationLoaded = true;
}

 /**
 * @brief obsevation model.
 *
 * Based on give postion of robot and postion of the landmark, return the expected values 3d feature observation
 *
 * @param robotPose robot pose
 * @param landmarkPosition landmark position
 * @param expectedMeasurement the expected values 3d feature detection
 * @return true if it is successful
 */
bool PositionPositionMotionMeasurementModel::directMeasurementModel(const Pose& robotPose,
    const Position& landmarkPosition, Measurement& expectedMeasurement)
{
    // if (!_sensorTranformationLoaded)
    // {
    //     return false;
    // }
    // Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix()* _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame

    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix(); // * _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame
    Eigen::Vector4d homogeneousLandmarkPosition;
    homogeneousLandmarkPosition.head<3>() = landmarkPosition.getPositionVector();
    homogeneousLandmarkPosition(3) = 1.0;

    Eigen::Vector4d landMarkInRobotCoordinate = robotTransformation * homogeneousLandmarkPosition;

    expectedMeasurement.position = Position(landMarkInRobotCoordinate.head<3>());

    return true;
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementToRobotJacobian(const Pose& robotPose)
{
    return Eigen::Matrix<double, 3, 3>::Identity();  //TODO: to be verified
}


Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementToMeasurementJacobian(const Pose& robotPose)
{
    Eigen::Quaterniond q(robotPose.quaternion.w, robotPose.quaternion.x, robotPose.quaternion.y, robotPose.quaternion.z);
    // Normalize the quaternion (optional if you know it's already normalized)
    q.normalize();
    return q.toRotationMatrix(); //TODO: to be verified
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementNoise()
{
    return Eigen::Matrix<double, 3, 3>::Identity()*0.2f; // Process noise covariance (tune this)
                                                         // this can be angular velocity of drone
}
