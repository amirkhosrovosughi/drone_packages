#include "filter/models/position_position_motion_measurement_model.hpp"
#include <iostream>

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
    // -> TODO: can make it more general, with passing idendity and using it later
    // measurement is in robot frame, so don't need to do that
    // _sensorTranformation = transform;
    // _sensorTranformationLoaded = true;
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
bool PositionPositionMotionMeasurementModel::directObservationModel(const Pose& robotPose,
    const Position& landmarkPosition, Measurement& expectedMeasurement)
{
    std::cout << " ------- directObservationModel ------" << std::endl;
    std::cout << " ... robotPose is:" << robotPose.position.getPositionVector() << std::endl;
    std::cout << " ... landmarkPosition is " << landmarkPosition.getPositionVector() << std::endl;

    // if (!_sensorTranformationLoaded)
    // {
    //     return false;
    // }
    // Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix()* _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame

    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix(); // * _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame
    Eigen::Vector4d homogeneousLandmarkPosition;
    homogeneousLandmarkPosition.head<3>() = landmarkPosition.getPositionVector();
    homogeneousLandmarkPosition(3) = 1.0; //AMIR-> why 1.0, should not be zero ???

    std::cout << " ... homogeneousLandmarkPosition is: \n " << homogeneousLandmarkPosition << std::endl;
    std::cout << " ... robotTransformation.inverse() is: \n " << robotTransformation.inverse() << std::endl;

    Eigen::Vector4d landMarkInRobotCoordinate = robotTransformation.inverse() * homogeneousLandmarkPosition; // something is wrong here, should not be transpose???
    std::cout << " ... landMarkInRobotCoordinate is:" << landMarkInRobotCoordinate << std::endl;
    std::cout << " -------END directObservationModel ------" << std::endl;


    expectedMeasurement.position = Position(landMarkInRobotCoordinate.head<3>());

    return true;
}

Position PositionPositionMotionMeasurementModel::inverseObservationModel(const Pose& robotPose,
    const Measurement measurement)
{
    std::cout << " --- robotPose position is:" << robotPose.position.getPositionVector() << std::endl;
    std::cout << " --- robotPose quaternion is:" << robotPose.quaternion.getVector() << std::endl;
    std::cout << " --- measurement is " << measurement.position.getPositionVector() << std::endl;
    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix();

    std::cout << "--- robotTransformation is:"<< "\n" << robotTransformation << std::endl;

    Eigen::Vector4d homogeneousLandmarkPosition;
    homogeneousLandmarkPosition.head<3>() = measurement.position.getPositionVector();
    homogeneousLandmarkPosition(3) = 1.0;

    Eigen::Vector4d landmarkPostion = robotTransformation * homogeneousLandmarkPosition;
    std::cout << " --- landmarkPostion is:" << landmarkPostion << std::endl;

    return Position(landmarkPostion.head<3>().eval());
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
