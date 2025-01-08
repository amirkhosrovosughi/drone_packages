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
    // std::cout << " ------- directObservationModel ------" << std::endl;
    // std::cout << " ... robotPose is:" << robotPose.position.getPositionVector() << std::endl;
    // std::cout << " ... landmarkPosition is " << landmarkPosition.getPositionVector() << std::endl;
 
    // if (!_sensorTranformationLoaded) // TODO: uncomment after passing  _sensorTranformation
    // {
    //     return false;
    // }
    // Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix()* _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame

    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix(); // * _sensorTranformation; -> not necessary as we send landmark coordinate in robot frame
    Eigen::Vector4d homogeneousLandmarkPosition;
    homogeneousLandmarkPosition.head<3>() = landmarkPosition.getPositionVector();
    homogeneousLandmarkPosition(3) = 1.0; //AMIR-> why 1.0, should not be zero ???

    // std::cout << " ... homogeneousLandmarkPosition is: \n " << homogeneousLandmarkPosition << std::endl;
    // std::cout << " ... robotTransformation.inverse() is: \n " << robotTransformation.inverse() << std::endl;

    Eigen::Vector4d landMarkInRobotCoordinate = robotTransformation.inverse() * homogeneousLandmarkPosition; // something is wrong here, should not be transpose???
    // std::cout << "directObservationModel ... landMark In Robot Coordinate is:" << landMarkInRobotCoordinate << std::endl;
    // std::cout << " -------END directObservationModel ------" << std::endl;


    expectedMeasurement.position = Position(landMarkInRobotCoordinate.head<3>());

    return true;
}

Position PositionPositionMotionMeasurementModel::inverseObservationModel(const Pose& robotPose,
    const Measurement measurement)
{
    // std::cout << " --- robotPose position is:" << robotPose.position.getPositionVector() << std::endl;
    // std::cout << " --- robotPose quaternion is:" << robotPose.quaternion.getVector() << std::endl;
    // std::cout << " --- measurement is " << measurement.position.getPositionVector() << std::endl;
    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix();

    // std::cout << "--- robotTransformation is:"<< "\n" << robotTransformation << std::endl;

    Eigen::Vector4d homogeneousLandmarkPosition;
    homogeneousLandmarkPosition.head<3>() = measurement.position.getPositionVector();
    homogeneousLandmarkPosition(3) = 1.0;

    Eigen::Vector4d landmarkPostion = robotTransformation * homogeneousLandmarkPosition;
    // std::cout << "inverseObservationModel --- landmark absolute Postion is:" << landmarkPostion << std::endl;

    return Position(landmarkPostion.head<3>().eval());
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementToRobotJacobian(const Pose& robotPose)
{
    Eigen::Quaterniond q(robotPose.quaternion.w, robotPose.quaternion.x, robotPose.quaternion.y, robotPose.quaternion.z);
    // Normalize the quaternion (optional if you know it's already normalized)
    q.normalize();
    return -1*q.toRotationMatrix().inverse();

    /*
    Background for Jacobian Calculation:

    Given:
        y = R^(-1) * L
        - y is a 4x1 vector: [y1, y2, y3, y4]
        - R is a 4x4 transformation matrix:
            R = [ R_3x3  t ]
                [  0    1 ]
        where:
            - R_3x3 is the top-left 3x3 rotation submatrix.
            - t = [t1, t2, t3]^T represents the translation components (first three rows of the last column of R).
        - L is a 4x1 position vector in homogeneous coordinates:
            L = [ l1, l2, l3, 1 ]^T,
        where [l1, l2, l3]^T is the 3D position.

    We are interested in:
        y_truncated = [y1, y2, y3]^T
        and its Jacobian with respect to t = [t1, t2, t3]^T.

    Step 1: Simplified y_truncated
        Using the block structure of R^(-1), the first three elements of y are:
            y_truncated = R_3x3^(-1) * l - R_3x3^(-1) * t,
        where:
            - l = [l1, l2, l3]^T (first three elements of L).
            - t = [t1, t2, t3]^T (translation components of R).

    Step 2: Jacobian of y_truncated w.r.t. t
        The derivative of y_truncated with respect to t is:
            J_truncated = ∂y_truncated / ∂t
        Expanding:
            J_truncated = -R_3x3^(-1).

    Final Result:
        - If L is a position vector in homogeneous coordinates, the 4th element of L is always 1.
        - Therefore, the Jacobian of y_truncated with respect to t simplifies to:
            J_truncated = -R_3x3^(-1).
    */
}


Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementToMeasurementJacobian(const Pose& robotPose)
{
    Eigen::Quaterniond q(robotPose.quaternion.w, robotPose.quaternion.x, robotPose.quaternion.y, robotPose.quaternion.z);
    // Normalize the quaternion (optional if you know it's already normalized)
    q.normalize();
    // std::cout << ">---> q.toRotationMatrix() is:"<< "\n" << q.toRotationMatrix().inverse() << std::endl;

    Eigen::Matrix4d robotTransformation = robotPose.getTransformationMatrix();
    Eigen::Matrix4d robotTransformationInserve = robotTransformation.inverse();
    // std::cout << ">---> robotTransformationInserve is:"<< "\n" << robotTransformationInserve << std::endl;

    return q.toRotationMatrix().inverse();
    /*
    Background for jacobian calculation:

    From aboive
    y_truncated = R_3x3^(-1) * l - R_3x3^(-1) * t,
    */
}

Eigen::MatrixXd PositionPositionMotionMeasurementModel::getMeasurementNoise()
{
    return Eigen::Matrix<double, 3, 3>::Identity()*0.2f; // Process noise covariance (tune this)
                                                         // this can be angular velocity of drone
}
