#include "common_utilities/transform_util.hpp"

Eigen::Matrix4f TransformUtil::nedToEnuTransform()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = nedToEnuRotation();
    return transform;
}

Eigen::Matrix4f TransformUtil::enuToNedTransform()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = enuToNedRotation();
    return transform;
}

Eigen::Matrix3f TransformUtil::nedToEnuRotation()
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    // Rotate pi/2 around Z-axis
    rotation= Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // Rotate pi around X-axis
    rotation *= Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    return rotation;
}

Eigen::Matrix3f TransformUtil::enuToNedRotation()
{
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();

    // Rotate -pi/2 around Z-axis
    rotation = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // Rotate -pi around X-axis
    rotation *= Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    return rotation;
}

Eigen::Vector3f TransformUtil::nedToEnu(const Eigen::Vector3f& ned_coordinates)
{
    return nedToEnuRotation()*ned_coordinates;
}

Eigen::Vector3f TransformUtil::enuToNed(const Eigen::Vector3f& enu_coordinates)
{
    return enuToNedRotation()*enu_coordinates;
}

Eigen::Vector2f TransformUtil::rotate2D(const Eigen::Vector2f& coordinate, const float rotate)
{
    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << cos(rotate), -sin(rotate),
                      sin(rotate), cos(rotate);

    return rotationMatrix * coordinate;
}

Eigen::Matrix4f TransformUtil::rotateAround(const Eigen::Matrix4f& matrix, const float value, const Orientation direction)
{
    Eigen::Matrix3f rotationMatrix;
    switch (direction) {
        case PITCH:
            rotationMatrix = createRotationMatrix(value, 0.0, 0.0);
            break;
        case ROLL:
            rotationMatrix = createRotationMatrix(0.0, value, 0.0);
            break;
        case YAW:
            rotationMatrix = createRotationMatrix(0.0, 0.0, value);
            break;
        default:
            rotationMatrix = createRotationMatrix(0.0, 0.0, 0.0);
            break;
    }
    Eigen::Matrix4f transitionMatrix;
    transitionMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transitionMatrix(3,3) = 1;
    return matrix * transitionMatrix;
}

Eigen::Matrix3f TransformUtil::createRotationMatrix(double pitch, double roll, double yaw)
{
    Eigen::AngleAxisd rotation_pitch(pitch, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotation_roll(roll, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotation_yaw(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond quaternion = rotation_yaw * rotation_roll * rotation_pitch;
    Eigen::Matrix3d rotation_matrix = quaternion.matrix();
    Eigen::Matrix3f result = rotation_matrix.cast<float>();;

    return result;
}

Eigen::Matrix4f TransformUtil::nedToEnu(const Eigen::Matrix4f& ned_matrix)
{
    return nedToEnuTransform()*ned_matrix;
}

Eigen::Matrix4f TransformUtil::enuToNed(const Eigen::Matrix4f& enu_matrix)
{
    return enuToNedTransform()*enu_matrix;
}

Eigen::Matrix3f TransformUtil::nedToEnu(const Eigen::Matrix3f& ned_matrix)
{
    return nedToEnuRotation()*ned_matrix;
}

Eigen::Matrix3f TransformUtil::enuToNed(const Eigen::Matrix3f& enu_matrix)
{
    return enuToNedRotation()*enu_matrix;
}

// Function to convert yaw from ENU to NED, assuming pitch and roll equal zero
float TransformUtil::convertYawEnuToNed(float yawEnu) {
    float yawNed = -1.0 *yawEnu + (M_PI / 2);
    // Normalize yaw to the range [-PI, PI]
    if (yawNed > M_PI) {
        yawNed -= 2 * M_PI;
    } else if (yawNed < -M_PI) {
        yawNed += 2 * M_PI;
    }
    return yawNed;
}

// Function to convert yaw from NED to ENU, assuming pitch and roll equal zero
float TransformUtil::convertYawNedToEnu(float yawNed) {
    float yawEnu = -1.0 * yawNed + (M_PI / 2);
    // Normalize yaw to the range [-PI, PI]
    if (yawEnu > M_PI) {
        yawEnu -= 2 * M_PI;
    } else if (yawEnu < -M_PI) {
        yawEnu += 2 * M_PI;
    }
    return yawEnu;
}

