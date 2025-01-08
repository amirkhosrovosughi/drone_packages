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
    if (yawNed > M_PI)
    {
        yawNed -= 2 * M_PI;
    } else if (yawNed < -M_PI)
    {
        yawNed += 2 * M_PI;
    }
    return yawNed;
}

// Function to convert yaw from NED to ENU, assuming pitch and roll equal zero
float TransformUtil::convertYawNedToEnu(float yawNed) {
    float yawEnu = -1.0 * yawNed + (M_PI / 2);
    // Normalize yaw to the range [-PI, PI]
    if (yawEnu > M_PI)
    {
        yawEnu -= 2 * M_PI;
    } else if (yawEnu < -M_PI)
    {
        yawEnu += 2 * M_PI;
    }
    return yawEnu;
}

/**
 * @brief Converts a quaternion from the NED (North-East-Down) coordinate system 
 *        to the ENU (East-North-Up) coordinate system.
 * 
 * This function takes a quaternion representing a rotation in the NED frame
 * and transforms it into the equivalent quaternion in the ENU frame by applying:
 * - A π/2 rotation about the Z-axis (down).
 * - A π rotation about the X-axis (old North → new East).
 * 
 * The transformation is performed using the following quaternions:
 * Q_Z (90° rotation about Z-axis): [sqrt(0.5), 0, 0, sqrt(0.5)].
 * Q_X (180° rotation about X-axis): [0, 1, 0, 0].
 * 
 * Transformation:
 * Q_ENU = Q_X * Q_Z * Q_NED * Q_Z^(-1) * Q_X^(-1)
 * 
 * @param nedQuat Eigen::Vector4d representing the quaternion in NED coordinates
 *                 in the order [w, x, y, z].
 * @return Eigen::Vector4d representing the quaternion in ENU coordinates
 *         in the order [w, x, y, z].
 */
Eigen::Vector4d TransformUtil::nedToEnuQuaternion(const Eigen::Vector4d& nedQuat)
{
    // Define the transformation quaternion for a 90-degree rotation about Z-axis
    Eigen::Vector4d qZ;  // [w, x, y, z]
    qZ << std::sqrt(0.5), 0, 0, std::sqrt(0.5);

    // Define the transformation quaternion for a 180-degree rotation about X-axis
    Eigen::Vector4d qX;  // [w, x, y, z]
    qX << 0, 1, 0, 0;

    // Compute the conjugates of qZ and qX
    Eigen::Vector4d qZConj, qXConj;
    qZConj << qZ(0), -qZ(1), -qZ(2), -qZ(3);
    qXConj << qX(0), -qX(1), -qX(2), -qX(3);

    // Step 1: Apply qZ to rotate 90° around Z-axis
    Eigen::Vector4d tempQuat = quaternionMultiplication(qZ, nedQuat);
    tempQuat = quaternionMultiplication(tempQuat, qZConj);

    // Step 2: Apply qX to rotate 180° around X-axis
    Eigen::Vector4d enuQuat = quaternionMultiplication(qX, tempQuat);
    enuQuat = quaternionMultiplication(enuQuat, qXConj);

    // Normalize the resulting quaternion
    enuQuat.normalize();

    return enuQuat;
}

/**
 * @brief Helper function to perform quaternion multiplication.
 * 
 * @param q1 First quaternion as Eigen::Vector4d [w, x, y, z].
 * @param q2 Second quaternion as Eigen::Vector4d [w, x, y, z].
 * @return Eigen::Vector4d representing the product q1 * q2.
 */
Eigen::Vector4d TransformUtil::quaternionMultiplication(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2)
{
    double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
    double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);

    double w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    double x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    double y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    double z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

    return Eigen::Vector4d(w, x, y, z);
}
