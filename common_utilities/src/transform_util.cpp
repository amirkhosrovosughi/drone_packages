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

Eigen::Vector3f TransformUtil::nedToEnu(const Eigen::Vector3f& nedVector)
{
    return nedToEnuRotation()*nedVector;
}

Eigen::Vector3f TransformUtil::enuToNed(const Eigen::Vector3f& enuVector)
{
    return enuToNedRotation()*enuVector;
}

Eigen::Vector2f TransformUtil::rotate2D(const Eigen::Vector2f& coordinate, const float rotate)
{
    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << cos(rotate), -sin(rotate),
                      sin(rotate), cos(rotate);

    return rotationMatrix * coordinate;
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

 Eigen::Vector4d TransformUtil::nedToEnuQuaternion(const Eigen::Vector4d& nedQuat) {
        // Input quaternion is in (w, x, y, z) order
        Eigen::Quaterniond qNed(nedQuat[0], nedQuat[1], nedQuat[2], nedQuat[3]);

        // Coordinate frame conversion from NED to ENU:
        // ENU = R_world * NED * R_body⁻¹, 

        // Rotation matrix to convert NED to ENU world frame:
        // 1) +π/2 rotation along z, 2) π rotation along x
        Eigen::Matrix3d R_ned_to_enu;
        R_ned_to_enu <<  0, 1,  0,
                         1, 0,  0,
                         0, 0, -1;

        // Rotation matrix to convert NED to ENU body frame:
        // 1) π rotation along x
        Eigen::Matrix3d R_implace_rotation;
        R_implace_rotation <<  1.0, 0.0,  0.0,
                         0.0, -1.0,  0.0,
                         0.0, 0.0, -1.0;

        // Apply the frame transformation: R * qNed * R⁻¹
        Eigen::Matrix3d rot_ned = qNed.toRotationMatrix();
        // Apply the frame transformation: R_world * qNed * R_body⁻¹
        Eigen::Matrix3d rot_enu = R_ned_to_enu.transpose() * rot_ned * R_implace_rotation; // * R_ned_to_enu;

        Eigen::Quaterniond q_enu(rot_enu);
        q_enu.normalize();

        // Output as (w, x, y, z)
        return Eigen::Vector4d(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
    }

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
