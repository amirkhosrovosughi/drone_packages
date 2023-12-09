#include "common_utilities/transform_util.hpp"

Eigen::Matrix4f TransformUtil::nedToEnuTransform()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Rotate pi/2 around Z-axis
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // Rotate pi around X-axis
    transform.block<3, 3>(0, 0) *= Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    return transform;
}

Eigen::Matrix4f TransformUtil::enuToNedTransform()
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Rotate -pi/2 around Z-axis
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // Rotate -pi around X-axis
    transform.block<3, 3>(0, 0) *= Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    return transform;
}

Eigen::Vector3f TransformUtil::nedToEnu(const Eigen::Vector3f& ned_coordinates)
{
    Eigen::Matrix4f transform = nedToEnuTransform();
    Eigen::Vector4f ned_vector;
    ned_vector << ned_coordinates, 1.0;

    Eigen::Vector4f enu_vector = transform * ned_vector;

    return enu_vector.head(3);
}

Eigen::Vector3f TransformUtil::enuToNed(const Eigen::Vector3f& enu_coordinates)
{
    Eigen::Matrix4f transform = enuToNedTransform();
    Eigen::Vector4f enu_vector;
    enu_vector << enu_coordinates, 1.0;

    Eigen::Vector4f ned_vector = transform * enu_vector;

    return ned_vector.head(3);
}

Eigen::Vector2f TransformUtil::rotate2D(const Eigen::Vector2f& coordinate, const float rotate)
{
    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << cos(rotate), -sin(rotate),
                      sin(rotate), cos(rotate);

    return rotationMatrix * coordinate;
}


