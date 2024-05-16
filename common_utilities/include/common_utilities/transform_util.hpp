#ifndef COMMON_UTILITIES_TRANSFORM_UTIL_HPP_
#define COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

#include <Eigen/Dense>

class TransformUtil {
private:
    static Eigen::Matrix4f nedToEnuTransform();
    static Eigen::Matrix4f enuToNedTransform();
    static Eigen::Matrix3f createRotationMatrix(double pitch, double roll, double yaw);

public:
    enum Orientation {
        PITCH,
        ROLL,
        YAW
    };

    static Eigen::Vector3f nedToEnu(const Eigen::Vector3f& ned_coordinates);
    static Eigen::Vector3f enuToNed(const Eigen::Vector3f& enu_coordinates);
    static Eigen::Vector2f rotate2D(const Eigen::Vector2f& coordinate, const float rotate);
    static Eigen::Matrix4f rotateAround(const Eigen::Matrix4f& matrix, const float value, const Orientation axis);
    static Eigen::Matrix4f nedToEnu(const Eigen::Matrix4f& ned_coordinates);
    static Eigen::Matrix4f enuToNed(const Eigen::Matrix4f& enu_coordinates);
};

#endif  // COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

