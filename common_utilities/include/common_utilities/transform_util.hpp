#ifndef COMMON_UTILITIES_TRANSFORM_UTIL_HPP_
#define COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

#include <Eigen/Dense>

class TransformUtil {
private:
    static Eigen::Matrix4f nedToEnuTransform();
    static Eigen::Matrix4f enuToNedTransform();
public:
    static Eigen::Vector3f nedToEnu(const Eigen::Vector3f& ned_coordinates);
    static Eigen::Vector3f enuToNed(const Eigen::Vector3f& enu_coordinates);
    static Eigen::Vector2f rotate2D(const Eigen::Vector2f& coordinate, const float rotate);
};

#endif  // COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

