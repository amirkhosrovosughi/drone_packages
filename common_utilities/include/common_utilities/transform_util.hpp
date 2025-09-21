#ifndef COMMON_UTILITIES_TRANSFORM_UTIL_HPP_
#define COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

#include <Eigen/Dense>

/**
 * @brief Utility class for coordinate frame transformations
 *        between NED (North-East-Down) and ENU (East-North-Up).
 */
class TransformUtil {
private:
    /**
     * @brief Utility class for coordinate frame transformations
     *        between NED (North-East-Down) and ENU (East-North-Up).
     */
    static Eigen::Matrix4f nedToEnuTransform();

    /**
     * @brief Generate homogeneous transformation matrix from ENU to NED.
     * @return 4x4 transformation matrix.
     */
    static Eigen::Matrix4f enuToNedTransform();

    /**
     * @brief Generate rotation matrix from NED to ENU.
     * @return 3x3 rotation matrix.
     */
    static Eigen::Matrix3f nedToEnuRotation();

    /**
     * @brief Generate rotation matrix from ENU to NED.
     * @return 3x3 rotation matrix.
     */
    static Eigen::Matrix3f enuToNedRotation();

public:
    /**
     * @brief Enumeration of orientation angles.
     */ 
    enum Orientation {
        PITCH, ///< Pitch angle
        ROLL,  ///< Roll angle
        YAW    ///< Yaw angle
    };

    /**
     * @brief Convert vector from NED to ENU.
     * @param nedVector Input vector in NED.
     * @return Equivalent vector in ENU.
     */
    static Eigen::Vector3f nedToEnu(const Eigen::Vector3f& nedVector);

    /**
     * @brief Convert vector from ENU to NED.
     * @param enuVector Input vector in ENU.
     * @return Equivalent vector in NED.
     */
    static Eigen::Vector3f enuToNed(const Eigen::Vector3f& enuVector);

    /**
     * @brief Rotate a 2D coordinate.
     * @param coordinate Input 2D coordinate.
     * @param rotate Rotation angle (radians).
     * @return Rotated coordinate.
     */
    static Eigen::Vector2f rotate2D(const Eigen::Vector2f& coordinate, const float rotate);

    /**
     * @brief Convert homogeneous transform from NED to ENU.
     * @param nedMatrix 4x4 NED matrix.
     * @return 4x4 ENU matrix.
     */
    static Eigen::Matrix4f nedToEnu(const Eigen::Matrix4f& ned_matrix);

    /**
     * @brief Convert homogeneous transform from ENU to NED.
     * @param enuMatrix 4x4 ENU matrix.
     * @return 4x4 NED matrix.
     */
    static Eigen::Matrix4f enuToNed(const Eigen::Matrix4f& enu_matrix);

    /**
     * @brief Convert rotation matrix from NED to ENU.
     * @param nedRotation 3x3 NED rotation matrix.
     * @return 3x3 ENU rotation matrix.
     */
    static Eigen::Matrix3f nedToEnu(const Eigen::Matrix3f& ned_rotation);

    /**
     * @brief Convert rotation matrix from ENU to NED.
     * @param enuRotation 3x3 ENU rotation matrix.
     * @return 3x3 NED rotation matrix.
     */
    static Eigen::Matrix3f enuToNed(const Eigen::Matrix3f& enu_rotation);

    /**
     * @brief Convert yaw angle from ENU to NED.
     * @param yawEnu Input yaw in ENU (radians).
     * @return Equivalent yaw in NED (radians).
     */
    static float convertYawEnuToNed(float yawEnu);

    /**
     * @brief Convert yaw angle from NED to ENU.
     * @param yawNed Input yaw in NED (radians).
     * @return Equivalent yaw in ENU (radians).
     */
    static float convertYawNedToEnu(float yawNed);

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
     * Q_ENU = Q_X * Q_Z * Q_NED * Q_X^(-1)
     * Reference: https://docs.px4.io/main/en/ros/external_position_estimation
     * 
     * @param nedQuat Eigen::Vector4d representing the quaternion in NED coordinates
     *                 in the order [w, x, y, z].
     * @return Eigen::Vector4d representing the quaternion in ENU coordinates
     *         in the order [w, x, y, z].
     */
    static Eigen::Vector4d nedToEnuQuaternion(const Eigen::Vector4d& nedQuat);

    /**
     * @brief Multiply two quaternions.
     * @param q1 First quaternion [w, x, y, z].
     * @param q2 Second quaternion [w, x, y, z].
     * @return Product quaternion representing the product q1 * q2 [w, x, y, z].
     */
    static Eigen::Vector4d quaternionMultiplication(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

    /**
     * @brief Convert matrix to string for logging/debugging.
     * @tparam Derived Eigen type.
     * @param matrix Input Eigen matrix.
     * @return String representation.
     */
    template<typename Derived>
    static std::string matrixToString(const Eigen::MatrixBase<Derived>& matrix)
    {
        std::stringstream ss;
        ss << matrix;
        return ss.str();
    }
};

#endif  // COMMON_UTILITIES_TRANSFORM_UTIL_HPP_

