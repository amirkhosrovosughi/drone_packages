#include <gtest/gtest.h>
#include "common_utilities/transform_util.hpp"

// using namespace TransformUtil;

// 1. nedToEnu with unit vector along NED x-axis
TEST(TransformUtilTest, NedToEnu_XAxis) {
    Eigen::Vector3f ned(1.0f, 0.0f, 0.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], 0.0f, 1e-5);
    EXPECT_NEAR(enu[1], 1.0f, 1e-5);
    EXPECT_NEAR(enu[2], -0.0f, 1e-5);
}

// 2. nedToEnu with unit vector along NED y-axis
TEST(TransformUtilTest, NedToEnu_YAxis) {
    Eigen::Vector3f ned(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], 1.0f, 1e-5);
    EXPECT_NEAR(enu[1], 0.0f, 1e-5);
    EXPECT_NEAR(enu[2], -0.0f, 1e-5);
}

// 3. nedToEnu with unit vector along NED z-axis
TEST(TransformUtilTest, NedToEnu_ZAxis) {
    Eigen::Vector3f ned(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], 0.0f, 1e-5);
    EXPECT_NEAR(enu[1], 0.0f, 1e-5);
    EXPECT_NEAR(enu[2], -1.0f, 1e-5);
}

TEST(TransformUtilTest, NedToEnu_NegativeAxes) {
    Eigen::Vector3f ned(-1.0f, -1.0f, -1.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], -1.0f, 1e-5);
    EXPECT_NEAR(enu[1], -1.0f, 1e-5);
    EXPECT_NEAR(enu[2], 1.0f, 1e-5);
}

TEST(TransformUtilTest, NedToEnu_ArbitraryVector) {
    Eigen::Vector3f ned(5.5f, -3.2f, 7.8f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], -3.2f, 1e-5);
    EXPECT_NEAR(enu[1], 5.5f, 1e-5);
    EXPECT_NEAR(enu[2], -7.8f, 1e-5);
}

TEST(TransformUtilTest, NedToEnu_ZeroVector) {
    Eigen::Vector3f ned(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], 0.0f, 1e-5);
    EXPECT_NEAR(enu[1], 0.0f, 1e-5);
    EXPECT_NEAR(enu[2], 0.0f, 1e-5);
}

TEST(TransformUtilTest, NedToEnu_CustomVector) {
    Eigen::Vector3f ned(2.0f, -1.0f, 3.0f);
    Eigen::Vector3f enu = TransformUtil::nedToEnu(ned);
    EXPECT_NEAR(enu[0], -1.0f, 1e-5);  // y → x
    EXPECT_NEAR(enu[1], 2.0f, 1e-5);   // x → y
    EXPECT_NEAR(enu[2], -3.0f, 1e-5);  // -z → z
}


// 4. 2D Rotation by 90 degrees counter-clockwise
TEST(TransformUtilTest, Rotate2D_90Degrees) {
    Eigen::Vector2f point(1.0f, 0.0f);
    float angle_rad = M_PI / 2.0f; // 90 degrees
    Eigen::Vector2f rotated = TransformUtil::rotate2D(point, angle_rad);
    EXPECT_NEAR(rotated.x(), 0.0f, 1e-5);
    EXPECT_NEAR(rotated.y(), 1.0f, 1e-5);
}



// 5. Quaternion NED to ENU conversion
TEST(TransformUtilTest, NedToEnuQuaternion_FacingPosX) {
    // 0° yaw in NED = (cos(0), 0, 0, sin(0)) = (0, 0, 0, 1)
    Eigen::Vector4d quat_ned( 1.0, 0.0, 0.0, 0.0);

    Eigen::Vector4d quat_enu = TransformUtil::nedToEnuQuaternion(quat_ned);
    Eigen::Quaterniond q_enu(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]);

    // Expected: facing +Y in ENU
    Eigen::Matrix3d actual = q_enu.toRotationMatrix(); // * Eigen::Vector3d::UnitX();
    // Eigen::Vector3d expected(0, 1, 0);
    Eigen::Matrix3d expected;
    expected << 0.0f, -1.0f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f;

    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-5);
}

TEST(TransformUtilTest, NedToEnuQuaternion_FacingNegX) {
    // 180° yaw in NED = (cos(π/2), 0, 0, sin(π/2)) = (0, 0, 0, 1)
    Eigen::Vector4d quat_ned(0.0, 0.0, 0.0, 1.0);

    Eigen::Vector4d quat_enu = TransformUtil::nedToEnuQuaternion(quat_ned);
    Eigen::Quaterniond q_enu(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]);

    Eigen::Matrix3d actual = q_enu.toRotationMatrix();
    Eigen::Matrix3d expected;
    expected <<  0.0,  1.0, 0.0,
                -1.0,  0.0, 0.0,
                 0.0,  0.0, 1.0;

    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-5);
}


TEST(TransformUtilTest, NedToEnuQuaternion_FacingPosY) {
    // 90° yaw in NED = (cos(π/4), 0, 0, sin(π/4)) = (0.7071, 0, 0, 0.7071)
    Eigen::Vector4d quat_ned(std::cos(M_PI_4), 0.0, 0.0, std::sin(M_PI_4));

    Eigen::Vector4d quat_enu = TransformUtil::nedToEnuQuaternion(quat_ned);
    Eigen::Quaterniond q_enu(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]);

    Eigen::Matrix3d actual = q_enu.toRotationMatrix();
    Eigen::Matrix3d expected;
    expected << 1.0,  0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0,  0.0, 1.0;

    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-5);
}

TEST(TransformUtilTest, NedToEnuQuaternion_FacingNegY) {
    // -90° yaw in NED = (cos(-π/4), 0, 0, sin(-π/4)) = (0.7071, 0, 0, -0.7071)
    Eigen::Vector4d quat_ned(std::cos(M_PI_4), 0.0, 0.0, -std::sin(M_PI_4));

    Eigen::Vector4d quat_enu = TransformUtil::nedToEnuQuaternion(quat_ned);
    Eigen::Quaterniond q_enu(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]);

    Eigen::Matrix3d actual = q_enu.toRotationMatrix();
    Eigen::Matrix3d expected;
    expected <<  -1.0,  0.0, 0.0,
                 0.0,  -1.0, 0.0,
                 0.0,  0.0, 1.0;

    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-5);
}

TEST(TransformUtilTest, NedToEnuQuaternion_RandomYaw45) {
    double yaw_rad = M_PI / 4.0;
    Eigen::Vector4d quat_ned(std::cos(yaw_rad / 2.0), 0.0, 0.0, std::sin(yaw_rad / 2.0));  // 45° NED

    Eigen::Vector4d quat_enu = TransformUtil::nedToEnuQuaternion(quat_ned);
    Eigen::Quaterniond q_enu(quat_enu[0], quat_enu[1], quat_enu[2], quat_enu[3]);

    Eigen::Matrix3d actual = q_enu.toRotationMatrix();

    // ENU yaw = 45°
    double yaw_enu = yaw_rad;
    Eigen::Matrix3d expected;
    expected << std::cos(yaw_enu), -std::sin(yaw_enu), 0.0,
                std::sin(yaw_enu),  std::cos(yaw_enu), 0.0,
                0.0,                0.0,               1.0;

    EXPECT_NEAR((actual - expected).norm(), 0.0, 1e-5);
}



