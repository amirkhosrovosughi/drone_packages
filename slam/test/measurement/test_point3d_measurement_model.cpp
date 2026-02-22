#include <gtest/gtest.h>

#include "measurement/point3d_measurement_model.hpp"
#include "common/def_slam.hpp"
#include <Eigen/Geometry>
#include <cmath>

TEST(Point3DMeasurementModelTest, PredictProducesRelativePosition)
{
    auto model = std::make_shared<Point3DMeasurementModel>();
    Pose robot;
    robot.position = Position(1.0, 2.0, 3.0);

    Position landmark(2.0, 4.0, 5.5);

    Measurement zhat = model->predict(robot, landmark);
    ASSERT_EQ(zhat.payload.size(), 3);
    EXPECT_DOUBLE_EQ(zhat.payload(0), 1.0);
    EXPECT_DOUBLE_EQ(zhat.payload(1), 2.0);
    EXPECT_DOUBLE_EQ(zhat.payload(2), 2.5);
}

TEST(Point3DMeasurementModelTest, JacobiansAreIdentityAndMinusIdentity)
{
    Point3DMeasurementModel model;
    Pose robot;
    Position landmark;

    Eigen::MatrixXd Jr = model.jacobianWrtRobot(robot, landmark);
    Eigen::MatrixXd Jl = model.jacobianWrtLandmark(robot, landmark);

    EXPECT_TRUE(Jr.isApprox(-Eigen::MatrixXd::Identity(3,3)));
    EXPECT_TRUE(Jl.isApprox(Eigen::MatrixXd::Identity(3,3)));
}

class Point3DMeasurementModelYawTest : public ::testing::TestWithParam<double>
{
protected:
    static Pose makeRobotPose(double yaw_rad)
    {
        Pose robot;
        robot.position = Position(2.0, -1.0, 0.5);
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
        robot.quaternion = Quaternion(q.w(), q.x(), q.y(), q.z());
        return robot;
    }
};

TEST_P(Point3DMeasurementModelYawTest, PredictUsesRobotOrientationForRelativeMeasurement)
{
    auto model = std::make_shared<Point3DMeasurementModel>();
    const double yaw_rad = GetParam();

    Pose robot = makeRobotPose(yaw_rad);
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));

    Position landmark_world(3.0, 2.0, 1.2);

    Measurement zhat = model->predict(robot, landmark_world);
    ASSERT_EQ(zhat.payload.size(), 3);

    Eigen::Vector3d p_r(robot.position.x, robot.position.y, robot.position.z);
    Eigen::Vector3d p_l(landmark_world.x, landmark_world.y, landmark_world.z);
    Eigen::Vector3d expected = q.toRotationMatrix().transpose() * (p_l - p_r);

    EXPECT_NEAR(zhat.payload(0), expected(0), 1e-9);
    EXPECT_NEAR(zhat.payload(1), expected(1), 1e-9);
    EXPECT_NEAR(zhat.payload(2), expected(2), 1e-9);
}

TEST_P(Point3DMeasurementModelYawTest, PredictInverseRoundTripAcrossHeadings)
{
    auto model = std::make_shared<Point3DMeasurementModel>();
    const double yaw_rad = GetParam();

    Pose robot = makeRobotPose(yaw_rad);

    Position landmark_world(4.0, 2.0, 1.2);

    Measurement zhat = model->predict(robot, landmark_world);
    auto recovered = model->inverse(robot, zhat);

    ASSERT_TRUE(recovered.has_value());
    EXPECT_NEAR(recovered->x, landmark_world.x, 1e-9);
    EXPECT_NEAR(recovered->y, landmark_world.y, 1e-9);
    EXPECT_NEAR(recovered->z, landmark_world.z, 1e-9);
}

TEST_P(Point3DMeasurementModelYawTest, JacobiansMatchRotationAcrossHeadings)
{
    Point3DMeasurementModel model;
    const double yaw_rad = GetParam();

    Pose robot = makeRobotPose(yaw_rad);
    Position landmark(4.0, 2.0, 1.2);

    Eigen::MatrixXd Jr = model.jacobianWrtRobot(robot, landmark);
    Eigen::MatrixXd Jl = model.jacobianWrtLandmark(robot, landmark);

    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d Rt = q.toRotationMatrix().transpose();

    EXPECT_TRUE(Jr.isApprox(-Rt, 1e-12));
    EXPECT_TRUE(Jl.isApprox(Rt, 1e-12));
}

INSTANTIATE_TEST_SUITE_P(
    MultipleYawAngles,
    Point3DMeasurementModelYawTest,
    ::testing::Values(
        -M_PI,
        -M_PI_2,
        -M_PI_4,
        0.0,
        M_PI_4,
        M_PI_2,
        M_PI));
