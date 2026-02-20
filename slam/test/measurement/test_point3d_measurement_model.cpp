#include <gtest/gtest.h>

#include "measurement/point3d_measurement_model.hpp"
#include "common/def_slam.hpp"

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
