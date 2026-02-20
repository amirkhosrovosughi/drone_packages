#include <gtest/gtest.h>

#include "measurement/bearing_measurement_model.hpp"
#include "common/def_slam.hpp"

TEST(BearingMeasurementModelTest, PredictFrontLandmark)
{
    auto model = std::make_shared<BearingMeasurementModel>();
    CameraInfo cam;
    cam.extrinsics = Eigen::Matrix4d::Identity();
    model->setCameraInfo(cam);

    Pose robot; // identity pose
    Position landmark(1.0, 0.0, 1.0); // in front-right and at same depth

    Measurement zhat = model->predict(robot, landmark);
    ASSERT_EQ(zhat.payload.size(), 2);
    double yaw = zhat.payload(0);
    double pitch = zhat.payload(1);
    EXPECT_NEAR(yaw, std::atan(1.0), 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
}
