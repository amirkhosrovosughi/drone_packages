#include <gtest/gtest.h>

#include "measurement/measurement_factory.hpp"
#include "measurement/bbox_measurement_model.hpp"
#include "measurement/bearing_measurement_model.hpp"

using namespace slam;

TEST(MeasurementFactoryTest, BuildRoutesObservationPayloads)
{
  MeasurementFactory factory;

  Observation pObs(0.0, Point3D{Eigen::Vector3d(1.0, 2.0, 3.0)});

  BoundingBox box;
  box.x = 10.0f;
  box.y = 20.0f;
  box.width = 30.0f;
  box.height = 40.0f;
  Observation bObs(0.0, box);

  Bearing bearing;
  bearing.yaw = 0.3;
  bearing.pitch = -0.1;
  Observation brObs(0.0, bearing);

  auto measurements = factory.build(Observations{pObs, bObs, brObs});
  ASSERT_EQ(measurements.size(), 3u);

  ASSERT_EQ(measurements[0].payload.size(), 3);
  EXPECT_DOUBLE_EQ(measurements[0].payload(0), 1.0);
  EXPECT_DOUBLE_EQ(measurements[0].payload(1), 2.0);
  EXPECT_DOUBLE_EQ(measurements[0].payload(2), 3.0);
  EXPECT_NE(measurements[0].model, nullptr);

  ASSERT_EQ(measurements[1].payload.size(), 4);
  EXPECT_FLOAT_EQ(measurements[1].payload(0), 10.0f);
  EXPECT_FLOAT_EQ(measurements[1].payload(1), 20.0f);
  EXPECT_FLOAT_EQ(measurements[1].payload(2), 30.0f);
  EXPECT_FLOAT_EQ(measurements[1].payload(3), 40.0f);
  EXPECT_NE(measurements[1].model, nullptr);

  ASSERT_EQ(measurements[2].payload.size(), 2);
  EXPECT_DOUBLE_EQ(measurements[2].payload(0), 0.3);
  EXPECT_DOUBLE_EQ(measurements[2].payload(1), -0.1);
  EXPECT_NE(measurements[2].model, nullptr);
}

TEST(MeasurementFactoryTest, SetCameraInfoConfiguresBBoxAndBearingModels)
{
  MeasurementFactory factory;

  Observation bboxObs(0.0, BoundingBox{});
  Observation bearingObs(0.0, Bearing{});

  auto before = factory.build(Observations{bboxObs, bearingObs});
  auto bboxBefore = std::dynamic_pointer_cast<BBoxMeasurementModel>(before[0].model);
  auto bearingBefore = std::dynamic_pointer_cast<BearingMeasurementModel>(before[1].model);
  ASSERT_NE(bboxBefore, nullptr);
  ASSERT_NE(bearingBefore, nullptr);

  Pose robot;
  Position landmark(1.0, 0.0, 1.0);
  EXPECT_THROW(bboxBefore->predict(robot, landmark), std::runtime_error);
  EXPECT_THROW(bearingBefore->predict(robot, landmark), std::runtime_error);

  CameraInfo cam;
  cam.extrinsics = Eigen::Matrix4d::Identity();
  cam.intrinsic = CameraIntrinsic(640, 480, 320.0, 320.0, 320.0, 240.0);
  factory.setCameraInfo(cam);

  auto after = factory.build(Observations{bboxObs, bearingObs});
  auto bboxAfter = std::dynamic_pointer_cast<BBoxMeasurementModel>(after[0].model);
  auto bearingAfter = std::dynamic_pointer_cast<BearingMeasurementModel>(after[1].model);
  ASSERT_NE(bboxAfter, nullptr);
  ASSERT_NE(bearingAfter, nullptr);

  EXPECT_NO_THROW({
    Measurement z1 = bboxAfter->predict(robot, landmark);
    EXPECT_EQ(z1.payload.size(), 2);
  });

  EXPECT_NO_THROW({
    Measurement z2 = bearingAfter->predict(robot, landmark);
    EXPECT_EQ(z2.payload.size(), 2);
  });
}
