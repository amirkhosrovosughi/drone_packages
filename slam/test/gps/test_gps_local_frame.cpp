#include <gtest/gtest.h>

#include <stdexcept>

#include <Eigen/Dense>

#include <px4_msgs/msg/sensor_gps.hpp>

#include "gps/gps_local_frame.hpp"

namespace
{

constexpr double ANCHOR_LATITUDE_DEG = 48.1372;
constexpr double ANCHOR_LONGITUDE_DEG = 11.5755;
constexpr double ANCHOR_ALTITUDE_M = 520.0;
constexpr std::uint64_t ANCHOR_TIMESTAMP_US = 1234567;
constexpr double INITIAL_ENU_X_M = 1.0;
constexpr double INITIAL_ENU_Y_M = 2.0;
constexpr double INITIAL_ENU_Z_M = 3.0;

LocalFrameAnchor makeAnchor()
{
  LocalFrameAnchor anchor;
  anchor.anchorReference.latitudeDeg = ANCHOR_LATITUDE_DEG;
  anchor.anchorReference.longitudeDeg = ANCHOR_LONGITUDE_DEG;
  anchor.anchorReference.altitudeM = ANCHOR_ALTITUDE_M;
  anchor.anchorTimestampUs = ANCHOR_TIMESTAMP_US;
  anchor.initialEnuPosition = Eigen::Vector3d(
    INITIAL_ENU_X_M,
    INITIAL_ENU_Y_M,
    INITIAL_ENU_Z_M);
  return anchor;
}

px4_msgs::msg::SensorGps makeAnchorSample()
{
  px4_msgs::msg::SensorGps msg;
  msg.latitude_deg = ANCHOR_LATITUDE_DEG;
  msg.longitude_deg = ANCHOR_LONGITUDE_DEG;
  msg.altitude_msl_m = ANCHOR_ALTITUDE_M;
  return msg;
}

}  // namespace

TEST(GpsLocalFrameTest, AnchorThrowsBeforeInitialization)
{
  GpsLocalFrame localFrame;

  EXPECT_THROW(localFrame.anchor(), std::logic_error);
}

TEST(GpsLocalFrameTest, ToEnuThrowsBeforeInitialization)
{
  GpsLocalFrame localFrame;
  GpsReference reference;

  EXPECT_THROW(localFrame.toEnu(reference), std::logic_error);
}

TEST(GpsLocalFrameTest, SetAnchorMarksAnchorAvailable)
{
  GpsLocalFrame localFrame;
  localFrame.setAnchor(makeAnchor());

  EXPECT_TRUE(localFrame.hasAnchor());
  EXPECT_EQ(localFrame.anchor().anchorTimestampUs, ANCHOR_TIMESTAMP_US);
}

TEST(GpsLocalFrameTest, ToEnuWithAnchorReferenceReturnsInitialEnuPosition)
{
  GpsLocalFrame localFrame;
  localFrame.setAnchor(makeAnchor());

  const Eigen::Vector3d position = localFrame.toEnu(localFrame.anchor().anchorReference);

  EXPECT_NEAR(position.x(), INITIAL_ENU_X_M, 1e-6);
  EXPECT_NEAR(position.y(), INITIAL_ENU_Y_M, 1e-6);
  EXPECT_NEAR(position.z(), INITIAL_ENU_Z_M, 1e-6);
}

TEST(GpsLocalFrameTest, ToEnuProjectsSensorGpsSample)
{
  GpsLocalFrame localFrame;
  localFrame.setAnchor(makeAnchor());

  px4_msgs::msg::SensorGps msg = makeAnchorSample();
  msg.longitude_deg += 0.0001;

  const Eigen::Vector3d position = localFrame.toEnu(msg);

  EXPECT_GT(position.x(), INITIAL_ENU_X_M);
  EXPECT_NEAR(position.y(), INITIAL_ENU_Y_M, 0.2);
  EXPECT_NEAR(position.z(), INITIAL_ENU_Z_M, 0.2);
}