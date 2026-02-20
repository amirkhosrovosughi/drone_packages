#include <gtest/gtest.h>

#include "observation/observation_builder.hpp"
#include "common/def_slam.hpp"

#include <drone_msgs/msg/point_list.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <drone_msgs/msg/bounding_boxes.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

using namespace slam;

TEST(ObservationBuilderTest, FromPointListSimple)
{
    drone_msgs::msg::PointList msg;
    geometry_msgs::msg::Point pt;
    pt.x = 1.0; pt.y = 2.0; pt.z = 3.0;
    msg.points.push_back(pt);

    auto observations = ObservationBuilder::fromPointList(msg, 0.123);
    ASSERT_EQ(observations.size(), 1);
    EXPECT_TRUE(std::holds_alternative<slam::Point3D>(observations[0].payload));
    const slam::Point3D &p = std::get<slam::Point3D>(observations[0].payload);
    EXPECT_TRUE(p.position.isApprox(Eigen::Vector3d(1.0,2.0,3.0)));
}

TEST(ObservationBuilderTest, FromBboxArrayWithCameraInfo)
{
    // Set camera intrinsics
    CameraInfo cam;
    cam.intrinsic.width = 640;
    cam.intrinsic.height = 480;
    cam.intrinsic.fx = 320.0;
    cam.intrinsic.fy = 320.0;
    cam.intrinsic.cx = 320.0;
    cam.intrinsic.cy = 240.0;
    ObservationBuilder::setCameraInfo(cam);

    vision_msgs::msg::Detection3DArray arr;
    vision_msgs::msg::Detection3D det;
    // normalized center at (0.5, 0.5) -> pixel center
    det.bbox.center.position.x = 0.5;
    det.bbox.center.position.y = 0.5;
    arr.detections.push_back(det);

    auto observations = ObservationBuilder::fromBboxArray(arr, 0.5);
    ASSERT_EQ(observations.size(), 1);
    EXPECT_TRUE(std::holds_alternative<slam::Bearing>(observations[0].payload));
}
