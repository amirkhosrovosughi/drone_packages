#include <gtest/gtest.h>

#include "filter/extended_kalman_filter.hpp"
#include "motion/position_only_motion_model.hpp"
#include "measurement/point3d_measurement_model.hpp"
#include "common/mock_slam_logger.hpp"

#include <future>
#include <chrono>

TEST(ExtendedKalmanFilterTest, PredictionUpdatesRobotPose)
{
    auto motion = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekf(motion);
    ekf.setLogger(std::make_shared<MockSlamLogger>());

    std::promise<MapSummary> prom;
    auto fut = prom.get_future();
    ekf.registerCallback([&prom](const MapSummary &map)
            {
                try { prom.set_value(map); } catch (...) {}
            });

    PredictionInput p;
    p.delta_position = Eigen::Vector3d(1.0, 2.0, -0.5);
    p.orientation = Eigen::Quaterniond::Identity();

    ekf.prediction(p);

    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    MapSummary map = fut.get();

    EXPECT_NEAR(map.robot.pose.position.x, 1.0, 1e-9);
    EXPECT_NEAR(map.robot.pose.position.y, 2.0, 1e-9);
    EXPECT_NEAR(map.robot.pose.position.z, -0.5, 1e-9);
}

TEST(ExtendedKalmanFilterTest, CorrectionAddsNewLandmark)
{
    auto motion = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekf(motion);
    ekf.setLogger(std::make_shared<MockSlamLogger>());

    std::promise<MapSummary> prom;
    auto fut = prom.get_future();
    ekf.registerCallback([&prom](const MapSummary &map)
                         {
        try { prom.set_value(map); } catch (...) {} });

    auto model = std::make_shared<Point3DMeasurementModel>();
    Measurement m;
    m.payload = Eigen::VectorXd(3);
    m.payload << 2.0, 0.0, 1.0;
    m.model = model;

    AssignedMeasurement am(m, 0);
    am.isNew = true;

    ekf.correction(AssignedMeasurements{am});

    ASSERT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    MapSummary map = fut.get();

    ASSERT_EQ(map.landmarks.size(), 1u);
    EXPECT_EQ(map.landmarks[0].id, 0);
    EXPECT_NEAR(map.landmarks[0].position.x, 2.0, 1e-9);
    EXPECT_NEAR(map.landmarks[0].position.y, 0.0, 1e-9);
    EXPECT_NEAR(map.landmarks[0].position.z, 1.0, 1e-9);
}
