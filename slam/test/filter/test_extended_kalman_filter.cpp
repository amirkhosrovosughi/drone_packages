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
    p.deltaPosition = Eigen::Vector3d(1.0, 2.0, -0.5);
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

TEST(ExtendedKalmanFilterTest, ApplyGpsCorrectionMovesRobotTowardMeasurementAndReducesVariance)
{
    constexpr double kTargetX = 2.0;
    constexpr double kSigmaXyM = 0.5;
    constexpr double kSigmaZM = 1.0;

    auto motion = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekf(motion);
    ekf.setLogger(std::make_shared<MockSlamLogger>());

    PredictionInput p;
    p.deltaPosition = Eigen::Vector3d::Zero();
    p.orientation = Eigen::Quaterniond::Identity();
    ekf.prediction(p);

    const MapSummary before = ekf.getMap();

    GpsConstraint gps;
    gps.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gps.sigmaXyM = kSigmaXyM;
    gps.sigmaZM = kSigmaZM;
    ekf.applyGpsCorrection(gps);

    const MapSummary after = ekf.getMap();
    EXPECT_GT(after.robot.pose.position.x, before.robot.pose.position.x);
    EXPECT_LT(std::abs(kTargetX - after.robot.pose.position.x),
              std::abs(kTargetX - before.robot.pose.position.x));
    EXPECT_LT(after.robot.variance.xx, before.robot.variance.xx);
}

TEST(ExtendedKalmanFilterTest, ApplyGpsCorrectionUsesSigmaAsExpected)
{
    constexpr double kTargetX = 2.0;
    constexpr double kSmallSigmaM = 0.2;
    constexpr double kLargeSigmaM = 5.0;
    constexpr double kSigmaZM = 1.0;

    auto motionA = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfSmallSigma(motionA);
    ekfSmallSigma.setLogger(std::make_shared<MockSlamLogger>());

    PredictionInput p;
    p.deltaPosition = Eigen::Vector3d::Zero();
    p.orientation = Eigen::Quaterniond::Identity();
    ekfSmallSigma.prediction(p);

    GpsConstraint gpsSmallSigma;
    gpsSmallSigma.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsSmallSigma.sigmaXyM = kSmallSigmaM;
    gpsSmallSigma.sigmaZM = kSigmaZM;
    ekfSmallSigma.applyGpsCorrection(gpsSmallSigma);

    const MapSummary mapSmallSigma = ekfSmallSigma.getMap();

    auto motionB = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfLargeSigma(motionB);
    ekfLargeSigma.setLogger(std::make_shared<MockSlamLogger>());
    ekfLargeSigma.prediction(p);

    GpsConstraint gpsLargeSigma;
    gpsLargeSigma.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsLargeSigma.sigmaXyM = kLargeSigmaM;
    gpsLargeSigma.sigmaZM = kSigmaZM;
    ekfLargeSigma.applyGpsCorrection(gpsLargeSigma);

    const MapSummary mapLargeSigma = ekfLargeSigma.getMap();

    EXPECT_GT(mapSmallSigma.robot.pose.position.x, mapLargeSigma.robot.pose.position.x);
    EXPECT_LT(mapSmallSigma.robot.variance.xx, mapLargeSigma.robot.variance.xx);
}
