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

TEST(ExtendedKalmanFilterTest, ApplyAbsolutePositionCorrectionMovesRobotTowardMeasurementAndReducesVariance)
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

    AbsolutePositionConstraint gps;
    gps.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gps.sigmaXyM = kSigmaXyM;
    gps.sigmaZM = kSigmaZM;
    ekf.applyAbsolutePositionCorrection(gps);

    const MapSummary after = ekf.getMap();
    EXPECT_GT(after.robot.pose.position.x, before.robot.pose.position.x);
    EXPECT_LT(std::abs(kTargetX - after.robot.pose.position.x),
              std::abs(kTargetX - before.robot.pose.position.x));
    EXPECT_LT(after.robot.variance.xx, before.robot.variance.xx);
}

TEST(ExtendedKalmanFilterTest, ApplyAbsolutePositionCorrectionUsesSigmaAsExpected)
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

    AbsolutePositionConstraint gpsSmallSigma;
    gpsSmallSigma.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsSmallSigma.sigmaXyM = kSmallSigmaM;
    gpsSmallSigma.sigmaZM = kSigmaZM;
    ekfSmallSigma.applyAbsolutePositionCorrection(gpsSmallSigma);

    const MapSummary mapSmallSigma = ekfSmallSigma.getMap();

    auto motionB = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfLargeSigma(motionB);
    ekfLargeSigma.setLogger(std::make_shared<MockSlamLogger>());
    ekfLargeSigma.prediction(p);

    AbsolutePositionConstraint gpsLargeSigma;
    gpsLargeSigma.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsLargeSigma.sigmaXyM = kLargeSigmaM;
    gpsLargeSigma.sigmaZM = kSigmaZM;
    ekfLargeSigma.applyAbsolutePositionCorrection(gpsLargeSigma);

    const MapSummary mapLargeSigma = ekfLargeSigma.getMap();

    EXPECT_GT(mapSmallSigma.robot.pose.position.x, mapLargeSigma.robot.pose.position.x);
    EXPECT_LT(mapSmallSigma.robot.variance.xx, mapLargeSigma.robot.variance.xx);
}

TEST(ExtendedKalmanFilterTest, ApplyAbsolutePositionCorrectionClampsTooSmallSigmaToMinBound)
{
    constexpr double kTargetX = 2.0;
    constexpr double kSigmaXyMinM = 0.2;
    constexpr double kSigmaZMinM = 0.3;
    constexpr double kSigmaBelowMin = 0.01;

    PredictionInput p;
    p.deltaPosition = Eigen::Vector3d::Zero();
    p.orientation = Eigen::Quaterniond::Identity();

    auto motionA = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfBelowMin(motionA);
    ekfBelowMin.setLogger(std::make_shared<MockSlamLogger>());
    ekfBelowMin.prediction(p);
    AbsolutePositionConstraint gpsBelowMin;
    gpsBelowMin.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsBelowMin.sigmaXyM = kSigmaBelowMin;
    gpsBelowMin.sigmaZM = kSigmaBelowMin;
    ekfBelowMin.applyAbsolutePositionCorrection(gpsBelowMin);
    const MapSummary mapBelowMin = ekfBelowMin.getMap();

    auto motionB = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfAtMin(motionB);
    ekfAtMin.setLogger(std::make_shared<MockSlamLogger>());
    ekfAtMin.prediction(p);
    AbsolutePositionConstraint gpsAtMin;
    gpsAtMin.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsAtMin.sigmaXyM = kSigmaXyMinM;
    gpsAtMin.sigmaZM = kSigmaZMinM;
    ekfAtMin.applyAbsolutePositionCorrection(gpsAtMin);
    const MapSummary mapAtMin = ekfAtMin.getMap();

    EXPECT_NEAR(mapBelowMin.robot.pose.position.x, mapAtMin.robot.pose.position.x, 1e-9);
    EXPECT_NEAR(mapBelowMin.robot.variance.xx, mapAtMin.robot.variance.xx, 1e-9);
    EXPECT_GT(mapBelowMin.robot.pose.position.x, 0.0);
    EXPECT_LT(mapBelowMin.robot.pose.position.x, kTargetX);
}

TEST(ExtendedKalmanFilterTest, ApplyAbsolutePositionCorrectionClampsTooLargeSigmaToMaxBound)
{
    constexpr double kTargetX = 2.0;
    constexpr double kSigmaXyMaxM = 20.0;
    constexpr double kSigmaZMaxM = 30.0;
    constexpr double kSigmaAboveMax = 1000.0;

    PredictionInput p;
    p.deltaPosition = Eigen::Vector3d::Zero();
    p.orientation = Eigen::Quaterniond::Identity();

    auto motionA = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfAboveMax(motionA);
    ekfAboveMax.setLogger(std::make_shared<MockSlamLogger>());
    ekfAboveMax.prediction(p);
    AbsolutePositionConstraint gpsAboveMax;
    gpsAboveMax.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsAboveMax.sigmaXyM = kSigmaAboveMax;
    gpsAboveMax.sigmaZM = kSigmaAboveMax;
    ekfAboveMax.applyAbsolutePositionCorrection(gpsAboveMax);
    const MapSummary mapAboveMax = ekfAboveMax.getMap();

    auto motionB = std::make_shared<PositionOnlyMotionModel>();
    ExtendedKalmanFilter ekfAtMax(motionB);
    ekfAtMax.setLogger(std::make_shared<MockSlamLogger>());
    ekfAtMax.prediction(p);
    AbsolutePositionConstraint gpsAtMax;
    gpsAtMax.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
    gpsAtMax.sigmaXyM = kSigmaXyMaxM;
    gpsAtMax.sigmaZM = kSigmaZMaxM;
    ekfAtMax.applyAbsolutePositionCorrection(gpsAtMax);
    const MapSummary mapAtMax = ekfAtMax.getMap();

    EXPECT_NEAR(mapAboveMax.robot.pose.position.x, mapAtMax.robot.pose.position.x, 1e-9);
    EXPECT_NEAR(mapAboveMax.robot.variance.xx, mapAtMax.robot.variance.xx, 1e-9);
    EXPECT_GT(mapAboveMax.robot.pose.position.x, 0.0);
    EXPECT_LT(mapAboveMax.robot.pose.position.x, 0.01);
}

TEST(ExtendedKalmanFilterTest, ApplyAbsolutePositionCorrectionIsMonotonicAroundClampThresholds)
{
    constexpr double kTargetX = 2.0;
    constexpr double kSigmaXyBelowMin = 0.10;
    constexpr double kSigmaXyAtMin = 0.20;
    constexpr double kSigmaXyAboveMin = 0.30;
    constexpr double kSigmaXyBelowMax = 10.0;
    constexpr double kSigmaXyAtMax = 20.0;
    constexpr double kSigmaXyAboveMax = 30.0;
    constexpr double kSigmaZFixed = 1.0;

    auto runCase = [&](double sigmaXyM) {
        auto motion = std::make_shared<PositionOnlyMotionModel>();
        ExtendedKalmanFilter ekf(motion);
        ekf.setLogger(std::make_shared<MockSlamLogger>());

        PredictionInput p;
        p.deltaPosition = Eigen::Vector3d::Zero();
        p.orientation = Eigen::Quaterniond::Identity();
        ekf.prediction(p);

        AbsolutePositionConstraint gps;
        gps.enuPosition = Eigen::Vector3d(kTargetX, 0.0, 0.0);
        gps.sigmaXyM = sigmaXyM;
        gps.sigmaZM = kSigmaZFixed;
        ekf.applyAbsolutePositionCorrection(gps);
        return ekf.getMap();
    };

    const MapSummary mapBelowMin = runCase(kSigmaXyBelowMin);
    const MapSummary mapAtMin = runCase(kSigmaXyAtMin);
    const MapSummary mapAboveMin = runCase(kSigmaXyAboveMin);
    const MapSummary mapBelowMax = runCase(kSigmaXyBelowMax);
    const MapSummary mapAtMax = runCase(kSigmaXyAtMax);
    const MapSummary mapAboveMax = runCase(kSigmaXyAboveMax);

    // At the lower threshold: below-min should match min-clamped behavior.
    EXPECT_NEAR(mapBelowMin.robot.pose.position.x, mapAtMin.robot.pose.position.x, 1e-9);
    EXPECT_NEAR(mapBelowMin.robot.variance.xx, mapAtMin.robot.variance.xx, 1e-9);
    EXPECT_GT(mapAtMin.robot.pose.position.x, mapAboveMin.robot.pose.position.x);
    EXPECT_LT(mapAtMin.robot.variance.xx, mapAboveMin.robot.variance.xx);

    // At the upper threshold: above-max should match max-clamped behavior.
    EXPECT_GT(mapBelowMax.robot.pose.position.x, mapAtMax.robot.pose.position.x);
    EXPECT_LT(mapBelowMax.robot.variance.xx, mapAtMax.robot.variance.xx);
    EXPECT_NEAR(mapAtMax.robot.pose.position.x, mapAboveMax.robot.pose.position.x, 1e-9);
    EXPECT_NEAR(mapAtMax.robot.variance.xx, mapAboveMax.robot.variance.xx, 1e-9);
}
