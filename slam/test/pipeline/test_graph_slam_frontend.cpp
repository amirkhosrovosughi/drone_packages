#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "association/base_association.hpp"
#include "pipeline/graph_slam_frontend.hpp"

namespace slam
{

namespace
{
constexpr double kTranslationThresholdM = 0.5;
constexpr double kRotationThresholdRad = 10.0 * M_PI / 180.0;
constexpr double kTranslationEpsilonM = 0.01;
constexpr double kRotationEpsilonRad = 1.0 * M_PI / 180.0;
constexpr double kGpsSigmaXy = 0.8;
constexpr double kGpsSigmaZ  = 1.5;
}

class FakeAssociation : public BaseAssociation
{
public:
  void onReceiveMeasurement(const Measurements& meas) override
  {
    receivedCount += 1;
    lastMeasurements = meas;
  }

  void handleUpdate(const MapSummary& map) override
  {
    handleUpdateCalled = true;
    lastMap = map;
  }

  void registerCallback(std::function<void(AssignedMeasurements)> callback) override
  {
    storedCallback = std::move(callback);
  }

  void setLogger(LoggerPtr logger) override
  {
    loggerSet = logger;
  }

  int receivedCount = 0;
  bool handleUpdateCalled = false;
  Measurements lastMeasurements;
  MapSummary lastMap;
  LoggerPtr loggerSet;

private:
  void processMeasurement(const Measurements&) override {}
  std::function<void(AssignedMeasurements)> storedCallback;
};

static MotionConstraint makeMotion(
  const Eigen::Vector3d& delta,
  const Eigen::Quaterniond& orientation)
{
  MotionConstraint m;
  m.deltaPosition = delta;
  m.orientation = orientation;
  return m;
}

static GpsConstraint makeGps(
  double x, double y, double z,
  double sigmaXy = kGpsSigmaXy, double sigmaZ = kGpsSigmaZ)
{
  GpsConstraint g;
  g.enuPosition = Eigen::Vector3d(x, y, z);
  g.sigmaXyM = sigmaXy;
  g.sigmaZM  = sigmaZ;
  return g;
}

TEST(GraphSlamFrontendTest, InitializeThrowsWhenDependenciesMissing)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EXPECT_THROW((GraphSlamFrontend(nullptr, factory)).initialize(), std::runtime_error);
  EXPECT_THROW((GraphSlamFrontend(association, nullptr)).initialize(), std::runtime_error);
}

TEST(GraphSlamFrontendTest, BelowThresholdDoesNotEmitKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int motionCallbackCount = 0;
  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint&)
    {
      motionCallbackCount += 1;
    });

  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM - kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  EXPECT_EQ(motionCallbackCount, 0);
}

TEST(GraphSlamFrontendTest, TranslationAtThresholdDoesNotEmitKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int motionCallbackCount = 0;
  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint&)
    {
      motionCallbackCount += 1;
    });

  frontend.onMotion(makeMotion(Eigen::Vector3d(kTranslationThresholdM, 0.0, 0.0), Eigen::Quaterniond::Identity()));

  EXPECT_EQ(motionCallbackCount, 0);
}

TEST(GraphSlamFrontendTest, TranslationOverThresholdEmitsKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int motionCallbackCount = 0;
  MotionConstraint emittedMotion;

  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint& m)
    {
      motionCallbackCount += 1;
      emittedMotion = m;
    });

  const double firstStepM = 0.30;
  const double secondStepM = (kTranslationThresholdM + kTranslationEpsilonM) - firstStepM;

  frontend.onMotion(makeMotion(Eigen::Vector3d(firstStepM, 0.0, 0.0), Eigen::Quaterniond::Identity()));
  frontend.onMotion(makeMotion(Eigen::Vector3d(secondStepM, 0.0, 0.0), Eigen::Quaterniond::Identity()));

  EXPECT_EQ(motionCallbackCount, 1);
  EXPECT_NEAR(emittedMotion.deltaPosition.norm(), kTranslationThresholdM + kTranslationEpsilonM, 1e-9);
}

TEST(GraphSlamFrontendTest, RotationAtThresholdDoesNotEmitKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int motionCallbackCount = 0;
  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint&)
    {
      motionCallbackCount += 1;
    });

  frontend.onMotion(makeMotion(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));

  Eigen::Quaterniond q(Eigen::AngleAxisd(kRotationThresholdRad, Eigen::Vector3d::UnitZ()));
  frontend.onMotion(makeMotion(Eigen::Vector3d::Zero(), q));

  EXPECT_EQ(motionCallbackCount, 0);
}

TEST(GraphSlamFrontendTest, RotationOverThresholdEmitsKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int motionCallbackCount = 0;
  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint&)
    {
      motionCallbackCount += 1;
    });

  frontend.onMotion(makeMotion(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));

  const double yawRad = kRotationThresholdRad + kRotationEpsilonRad;
  Eigen::Quaterniond q(Eigen::AngleAxisd(yawRad, Eigen::Vector3d::UnitZ()));
  frontend.onMotion(makeMotion(Eigen::Vector3d::Zero(), q));

  EXPECT_EQ(motionCallbackCount, 1);
}

TEST(GraphSlamFrontendTest, ObservationAfterTimeoutIsDropped)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  Observation obs(1.0, Point3D{Eigen::Vector3d(1.0, 2.0, 3.0)});
  frontend.onObservation(Observations{obs});

  EXPECT_EQ(association->receivedCount, 0);
}

TEST(GraphSlamFrontendTest, ObservationWithinWindowIsForwardedToAssociation)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  Observation obs(1.0, Point3D{Eigen::Vector3d(4.0, 5.0, 6.0)});
  frontend.onObservation(Observations{obs});

  ASSERT_EQ(association->receivedCount, 1);
  ASSERT_EQ(association->lastMeasurements.size(), 1u);
  EXPECT_DOUBLE_EQ(association->lastMeasurements[0].payload(0), 4.0);
  EXPECT_DOUBLE_EQ(association->lastMeasurements[0].payload(1), 5.0);
  EXPECT_DOUBLE_EQ(association->lastMeasurements[0].payload(2), 6.0);
}

// ---------------------------------------------------------------------------
// GPS prior buffer tests
// ---------------------------------------------------------------------------

TEST(GraphSlamFrontendTest, GpsPriorNotFiredWithoutKeyframeCommit)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int gpsPriorCount = 0;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint&) { gpsPriorCount += 1; });
  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  frontend.onGpsMeasurement(makeGps(10.0, 5.0, 2.0));

  // Motion below threshold — no keyframe commit.
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM - kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  EXPECT_EQ(gpsPriorCount, 0);
}

TEST(GraphSlamFrontendTest, GpsPriorFiredAtKeyframeCommitWithCorrectValues)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int gpsPriorCount = 0;
  AbsolutePositionConstraint received;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint& c)
    {
      gpsPriorCount += 1;
      received = c;
    });
  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  constexpr double kX = 12.5, kY = -3.7, kZ = 1.2;
  frontend.onGpsMeasurement(makeGps(kX, kY, kZ, kGpsSigmaXy, kGpsSigmaZ));
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  ASSERT_EQ(gpsPriorCount, 1);
  EXPECT_DOUBLE_EQ(received.enuPosition.x(), kX);
  EXPECT_DOUBLE_EQ(received.enuPosition.y(), kY);
  EXPECT_DOUBLE_EQ(received.enuPosition.z(), kZ);
  EXPECT_DOUBLE_EQ(received.sigmaXyM, kGpsSigmaXy);
  EXPECT_DOUBLE_EQ(received.sigmaZM, kGpsSigmaZ);
}

TEST(GraphSlamFrontendTest, GpsPriorFiredBeforeMotionCallback)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  std::vector<std::string> order;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint&) { order.push_back("gps"); });
  frontend.setMotionConstraintCallback(
    [&](const MotionConstraint&) { order.push_back("motion"); });

  frontend.onGpsMeasurement(makeGps(1.0, 2.0, 3.0));
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  ASSERT_EQ(order.size(), 2u);
  EXPECT_EQ(order[0], "gps");
  EXPECT_EQ(order[1], "motion");
}

TEST(GraphSlamFrontendTest, GpsPriorBuffersOnlyLatestFixPerKeyframe)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int gpsPriorCount = 0;
  AbsolutePositionConstraint received;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint& c)
    {
      gpsPriorCount += 1;
      received = c;
    });
  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  // Two GPS fixes before a single keyframe commit — only the last must survive.
  frontend.onGpsMeasurement(makeGps(1.0, 2.0, 3.0, 2.0, 4.0));
  frontend.onGpsMeasurement(makeGps(10.0, 20.0, 30.0, 0.3, 0.6));

  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  ASSERT_EQ(gpsPriorCount, 1);
  EXPECT_DOUBLE_EQ(received.enuPosition.x(), 10.0);
  EXPECT_DOUBLE_EQ(received.enuPosition.y(), 20.0);
  EXPECT_DOUBLE_EQ(received.enuPosition.z(), 30.0);
  EXPECT_DOUBLE_EQ(received.sigmaXyM, 0.3);
  EXPECT_DOUBLE_EQ(received.sigmaZM, 0.6);
}

TEST(GraphSlamFrontendTest, GpsPriorClearedAfterKeyframeCommit)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int gpsPriorCount = 0;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint&) { gpsPriorCount += 1; });
  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  // Keyframe 1: GPS buffered — GPS callback must fire.
  frontend.onGpsMeasurement(makeGps(5.0, 0.0, 0.0));
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));
  ASSERT_EQ(gpsPriorCount, 1);

  // Keyframe 2: no new GPS — GPS callback must NOT fire again.
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));
  EXPECT_EQ(gpsPriorCount, 1);
}

TEST(GraphSlamFrontendTest, GpsPriorNoCallbackRegisteredDoesNotCrash)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});
  // Deliberately omit setGpsPriorCallback.

  frontend.onGpsMeasurement(makeGps(3.0, 4.0, 5.0));

  EXPECT_NO_THROW(
    frontend.onMotion(makeMotion(
      Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
      Eigen::Quaterniond::Identity())));
}

TEST(GraphSlamFrontendTest, GpsPriorClearedOnReset)
{
  auto association = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  GraphSlamFrontend frontend(association, factory);

  int gpsPriorCount = 0;
  frontend.setGpsPriorCallback(
    [&](const AbsolutePositionConstraint&) { gpsPriorCount += 1; });
  frontend.setMotionConstraintCallback([](const MotionConstraint&) {});

  frontend.onGpsMeasurement(makeGps(7.0, 8.0, 9.0));
  frontend.reset();

  // After reset the buffered prior must be discarded — no GPS callback on next keyframe.
  frontend.onMotion(makeMotion(
    Eigen::Vector3d(kTranslationThresholdM + kTranslationEpsilonM, 0.0, 0.0),
    Eigen::Quaterniond::Identity()));

  EXPECT_EQ(gpsPriorCount, 0);
}

}  // namespace slam
