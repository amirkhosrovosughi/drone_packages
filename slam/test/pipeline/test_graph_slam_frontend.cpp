#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

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
  m.delta_position = delta;
  m.orientation = orientation;
  return m;
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
  EXPECT_NEAR(emittedMotion.delta_position.norm(), kTranslationThresholdM + kTranslationEpsilonM, 1e-9);
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

}  // namespace slam
