#include <gtest/gtest.h>

#include "backend/ekf_slam_backend.hpp"
#include "association/base_association.hpp"
#include "association/ekf_bearing_initialization_strategy.hpp"
#include "common/mock_slam_logger.hpp"
#include "motion/position_only_motion_model.hpp"
#include "association/nearest_neighbor_association.hpp"

#include <future>
#include <chrono>
#include <thread>
#include <Eigen/Geometry>
#include <cmath>
#include <tuple>

namespace slam
{

class SpyExtendedKalmanFilter : public ExtendedKalmanFilter
{
public:
  explicit SpyExtendedKalmanFilter(std::shared_ptr<MotionModel> motion)
  : ExtendedKalmanFilter(std::move(motion))
  {}

  void prediction(const PredictionInput& predictionInput) override
  {
    predictionCalled = true;
    lastPrediction = predictionInput;
  }

  void correction(const AssignedMeasurements& meas) override
  {
    correctionCalled = true;
    lastCorrection = meas;
  }

  void registerCallback(std::function<void(const MapSummary& map)> callback) override
  {
    storedCallback = std::move(callback);
  }

  void setSensorInfo(const Eigen::Matrix4d&) override {}

  void setLogger(LoggerPtr logger) override
  {
    loggerSet = logger;
  }

  void emitMapUpdate(const MapSummary& map)
  {
    if (storedCallback)
    {
      storedCallback(map);
    }
  }

  bool predictionCalled = false;
  bool correctionCalled = false;
  PredictionInput lastPrediction;
  AssignedMeasurements lastCorrection;
  LoggerPtr loggerSet;

private:
  std::function<void(const MapSummary& map)> storedCallback;
};

class FakeAssociation : public BaseAssociation
{
public:
  void onReceiveMeasurement(const Measurements& meas) override
  {
    onReceiveCalled = true;
    receivedMeasurements = meas;
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

  void emitAssigned(const AssignedMeasurements& assigned)
  {
    if (storedCallback)
    {
      storedCallback(assigned);
    }
  }

  bool onReceiveCalled = false;
  bool handleUpdateCalled = false;
  Measurements receivedMeasurements;
  MapSummary lastMap;
  LoggerPtr loggerSet;

private:
  void processMeasurement(const Measurements&) override {}
  std::function<void(AssignedMeasurements)> storedCallback;
};

TEST(EkfSlamBackendTest, InitializeThrowsWhenDependenciesMissing)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<SpyExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EXPECT_THROW(EkfSlamBackend(nullptr, assoc, factory).initialize(), std::runtime_error);
  EXPECT_THROW(EkfSlamBackend(ekf, nullptr, factory).initialize(), std::runtime_error);
  EXPECT_THROW(EkfSlamBackend(ekf, assoc, nullptr).initialize(), std::runtime_error);
}

TEST(EkfSlamBackendTest, ProcessMotionForwardsToFilterPrediction)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<SpyExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);

  MotionConstraint m;
  m.delta_position = Eigen::Vector3d(1.0, -2.0, 0.5);
  m.orientation = Eigen::Quaterniond::Identity();

  backend.processMotion(m);

  EXPECT_TRUE(ekf->predictionCalled);
  EXPECT_TRUE(ekf->lastPrediction.delta_position.isApprox(m.delta_position));
}

TEST(EkfSlamBackendTest, ProcessObservationBuildsAndForwardsMeasurements)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<SpyExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);

  Observation obs(0.0, Point3D{Eigen::Vector3d(2.0, 3.0, 4.0)});
  Observations observations{obs};

  backend.processObservation(observations);

  EXPECT_TRUE(assoc->onReceiveCalled);
  ASSERT_EQ(assoc->receivedMeasurements.size(), 1u);
  ASSERT_EQ(assoc->receivedMeasurements[0].payload.size(), 3);
  EXPECT_DOUBLE_EQ(assoc->receivedMeasurements[0].payload(0), 2.0);
  EXPECT_DOUBLE_EQ(assoc->receivedMeasurements[0].payload(1), 3.0);
  EXPECT_DOUBLE_EQ(assoc->receivedMeasurements[0].payload(2), 4.0);
}

TEST(EkfSlamBackendTest, InitializeWiresCallbacksAssociationToFilterAndFilterToAssociation)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<SpyExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);
  backend.initialize();

  AssignedMeasurement am;
  am.id = 0;
  am.isNew = true;
  assoc->emitAssigned(AssignedMeasurements{am});
  EXPECT_TRUE(ekf->correctionCalled);
  ASSERT_EQ(ekf->lastCorrection.size(), 1u);
  EXPECT_EQ(ekf->lastCorrection[0].id, 0);

  MapSummary map;
  ekf->emitMapUpdate(map);
  EXPECT_TRUE(assoc->handleUpdateCalled);
}

TEST(EkfSlamBackendTest, SetLoggerPropagatesToSubcomponents)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<SpyExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<FakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);

  auto logger = std::make_shared<MockSlamLogger>();
  backend.setLogger(logger);

  EXPECT_EQ(ekf->loggerSet, logger);
  EXPECT_EQ(assoc->loggerSet, logger);
}

TEST(EkfSlamBackendTest, EndToEndObservationCreatesLandmarkAndResetClearsIt)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<ExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<NearestNeighborAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);
  backend.setLogger(std::make_shared<MockSlamLogger>());
  backend.initialize();

  MotionConstraint motionConstraint;
  motionConstraint.delta_position = Eigen::Vector3d::Zero();
  motionConstraint.orientation = Eigen::Quaterniond::Identity();
  backend.processMotion(motionConstraint);

  Observation obs(0.0, Point3D{Eigen::Vector3d(1.0, 0.0, 1.0)});
  for (int i = 0; i < 5; ++i)
  {
    backend.processObservation(Observations{obs});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool gotLandmark = false;
  for (int i = 0; i < 20; ++i)
  {
    auto map = backend.getMap();
    if (!map.landmarks.empty())
    {
      gotLandmark = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_TRUE(gotLandmark);

  backend.reset();
  auto mapAfterReset = backend.getMap();
  EXPECT_TRUE(mapAfterReset.landmarks.empty());
}

TEST(EkfSlamBackendTest, EndToEndBearingObservationCreatesLandmarkWithStrategy)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<ExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<NearestNeighborAssociation>();
  assoc->setUnderConstrainedInitializationStrategy(std::make_shared<EkfBearingInitializationStrategy>());
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);
  backend.setLogger(std::make_shared<MockSlamLogger>());
  backend.initialize();

  MotionConstraint motionConstraint;
  motionConstraint.delta_position = Eigen::Vector3d::Zero();
  motionConstraint.orientation = Eigen::Quaterniond::Identity();
  backend.processMotion(motionConstraint);

  Bearing bearing;
  bearing.yaw = 0.1;
  bearing.pitch = -0.05;
  Observation obs(0.0, bearing);

  for (int i = 0; i < 5; ++i)
  {
    backend.processObservation(Observations{obs});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  bool gotLandmark = false;
  for (int i = 0; i < 30; ++i)
  {
    auto map = backend.getMap();
    if (!map.landmarks.empty())
    {
      gotLandmark = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(gotLandmark);
}

TEST(EkfSlamBackendTest, NoisyBearingObservationsStillConfirmTentativeLandmark)
{
  auto motion = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<ExtendedKalmanFilter>(motion);
  auto assoc = std::make_shared<NearestNeighborAssociation>();
  assoc->setUnderConstrainedInitializationStrategy(std::make_shared<EkfBearingInitializationStrategy>());
  auto factory = std::make_shared<MeasurementFactory>();

  EkfSlamBackend backend(ekf, assoc, factory);
  backend.setLogger(std::make_shared<MockSlamLogger>());
  backend.initialize();

  MotionConstraint motionConstraint;
  motionConstraint.delta_position = Eigen::Vector3d::Zero();
  motionConstraint.orientation = Eigen::Quaterniond::Identity();
  backend.processMotion(motionConstraint);

  const std::vector<std::pair<double, double>> jitteredBearings = {
    {0.10, -0.05},
    {0.12, -0.04},
    {0.09, -0.06},
    {0.11, -0.05},
    {0.13, -0.03},
    {0.10, -0.04},
    {0.08, -0.05},
    {0.12, -0.06}
  };

  for (const auto& sample : jitteredBearings)
  {
    Bearing bearing;
    bearing.yaw = sample.first;
    bearing.pitch = sample.second;
    Observation obs(0.0, bearing);
    backend.processObservation(Observations{obs});
    std::this_thread::sleep_for(std::chrono::milliseconds(12));
  }

  bool gotLandmark = false;
  for (int i = 0; i < 30; ++i)
  {
    auto map = backend.getMap();
    if (!map.landmarks.empty())
    {
      gotLandmark = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  EXPECT_TRUE(gotLandmark);
}

class EkfSlamBackendHeadingTest : public ::testing::TestWithParam<double>
{
protected:
  static void runRepeatedStaticObservationScenario(double yaw_rad)
  {
    auto motion = std::make_shared<PositionOnlyMotionModel>();
    auto ekf = std::make_shared<ExtendedKalmanFilter>(motion);
    auto assoc = std::make_shared<NearestNeighborAssociation>();
    auto factory = std::make_shared<MeasurementFactory>();

    EkfSlamBackend backend(ekf, assoc, factory);
    backend.setLogger(std::make_shared<MockSlamLogger>());
    backend.initialize();

    MotionConstraint motionConstraint;
    motionConstraint.delta_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond yaw(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    motionConstraint.orientation = yaw;

    Observation obs(0.0, Point3D{Eigen::Vector3d(1.0, 0.0, 1.0)});
    for (int i = 0; i < 12; ++i)
    {
      backend.processMotion(motionConstraint);
      backend.processObservation(Observations{obs});
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    bool gotAnyLandmark = false;
    MapSummary map;
    for (int i = 0; i < 30; ++i)
    {
      map = backend.getMap();
      if (!map.landmarks.empty())
      {
        gotAnyLandmark = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EXPECT_TRUE(gotAnyLandmark);
    EXPECT_LE(map.landmarks.size(), 1u);
  }
};

TEST_P(EkfSlamBackendHeadingTest, RepeatedStaticObservationAcrossHeadingsDoesNotCreateManyLandmarks)
{
  runRepeatedStaticObservationScenario(GetParam());
}

INSTANTIATE_TEST_SUITE_P(
    MultipleYawAngles,
    EkfSlamBackendHeadingTest,
    ::testing::Values(
        -M_PI,
        -M_PI_2,
        -M_PI_4,
        M_PI_4,
        M_PI_2,
        M_PI));

class EkfSlamBackendHeadingWithDriftTest
  : public ::testing::TestWithParam<std::tuple<double, double, double>>
{
protected:
  static void runRepeatedObservationWithDriftScenario(double yaw_rad, double drift_dx, double drift_dy)
  {
    auto motion = std::make_shared<PositionOnlyMotionModel>();
    auto ekf = std::make_shared<ExtendedKalmanFilter>(motion);
    auto assoc = std::make_shared<NearestNeighborAssociation>();
    auto factory = std::make_shared<MeasurementFactory>();

    EkfSlamBackend backend(ekf, assoc, factory);
    backend.setLogger(std::make_shared<MockSlamLogger>());
    backend.initialize();

    Eigen::Quaterniond yaw(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d robot_pos = Eigen::Vector3d::Zero();
    const Eigen::Vector3d landmark_world(3.0, 1.0, 1.0);
    const Eigen::Vector3d drift_step(drift_dx, drift_dy, 0.0);

    for (int i = 0; i < 12; ++i)
    {
      MotionConstraint motionConstraint;
      motionConstraint.delta_position = drift_step;
      motionConstraint.orientation = yaw;
      backend.processMotion(motionConstraint);

      robot_pos += drift_step;
      const Eigen::Vector3d measured_in_robot =
        yaw.toRotationMatrix().transpose() * (landmark_world - robot_pos);

      Observation obs(0.0, Point3D{measured_in_robot});
      backend.processObservation(Observations{obs});
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    bool gotAnyLandmark = false;
    MapSummary map;
    for (int i = 0; i < 30; ++i)
    {
      map = backend.getMap();
      if (!map.landmarks.empty())
      {
        gotAnyLandmark = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    EXPECT_TRUE(gotAnyLandmark);
    EXPECT_LE(map.landmarks.size(), 1u);
  }
};

TEST_P(EkfSlamBackendHeadingWithDriftTest, RepeatedObservationWithSmallDriftAndRotationDoesNotCreateManyLandmarks)
{
  const auto [yaw_rad, drift_dx, drift_dy] = GetParam();
  runRepeatedObservationWithDriftScenario(yaw_rad, drift_dx, drift_dy);
}

INSTANTIATE_TEST_SUITE_P(
    MultipleYawAndDriftCases,
    EkfSlamBackendHeadingWithDriftTest,
    ::testing::Values(
        std::make_tuple(-M_PI_2, 0.005, 0.0),
        std::make_tuple(-M_PI_2, 0.0, 0.005),
        std::make_tuple(M_PI_4, 0.004, -0.003),
        std::make_tuple(M_PI_2, 0.005, 0.0),
        std::make_tuple(M_PI_2, 0.0, -0.005),
        std::make_tuple(M_PI, 0.003, 0.003)));

}  // namespace slam
