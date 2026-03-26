#include <gtest/gtest.h>

#include "association/nearest_neighbor_association.hpp"
#include "association/ekf_nearest_neighbor_association.hpp"
#include "association/under_constrained_initialization_strategy.hpp"
#include "measurement/measurement_model.hpp"
#include "measurement/point3d_measurement_model.hpp"
#include "common/mock_slam_logger.hpp"

#include <condition_variable>
#include <future>
#include <chrono>
#include <cmath>

using namespace slam;

namespace {

// Named constants for EKF-specific threshold values used in the test adapter.
// Keep these aligned with test intent; these may intentionally differ from
// production values when a shorter confirmation horizon is desired in unit tests.
static constexpr double EKF_TEST_GATING_DISTANCE = 0.5;
static constexpr double EKF_TEST_BEARING_GATING = 0.22;
static constexpr double EKF_TEST_BEARING_FALLBACK = 0.40;
static constexpr int EKF_TEST_MIN_CONFIRM_OBS = 5;
static constexpr double EKF_TEST_MAX_COVARIANCE = 0.10;
static constexpr double EKF_TEST_UNDER_CONSTRAINED_COVARIANCE = 0.35;
static constexpr std::size_t EKF_TEST_MIN_TRIANGULATION_OBS = 4;
static constexpr double EKF_TEST_MIN_PARALLAX = 0.08;
static constexpr double EKF_TEST_MIN_BASELINE = 0.20;
static constexpr double EKF_TEST_MAX_RAY_RESIDUAL = 0.75;

class TestEkfNearestNeighborAssociation : public EkfNearestNeighborAssociation {
public:
    TestEkfNearestNeighborAssociation() = default;

    explicit TestEkfNearestNeighborAssociation(UnderConstrainedInitializationStrategyPtr strategy)
        : EkfNearestNeighborAssociation()
    {
        _underConstrainedInitializationStrategy = strategy;
    }

private:
    void processPointMeasurement(
        const Measurement& measurement,
        std::vector<int>& assignedFeature,
        AssignedMeasurements& assignedMeasurements) override
    {
        EkfNearestNeighborAssociation::processPointMeasurement(measurement, assignedFeature, assignedMeasurements);
    }

    int findNearestTentativeBearingCandidate(
        const Eigen::Vector3d& rayOriginWorld,
        const Eigen::Vector3d& rayDirectionWorld) const override
    {
        return EkfNearestNeighborAssociation::findNearestTentativeBearingCandidate(rayOriginWorld, rayDirectionWorld);
    }

    double getGatingDistance() const override { return EKF_TEST_GATING_DISTANCE; }
    double getBearingGatingDistance() const override { return EKF_TEST_BEARING_GATING; }
    double getBearingRelaxedFallbackDistance() const override { return EKF_TEST_BEARING_FALLBACK; }
    int getMinConfirmationObservations() const override { return EKF_TEST_MIN_CONFIRM_OBS; }
    double getMaxTentativeCovarianceTrace() const override { return EKF_TEST_MAX_COVARIANCE; }
    double getUnderConstrainedMaxCovarianceTrace() const override { return EKF_TEST_UNDER_CONSTRAINED_COVARIANCE; }
    std::size_t getMinTriangulationObservations() const override { return EKF_TEST_MIN_TRIANGULATION_OBS; }
    double getMinTriangulationParallaxRadians() const override { return EKF_TEST_MIN_PARALLAX; }
    double getMinTriangulationBaselineMeters() const override { return EKF_TEST_MIN_BASELINE; }
    double getMaxTriangulationMeanRayResidual() const override { return EKF_TEST_MAX_RAY_RESIDUAL; }
};

class UnderConstrainedTestMeasurementModel : public MeasurementModel {
public:
    int measurementDimension() const override
    {
        return 2;
    }

    Measurement predict(const Pose&, const Position&) override
    {
        Measurement z_hat;
        z_hat.payload = Eigen::VectorXd(2);
        z_hat.payload << 0.0, 0.0;
        z_hat.model = shared_from_this();
        return z_hat;
    }

    Eigen::MatrixXd jacobianWrtRobot(const Pose&, const Position&) const override
    {
        return Eigen::MatrixXd::Zero(2, 3);
    }

    Eigen::MatrixXd jacobianWrtLandmark(const Pose&, const Position&) const override
    {
        return Eigen::MatrixXd::Zero(2, 3);
    }

    Eigen::MatrixXd measurementNoise() const override
    {
        return Eigen::MatrixXd::Identity(2, 2);
    }

    std::optional<Position> inverse(const Pose&, const Measurement&) const override
    {
        return std::nullopt;
    }
};

class FixedPointUnderConstrainedStrategy : public UnderConstrainedInitializationStrategy {
public:
    std::optional<Position> initialize(const Measurement&, const Pose&) const override
    {
        return Position(5.0, -1.0, 2.0);
    }
};

}  // namespace

TEST(NearestNeighborAssociationTest, CreatesThenMatchesLandmark)
{
    TestEkfNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    auto sendAndGet = [&assoc](const Measurements& measurements) {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am)
                               {
            try {
                prom.set_value(am);
            } catch (...) {
                // ignore if promise already satisfied
            } });
        assoc.onReceiveMeasurement(measurements);
        auto status = fut.wait_for(std::chrono::seconds(5));
        EXPECT_EQ(status, std::future_status::ready);
        return fut.get();
    };

    // Build a single point measurement at (1,2,3)
    Measurement meas;
    meas.payload = Eigen::VectorXd(3);
    meas.payload << 1.0, 2.0, 3.0;
    meas.model = std::make_shared<Point3DMeasurementModel>();

    // Point3D measurements now follow tentative-first confirmation.
    for (int i = 0; i < EKF_TEST_MIN_CONFIRM_OBS - 1; i++)
    {
        AssignedMeasurements tentative = sendAndGet(Measurements{meas});
        EXPECT_TRUE(tentative.empty());
    }

    AssignedMeasurements firstResult = sendAndGet(Measurements{meas});
    ASSERT_EQ(firstResult.size(), 1u);
    EXPECT_TRUE(firstResult[0].isNew);
    int firstId = firstResult[0].id;

    // Next call: should match existing confirmed landmark
    AssignedMeasurements result2 = sendAndGet(Measurements{meas});
    ASSERT_EQ(result2.size(), 1u);
    EXPECT_FALSE(result2[0].isNew);
    EXPECT_EQ(result2[0].id, firstId);
}

TEST(NearestNeighborAssociationTest, EuclideanDistanceAndHandleUpdate)
{
    // expose protected euclideanDistance via derived test class
    struct TestAssoc : public TestEkfNearestNeighborAssociation
    {
        using TestEkfNearestNeighborAssociation::euclideanDistance;
        using TestEkfNearestNeighborAssociation::handleUpdate;
    } assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    // simple euclidean distance check
    Landmark a;
    Landmark b;
    a.position = Position(0.0, 0.0, 0.0);
    b.position = Position(3.0, 4.0, 0.0);
    double d = assoc.euclideanDistance(a, b);
    EXPECT_NEAR(d, 5.0, 1e-9);

    auto sendAndGet = [&assoc](const Measurements& measurements) {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am)
                               { try { prom.set_value(am);} catch(...){} });
        assoc.onReceiveMeasurement(measurements);
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        return fut.get();
    };

    // Create initial landmark at (0,0,0) via tentative confirmation.
    Measurement meas;
    meas.payload = Eigen::VectorXd(3);
    meas.payload << 0.0, 0.0, 0.0;
    meas.model = std::make_shared<Point3DMeasurementModel>();
    for (int i = 0; i < EKF_TEST_MIN_CONFIRM_OBS - 1; i++)
    {
        AssignedMeasurements tentative = sendAndGet(Measurements{meas});
        EXPECT_TRUE(tentative.empty());
    }
    AssignedMeasurements res = sendAndGet(Measurements{meas});
    ASSERT_EQ(res.size(), 1u);
    int initialId = res[0].id;

    // Create a map where the landmark moved to (10,0,0)
    MapSummary map;
    map.robot = RobotState();
    Landmark lm;
    lm.id = initialId;
    lm.position = Position(10.0, 0.0, 0.0);
    map.landmarks.push_back(lm);

    // Call handleUpdate to apply the new landmark position
    assoc.handleUpdate(map);

    // Now send measurement near (10,0,0) and expect a match (not new)
    Measurement meas2;
    meas2.payload = Eigen::VectorXd(3);
    meas2.payload << 10.0, 0.0, 0.0;
    meas2.model = std::make_shared<Point3DMeasurementModel>();
    AssignedMeasurements res2 = sendAndGet(Measurements{meas2});
    ASSERT_EQ(res2.size(), 1u);
    EXPECT_FALSE(res2[0].isNew);
    EXPECT_EQ(res2[0].id, initialId);
}

TEST(NearestNeighborAssociationTest, Point3DMeasurementsRequireTentativeConfirmation)
{
    TestEkfNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    auto sendAndGet = [&assoc](const Measurements& measurements) {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am)
                               { try { prom.set_value(am);} catch(...){} });
        assoc.onReceiveMeasurement(measurements);
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        return fut.get();
    };

    Measurement pointA;
    pointA.payload = Eigen::VectorXd(3);
    pointA.payload << 1.0, 2.0, 3.0;
    pointA.model = std::make_shared<Point3DMeasurementModel>();

    for (int i = 0; i < EKF_TEST_MIN_CONFIRM_OBS - 1; i++)
    {
        AssignedMeasurements tentative = sendAndGet(Measurements{pointA});
        EXPECT_TRUE(tentative.empty());
    }

    AssignedMeasurements created = sendAndGet(Measurements{pointA});
    ASSERT_EQ(created.size(), 1u);
    EXPECT_TRUE(created[0].isNew);

    AssignedMeasurements matched = sendAndGet(Measurements{pointA});
    ASSERT_EQ(matched.size(), 1u);
    EXPECT_FALSE(matched[0].isNew);
    EXPECT_EQ(matched[0].id, created[0].id);
}

TEST(NearestNeighborAssociationTest, UsesUnderConstrainedInitializationStrategyWhenInverseFails)
{
    TestEkfNearestNeighborAssociation assoc(std::make_shared<FixedPointUnderConstrainedStrategy>());
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    auto sendAndGet = [&assoc](const Measurements& measurements) {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am)
                               { try { prom.set_value(am);} catch(...){} });
        assoc.onReceiveMeasurement(measurements);
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        return fut.get();
    };

    Measurement meas;
    meas.payload = Eigen::VectorXd(2);
    meas.payload << 0.1, -0.2;
    meas.model = std::make_shared<UnderConstrainedTestMeasurementModel>();

    for (int i = 0; i < EKF_TEST_MIN_CONFIRM_OBS - 1; i++)
    {
        AssignedMeasurements tentativeResult = sendAndGet(Measurements{meas});
        EXPECT_TRUE(tentativeResult.empty());
    }

    AssignedMeasurements confirmedResult = sendAndGet(Measurements{meas});
    ASSERT_EQ(confirmedResult.size(), 1u);
    EXPECT_TRUE(confirmedResult[0].isNew);

    AssignedMeasurements matchedResult = sendAndGet(Measurements{meas});
    ASSERT_EQ(matchedResult.size(), 1u);
    EXPECT_FALSE(matchedResult[0].isNew);
    EXPECT_EQ(matchedResult[0].id, confirmedResult[0].id);
}
