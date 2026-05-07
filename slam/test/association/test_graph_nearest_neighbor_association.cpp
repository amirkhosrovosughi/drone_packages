#include <gtest/gtest.h>

#include "association/association_profile.hpp"
#include "association/graph_nearest_neighbor_association.hpp"
#include "measurement/point3d_measurement_model.hpp"
#include "common/mock_slam_logger.hpp"

#include <future>
#include <chrono>
#include <cmath>

using namespace slam;

namespace {

// Named constants for Graph-specific threshold values used in the test adapter.
// Keep these aligned with Graph production constants to stay representative.
static constexpr double GRAPH_TEST_GATING_DISTANCE = 0.6;
static constexpr double GRAPH_TEST_BEARING_GATING = 0.25;
static constexpr double GRAPH_TEST_BEARING_FALLBACK = 0.45;
static constexpr int GRAPH_TEST_MIN_CONFIRM_OBS = 6;
static constexpr double GRAPH_TEST_MAX_COVARIANCE = 0.15;
static constexpr double GRAPH_TEST_UNDER_CONSTRAINED_COVARIANCE = 0.40;
static constexpr std::size_t GRAPH_TEST_MIN_TRIANGULATION_OBS = 4;
static constexpr double GRAPH_TEST_MIN_PARALLAX = 0.08;
static constexpr double GRAPH_TEST_MIN_BASELINE = 0.20;
static constexpr double GRAPH_TEST_MAX_RAY_RESIDUAL = 0.8;

// ─── Test adapter ────────────────────────────────────────────────────────────
// Exposes internals needed for white-box bearing candidate tests.
class TestGraphNearestNeighborAssociation : public GraphNearestNeighborAssociation {
public:
    TestGraphNearestNeighborAssociation() = default;

    // Expose protected bearing candidate lookup for direct unit testing.
    int exposeFindNearestTentativeBearingCandidate(
        const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const
    {
        return findNearestTentativeBearingCandidate(origin, direction);
    }

    // Direct injection of tentative entries so bearing-candidate tests don't
    // require wiring up a full bearing measurement pipeline.
    void injectTentativeCandidate(int id, const Eigen::Vector3d& directionWorld)
    {
        TentativeLandmark candidate;
        candidate.candidateId = id;
        candidate.isUnderConstrained = true;
        candidate.hasTriangulatedPosition = false;

        BearingRayObservation obs;
        obs.originWorld = Eigen::Vector3d::Zero();
        obs.directionWorld = directionWorld.normalized();
        candidate.bearingObservations.push_back(obs);

        _tentativeLandmarks[id] = candidate;
    }

private:
    double getGatingDistance() const override { return GRAPH_TEST_GATING_DISTANCE; }
    double getBearingGatingDistance() const override { return GRAPH_TEST_BEARING_GATING; }
    double getBearingRelaxedFallbackDistance() const override { return GRAPH_TEST_BEARING_FALLBACK; }
    int getMinConfirmationObservations() const override { return GRAPH_TEST_MIN_CONFIRM_OBS; }
    double getMaxTentativeCovarianceTrace() const override { return GRAPH_TEST_MAX_COVARIANCE; }
    double getUnderConstrainedMaxCovarianceTrace() const override { return GRAPH_TEST_UNDER_CONSTRAINED_COVARIANCE; }
    std::size_t getMinTriangulationObservations() const override { return GRAPH_TEST_MIN_TRIANGULATION_OBS; }
    double getMinTriangulationParallaxRadians() const override { return GRAPH_TEST_MIN_PARALLAX; }
    double getMinTriangulationBaselineMeters() const override { return GRAPH_TEST_MIN_BASELINE; }
    double getMaxTriangulationMeanRayResidual() const override { return GRAPH_TEST_MAX_RAY_RESIDUAL; }
};

}  // namespace

// ─── Tests ───────────────────────────────────────────────────────────────────

// Smoke test: fully-constrained Point3D measurements also go through
// tentative-first confirmation before becoming confirmed landmarks.
TEST(GraphNearestNeighborAssociationTest, Point3DRequiresTentativeConfirmation)
{
    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    auto sendAndGet = [&assoc](const Measurements& measurements) {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am) {
            try { prom.set_value(am); } catch (...) {}
        });
        assoc.onReceiveMeasurement(measurements);
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(5)), std::future_status::ready);
        return fut.get();
    };

    Measurement meas;
    meas.payload = Eigen::VectorXd(3);
    meas.payload << 2.0, 3.0, 4.0;
    meas.model = std::make_shared<Point3DMeasurementModel>();

    for (int i = 0; i < 5; i++)
    {
        AssignedMeasurements tentative = sendAndGet(Measurements{meas});
        EXPECT_TRUE(tentative.empty());
    }

    AssignedMeasurements created = sendAndGet(Measurements{meas});
    ASSERT_EQ(created.size(), 1u);
    EXPECT_TRUE(created[0].isNew);
    const int landmarkId = created[0].id;

    // Second call with same position: must match the confirmed landmark.
    AssignedMeasurements matched = sendAndGet(Measurements{meas});
    ASSERT_EQ(matched.size(), 1u);
    EXPECT_FALSE(matched[0].isNew);
    EXPECT_EQ(matched[0].id, landmarkId);
}

// White-box test: Graph's bearing candidate selection uses pure minimum angular
// residual. Given two tentative candidates at different directions, the one
// closer in angle to the query ray wins — regardless of baseline.
TEST(GraphNearestNeighborAssociationTest, BearingCandidateSelectsMinimumAngle)
{
    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());

    // Candidate A: pointing straight forward (+Z axis in world).
    const int candidateIdA = 0;
    const Eigen::Vector3d dirA(0.0, 0.0, 1.0);
    assoc.injectTentativeCandidate(candidateIdA, dirA);

    // Candidate B: slightly off-forward — close enough that a query midway
    // between them keeps both within the angular gate (~0.16 rad).
    // angle(A, B) ≈ atan(0.15) ≈ 0.149 rad  <  0.16 rad gate.
    const int candidateIdB = 1;
    const Eigen::Vector3d dirB(0.15, 0.0, 1.0);
    assoc.injectTentativeCandidate(candidateIdB, dirB);

    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

    // Query very close to A (small x offset) → must select A.
    {
        const Eigen::Vector3d queryNearA(0.02, 0.0, 1.0);
        const int result = assoc.exposeFindNearestTentativeBearingCandidate(origin, queryNearA);
        EXPECT_EQ(result, candidateIdA)
            << "Query near A should select candidate A (minimum angle wins)";
    }

    // Query close to B (x offset between A and B but nearer to B) → must select B.
    {
        const Eigen::Vector3d queryNearB(0.12, 0.0, 1.0);
        const int result = assoc.exposeFindNearestTentativeBearingCandidate(origin, queryNearB);
        EXPECT_EQ(result, candidateIdB)
            << "Query near B should select candidate B (minimum angle wins)";
    }

    // Query pointing backward — outside the gate for both candidates → no match.
    {
        const Eigen::Vector3d queryFar(0.0, 0.0, -1.0);
        const int result = assoc.exposeFindNearestTentativeBearingCandidate(origin, queryFar);
        EXPECT_EQ(result, -1)
            << "Query outside the angular gate should return -1 (no match)";
    }
}

// ─── Ambiguity gate tests ─────────────────────────────────────────────────────

// Helper: build a Point3D measurement at position (x, y, z).
static Measurement makePoint3DMeasurement(double x, double y, double z)
{
    Measurement m;
    m.payload = Eigen::VectorXd(3);
    m.payload << x, y, z;
    m.model = std::make_shared<Point3DMeasurementModel>();
    return m;
}

// Helper: send measurements repeatedly until the association confirms a landmark
// (returns non-empty with isNew=true) or we give up after maxRounds.
// Returns the confirmed assigned measurement, or empty if none arrived.
static AssignedMeasurements seedConfirmedLandmark(
    TestGraphNearestNeighborAssociation& assoc,
    const Measurements& meas,
    int maxRounds = 10)
{
    for (int i = 0; i < maxRounds; ++i)
    {
        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am) {
            try { prom.set_value(am); } catch (...) {}
        });
        assoc.onReceiveMeasurement(meas);
        if (fut.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto result = fut.get();
            if (!result.empty() && result[0].isNew)
            {
                return result;
            }
        }
    }
    return {};
}

// Gate disabled (default): even when two landmarks are at equal distances the
// best match is still accepted unconditionally.
TEST(GraphNearestNeighborAssociationTest, AmbiguityGateDisabledAlwaysAccepts)
{
    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());
    // Gate is disabled by default — do not call setAmbiguityGate.

    // Landmarks are 0.8 m apart (> gating distance 0.6) so each seed
    // measurement is outside the gate of the other confirmed landmark.
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement(-0.4, 0.0, 5.0)}).empty());
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement( 0.4, 0.0, 5.0)}).empty());

    // A query exactly at the midpoint is equidistant (0.4 m) from both landmarks,
    // each within the gate. Gate is off → should accept the nearest.
    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();
    assoc.registerCallback([&prom](AssignedMeasurements am) {
        try { prom.set_value(am); } catch (...) {}
    });
    assoc.onReceiveMeasurement({makePoint3DMeasurement(0.0, 0.0, 5.0)});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    const auto result = fut.get();
    EXPECT_FALSE(result.empty()) << "Gate disabled: ambiguous measurement should still be accepted";
}

// Gate enabled, single candidate: no top2 exists, so the match must be
// accepted unconditionally regardless of the configured margin.
TEST(GraphNearestNeighborAssociationTest, AmbiguityGateSingleCandidateAlwaysAccepts)
{
    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());
    assoc.setAmbiguityGate(true, 0.5);  // strict margin

    // Seed exactly one confirmed landmark.
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement(0.0, 0.0, 5.0)}).empty());

    // Query close to that single landmark — must match.
    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();
    assoc.registerCallback([&prom](AssignedMeasurements am) {
        try { prom.set_value(am); } catch (...) {}
    });
    assoc.onReceiveMeasurement({makePoint3DMeasurement(0.05, 0.0, 5.0)});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    const auto result = fut.get();
    EXPECT_FALSE(result.empty()) << "Single candidate should always be accepted";
}

// Gate enabled, margin too small (ambiguous): measurement must be deferred.
TEST(GraphNearestNeighborAssociationTest, AmbiguityGateDefearsAmbiguousMatch)
{
    // Margin large enough that the equal-distance case fails the gate.
    static constexpr double kMargin = 0.3;

    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());
    assoc.setAmbiguityGate(true, kMargin);

    // Landmarks are 0.8 m apart (> gating distance 0.6) so seeding succeeds
    // without the gate interfering (single candidate per seed round).
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement(-0.4, 0.0, 5.0)}).empty());
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement( 0.4, 0.0, 5.0)}).empty());

    // Query exactly at the midpoint → top1 = top2 = 0.4 m →
    // margin = 0 < kMargin (0.3) → should defer.
    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();
    assoc.registerCallback([&prom](AssignedMeasurements am) {
        try { prom.set_value(am); } catch (...) {}
    });
    assoc.onReceiveMeasurement({makePoint3DMeasurement(0.0, 0.0, 5.0)});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    const auto result = fut.get();
    EXPECT_TRUE(result.empty()) << "Ambiguous match (margin below threshold) should be deferred";
}

// Gate enabled, margin clearly sufficient: unambiguous best match is accepted.
TEST(GraphNearestNeighborAssociationTest, AmbiguityGateAcceptsClearMatch)
{
    static constexpr double kMargin = 0.05;  // small margin, easily satisfied

    TestGraphNearestNeighborAssociation assoc;
    assoc.setLogger(std::make_shared<MockSlamLogger>());
    assoc.setAmbiguityGate(true, kMargin);

    // Seed two landmarks: one very close to our future query, one far away.
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement(0.0, 0.0, 5.0)}).empty());
    ASSERT_FALSE(seedConfirmedLandmark(assoc, {makePoint3DMeasurement(3.0, 0.0, 5.0)}).empty());

    // Query close to landmark 1 → top1 much smaller than top2 → margin wide → accept.
    std::promise<AssignedMeasurements> prom;
    auto fut = prom.get_future();
    assoc.registerCallback([&prom](AssignedMeasurements am) {
        try { prom.set_value(am); } catch (...) {}
    });
    assoc.onReceiveMeasurement({makePoint3DMeasurement(0.05, 0.0, 5.0)});
    ASSERT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    const auto result = fut.get();
    EXPECT_FALSE(result.empty()) << "Clear unambiguous match should be accepted";
}

// Duplicate suppression gate must stay strict regardless of association profile.
// This verifies behavior parity between Baseline and GpsEnabled modes.
TEST(GraphNearestNeighborAssociationTest, DuplicateSuppressionUnchangedAcrossProfiles)
{
    auto runOnce = [](slam::AssociationProfileMode mode) {
        auto logger = std::make_shared<MockSlamLogger>();
        GraphNearestNeighborAssociation assoc(
            slam::makeGraphAssociationConfirmationConfig(mode));
        assoc.setLogger(logger);

        // Force ambiguous nearest-neighbor outcome so control reaches duplicate
        // suppression path in processPointMeasurement.
        assoc.setAmbiguityGate(true, 0.3);

        MapSummary map;
        map.robot = RobotState();
        Landmark l1;
        l1.id = 0;
        l1.position = Position(-0.05, 0.0, 5.0);
        Landmark l2;
        l2.id = 1;
        l2.position = Position(0.05, 0.0, 5.0);
        map.landmarks.push_back(l1);
        map.landmarks.push_back(l2);
        assoc.handleUpdate(map);

        std::promise<AssignedMeasurements> prom;
        auto fut = prom.get_future();
        assoc.registerCallback([&prom](AssignedMeasurements am) {
            try { prom.set_value(am); } catch (...) {}
        });

        // Midpoint: equal distance to both landmarks (0.05 m), which is
        // within strict duplicate suppression gate (0.08 m).
        assoc.onReceiveMeasurement({makePoint3DMeasurement(0.0, 0.0, 5.0)});
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
        return fut.get();
    };

    const AssignedMeasurements baselineResult = runOnce(slam::AssociationProfileMode::Baseline);
    const AssignedMeasurements gpsEnabledResult = runOnce(slam::AssociationProfileMode::GpsEnabled);

    EXPECT_TRUE(baselineResult.empty());
    EXPECT_TRUE(gpsEnabledResult.empty());
}
