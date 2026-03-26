#include <gtest/gtest.h>

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
