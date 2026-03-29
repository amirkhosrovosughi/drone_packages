#include "association/graph_nearest_neighbor_association.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

// ============================================================================
// Graph-Specific Pipeline Constants
// ============================================================================
// These constants are tuned for the Graph SLAM backend.

// Point measurement gating for graph-based optimization
static const double GRAPH_GATING_DISTANCE = 0.6;  // Slightly relaxed for graph bundle adjustment

// Bearing measurement gating: optimized for ray-space consistency in graph
static const double GRAPH_BEARING_GATING_DISTANCE = 0.25;  // ~14.3 deg in yaw/pitch

// Bearing relaxed fallback: larger gate to manage dynamic objects in graph
static const double GRAPH_BEARING_RELAXED_FALLBACK_DISTANCE = 0.45;  // ~25.8 deg

// Graph confirms candidates quickly and relies on backend optimization to refine them.
static const int GRAPH_MIN_CONFIRMATION_OBSERVATIONS = 2;  // Fewer required (graph more robust)
static const double GRAPH_MAX_TENTATIVE_COVARIANCE_TRACE = 0.30;
static const double GRAPH_UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE = 1.20;

// Triangulation quality gates retained for geometry estimation, but Graph can
// choose to skip requiring them at confirmation via the pipeline hook.
static const std::size_t GRAPH_MIN_TRIANGULATION_OBSERVATIONS = 4;
static const double GRAPH_MIN_TRIANGULATION_PARALLAX_RADIANS = 0.08;
static const double GRAPH_MIN_TRIANGULATION_BASELINE_METERS = 0.20;
static const double GRAPH_MAX_TRIANGULATION_MEAN_RAY_RESIDUAL = 0.8;  // Slightly relaxed for bundle adjustment
static const double GRAPH_POINT_DUPLICATE_SUPPRESS_GATE = 0.08;

static const LogLevel GRAPH_HIGH_LEVEL = LogLevel::INFO;
static const LogLevel GRAPH_LOW_LEVEL = LogLevel::DEBUG;
static const std::string GRAPH_LOG_SUBSECTION = "[association] - ";

namespace {
constexpr double kGraphUnderConstrainedGatingDistance = 2.0;
constexpr double kGraphBearingDirectionGateRadians = 0.16;
constexpr double kGraphTriangulatedRayDistanceGateMeters = 0.5;
}  // namespace

void GraphNearestNeighborAssociation::processPointMeasurement(
    const Measurement& measurement,
    std::vector<int>& assignedFeature,
    AssignedMeasurements& assignedMeasurements)
{
    Landmark landmark;
    bool isUnderConstrainedInitialization = false;
    if (!tryInitializeLandmarkFromMeasurement(measurement, landmark, isUnderConstrainedInitialization))
    {
        _logger->log(
            GRAPH_HIGH_LEVEL,
            GRAPH_LOG_SUBSECTION,
            "Skip point measurement as landmark cannot be constructed from it.");
        return;
    }

    std::size_t startingLandmarkIndex = 0;
    auto it = std::find_if(assignedFeature.begin(), assignedFeature.end(), [](int value) { return value == 0; });
    if (it != assignedFeature.end())
    {
        startingLandmarkIndex = static_cast<std::size_t>(std::distance(assignedFeature.begin(), it));
    }
    else
    {
        trackOrConfirmTentativeLandmark(
            measurement,
            landmark,
            isUnderConstrainedInitialization,
            assignedMeasurements,
            _robotPose);
        return;
    }

    double shortestDistance = std::numeric_limits<double>::infinity();
    bool foundComparableLandmark = false;
    std::size_t nearestIndex = startingLandmarkIndex;
    for (std::size_t i = startingLandmarkIndex; i < _landmarks.size(); ++i)
    {
        if (assignedFeature[i] != 0)
        {
            continue;
        }

        const double distance = euclideanDistance(landmark, _landmarks[i]);
        foundComparableLandmark = true;
        if (distance < shortestDistance)
        {
            shortestDistance = distance;
            nearestIndex = i;
        }
    }

    int landmarkId = 0;
    if (foundComparableLandmark && shortestDistance < this->getGatingDistance())
    {
        _landmarks[nearestIndex].observeRepeat++;
        landmarkId = static_cast<int>(nearestIndex);

        AssignedMeasurement meas(measurement, _landmarks[nearestIndex].id);
        meas.isNew = false;
        assignedMeasurements.push_back(meas);
        assignedFeature[nearestIndex] = 1;
    }
    else
    {
        if (foundComparableLandmark && shortestDistance < GRAPH_POINT_DUPLICATE_SUPPRESS_GATE)
        {
            _logger->log(
                GRAPH_LOW_LEVEL,
                GRAPH_LOG_SUBSECTION,
                "Point measurement suppressed (",
                shortestDistance,
                "m from confirmed landmark, within duplicate gate).");
            return;
        }

        trackOrConfirmTentativeLandmark(
            measurement,
            landmark,
            isUnderConstrainedInitialization,
            assignedMeasurements,
            _robotPose,
            &landmarkId);
    }

#ifdef STORE_DEBUG_DATA
    data_logging_utils::DataLogger::log("landmarkId", landmarkId);
#endif
}

int GraphNearestNeighborAssociation::findNearestTentativeCandidate(
    const Landmark& measurementLandmark,
    bool isUnderConstrainedInitialization) const
{
    int nearestCandidateId = -1;
    double shortestDistance = std::numeric_limits<double>::infinity();
    const double gatingDistance = isUnderConstrainedInitialization
        ? kGraphUnderConstrainedGatingDistance
        : this->getGatingDistance();
    const Eigen::Vector3d measurementVector = measurementLandmark.position.getPositionVector();

    for (const auto& entry : _tentativeLandmarks)
    {
        const TentativeLandmark& candidate = entry.second;
        if (candidate.isUnderConstrained != isUnderConstrainedInitialization)
        {
            continue;
        }

        const double distance = (measurementVector - candidate.position.getPositionVector()).norm();
        if (distance < shortestDistance)
        {
            shortestDistance = distance;
            nearestCandidateId = entry.first;
        }
    }

    return shortestDistance < gatingDistance ? nearestCandidateId : -1;
}

int GraphNearestNeighborAssociation::findNearestTentativeBearingCandidate(
    const Eigen::Vector3d& rayOriginWorld,
    const Eigen::Vector3d& rayDirectionWorld) const
{
    const Eigen::Vector3d normalizedDirection = rayDirectionWorld.normalized();
    int nearestCandidateId = -1;
    double bestAngularResidual = std::numeric_limits<double>::infinity();

    for (const auto& entry : _tentativeLandmarks)
    {
        const TentativeLandmark& candidate = entry.second;
        if (!candidate.isUnderConstrained || candidate.bearingObservations.empty())
        {
            continue;
        }

        const Eigen::Vector3d referenceDirection =
            candidate.bearingObservations.back().directionWorld.normalized();
        const double dot = std::max(-1.0, std::min(1.0, referenceDirection.dot(normalizedDirection)));
        const double angle = std::acos(dot);
        if (angle > kGraphBearingDirectionGateRadians)
        {
            continue;
        }

        if (candidate.hasTriangulatedPosition)
        {
            const double rayDistance = pointToRayDistance(
                candidate.position.getPositionVector(),
                rayOriginWorld,
                normalizedDirection);
            if (rayDistance > kGraphTriangulatedRayDistanceGateMeters)
            {
                continue;
            }
        }

        if (angle < bestAngularResidual)
        {
            bestAngularResidual = angle;
            nearestCandidateId = entry.first;
        }
    }

    return nearestCandidateId;
}

// ============================================================================
// Graph Configuration Getter Implementations
// ============================================================================

double GraphNearestNeighborAssociation::getGatingDistance() const
{
    return GRAPH_GATING_DISTANCE;
}

double GraphNearestNeighborAssociation::getBearingGatingDistance() const
{
    return GRAPH_BEARING_GATING_DISTANCE;
}

double GraphNearestNeighborAssociation::getBearingRelaxedFallbackDistance() const
{
    return GRAPH_BEARING_RELAXED_FALLBACK_DISTANCE;
}

int GraphNearestNeighborAssociation::getMinConfirmationObservations() const
{
    return GRAPH_MIN_CONFIRMATION_OBSERVATIONS;
}

double GraphNearestNeighborAssociation::getMaxTentativeCovarianceTrace() const
{
    return GRAPH_MAX_TENTATIVE_COVARIANCE_TRACE;
}

double GraphNearestNeighborAssociation::getUnderConstrainedMaxCovarianceTrace() const
{
    return GRAPH_UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE;
}

std::size_t GraphNearestNeighborAssociation::getMinTriangulationObservations() const
{
    return GRAPH_MIN_TRIANGULATION_OBSERVATIONS;
}

double GraphNearestNeighborAssociation::getMinTriangulationParallaxRadians() const
{
    return GRAPH_MIN_TRIANGULATION_PARALLAX_RADIANS;
}

double GraphNearestNeighborAssociation::getMinTriangulationBaselineMeters() const
{
    return GRAPH_MIN_TRIANGULATION_BASELINE_METERS;
}

double GraphNearestNeighborAssociation::getMaxTriangulationMeanRayResidual() const
{
    return GRAPH_MAX_TRIANGULATION_MEAN_RAY_RESIDUAL;
}
