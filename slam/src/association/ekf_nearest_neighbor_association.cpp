#include "association/ekf_nearest_neighbor_association.hpp"
#include <algorithm>
#include <limits>

// ============================================================================
// EKF-Specific Pipeline Constants
// ============================================================================
// These constants are tuned for the EKF SLAM pipeline and represent the
// confirmation thresholds, gating distances, and triangulation quality gates.

// Point measurement gating: EKF uses consistent Euclidean distance threshold
static const double EKF_GATING_DISTANCE = 0.5;

// Bearing measurement gating: strict bearing residual threshold for EKF
static const double EKF_BEARING_GATING_DISTANCE = 0.22;  // ~12.6 deg in yaw/pitch

// Bearing relaxed fallback: larger gate to avoid duplicate creation
static const double EKF_BEARING_RELAXED_FALLBACK_DISTANCE = 0.40;  // ~22.9 deg

// Confirmation thresholds tuned for EKF covariance stability
static const int EKF_MIN_CONFIRMATION_OBSERVATIONS = 8;
static const double EKF_MAX_TENTATIVE_COVARIANCE_TRACE = 0.10;
static const double EKF_UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE = 0.35;

// EKF bearing triangulation quality gates
static const std::size_t EKF_MIN_TRIANGULATION_OBSERVATIONS = 4;
static const double EKF_MIN_TRIANGULATION_PARALLAX_RADIANS = 0.08;
static const double EKF_MIN_TRIANGULATION_BASELINE_METERS = 0.20;
static const double EKF_MAX_TRIANGULATION_MEAN_RAY_RESIDUAL = 0.75;
static const double EKF_POINT_DUPLICATE_SUPPRESS_GATE = 0.08;
static const double EKF_BEARING_TENTATIVE_DIRECTION_GATING_RADIANS = 0.12;
static const double EKF_BEARING_TRIANGULATED_RAY_DISTANCE_GATING_METERS = 0.35;

static const LogLevel EKF_HIGH_LEVEL = LogLevel::INFO;
static const LogLevel EKF_LOW_LEVEL = LogLevel::DEBUG;
static const std::string EKF_LOG_SUBSECTION = "[association] - ";

namespace {
constexpr double kEkfUnderConstrainedGatingDistance = 2.0;
constexpr double kEkfVarianceEpsilon = 1e-6;
}  // namespace

void EkfNearestNeighborAssociation::processPointMeasurement(
    const Measurement& measurement,
    std::vector<int>& assignedFeature,
    AssignedMeasurements& assignedMeasurements)
{
    Landmark landmark;
    bool isUnderConstrainedInitialization = false;
    if (!tryInitializeLandmarkFromMeasurement(measurement, landmark, isUnderConstrainedInitialization))
    {
        _logger->log(EKF_HIGH_LEVEL, EKF_LOG_SUBSECTION, "Skip point measurement as landmark cannot be constructed from it.");
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
        if (foundComparableLandmark && shortestDistance < EKF_POINT_DUPLICATE_SUPPRESS_GATE)
        {
            _logger->log(
                EKF_LOW_LEVEL,
                EKF_LOG_SUBSECTION,
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

int EkfNearestNeighborAssociation::findNearestTentativeCandidate(
    const Landmark& measurementLandmark,
    bool isUnderConstrainedInitialization) const
{
    int nearestCandidateId = -1;
    double bestCost = std::numeric_limits<double>::infinity();
    const double gatingDistance =
        isUnderConstrainedInitialization ? kEkfUnderConstrainedGatingDistance : this->getGatingDistance();
    const Eigen::Vector3d measurementVector = measurementLandmark.position.getPositionVector();

    for (const auto& entry : _tentativeLandmarks)
    {
        const TentativeLandmark& candidate = entry.second;
        if (candidate.isUnderConstrained != isUnderConstrainedInitialization)
        {
            continue;
        }

        const Eigen::Vector3d delta = measurementVector - candidate.position.getPositionVector();
        const double euclidean = delta.norm();
        const double covarianceTrace = std::max(kEkfVarianceEpsilon, candidate.variance.xx + candidate.variance.yy);
        const double mahalanobisLikeCost = euclidean / std::sqrt(covarianceTrace);

        if (mahalanobisLikeCost < bestCost)
        {
            bestCost = mahalanobisLikeCost;
            nearestCandidateId = entry.first;
        }
    }

    if (nearestCandidateId < 0)
    {
        return -1;
    }

    const TentativeLandmark& best = _tentativeLandmarks.at(nearestCandidateId);
    const double confirmationDistance =
        (measurementVector - best.position.getPositionVector()).norm();
    return confirmationDistance < gatingDistance ? nearestCandidateId : -1;
}

int EkfNearestNeighborAssociation::findNearestTentativeBearingCandidate(
    const Eigen::Vector3d& rayOriginWorld,
    const Eigen::Vector3d& rayDirectionWorld) const
{
    const Eigen::Vector3d normalizedDirection = rayDirectionWorld.normalized();
    int nearestCandidateId = -1;
    double bestScore = -std::numeric_limits<double>::infinity();

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

        if (angle > EKF_BEARING_TENTATIVE_DIRECTION_GATING_RADIANS)
        {
            continue;
        }

        double baselineScore = 0.0;
        if (candidate.bearingObservations.size() > 1)
        {
            double avgBaseline = 0.0;
            std::size_t pairCount = 0;
            for (std::size_t i = 0; i < candidate.bearingObservations.size(); ++i)
            {
                for (std::size_t j = i + 1; j < candidate.bearingObservations.size(); ++j)
                {
                    avgBaseline +=
                        (candidate.bearingObservations[i].originWorld - candidate.bearingObservations[j].originWorld)
                            .norm();
                    pairCount++;
                }
            }

            if (pairCount > 0)
            {
                avgBaseline /= static_cast<double>(pairCount);
                baselineScore = std::min(0.3, avgBaseline * 0.8);
            }
        }

        if (candidate.hasTriangulatedPosition)
        {
            const double rayDistance = pointToRayDistance(
                candidate.position.getPositionVector(),
                rayOriginWorld,
                normalizedDirection);
            if (rayDistance > EKF_BEARING_TRIANGULATED_RAY_DISTANCE_GATING_METERS)
            {
                continue;
            }
        }

        const double angleScore =
            (EKF_BEARING_TENTATIVE_DIRECTION_GATING_RADIANS - angle) / EKF_BEARING_TENTATIVE_DIRECTION_GATING_RADIANS;
        const double baselineWeight = candidate.bearingObservations.size() > 8 ? 0.4 : 0.2;
        const double angleWeight = 1.0 - baselineWeight;
        const double totalScore = angleScore * angleWeight + baselineScore * baselineWeight;

        if (totalScore > bestScore)
        {
            bestScore = totalScore;
            nearestCandidateId = entry.first;
        }
    }

    return nearestCandidateId;
}

// ============================================================================
// EKF Configuration Getter Implementations
// ============================================================================

double EkfNearestNeighborAssociation::getGatingDistance() const
{
    return EKF_GATING_DISTANCE;
}

double EkfNearestNeighborAssociation::getBearingGatingDistance() const
{
    return EKF_BEARING_GATING_DISTANCE;
}

double EkfNearestNeighborAssociation::getBearingRelaxedFallbackDistance() const
{
    return EKF_BEARING_RELAXED_FALLBACK_DISTANCE;
}

int EkfNearestNeighborAssociation::getMinConfirmationObservations() const
{
    return EKF_MIN_CONFIRMATION_OBSERVATIONS;
}

double EkfNearestNeighborAssociation::getMaxTentativeCovarianceTrace() const
{
    return EKF_MAX_TENTATIVE_COVARIANCE_TRACE;
}

double EkfNearestNeighborAssociation::getUnderConstrainedMaxCovarianceTrace() const
{
    return EKF_UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE;
}

std::size_t EkfNearestNeighborAssociation::getMinTriangulationObservations() const
{
    return EKF_MIN_TRIANGULATION_OBSERVATIONS;
}

double EkfNearestNeighborAssociation::getMinTriangulationParallaxRadians() const
{
    return EKF_MIN_TRIANGULATION_PARALLAX_RADIANS;
}

double EkfNearestNeighborAssociation::getMinTriangulationBaselineMeters() const
{
    return EKF_MIN_TRIANGULATION_BASELINE_METERS;
}

double EkfNearestNeighborAssociation::getMaxTriangulationMeanRayResidual() const
{
    return EKF_MAX_TRIANGULATION_MEAN_RAY_RESIDUAL;
}
