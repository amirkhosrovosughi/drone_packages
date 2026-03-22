#include "association/nearest_neighbor_association.hpp"
#include "measurement/bearing_measurement_model.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <chrono>
#include <algorithm>
#include <limits>
#include <stdexcept>

#ifdef STORE_DEBUG_DATA
#include <atomic>
#include <map>
#endif


static const double GATING_DISTANCE = 0.5;
static const double BEARING_GATING_DISTANCE = 0.22; // ~12.6 deg in yaw/pitch residual norm
static const double BEARING_RELAXED_FALLBACK_DISTANCE = 0.40; // ~22.9 deg fallback to avoid duplicate landmark creation
static const double QUATERNION_RATE_LIMIT = 0.1;
static const int MIN_CONFIRMATION_OBSERVATIONS = 8;
static const int MAX_TENTATIVE_MISSED_FRAMES = 6;
static const double MAX_TENTATIVE_COVARIANCE_TRACE = 0.10;
static const double UNDER_CONSTRAINED_GATING_DISTANCE = 2.0;
static const double UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE = 0.35;
static const std::size_t MAX_BEARING_OBSERVATION_BUFFER = 20;
static const std::size_t MIN_TRIANGULATION_OBSERVATIONS = 4;
static const double MIN_TRIANGULATION_PARALLAX_RADIANS = 0.08;
static const double MIN_TRIANGULATION_BASELINE_METERS = 0.20;
static const double MIN_TRIANGULATION_FORWARD_DEPTH_METERS = 1.00;
static const double MAX_TRIANGULATION_MEAN_RAY_RESIDUAL = 0.75;
static const double MIN_NEW_LANDMARK_SEPARATION_METERS = 1.00;
static const double BEARING_TENTATIVE_DIRECTION_GATING_RADIANS = 0.12;
static const double BEARING_TRIANGULATED_RAY_DISTANCE_GATING_METERS = 0.35;
static const std::size_t EARLY_REJECTION_OBSERVATION_THRESHOLD = 15;
static const double EARLY_REJECTION_PARALLAX_THRESHOLD = 0.06;
static const double CLUTTER_MEASUREMENT_THRESHOLD = 2.0;
static const double BEARING_RELAXED_FALLBACK_DISTANCE_HIGH_CLUTTER = 0.30;

static const LogLevel HIGH_LEVEL = LogLevel::INFO;
static const LogLevel LOW_LEVEL = LogLevel::DEBUG;
static const std::string LOG_SUBSECTION = "[association] - ";

namespace {
constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

bool isBearingMeasurement(const Measurement& measurement)
{
    return std::dynamic_pointer_cast<BearingMeasurementModel>(measurement.model) != nullptr;
}
} // namespace

NearestNeighborAssociation::NearestNeighborAssociation()
    : NearestNeighborAssociation(std::make_shared<NoOpUnderConstrainedInitializationStrategy>())
{
}

NearestNeighborAssociation::NearestNeighborAssociation(
    UnderConstrainedInitializationStrategyPtr strategy)
{
    if (strategy)
    {
        _underConstrainedInitializationStrategy = std::move(strategy);
    }
    else
    {
        _underConstrainedInitializationStrategy = std::make_shared<NoOpUnderConstrainedInitializationStrategy>();
    }
}

void NearestNeighborAssociation::onReceiveMeasurement(const Measurements& meas) 
{
    #ifdef STORE_DEBUG_DATA
    static std::atomic<uint64_t> receive_seq{0};
    std::map<std::string, double> mapLog;
    mapLog["timeline.association.receive.seq"] = static_cast<double>(++receive_seq);
    mapLog["timeline.association.receive.wall_time"] =
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    mapLog["timeline.association.receive.measurement_count"] = static_cast<double>(meas.size());
    data_logging_utils::DataLogger::log(mapLog);
    #endif

    processMeasurement(meas);
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, LOG_SUBSECTION, "onReceiveMeasurement.");
}

void NearestNeighborAssociation::handleUpdate(const MapSummary& map)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _robotPose = map.robot.pose;
    static Quaternion preQuaternion;
        
    Eigen::Vector4d QuDef = _robotPose.quaternion.getVector() - preQuaternion.getVector();
    _quaternionRate = QuDef.norm();

    #ifdef STORE_DEBUG_DATA
    data_logging_utils::DataLogger::log("_quaternionRate", _quaternionRate);
    #endif

    preQuaternion = _robotPose.quaternion;

    // Copy updated landmarks from the provided map for matching
    Landmarks updatedLandmarks = map.landmarks;

    // Iterate by reference so we update the stored landmarks in-place
    // Instead of this we can do: _landmarks = map.landmarks
    // But keep it like that to catch potential id mismatch bugs
    for (Landmark& landmark : _landmarks)
    {
        bool foundMatchedLandmark = false;
        for (const Landmark& updatedLandmark : updatedLandmarks)
        {
            if (landmark.id == updatedLandmark.id)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!--! landmark:" , landmark.id, " has been updated from (",
                            landmark.position.x, ", ", landmark.position.y, ", ", landmark.position.z, " to ",
                            updatedLandmark.position.x, ", ", updatedLandmark.position.y, ", ",
                            updatedLandmark.position.z, ") \n");
                landmark.position = updatedLandmark.position;
                foundMatchedLandmark = true;
                break;
            }
        }

        if (!foundMatchedLandmark)
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "landmark id ", landmark.id,
                         " not found in map update yet, keep local estimate temporarily.");
        }
    }

    _numberLandmarks = static_cast<int>(updatedLandmarks.size());
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "handleUpdate.");
}

void NearestNeighborAssociation::processMeasurement(const Measurements& measurements)
{
    AssignedMeasurements  assignedMeasurements;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _frameCounter++;
        markTentativeCandidatesUnseenForCurrentFrame();

        
        // If angle moves too fast, skip detection update for this frame.
        if (_quaternionRate > QUATERNION_RATE_LIMIT) {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Drone angle moving so fast: ", _quaternionRate ,", skip the detection.\n");
            pruneTentativeLandmarks();
            return;
        }

        #ifdef STORE_DEBUG_DATA
        std::map<std::string, double> timelineLog;
        timelineLog["timeline.association.process.wall_time"] =
            std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        timelineLog["timeline.association.process.measurement_count"] = static_cast<double>(measurements.size());
        timelineLog["timeline.association.process.quaternion_rate"] = _quaternionRate;
        timelineLog["timeline.association.process.robot_pose.x"] = _robotPose.position.x;
        timelineLog["timeline.association.process.robot_pose.y"] = _robotPose.position.y;
        timelineLog["timeline.association.process.robot_pose.z"] = _robotPose.position.z;
        timelineLog["timeline.association.process.robot_q.w"] = _robotPose.quaternion.w;
        timelineLog["timeline.association.process.robot_q.x"] = _robotPose.quaternion.x;
        timelineLog["timeline.association.process.robot_q.y"] = _robotPose.quaternion.y;
        timelineLog["timeline.association.process.robot_q.z"] = _robotPose.quaternion.z;
        data_logging_utils::DataLogger::log(timelineLog);
        #endif

        std::vector<int> assignedFeature(_landmarks.size(), 0);

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "size measurements is:", measurements.size(), ", size Landmark is: (( ", _landmarks.size(), " ))");

        #ifdef STORE_DEBUG_DATA
        int measurementCounter = 0;
        #endif

        for (const Measurement& measurement : measurements)
        {
            #ifdef STORE_DEBUG_DATA
            std::string plotTagMeas =  "MeasurementPosition_" +  std::to_string(measurementCounter++) + "_";
            data_logging_utils::DataLogger::log(plotTagMeas + "dim", static_cast<double>(measurement.payload.size()));
            if (measurement.payload.size() > 0)
            {
                data_logging_utils::DataLogger::log(plotTagMeas + "x", measurement.payload[0]);
            }
            if (measurement.payload.size() > 1)
            {
                data_logging_utils::DataLogger::log(plotTagMeas + "y", measurement.payload[1]);
            }
            if (measurement.payload.size() > 2)
            {
                data_logging_utils::DataLogger::log(plotTagMeas + "z", measurement.payload[2]);
            }
            #endif

            if (isBearingMeasurement(measurement))
            {
                processBearingMeasurement(measurement, assignedFeature, assignedMeasurements);
            }
            else
            {
                processPointMeasurement(measurement, assignedFeature, assignedMeasurements);
            }
        }
        pruneTentativeLandmarks();
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Measurement are processed.\n");
    }
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Number of landmarks is ", _landmarks.size());

    if (_callback)
    {
        _callback(assignedMeasurements);
    }

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "callback measurement is called.");
}

void NearestNeighborAssociation::processPointMeasurement(
    const Measurement& measurement,
    std::vector<int>& assignedFeature,
    AssignedMeasurements& assignedMeasurements)
{
    Landmark landmark;
    bool isUnderConstrainedInitialization = false;
    if (!tryInitializeLandmarkFromMeasurement(measurement, landmark, isUnderConstrainedInitialization))
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Skip point measurement as landmark cannot be constructed from it.");
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
    if (foundComparableLandmark && shortestDistance < GATING_DISTANCE)
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

void NearestNeighborAssociation::processBearingMeasurement(
    const Measurement& measurement,
    std::vector<int>& assignedFeature,
    AssignedMeasurements& assignedMeasurements)
{
    auto bearingModel = std::dynamic_pointer_cast<BearingMeasurementModel>(measurement.model);
    if (!bearingModel)
    {
        return;
    }

    Eigen::Vector3d rayOriginWorld;
    Eigen::Vector3d rayDirectionWorld;
    if (!bearingModel->worldRayFromMeasurement(_robotPose, measurement, rayOriginWorld, rayDirectionWorld))
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Skip bearing measurement as ray cannot be constructed from it.");
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
        trackOrConfirmTentativeBearingLandmark(
            measurement,
            rayOriginWorld,
            rayDirectionWorld,
            assignedMeasurements,
            _robotPose);
        return;
    }

    double shortestDistance = std::numeric_limits<double>::infinity();
    bool foundComparableLandmark = false;
    std::size_t nearestIndex = startingLandmarkIndex;
    Measurement nearestPredictedMeasurement;
    bool hasNearestPredictedMeasurement = false;

    for (std::size_t i = startingLandmarkIndex; i < _landmarks.size(); ++i)
    {
        if (assignedFeature[i] != 0)
        {
            continue;
        }

        try
        {
            Measurement predicted = measurement.model->predict(_robotPose, _landmarks[i].position);
            if (measurement.payload.size() < 2 || predicted.payload.size() < 2)
            {
                continue;
            }

            const double dyaw = wrapAngle(measurement.payload(0) - predicted.payload(0));
            const double dpitch = wrapAngle(measurement.payload(1) - predicted.payload(1));
            const double distance = std::sqrt(dyaw * dyaw + dpitch * dpitch);

            foundComparableLandmark = true;
            if (distance < shortestDistance)
            {
                shortestDistance = distance;
                nearestIndex = i;
                nearestPredictedMeasurement = predicted;
                hasNearestPredictedMeasurement = true;
            }
        }
        catch (const std::runtime_error&)
        {
            continue;
        }
    }

    int landmarkId = 0;
    const bool strictAssociationMatch = foundComparableLandmark && shortestDistance < BEARING_GATING_DISTANCE;
    
    const double fallbackDistanceThreshold = BEARING_RELAXED_FALLBACK_DISTANCE;
    
    const bool relaxedBearingFallbackMatch =
        foundComparableLandmark &&
        shortestDistance >= BEARING_GATING_DISTANCE &&
        shortestDistance < fallbackDistanceThreshold;

    if (strictAssociationMatch || relaxedBearingFallbackMatch)
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
        trackOrConfirmTentativeBearingLandmark(
            measurement,
            rayOriginWorld,
            rayDirectionWorld,
            assignedMeasurements,
            _robotPose,
            &landmarkId);
    }

    #ifdef STORE_DEBUG_DATA
    data_logging_utils::DataLogger::log("landmarkId", landmarkId);
    #endif
}

bool NearestNeighborAssociation::tryInitializeLandmarkFromMeasurement(
    const Measurement& measurement,
    Landmark& landmark,
    bool& isUnderConstrainedInitialization) const
{
    isUnderConstrainedInitialization = false;

    auto capturedLandmarkPosition = measurement.model->inverse(_robotPose, measurement);
    if (!capturedLandmarkPosition)
    {
        if (_underConstrainedInitializationStrategy)
        {
            capturedLandmarkPosition = _underConstrainedInitializationStrategy->initialize(measurement, _robotPose);
        }

        if (!capturedLandmarkPosition)
        {
            return false;
        }

        isUnderConstrainedInitialization = true;
    }
    landmark.position = capturedLandmarkPosition.value();
    return true;
}

void NearestNeighborAssociation::appendNewLandmarkAssignment(
    const Measurement& measurement,
    const Landmark& landmark,
    AssignedMeasurements& assignedMeasurements) const
{
    AssignedMeasurement meas(measurement, landmark.id);
    meas.isNew = true;
    meas.position = landmark.position;
    meas.hasInitializedPosition = true;
    assignedMeasurements.push_back(meas);
}

void NearestNeighborAssociation::trackOrConfirmTentativeLandmark(
    const Measurement& measurement,
    const Landmark& landmark,
    bool isUnderConstrainedInitialization,
    AssignedMeasurements& assignedMeasurements,
    const Pose& robotPose,
    int* landmarkId)
{
    const int tentativeCandidateId =
        findNearestTentativeCandidate(landmark, isUnderConstrainedInitialization);
    if (tentativeCandidateId >= 0)
    {
        TentativeLandmark& candidate = _tentativeLandmarks[tentativeCandidateId];
        updateTentativeLandmark(candidate, landmark.position);
        updateBearingTriangulation(candidate, measurement, robotPose);
        candidate.seenInCurrentFrame = true;

        if (landmarkId)
        {
            *landmarkId = candidate.candidateId;
        }

        if (shouldConfirmTentativeLandmark(candidate))
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Confirming tentative landmark candidate ", candidate.candidateId,
                         " after ", candidate.consistentObservations, " observations.");

            int nearestExistingLandmarkIndex = -1;
            double nearestExistingDistance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < _landmarks.size(); ++i)
            {
                const double distance =
                    (candidate.position.getPositionVector() - _landmarks[i].position.getPositionVector()).norm();
                if (distance < nearestExistingDistance)
                {
                    nearestExistingDistance = distance;
                    nearestExistingLandmarkIndex = static_cast<int>(i);
                }
            }

            if (nearestExistingLandmarkIndex >= 0 &&
                nearestExistingDistance < MIN_NEW_LANDMARK_SEPARATION_METERS)
            {
                Landmark& mergedLandmark = _landmarks[static_cast<std::size_t>(nearestExistingLandmarkIndex)];
                mergedLandmark.observeRepeat++;

                AssignedMeasurement meas(measurement, mergedLandmark.id);
                meas.isNew = false;
                assignedMeasurements.push_back(meas);

                _logger->log(HIGH_LEVEL, LOG_SUBSECTION,

                             "Tentative candidate ", candidate.candidateId,
                             " merged with existing landmark id ", mergedLandmark.id,
                             " at distance ", nearestExistingDistance,
                             " (min separation gate=", MIN_NEW_LANDMARK_SEPARATION_METERS, ").");

                _tentativeLandmarks.erase(candidate.candidateId);
                return;
            }

            Landmark confirmedLandmark;
            confirmedLandmark.id = _numberLandmarks++;
            confirmedLandmark.position = candidate.position;
            confirmedLandmark.variance = candidate.variance;
            confirmedLandmark.observeRepeat = candidate.consistentObservations;

            _landmarks.push_back(confirmedLandmark);
            appendNewLandmarkAssignment(measurement, confirmedLandmark, assignedMeasurements);
            _tentativeLandmarks.erase(candidate.candidateId);
        }
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Creating tentative landmark candidate ...");
        TentativeLandmark candidate;
        candidate.candidateId = _nextTentativeId++;
        candidate.isUnderConstrained = isUnderConstrainedInitialization;
        updateTentativeLandmark(candidate, landmark.position);
        updateBearingTriangulation(candidate, measurement, robotPose);
        candidate.seenInCurrentFrame = true;

        if (landmarkId)
        {
            *landmarkId = candidate.candidateId;
        }

        _tentativeLandmarks[candidate.candidateId] = candidate;
    }
}

void NearestNeighborAssociation::trackOrConfirmTentativeBearingLandmark(
    const Measurement& measurement,
    const Eigen::Vector3d& rayOriginWorld,
    const Eigen::Vector3d& rayDirectionWorld,
    AssignedMeasurements& assignedMeasurements,
    const Pose& robotPose,
    int* landmarkId)
{
    const int tentativeCandidateId =
        findNearestTentativeBearingCandidate(rayOriginWorld, rayDirectionWorld);

    if (tentativeCandidateId >= 0)
    {
        TentativeLandmark& candidate = _tentativeLandmarks[tentativeCandidateId];
        candidate.consistentObservations++;
        candidate.missedFrames = 0;
        candidate.seenInCurrentFrame = true;
        updateBearingTriangulationFromRay(candidate, rayOriginWorld, rayDirectionWorld, robotPose);

        if (landmarkId)
        {
            *landmarkId = candidate.candidateId;
        }

        if (shouldConfirmTentativeLandmark(candidate))
        {
            int nearestExistingLandmarkIndex = -1;
            double nearestExistingDistance = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < _landmarks.size(); ++i)
            {
                const double distance =
                    (candidate.position.getPositionVector() - _landmarks[i].position.getPositionVector()).norm();
                if (distance < nearestExistingDistance)
                {
                    nearestExistingDistance = distance;
                    nearestExistingLandmarkIndex = static_cast<int>(i);
                }
            }

            if (nearestExistingLandmarkIndex >= 0 &&
                nearestExistingDistance < MIN_NEW_LANDMARK_SEPARATION_METERS)
            {
                Landmark& mergedLandmark = _landmarks[static_cast<std::size_t>(nearestExistingLandmarkIndex)];
                mergedLandmark.observeRepeat++;

                AssignedMeasurement meas(measurement, mergedLandmark.id);
                meas.isNew = false;
                assignedMeasurements.push_back(meas);
                _tentativeLandmarks.erase(candidate.candidateId);
                return;
            }

            Landmark confirmedLandmark;
            confirmedLandmark.id = _numberLandmarks++;
            confirmedLandmark.position = candidate.position;
            confirmedLandmark.variance = candidate.variance;
            confirmedLandmark.observeRepeat = candidate.consistentObservations;

            _landmarks.push_back(confirmedLandmark);
            appendNewLandmarkAssignment(measurement, confirmedLandmark, assignedMeasurements);
            _tentativeLandmarks.erase(candidate.candidateId);
        }
        return;
    }

    TentativeLandmark candidate;
    candidate.candidateId = _nextTentativeId++;
    candidate.isUnderConstrained = true;
    candidate.consistentObservations = 1;
    candidate.missedFrames = 0;
    candidate.seenInCurrentFrame = true;
    const double initialVariance = UNDER_CONSTRAINED_GATING_DISTANCE * UNDER_CONSTRAINED_GATING_DISTANCE;
    candidate.variance = Variance2D(initialVariance, 0.0, initialVariance);
    updateBearingTriangulationFromRay(candidate, rayOriginWorld, rayDirectionWorld, robotPose);

    if (landmarkId)
    {
        *landmarkId = candidate.candidateId;
    }

    _tentativeLandmarks[candidate.candidateId] = candidate;
}

double NearestNeighborAssociation::euclideanDistance(const Landmark& meas, Landmark feature)
{
    Eigen::Vector3d vec1 = meas.position.getPositionVector();
    Eigen::Vector3d vec2 = feature.position.getPositionVector();
    return (vec1 - vec2).norm();
}

double NearestNeighborAssociation::mahalanobisDistance(
    [[maybe_unused]] const Landmark& meas,
    [[maybe_unused]] Landmark feature)
{
    return 0.0;
}

double NearestNeighborAssociation::matchingScore(double distance)
{
    return std::exp(-std::pow(distance, 2));
}

void NearestNeighborAssociation::markTentativeCandidatesUnseenForCurrentFrame()
{
    for (auto& candidateEntry : _tentativeLandmarks)
    {
        candidateEntry.second.seenInCurrentFrame = false;
    }
}

void NearestNeighborAssociation::updateTentativeLandmark(TentativeLandmark& candidate, const Position& measurementPosition)
{
    candidate.sampleCount++;
    const double sampleCount = static_cast<double>(candidate.sampleCount);
    const Position previousMean(candidate.meanX, candidate.meanY, candidate.meanZ);
    const double consistencyDistance =
        (measurementPosition.getPositionVector() - previousMean.getPositionVector()).norm();

    if (candidate.sampleCount == 1)
    {
        candidate.consistentObservations = 1;
    }
    else if (consistencyDistance < (candidate.isUnderConstrained ? UNDER_CONSTRAINED_GATING_DISTANCE : GATING_DISTANCE))
    {
        candidate.consistentObservations++;
    }
    else
    {
        candidate.consistentObservations = 1;
    }

    const double dx = measurementPosition.x - candidate.meanX;
    candidate.meanX += dx / sampleCount;
    candidate.m2X += dx * (measurementPosition.x - candidate.meanX);

    const double dy = measurementPosition.y - candidate.meanY;
    candidate.meanY += dy / sampleCount;
    candidate.m2Y += dy * (measurementPosition.y - candidate.meanY);

    const double dz = measurementPosition.z - candidate.meanZ;
    candidate.meanZ += dz / sampleCount;

    candidate.position = Position(candidate.meanX, candidate.meanY, candidate.meanZ);
    candidate.missedFrames = 0;

    if (candidate.sampleCount > 1)
    {
        const double varianceX = candidate.m2X / static_cast<double>(candidate.sampleCount - 1);
        const double varianceY = candidate.m2Y / static_cast<double>(candidate.sampleCount - 1);
        candidate.variance = Variance2D(varianceX, 0.0, varianceY);
    }
    else
    {
        const double largeInitialVariance = GATING_DISTANCE * GATING_DISTANCE;
        candidate.variance = Variance2D(largeInitialVariance, 0.0, largeInitialVariance);
    }
}

void NearestNeighborAssociation::updateBearingTriangulation(
    TentativeLandmark& candidate,
    const Measurement& measurement,
    const Pose& robotPose)
{
    if (!candidate.isUnderConstrained)
    {
        return;
    }

    auto bearingModel = std::dynamic_pointer_cast<BearingMeasurementModel>(measurement.model);
    if (!bearingModel)
    {
        return;
    }

    Eigen::Vector3d rayOriginWorld;
    Eigen::Vector3d rayDirectionWorld;
    if (!bearingModel->worldRayFromMeasurement(robotPose, measurement, rayOriginWorld, rayDirectionWorld))
    {
        return;
    }

    updateBearingTriangulationFromRay(candidate, rayOriginWorld, rayDirectionWorld, robotPose);
}

void NearestNeighborAssociation::updateBearingTriangulationFromRay(
    TentativeLandmark& candidate,
    const Eigen::Vector3d& rayOriginWorld,
    const Eigen::Vector3d& rayDirectionWorld,
    const Pose& robotPose)
{
    if (!candidate.isUnderConstrained)
    {
        return;
    }

    candidate.bearingObservations.push_back(BearingRayObservation());
    BearingRayObservation& observation = candidate.bearingObservations.back();
    observation.originWorld = rayOriginWorld;
    observation.directionWorld = rayDirectionWorld;

    if (candidate.bearingObservations.size() > MAX_BEARING_OBSERVATION_BUFFER)
    {
        candidate.bearingObservations.pop_front();
    }

    Position triangulatedPosition;
    Variance2D triangulatedVariance;
    double residual = 0.0;
    double maxParallaxRadians = 0.0;
    double maxBaselineMeters = 0.0;
    double minForwardDepthMeters = 0.0;
    if (triangulateBearingCandidate(
            candidate,
            triangulatedPosition,
            triangulatedVariance,
            residual,
            maxParallaxRadians,
            maxBaselineMeters,
            minForwardDepthMeters))
    {
        candidate.position = triangulatedPosition;
        candidate.variance = triangulatedVariance;
        candidate.hasTriangulatedPosition = true;
        candidate.triangulationResidual = residual;
        candidate.maxParallaxRadians = maxParallaxRadians;
        candidate.maxBaselineMeters = maxBaselineMeters;
        candidate.minForwardDepthMeters = minForwardDepthMeters;
    }
}

bool NearestNeighborAssociation::triangulateBearingCandidate(
    const TentativeLandmark& candidate,
    Position& triangulatedPosition,
    Variance2D& triangulatedVariance,
    double& residual,
    double& maxParallaxRadians,
    double& maxBaselineMeters,
    double& minForwardDepthMeters) const
{
    const std::size_t observationCount = candidate.bearingObservations.size();
    if (observationCount < MIN_TRIANGULATION_OBSERVATIONS)
    {
        return false;
    }

    maxParallaxRadians = 0.0;
    maxBaselineMeters = 0.0;
    for (std::size_t i = 0; i < observationCount; ++i)
    {
        const Eigen::Vector3d d1 = candidate.bearingObservations[i].directionWorld.normalized();
        for (std::size_t j = i + 1; j < observationCount; ++j)
        {
            const Eigen::Vector3d d2 = candidate.bearingObservations[j].directionWorld.normalized();
            const double dot = std::max(-1.0, std::min(1.0, d1.dot(d2)));
            const double angle = std::acos(dot);
            if (angle > maxParallaxRadians)
            {
                maxParallaxRadians = angle;
            }

            const double baseline =
                (candidate.bearingObservations[i].originWorld - candidate.bearingObservations[j].originWorld).norm();
            if (baseline > maxBaselineMeters)
            {
                maxBaselineMeters = baseline;
            }
        }
    }

    if (maxParallaxRadians < MIN_TRIANGULATION_PARALLAX_RADIANS)
    {
        return false;
    }

    if (maxBaselineMeters < MIN_TRIANGULATION_BASELINE_METERS)
    {
        return false;
    }

    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const BearingRayObservation& observation : candidate.bearingObservations)
    {
        const Eigen::Vector3d direction = observation.directionWorld.normalized();
        const Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - direction * direction.transpose();
        A += projector;
        b += projector * observation.originWorld;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(A);
    if (eigensolver.info() != Eigen::Success)
    {
        return false;
    }

    const Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
    if (eigenvalues(0) < 1e-6)
    {
        return false;
    }

    const Eigen::Vector3d estimatedPosition = A.ldlt().solve(b);
    if (!estimatedPosition.allFinite())
    {
        return false;
    }

    minForwardDepthMeters = std::numeric_limits<double>::infinity();
    double residualSum = 0.0;
    for (const BearingRayObservation& observation : candidate.bearingObservations)
    {
        const double forwardDepth =
            (estimatedPosition - observation.originWorld).dot(observation.directionWorld.normalized());
        minForwardDepthMeters = std::min(minForwardDepthMeters, forwardDepth);

        residualSum += pointToRayDistance(
            estimatedPosition,
            observation.originWorld,
            observation.directionWorld);
    }

    if (minForwardDepthMeters < MIN_TRIANGULATION_FORWARD_DEPTH_METERS)
    {
        return false;
    }

    residual = residualSum / static_cast<double>(observationCount);
    triangulatedPosition = Position(estimatedPosition);

    const double varianceComponent = residual * residual;
    triangulatedVariance = Variance2D(varianceComponent, 0.0, varianceComponent);
    return true;
}

double NearestNeighborAssociation::pointToRayDistance(
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& rayOrigin,
    const Eigen::Vector3d& rayDirection)
{
    const Eigen::Vector3d direction = rayDirection.normalized();
    const Eigen::Vector3d offset = point - rayOrigin;
    const double t = std::max(0.0, offset.dot(direction));
    const Eigen::Vector3d closestPoint = rayOrigin + t * direction;
    return (point - closestPoint).norm();
}

bool NearestNeighborAssociation::shouldConfirmTentativeLandmark(const TentativeLandmark& candidate) const
{
    if (candidate.consistentObservations < MIN_CONFIRMATION_OBSERVATIONS)
    {
        return false;
    }

    const double covarianceTrace = candidate.variance.xx + candidate.variance.yy;

    if (candidate.isUnderConstrained)
    {
        if (!candidate.bearingObservations.empty())
        {
            if (!candidate.hasTriangulatedPosition)
            {
                return false;
            }

            if (candidate.bearingObservations.size() < MIN_TRIANGULATION_OBSERVATIONS)
            {
                return false;
            }

            if (candidate.maxParallaxRadians < MIN_TRIANGULATION_PARALLAX_RADIANS)
            {
                return false;
            }

            if (candidate.maxBaselineMeters < MIN_TRIANGULATION_BASELINE_METERS)
            {
                return false;
            }

            if (candidate.minForwardDepthMeters < MIN_TRIANGULATION_FORWARD_DEPTH_METERS)
            {
                return false;
            }

            if (candidate.triangulationResidual > MAX_TRIANGULATION_MEAN_RAY_RESIDUAL)
            {
                return false;
            }
        }

        return covarianceTrace <= UNDER_CONSTRAINED_MAX_COVARIANCE_TRACE;
    }

    return covarianceTrace <= MAX_TENTATIVE_COVARIANCE_TRACE;
}

void NearestNeighborAssociation::pruneTentativeLandmarks()
{
    std::vector<int> toRemove;
    toRemove.reserve(_tentativeLandmarks.size());

    for (auto& entry : _tentativeLandmarks)
    {
        TentativeLandmark& candidate = entry.second;
        if (!candidate.seenInCurrentFrame)
        {
            candidate.missedFrames++;
            candidate.consistentObservations = 0;
        }

        if (candidate.missedFrames > MAX_TENTATIVE_MISSED_FRAMES)
        {
            _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Rejecting tentative landmark candidate ", candidate.candidateId,
                         " after missed frames: ", candidate.missedFrames);
            toRemove.push_back(entry.first);
        }
        // Early rejection: if under-constrained bearing candidate has accumulated observations
        // but parallax remains low, reject it to avoid confirming poorly-triangulated landmarks
        else if (candidate.isUnderConstrained && candidate.bearingObservations.size() >= EARLY_REJECTION_OBSERVATION_THRESHOLD)
        {
            if (candidate.maxParallaxRadians < EARLY_REJECTION_PARALLAX_THRESHOLD)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Early rejecting tentative bearing candidate ", candidate.candidateId,
                             " due to low parallax (", candidate.maxParallaxRadians, " rad) after ",
                             candidate.bearingObservations.size(), " observations");
                toRemove.push_back(entry.first);
            }
        }
    }

    for (const int candidateId : toRemove)
    {
        _tentativeLandmarks.erase(candidateId);
    }
}

int NearestNeighborAssociation::findNearestTentativeCandidate(
    const Landmark& measurementLandmark,
    bool isUnderConstrainedInitialization) const
{
    int nearestCandidateId = -1;
    double shortestDistance = std::numeric_limits<double>::max();
    const double gatingDistance =
        isUnderConstrainedInitialization ? UNDER_CONSTRAINED_GATING_DISTANCE : GATING_DISTANCE;
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

    if (shortestDistance < gatingDistance)
    {
        return nearestCandidateId;
    }

    return -1;
}

int NearestNeighborAssociation::findNearestTentativeBearingCandidate(
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

        // Scoring: combine direction similarity AND baseline/origin consistency
        const Eigen::Vector3d referenceDirection =
            candidate.bearingObservations.back().directionWorld.normalized();
        const double dot = std::max(-1.0, std::min(1.0, referenceDirection.dot(normalizedDirection)));
        const double angle = std::acos(dot);
        
        // Hard gate: direction must be reasonably aligned
        if (angle > BEARING_TENTATIVE_DIRECTION_GATING_RADIANS)
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
                    avgBaseline += (candidate.bearingObservations[i].originWorld - 
                                   candidate.bearingObservations[j].originWorld).norm();
                    pairCount++;
                }
            }
            if (pairCount > 0)
            {
                avgBaseline /= pairCount;

                baselineScore = std::min(0.3, avgBaseline * 0.8);
            }
        }

        // For triangulated candidates, also check ray-distance gate
        if (candidate.hasTriangulatedPosition)
        {
            const double rayDistance = pointToRayDistance(
                candidate.position.getPositionVector(),
                rayOriginWorld,
                normalizedDirection);
            if (rayDistance > BEARING_TRIANGULATED_RAY_DISTANCE_GATING_METERS)
            {
                continue;
            }
        }

        const double angleScore = (BEARING_TENTATIVE_DIRECTION_GATING_RADIANS - angle) / BEARING_TENTATIVE_DIRECTION_GATING_RADIANS;
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