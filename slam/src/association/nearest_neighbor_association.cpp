#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <algorithm>
#include <limits>


static const double GATING_DISTANCE = 0.5;
static const double QUATERNION_RATE_LIMIT = 0.2;
static const int MIN_CONFIRMATION_OBSERVATIONS = 5;
static const int MAX_TENTATIVE_MISSED_FRAMES = 6;
static const double MAX_TENTATIVE_COVARIANCE_TRACE = 0.10;

static const LogLevel HIGH_LEVEL = LogLevel::INFO;
static const LogLevel LOW_LEVEL = LogLevel::DEBUG;
static const std::string LOG_SUBSECTION = "[association] - ";

NearestNeighborAssociation::NearestNeighborAssociation()
{
}

void NearestNeighborAssociation::onReceiveMeasurement(const Measurements& meas) 
{
    std::future<void> result = std::async(std::launch::async, &NearestNeighborAssociation::processMeasurement, this, meas);
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

        
        // if (roll/picht is bigger than a limit) -> skip as detection will not be proside
        if (_quaternionRate > QUATERNION_RATE_LIMIT) {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Drone angle moving so fast: ", _quaternionRate ,", skip the detection.\n");
            pruneTentativeLandmarks();
            return;
        }

        #ifdef STORE_DEBUG_DATA
        data_logging_utils::DataLogger::log("robotPose.position.x", _robotPose.position.x);
        data_logging_utils::DataLogger::log("robotPose.position.y", _robotPose.position.y);
        data_logging_utils::DataLogger::log("robotPose.position.z", _robotPose.position.z);
     
        data_logging_utils::DataLogger::log("robotPose.quaternion.w", _robotPose.quaternion.w);
        data_logging_utils::DataLogger::log("robotPose.quaternion.x", _robotPose.quaternion.x);
        data_logging_utils::DataLogger::log("robotPose.quaternion.y", _robotPose.quaternion.y);
        data_logging_utils::DataLogger::log("robotPose.quaternion.z", _robotPose.quaternion.z);
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
            data_logging_utils::DataLogger::log(plotTagMeas + "x", measurement.payload[0]);
            data_logging_utils::DataLogger::log(plotTagMeas + "y", measurement.payload[1]);
            data_logging_utils::DataLogger::log(plotTagMeas + "z", measurement.payload[2]);
            #endif

            auto capturedLandmarkPosition = measurement.model->inverse(_robotPose, measurement);
            if (!capturedLandmarkPosition)
            {
                
                // TODO:  The landmark should still be initialized, but as a high-uncertainty hypothesis 
                //along the bearing ray, and refined over multiple observations.

                // TODO (bearing-only / under-constrained measurement):
                // This measurement does not admit a unique inverse observation.
                // Landmark should still be initialized as a high-uncertainty hypothesis
                // (e.g., along the bearing ray with large covariance), or delayed until
                // sufficient parallax is available. This requires:
                //   - Landmark lifecycle states (uninitialized / tentative / confirmed)
                //   - EKF / SLAM support for large anisotropic covariance
                //   - Association logic that refines hypotheses over multiple observations
                // Graph-SLAM can naturally handle this via ray constraint factors.
                continue;
            }
            Landmark landmark;
            landmark.position = capturedLandmarkPosition.value();
            
            _logger->log(LOW_LEVEL, LOG_SUBSECTION, "landmark position is: \n (", landmark.position.x,
                        ", ", landmark.position.y, ", ", landmark.position.z, ") \n");

            #ifdef STORE_DEBUG_DATA
            std::string plotTag =  "landmarkPosition_" +  std::to_string(measurementCounter) + "_";
            data_logging_utils::DataLogger::log(plotTag + "x", landmark.position.x);
            data_logging_utils::DataLogger::log(plotTag + "y", landmark.position.y);
            data_logging_utils::DataLogger::log(plotTag + "z", landmark.position.z);
            #endif

            // Find the first unassigend feature
            int startingLandmarkIndex = 0;
            auto it = std::find_if(assignedFeature.begin(), assignedFeature.end(), [](int value) { return value == 0; });
            if (it != assignedFeature.end())
            {
                startingLandmarkIndex = std::distance(assignedFeature.begin(), it);
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "First unassigned landmark is: ", startingLandmarkIndex, "\n");

            }
            else
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!-------------------------------------!");
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "No unassigned confirmed landmark, track tentative candidate.\n");

                int tentativeCandidateId = findNearestTentativeCandidate(landmark);
                if (tentativeCandidateId >= 0)
                {
                    TentativeLandmark& candidate = _tentativeLandmarks[tentativeCandidateId];
                    updateTentativeLandmark(candidate, landmark.position);
                    candidate.seenInCurrentFrame = true;

                    if (shouldConfirmTentativeLandmark(candidate))
                    {
                        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Confirming tentative landmark candidate ", candidate.candidateId,
                                     " after ", candidate.consistentObservations, " observations.");

                        Landmark confirmedLandmark;
                        confirmedLandmark.id = _numberLandmarks++;
                        confirmedLandmark.position = candidate.position;
                        confirmedLandmark.variance = candidate.variance;
                        confirmedLandmark.observeRepeat = candidate.consistentObservations;

                        AssignedMeasurement meas(measurement, confirmedLandmark.id);
                        meas.isNew = true;
                        assignedMeasurements.push_back(meas);

                        _landmarks.push_back(confirmedLandmark);
                        _tentativeLandmarks.erase(candidate.candidateId);
                    }
                }
                else
                {
                    TentativeLandmark candidate;
                    candidate.candidateId = _nextTentativeId++;
                    updateTentativeLandmark(candidate, landmark.position);
                    candidate.seenInCurrentFrame = true;
                    _tentativeLandmarks[candidate.candidateId] = candidate;
                }
                continue;
            }

            double shortestDistance = euclideanDistance(landmark, _landmarks[startingLandmarkIndex]);
            double distance = 0;
            int nearestIndex = startingLandmarkIndex;

            for (int i = startingLandmarkIndex + 1; i < _landmarks.size(); i++)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "landmark ", i, " position is: (", _landmarks[i].position.x,
                            ", ", _landmarks[i].position.y, ", ", _landmarks[i].position.z, ") \n");
                distance = euclideanDistance(landmark, _landmarks[i]);
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                    nearestIndex = i;
                    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "--- nearestIndex so far: ", nearestIndex,
                                ", _landmarks[i].id: ", _landmarks[i].id, "Distance:", shortestDistance);
                }
            }

            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "========= index nearest: [[[ ", nearestIndex,
                        " ]]], shortestDistance: ", shortestDistance, " \n");

            int landmarkId = 0;
            if (shortestDistance < GATING_DISTANCE)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Find a matching landmark\n.");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Nearest machitng id is", _landmarks[nearestIndex].id, ", and it landmark ",
                            "position is: (", _landmarks[nearestIndex].position.x, ", ", _landmarks[nearestIndex].position.y,
                            ", ", _landmarks[nearestIndex].position.z, ") \n");

                _landmarks[nearestIndex].observeRepeat ++;
                landmarkId = nearestIndex;

                AssignedMeasurement meas(measurement, _landmarks[nearestIndex].id);
                meas.isNew = false;

                assignedMeasurements.push_back(meas);
                assignedFeature[nearestIndex] = 1;
            }
            else
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!-------------------------------------!");
                int tentativeCandidateId = findNearestTentativeCandidate(landmark);

                if (tentativeCandidateId >= 0)
                {
                    TentativeLandmark& candidate = _tentativeLandmarks[tentativeCandidateId];
                    updateTentativeLandmark(candidate, landmark.position);
                    candidate.seenInCurrentFrame = true;
                    landmarkId = candidate.candidateId;

                    if (shouldConfirmTentativeLandmark(candidate))
                    {
                        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Confirming tentative landmark candidate ", candidate.candidateId,
                                     " after ", candidate.consistentObservations, " observations.");

                        Landmark confirmedLandmark;
                        confirmedLandmark.id = _numberLandmarks++;
                        confirmedLandmark.position = candidate.position;
                        confirmedLandmark.variance = candidate.variance;
                        confirmedLandmark.observeRepeat = candidate.consistentObservations;

                        AssignedMeasurement meas(measurement, confirmedLandmark.id);
                        meas.isNew = true;
                        assignedMeasurements.push_back(meas);

                        _landmarks.push_back(confirmedLandmark);
                        _tentativeLandmarks.erase(candidate.candidateId);
                    }
                }
                else
                {
                    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Creating tentative landmark candidate ...");
                    TentativeLandmark candidate;
                    candidate.candidateId = _nextTentativeId++;
                    updateTentativeLandmark(candidate, landmark.position);
                    candidate.seenInCurrentFrame = true;
                    landmarkId = candidate.candidateId;
                    _tentativeLandmarks[candidate.candidateId] = candidate;
                }
            }

            #ifdef STORE_DEBUG_DATA
            data_logging_utils::DataLogger::log("landmarkId", landmarkId);
            #endif
        }
        pruneTentativeLandmarks();
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Measurement are processed.\n");
    }
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Number of landmarks is ", _landmarks.size());

    if (_callback)
    {
        std::async(std::launch::async, _callback, assignedMeasurements);
    }

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "callback measurement is called.");
}

double NearestNeighborAssociation::euclideanDistance(const Landmark& meas, Landmark feature)
{
    Eigen::Vector3d vec1 = meas.position.getPositionVector();
    Eigen::Vector3d vec2 = feature.position.getPositionVector();
    return (vec1 - vec2).norm();
}

double NearestNeighborAssociation::mahalanobisDistance(const Landmark& meas, Landmark feature)
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
    else if (consistencyDistance < GATING_DISTANCE)
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

bool NearestNeighborAssociation::shouldConfirmTentativeLandmark(const TentativeLandmark& candidate) const
{
    if (candidate.consistentObservations < MIN_CONFIRMATION_OBSERVATIONS)
    {
        return false;
    }

    const double covarianceTrace = candidate.variance.xx + candidate.variance.yy;
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
    }

    for (const int candidateId : toRemove)
    {
        _tentativeLandmarks.erase(candidateId);
    }
}

int NearestNeighborAssociation::findNearestTentativeCandidate(const Landmark& measurementLandmark) const
{
    int nearestCandidateId = -1;
    double shortestDistance = std::numeric_limits<double>::max();
    const Eigen::Vector3d measurementVector = measurementLandmark.position.getPositionVector();

    for (const auto& entry : _tentativeLandmarks)
    {
        const TentativeLandmark& candidate = entry.second;
        const double distance = (measurementVector - candidate.position.getPositionVector()).norm();
        if (distance < shortestDistance)
        {
            shortestDistance = distance;
            nearestCandidateId = entry.first;
        }
    }

    if (shortestDistance < GATING_DISTANCE)
    {
        return nearestCandidateId;
    }

    return -1;
}