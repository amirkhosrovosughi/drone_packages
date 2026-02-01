#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <algorithm>


static const double GATING_DISTANCE = 0.5;
static const double QUATERNION_RATE_LIMIT = 0.2;

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

    Landmarks updatedLandmarks = map.landmarks; // ==> commented out to make sure we do isolate error
    for (Landmark landmark : _landmarks)
    {
        bool foundMatchedLandmark = false;
        for (const Landmark updatedLandmark : updatedLandmarks) //why not simply say: _landmarks = map.landmarks, instead of this map???
                                                                // to catch error if that happens
        {
            if (landmark.id == updatedLandmark.id)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!--! landmark:" , landmark.id, " has been updated from (",
                            landmark.position.x, ", ", landmark.position.y, ", ", landmark.position.z, "to ",
                            updatedLandmark.position.x, ", ", updatedLandmark.position.y, ", ",
                            updatedLandmark.position.z, ") \n");
                landmark.position = updatedLandmark.position;
                foundMatchedLandmark = true;
                break;
            }
        }

        if (!foundMatchedLandmark)
        {
            throw std::logic_error("invalid landmark id");
        }
    }
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "handleUpdate.");
}

void NearestNeighborAssociation::processMeasurement(const Measurements& measurements)
{
    AssignedMeasurements  assignedMeasurements;
    {
        std::lock_guard<std::mutex> lock(_mutex);

        
        // if (roll/picht is bigger than a limit) -> skip as detection will not be proside
        if (_quaternionRate > QUATERNION_RATE_LIMIT) {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Drone angle moving so fast: ", _quaternionRate ,", skip the detection.\n");
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
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Adding a new landmark ...");
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "No unassigned landmark left, it is a new landmark.\n");
                landmark.id = _numberLandmarks++;
                landmark.observeRepeat = 1;
                AssignedMeasurement meas(measurement, landmark.id);
                meas.isNew = true;

                _landmarks.push_back(landmark);
                assignedMeasurements.push_back(meas);
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
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Adding a new landmark ...");
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Not match any of landmarks, marks as a new feature.\n");
                landmark.id = _numberLandmarks++;
                landmark.observeRepeat = 1;
                AssignedMeasurement meas(measurement, landmark.id);
                meas.isNew = true;
                landmarkId = landmark.id;

                _landmarks.push_back(landmark);
                assignedMeasurements.push_back(meas);
            }

            #ifdef STORE_DEBUG_DATA
            data_logging_utils::DataLogger::log("landmarkId", landmarkId);
            #endif
        }
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