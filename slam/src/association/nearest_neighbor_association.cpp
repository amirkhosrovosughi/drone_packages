#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>


static const double GATHING_DISTANCE = 0.5;
static const double QUATERNION_RATE_LIMIT = 0.2;

static const LogLevel HIGH_LEVEL = LogLevel::INFO;
static const LogLevel LOW_LEVEL = LogLevel::DEBUG;
static const std::string LOG_SUBSECTION = "[association] - ";

NearestNeighborAssociation::NearestNeighborAssociation()
{
#ifdef POSITION_MOTION_POSITION_MEASUREMENT
    _model = std::make_shared<PositionPositionMotionMeasurementModel>();
#elif POSE_MOTION_POSITION_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#elif POSITION_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#elif POSE_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#else
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Have not define compile tag for motion model");
    throw std::runtime_error("motion measurement model is not specified");
#endif


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
    Measurements  associatedMeasurement;
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

        std::vector<int> assingedFeature(_landmarks.size(), 0);

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "size measurements is:", measurements.size(), ", size Landmark is: (( ", _landmarks.size(), " ))");

        #ifdef STORE_DEBUG_DATA
        int measurementCounter = 0;
        #endif

        for (const Measurement measurement : measurements)
        {
            #ifdef STORE_DEBUG_DATA
            std::string plotTagMeas =  "MeasurementPosition_" +  std::to_string(measurementCounter++) + "_";
            data_logging_utils::DataLogger::log(plotTagMeas + "x", measurement.position.x);
            data_logging_utils::DataLogger::log(plotTagMeas + "y", measurement.position.y);
            data_logging_utils::DataLogger::log(plotTagMeas + "z", measurement.position.z);
            #endif


            _logger->log(LOW_LEVEL, LOG_SUBSECTION, "==> measurement is: (", measurement.position.x, ", ",
                        measurement.position.y, ", ", measurement.position.z, ") \n");

            Position capturedLandmarkPosition = _model->inverseObservationModel(_robotPose, measurement);
            Landmark landmark;
            landmark.position = capturedLandmarkPosition;
            
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
            auto it = std::find_if(assingedFeature.begin(), assingedFeature.end(), [](int value) { return value == 0; });
            if (it != assingedFeature.end())
            {
                startingLandmarkIndex = std::distance(assingedFeature.begin(), it);
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "First unassigned landmark is: ", startingLandmarkIndex, "\n");

            }
            else
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!-------------------------------------!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Adding a new landmark ...");
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "No unassigned landmark left, it is a new landmark.\n");
                landmark.id = numberLandmarks++;
                landmark.observeRepeat = 1;
                Measurement meas(landmark.id, measurement.position);
                meas.isNew = true;

                _landmarks.push_back(landmark);
                associatedMeasurement.push_back(meas);
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
            if (shortestDistance < GATHING_DISTANCE)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Find a matching landmark\n.");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Nearest machitng id is", _landmarks[nearestIndex].id, ", and it landmark ",
                            "position is: (", _landmarks[nearestIndex].position.x, ", ", _landmarks[nearestIndex].position.y,
                            ", ", _landmarks[nearestIndex].position.z, ") \n");

                _landmarks[nearestIndex].observeRepeat ++;
                landmarkId = nearestIndex;

                Measurement meas(_landmarks[nearestIndex].id, measurement.position);
                meas.isNew = false;

                associatedMeasurement.push_back(meas);
                assingedFeature[nearestIndex] = 1;
            }
            else
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!-------------------------------------!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Adding a new landmark ...");
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Not match any of landmarks, marks as a new feature.\n");
                landmark.id = numberLandmarks++;
                landmark.observeRepeat = 1;
                Measurement meas(landmark.id, measurement.position);
                meas.isNew = true;
                landmarkId = landmark.id;

                _landmarks.push_back(landmark);
                associatedMeasurement.push_back(meas);
            }

            #ifdef STORE_DEBUG_DATA
            data_logging_utils::DataLogger::log("landmarkId", landmarkId);
            #endif
        }
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Measurement are processed.\n");
    }
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Number of landmarks is ", _landmarks.size());

    if (_callback) // AMIR TO BE uncommented
    {
        std::async(std::launch::async, _callback, associatedMeasurement);
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