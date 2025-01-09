#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>


static const double GATHING_DISTANCE = 1.0; // TODO: To be tuned 0.25

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

    Landmarks updatedLandmarks = map.landmarks;
    for (Landmark landmark : _landmarks)
    {
        for (const Landmark updatedLandmark : updatedLandmarks)
        {
            bool foundMatchedLandmark = false;
            if (landmark.id == updatedLandmark.id)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "!--! landmark:" , landmark.id, " has been updated from (",
                            landmark.position.x, ", ", landmark.position.y, ", ", landmark.position.z, "to ",
                            updatedLandmark.position.x, ", ", updatedLandmark.position.y, ", ",
                            updatedLandmark.position.z, ") \n");
                landmark.position = updatedLandmark.position;
                foundMatchedLandmark = true;
            }
            if (!foundMatchedLandmark)
            {
                throw std::logic_error("invalid landmark id");
            }
        }
    }
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "handleUpdate.");
}

void NearestNeighborAssociation::processMeasurement(const Measurements& measurements)
{
    Measurements  associatedMeasurement;
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::vector<int> assingedFeature(_landmarks.size(), 0);

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "size measurements is:", measurements.size(), ", size Landmark is: (( ", _landmarks.size(), " ))");

        for (const Measurement measurement : measurements)
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "==> measurement is: (", measurement.position.x, ", ",
                        measurement.position.y, ", ", measurement.position.z, ") \n");

            // need to transfer measurement to landmark position, for that we need _model, robot Pose
            Position capturedLandmarkPosition = _model->inverseObservationModel(_robotPose, measurement);
            Landmark landmark;
            landmark.position = capturedLandmarkPosition;

            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "==> landmark position is: (", landmark.position.x,
                        ", ", landmark.position.y, ", ", landmark.position.z, ") \n");

            int startingLandmarkIndex = 0;
            auto it = std::find_if(assingedFeature.begin(), assingedFeature.end(), [](int value) { return value == 0; });
            if (it != assingedFeature.end())
            {
                startingLandmarkIndex = std::distance(assingedFeature.begin(), it);
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "First unassigned landmark is: ", startingLandmarkIndex, "\n");

            }
            else
            {
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "!!!!!!!!!!!!!---adding a new landmark---!!!!!!!!!!!!!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "No unassigned landmark left, it is a new landmark.\n");
                // Measurement meas = measurement;
                landmark.id = numberLandmarks++;
                // meas.isNew = true;
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
                    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "--- nearestIndex to far: ", nearestIndex,
                                ", _landmarks[i].id: ", _landmarks[i].id, "Distance:", shortestDistance);
                }
            }

            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "========= index nearest: [[[ ", nearestIndex,
                        " ]]], shortestDistance: ", shortestDistance, " \n");
            if (shortestDistance < GATHING_DISTANCE)
            {
                _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Find a matching landmark\n.");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Nearest machitng id is", _landmarks[nearestIndex].id, ", and it landmark ",
                            "position is: (", _landmarks[nearestIndex].position.x, ", ", _landmarks[nearestIndex].position.y,
                            ", ", _landmarks[nearestIndex].position.z, ") \n");
                // _landmarks[nearestIndex].isNew = false;
                _landmarks[nearestIndex].observeRepeat ++;

                Measurement meas(_landmarks[nearestIndex].id, measurement.position);
                // meas.id = _landmarks[nearestIndex].id;
                meas.isNew = false;
                // meas.observeRepeat = _landmarks[nearestIndex].observeRepeat;

                associatedMeasurement.push_back(meas);
                assingedFeature[nearestIndex] = 1;
            }
            else
            {
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "!!!!!!!!!!!!!---adding a new landmark---!!!!!!!!!!!!!");
                _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Not match any of landmarks, marks as a new feature.\n");
                // Measurement meas = measurement;
                landmark.id = numberLandmarks++;
                // meas.isNew = true;
                landmark.observeRepeat = 1;
                Measurement meas(landmark.id, measurement.position);
                meas.isNew = true;

                _landmarks.push_back(landmark);
                associatedMeasurement.push_back(meas);
            }
        }
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Measurement are processed.\n");
    }
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Number of landmarks is ", _landmarks.size());

    if (_callback)
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