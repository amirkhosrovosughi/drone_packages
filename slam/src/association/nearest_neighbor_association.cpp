#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>


static const double GATHING_DISTANCE = 0.25; // To be tuned

NearestNeighborAssociation::NearestNeighborAssociation()
{
}

void NearestNeighborAssociation::onReceiveMeasurement(const Measurements& meas) 
{
    std::future<void> result = std::async(std::launch::async, &NearestNeighborAssociation::processMeasurement, this, meas);
    std::cout << "onReceiveMeasurement" << std::endl;
}

void NearestNeighborAssociation::handleUpdate(const Measurements& updatedLandmarks)
{
    std::lock_guard<std::mutex> lock(_mutex);
    for (Measurement landmark : _landmarks)
    {
        for (const Measurement updatedLandmark : updatedLandmarks)
        {
            bool foundMatchedLandmark = false;
            if (landmark.id == updatedLandmark.id)
            {
                landmark.position = updatedLandmark.position;
                foundMatchedLandmark = true;
            }
            if (!foundMatchedLandmark)
            {
                throw std::logic_error("invalid landmark id");
            }
        }
    }
    std::cout << "handleUpdate" << std::endl;
}

void NearestNeighborAssociation::processMeasurement(const Measurements& measurements)
{
    Measurements  associatedMeasurement;
    {
        std::lock_guard<std::mutex> lock(_mutex);

        std::vector<int> assingedFeature(_landmarks.size(), 0);

        for (const Measurement measurement : measurements)
        {
            int startingLandmakIndex = 0;
            auto it = std::find_if(assingedFeature.begin(), assingedFeature.end(), [](int value) { return value != 0; });
            if (it != assingedFeature.end())
            {
                startingLandmakIndex = std::distance(assingedFeature.begin(), it);
                std::cout << "First unassigned landmark is: " << startingLandmakIndex << "\n";

            }
            else
            {
                std::cout << "No unassigned landmark left, it is a new landmark.\n";
                Measurement meas = measurement;
                meas.id = numberLandmarks++;
                meas.isNew = true;
                meas.observeRepeat = 1;

                _landmarks.push_back(meas);
                associatedMeasurement.push_back(meas);
                continue;
            }

            double shortestDistance = euclideanDistance(measurement, _landmarks[startingLandmakIndex]);
            double distance = 0;
            int nearestIndex = startingLandmakIndex;

            for (int i = startingLandmakIndex + 1; i < _landmarks.size(); i++)
            {
                distance = euclideanDistance(measurement, _landmarks[i]);
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                    nearestIndex = i;
                }
            }

            if (nearestIndex < GATHING_DISTANCE)
            {
                std::cout << "Find a matching landmark\n";
                _landmarks[nearestIndex].isNew = false;
                _landmarks[nearestIndex].observeRepeat ++;

                Measurement meas = measurement;
                meas.id = _landmarks[nearestIndex].id;
                meas.isNew = false;
                meas.observeRepeat = _landmarks[nearestIndex].observeRepeat;

                associatedMeasurement.push_back(meas);
                assingedFeature[nearestIndex] = 1;
            }
            else
            {
                std::cout << "Not match any of landmark, mark as a new feature\n";
                Measurement meas = measurement;
                meas.id = numberLandmarks++;
                meas.isNew = true;
                meas.observeRepeat = 1;

                _landmarks.push_back(meas);
                associatedMeasurement.push_back(meas);
            }
        }
        std::cout << "Measurement are processed" << std::endl;
    }

    if (_callback)
    {
        std::async(std::launch::async, _callback, associatedMeasurement);
    }

    std::cout << "callback measurement is called" << std::endl;
}

double NearestNeighborAssociation::euclideanDistance(const Measurement& meas, Measurement feature)
{
    Eigen::Vector3d vec1 = meas.position.getPositionVector();
    Eigen::Vector3d vec2 = feature.position.getPositionVector();
    return (vec1 - vec2).norm();
}

double NearestNeighborAssociation::mahalanobisDistance(const Measurement& meas, Measurement feature)
{
    return 0.0;
}

double NearestNeighborAssociation::matchingScore(double distance)
{
    return std::exp(-std::pow(distance, 2));
}