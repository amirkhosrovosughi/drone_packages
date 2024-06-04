#include "association/nearest_neighbor_association.hpp"
#include <iostream>
#include <thread>
#include <future>

NearestNeighborAssociation::NearestNeighborAssociation()
{
}

void NearestNeighborAssociation::onReceiveMeasurement(const Measurements& meas) 
{
    std::future<void> result = std::async(std::launch::async, &NearestNeighborAssociation::processMeasurement, this, meas);
    std::cout << "onReceiveMeasurement" << std::endl;
}

void NearestNeighborAssociation::handleUpdate(const Measurements& meas)
{
    std::cout << "handleUpdate" << std::endl;
}

void NearestNeighborAssociation::processMeasurement(const Measurements& meas)
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "processMeasurement" << std::endl;

    Measurements  meas_with_correct_id;
    if (_callback) _callback(meas_with_correct_id);
}