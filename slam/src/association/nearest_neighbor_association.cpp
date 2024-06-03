#include "association/nearest_neighbor_association.hpp"
#include <iostream>

NearestNeighborAssociation::NearestNeighborAssociation()
{
}

void NearestNeighborAssociation::onReceiveMeasurement() 
{
    std::cout << "onReceiveMeasurement" << std::endl;
}

void NearestNeighborAssociation::handleUpdate()
{
    std::cout << "handleUpdate" << std::endl;
}

void NearestNeighborAssociation::processMeasurement()
{
    std::cout << "processMeasurement" << std::endl;
}