#include "association/joint_compatibility_association.hpp"
#include <iostream>

JointCompatibilityAssociation::JointCompatibilityAssociation()
{
}

void JointCompatibilityAssociation::onReceiveMeasurement(const Measurements& meas) 
{
    std::cout << "onReceiveMeasurement" << std::endl;
}

void JointCompatibilityAssociation::handleUpdate(const MapSummary& map)
{
    std::cout << "handleUpdate" << std::endl;
}

void JointCompatibilityAssociation::processMeasurement(const Measurements& meas)
{
    std::cout << "processMeasurement" << std::endl;
}