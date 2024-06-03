#include "association/joint_compatibility_association.hpp"
#include <iostream>

JointCompatibilityAssociation::JointCompatibilityAssociation()
{
}

void JointCompatibilityAssociation::onReceiveMeasurement() 
{
    std::cout << "onReceiveMeasurement" << std::endl;
}

void JointCompatibilityAssociation::handleUpdate()
{
    std::cout << "handleUpdate" << std::endl;
}

void JointCompatibilityAssociation::processMeasurement()
{
    std::cout << "processMeasurement" << std::endl;
}