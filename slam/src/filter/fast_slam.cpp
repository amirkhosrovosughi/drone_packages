#include "filter/fast_slam.hpp"
#include <iostream>

FastSlam::FastSlam() {}

void FastSlam::prediction(const OdometryInfo& odom) {
    std::cout << "FastSlam prediction step" << std::endl;
    Map map;
    if (_callback) _callback(map);
}

void FastSlam::correction(const Measurements& meas)
{
    std::cout << "FastSlam correction step" << std::endl;
}

void FastSlam::registerCallback(std::function<void(const Map& map)> callback)
{
    _callback = callback;
}