#include "filter/fast_slam.hpp"
#include <iostream>

FastSlam::FastSlam() {}

void FastSlam::prediction() {
    std::cout << "FastSlam prediction step" << std::endl;
    if (_callback) _callback();
}

void FastSlam::correction() {
    std::cout << "FastSlam correction step" << std::endl;
}

void FastSlam::registerCallback(std::function<void()> callback) {
    _callback = callback;
}