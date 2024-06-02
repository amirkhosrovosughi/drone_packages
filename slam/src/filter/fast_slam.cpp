#include "filter/fast_slam.hpp"
#include <iostream>

FastSlam::FastSlam() {}

void FastSlam::prediction() {
    std::cout << "FastSlam prediction step" << std::endl;
    if (callback_) callback_();
}

void FastSlam::correction() {
    std::cout << "FastSlam correction step" << std::endl;
}

void FastSlam::registerCallback(std::function<void()> callback) {
    callback_ = callback;
}