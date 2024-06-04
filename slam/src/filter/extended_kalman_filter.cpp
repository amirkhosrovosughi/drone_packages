#include "filter/extended_kalman_filter.hpp"
#include <iostream>
#include <thread>
#include <future>

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::prediction()
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processPrediction, this); // , value  
}

void ExtendedKalmanFilter::correction(const Measurements& meas)
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processCorrection, this, meas); // .means
}

void ExtendedKalmanFilter::registerCallback(std::function<void(const Map& map)> callback)
{
    _callback = callback;
}

void ExtendedKalmanFilter::processPrediction()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "Extended Kalman Filter prediction step" << std::endl;
    Map map;
    if (_callback) _callback(map);
}

void ExtendedKalmanFilter::processCorrection(const Measurements& meas)
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "Extended Kalman Filter correction step" << std::endl;
    Map map;
    if (_callback) _callback(map);
}