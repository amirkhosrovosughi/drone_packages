#ifndef SLAM__FILTER_HPP_
#define SLAM__FILTER_HPP_

#include <functional>
#include "def_slam.hpp"

class BaseFilter {
public:
    virtual ~BaseFilter() = default;
    virtual void prediction() = 0;
    virtual void correction(const Measurements& meas) = 0;
    virtual void registerCallback(std::function<void(const Map& map)> callback) = 0;
};

#endif  // SLAM__FILTER_HPP_