#ifndef SLAM__ASSOCIATION_HPP_
#define SLAM__ASSOCIATION_HPP_

#include <functional>
#include "def_slam.hpp"

class BaseAssociation {
public:
    virtual ~BaseAssociation() = default;
    virtual void onReceiveMeasurement(const Measurements& meas) = 0;
    virtual void handleUpdate(const Measurements& meas) = 0;
    virtual void registerCallback(std::function<void(Measurements)> callback) = 0;
private:
    virtual void processMeasurement(const Measurements& meas) = 0;
};

#endif  // SLAM__ASSOCIATION_HPP_