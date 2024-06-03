#ifndef SLAM__ASSOCIATION_HPP_
#define SLAM__ASSOCIATION_HPP_

#include <functional>

class BaseAssociation {
public:
    virtual ~BaseAssociation() = default;
    virtual void onReceiveMeasurement() = 0;
    virtual void handleUpdate() = 0;
    virtual void registerCallback(std::function<void()> callback) = 0;
private:
    virtual void processMeasurement() = 0;
};

#endif  // SLAM__ASSOCIATION_HPP_