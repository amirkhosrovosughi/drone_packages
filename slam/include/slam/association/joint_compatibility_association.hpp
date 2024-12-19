#ifndef SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_
#define SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_

#include "base_association.hpp"

class JointCompatibilityAssociation : public BaseAssociation {
public:
    JointCompatibilityAssociation();
    void onReceiveMeasurement(const Measurements& meas) override;
    void handleUpdate(const MapSummary& map) override;
    void registerCallback(std::function<void(Measurements)> callback) override
    {
        _callback = callback;
    }
    void setLogger(LoggerPtr logger) override {}

private:
    void processMeasurement(const Measurements& meas) override;

private:
    std::function<void(Measurements)> _callback;
};

#endif  // SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_