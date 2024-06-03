#ifndef SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_
#define SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_

#include "base_association.hpp"

class JointCompatibilityAssociation : public BaseAssociation {
public:
    JointCompatibilityAssociation();
    void onReceiveMeasurement() override;
    void handleUpdate() override;
    void registerCallback(std::function<void()> callback) override
    {
        _callback = callback;
    }

private:
    void processMeasurement() override;

private:
    std::function<void()> _callback;
};

#endif  // SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_