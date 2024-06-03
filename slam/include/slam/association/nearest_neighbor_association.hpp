#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"

class NearestNeighborAssociation : public BaseAssociation {
public:
    NearestNeighborAssociation();
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

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_