#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"

#include <mutex>

class NearestNeighborAssociation : public BaseAssociation {
public:
    NearestNeighborAssociation();
    void onReceiveMeasurement(const Measurements& meas) override;
    void handleUpdate(const Measurements& meas) override;
    void registerCallback(std::function<void(Measurements)> callback) override
    {
        _callback = callback;
    }

private:
    void processMeasurement(const Measurements& meas) override;

private:
    std::function<void(Measurements)> _callback;
    std::mutex _mutex;
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_