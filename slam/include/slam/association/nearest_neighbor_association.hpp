#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"
#include <cmath>
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
    double euclideanDistance(const Measurement& meas, Measurement feature);
    double mahalanobisDistance(const Measurement& meas, Measurement feature);
    double matchingScore(double distance);

private:
    std::function<void(Measurements)> _callback;
    std::mutex _mutex;
    Measurements _landmarks;
    int numberLandmarks = 0;
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_