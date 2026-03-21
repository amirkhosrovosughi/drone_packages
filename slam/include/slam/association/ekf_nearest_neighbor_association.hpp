#ifndef SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "association/ekf_bearing_initialization_strategy.hpp"
#include "association/nearest_neighbor_association.hpp"

/**
 * @brief EKF-specific nearest-neighbor association entry point.
 *
 * This class keeps EKF wiring explicit while reusing the shared
 * nearest-neighbor association implementation.
 */
class EkfNearestNeighborAssociation : public NearestNeighborAssociation {
public:
    EkfNearestNeighborAssociation()
        : NearestNeighborAssociation(std::make_shared<EkfBearingInitializationStrategy>())
    {}

    ~EkfNearestNeighborAssociation() override = default;
};

#endif  // SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_