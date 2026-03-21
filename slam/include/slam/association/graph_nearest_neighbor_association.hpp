#ifndef SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "association/graph_bearing_initialization_strategy.hpp"
#include "association/nearest_neighbor_association.hpp"

/**
 * @brief Graph-specific nearest-neighbor association entry point.
 *
 * This class keeps Graph wiring explicit while reusing the shared
 * nearest-neighbor association implementation.
 */
class GraphNearestNeighborAssociation : public NearestNeighborAssociation {
public:
    GraphNearestNeighborAssociation()
        : NearestNeighborAssociation(std::make_shared<GraphBearingInitializationStrategy>())
    {}

    ~GraphNearestNeighborAssociation() override = default;
};

#endif  // SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_