#ifndef SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "association/graph_bearing_initialization_strategy.hpp"
#include "association/nearest_neighbor_association.hpp"

class GraphNearestNeighborAssociation : public NearestNeighborAssociation {
public:
    GraphNearestNeighborAssociation()
        : NearestNeighborAssociation(std::make_shared<GraphBearingInitializationStrategy>())
    {}

    ~GraphNearestNeighborAssociation() override = default;

protected:
    void processPointMeasurement(
        const Measurement& measurement,
        std::vector<int>& assignedFeature,
        AssignedMeasurements& assignedMeasurements) override;

    int findNearestTentativeCandidate(
        const Landmark& measurementLandmark,
        bool isUnderConstrainedInitialization) const override;

    int findNearestTentativeBearingCandidate(
        const Eigen::Vector3d& rayOriginWorld,
        const Eigen::Vector3d& rayDirectionWorld) const override;

private:
    double getGatingDistance() const override;
    double getBearingGatingDistance() const override;
    double getBearingRelaxedFallbackDistance() const override;
    int getMinConfirmationObservations() const override;
    double getMaxTentativeCovarianceTrace() const override;
    double getUnderConstrainedMaxCovarianceTrace() const override;
    std::size_t getMinTriangulationObservations() const override;
    double getMinTriangulationParallaxRadians() const override;
    double getMinTriangulationBaselineMeters() const override;
    double getMaxTriangulationMeanRayResidual() const override;
};

#endif  // SLAM__GRAPH_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
