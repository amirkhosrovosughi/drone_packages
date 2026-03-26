#ifndef SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "association/ekf_bearing_initialization_strategy.hpp"
#include "association/nearest_neighbor_association.hpp"

class EkfNearestNeighborAssociation : public NearestNeighborAssociation {
public:
    EkfNearestNeighborAssociation()
        : NearestNeighborAssociation(std::make_shared<EkfBearingInitializationStrategy>())
    {}

    ~EkfNearestNeighborAssociation() override = default;

protected:
    void processPointMeasurement(
        const Measurement &measurement,
        std::vector<int> &assignedFeature,
        AssignedMeasurements &assignedMeasurements) override;

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

#endif  // SLAM__EKF_NEAREST_NEIGHBOR_ASSOCIATION_HPP_
