#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"
#include "motion/motion_model.hpp"
#include "measurement/measurement_model.hpp"
#include "measurement/measurement.hpp"
#include "association/under_constrained_initialization_strategy.hpp"
#include "common/slam_logger.hpp"
#include <cmath>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <vector>

#ifdef STORE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif

/**
 * @brief Single bearing ray sample used to triangulate under-constrained landmarks.
 */
struct BearingRayObservation {
    Eigen::Vector3d originWorld;
    Eigen::Vector3d directionWorld;

    BearingRayObservation()
        : originWorld(Eigen::Vector3d::Zero()), directionWorld(Eigen::Vector3d::UnitX())
    {}
};

/**
 * @brief Tentative landmark candidate tracked before confirmation.
 *
 * Stores running statistics, observation history, and quality metrics used
 * by the association stage to either confirm or reject the candidate.
 */
struct TentativeLandmark {
    int candidateId;
    Position position;
    Variance2D variance;
    bool isUnderConstrained;
    int consistentObservations;
    int missedFrames;
    bool seenInCurrentFrame;
    int sampleCount;
    double meanX;
    double meanY;
    double meanZ;
    double m2X;
    double m2Y;
    std::deque<BearingRayObservation> bearingObservations;
    bool hasTriangulatedPosition;
    double triangulationResidual;
    double maxParallaxRadians;
    double maxBaselineMeters;
    double minForwardDepthMeters;

    TentativeLandmark()
        : candidateId(0),
          position(Position()),
          variance(Variance2D()),
          isUnderConstrained(false),
          consistentObservations(0),
          missedFrames(0),
          seenInCurrentFrame(false),
          sampleCount(0),
          meanX(0.0),
          meanY(0.0),
          meanZ(0.0),
          m2X(0.0),
          m2Y(0.0),
          bearingObservations(),
          hasTriangulatedPosition(false),
          triangulationResidual(0.0),
          maxParallaxRadians(0.0),
          maxBaselineMeters(0.0),
          minForwardDepthMeters(0.0)
    {}
};

using UnderConstrainedInitializationStrategyPtr =
    std::shared_ptr<UnderConstrainedInitializationStrategy>;

class NearestNeighborAssociation : public BaseAssociation {
public:
    /**
     * @brief Construct nearest-neighbor association with default under-constrained strategy.
     */
    NearestNeighborAssociation();

    /**
     * @brief Construct nearest-neighbor association with custom under-constrained strategy.
     *
     * @param strategy Strategy used when inverse measurement initialization fails.
     */
    explicit NearestNeighborAssociation(UnderConstrainedInitializationStrategyPtr strategy);
    virtual ~NearestNeighborAssociation() = default;

    /**
     * @brief Receive a measurement batch for association.
     */
    void onReceiveMeasurement(const Measurements& meas) override;

    /**
     * @brief Synchronize local association state from map summary updates.
     */
    void handleUpdate(const MapSummary& map) override;

    /**
     * @brief Register callback for association output.
     */
    void registerCallback(std::function<void(AssignedMeasurements)> callback) override
    {
        _callback = callback;
    }

    /**
     * @brief Set logger instance used by this component.
     */
    void setLogger(LoggerPtr logger) override
    {
        _logger = logger;
    }

private:
    /**
     * @brief Core measurement processing routine.
     */
    void processMeasurement(const Measurements& meas) override;

protected:
    /**
     * @brief Process one point measurement.
     *
     * Pipeline-specific behavior must be implemented by child classes.
     */
    virtual void processPointMeasurement(
        const Measurement& measurement,
        std::vector<int>& assignedFeature,
        AssignedMeasurements& assignedMeasurements) = 0;

    /**
     * @brief Process one bearing measurement.
     *
     * Kept in the base class for now because current EKF and Graph pipelines
     * still share the same high-level bearing flow (predict residual, gate,
     * then tentative tracking/confirmation).
     *
     * Child classes already specialize the parts that differ today via
     * threshold getters and tentative-candidate matching overrides.
     *
     * Move this to separate EKF/Graph implementations only when one pipeline
     * needs a different confirmed-landmark association metric, different
     * fallback semantics, or a different bearing tentative confirmation flow.
     */
    virtual void processBearingMeasurement(
        const Measurement& measurement,
        std::vector<int>& assignedFeature,
        AssignedMeasurements& assignedMeasurements);

    /**
     * @brief Find nearest tentative point candidate.
     */
    virtual int findNearestTentativeCandidate(
        const Landmark& measurementLandmark,
        bool isUnderConstrainedInitialization) const;

    /**
     * @brief Find nearest tentative bearing candidate.
     *
     * Pipeline-specific behavior must be implemented by child classes.
     */
    virtual int findNearestTentativeBearingCandidate(
        const Eigen::Vector3d& rayOriginWorld,
        const Eigen::Vector3d& rayDirectionWorld) const = 0;

    /**
     * @brief Check whether a tentative landmark should be confirmed.
     */
    virtual bool shouldConfirmTentativeLandmark(const TentativeLandmark& candidate) const;

    /**
     * @brief Triangulate a tentative bearing candidate.
     */
    virtual bool triangulateBearingCandidate(
        const TentativeLandmark& candidate,
        Position& triangulatedPosition,
        Variance2D& triangulatedVariance,
        double& residual,
        double& maxParallaxRadians,
        double& maxBaselineMeters,
        double& minForwardDepthMeters) const;

    /**
     * @brief Track/update a tentative point candidate and confirm when ready.
     */
    virtual void trackOrConfirmTentativeLandmark(
        const Measurement& measurement,
        const Landmark& landmark,
        bool isUnderConstrainedInitialization,
        AssignedMeasurements& assignedMeasurements,
        const Pose& robotPose,
        int* landmarkId = nullptr);

    /**
     * @brief Track/update a tentative bearing candidate and confirm when ready.
     */
    virtual void trackOrConfirmTentativeBearingLandmark(
        const Measurement& measurement,
        const Eigen::Vector3d& rayOriginWorld,
        const Eigen::Vector3d& rayDirectionWorld,
        AssignedMeasurements& assignedMeasurements,
        const Pose& robotPose,
        int* landmarkId = nullptr);

    /**
     * @brief Compute Euclidean distance between a measured landmark and feature.
     */
    double euclideanDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Compute Mahalanobis distance between a measured landmark and feature.
     */
    double mahalanobisDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Convert distance value into matching score.
     */
    double matchingScore(double distance);

    /**
     * @brief Utility distance from 3D point to ray.
     */
    static double pointToRayDistance(
        const Eigen::Vector3d& point,
        const Eigen::Vector3d& rayOrigin,
        const Eigen::Vector3d& rayDirection);

    /**
     * @brief Mark all tentative candidates unseen for current frame.
     */
    void markTentativeCandidatesUnseenForCurrentFrame();

    /**
     * @brief Remove stale tentative candidates.
     */
    void pruneTentativeLandmarks();

    /**
     * @brief Update running mean/variance for a tentative candidate.
     */
    void updateTentativeLandmark(TentativeLandmark& candidate, const Position& measurementPosition);

    /**
     * @brief Update triangulation state from a new measurement.
     */
    void updateBearingTriangulation(
        TentativeLandmark& candidate,
        const Measurement& measurement,
        const Pose& robotPose);

    /**
     * @brief Update triangulation state from a world-frame ray.
     */
    void updateBearingTriangulationFromRay(
        TentativeLandmark& candidate,
        const Eigen::Vector3d& rayOriginWorld,
        const Eigen::Vector3d& rayDirectionWorld,
        const Pose& robotPose);

    /**
     * @brief Initialize landmark estimate from measurement or fallback strategy.
     */
    bool tryInitializeLandmarkFromMeasurement(
        const Measurement& measurement,
        Landmark& landmark,
        bool& isUnderConstrainedInitialization) const;

    /**
     * @brief Append a new-landmark assignment output item.
     */
    void appendNewLandmarkAssignment(
        const Measurement& measurement,
        const Landmark& landmark,
        AssignedMeasurements& assignedMeasurements) const;

private:
    // Pipeline-specific thresholds are abstract and supplied only by child classes.
    virtual double getGatingDistance() const = 0;
    virtual double getBearingGatingDistance() const = 0;
    virtual double getBearingRelaxedFallbackDistance() const = 0;
    virtual int getMinConfirmationObservations() const = 0;
    virtual double getMaxTentativeCovarianceTrace() const = 0;
    virtual double getUnderConstrainedMaxCovarianceTrace() const = 0;
    virtual std::size_t getMinTriangulationObservations() const = 0;
    virtual double getMinTriangulationParallaxRadians() const = 0;
    virtual double getMinTriangulationBaselineMeters() const = 0;
    virtual double getMaxTriangulationMeanRayResidual() const = 0;

protected:
    // Shared state intended for child-class specialization logic.
    std::function<void(AssignedMeasurements)> _callback;
    std::mutex _mutex;
    Landmarks _landmarks;
    std::unordered_map<int, TentativeLandmark> _tentativeLandmarks;
    int _numberLandmarks = 0;
    int _nextTentativeId = 0;
    std::size_t _frameCounter = 0;
    Pose _robotPose;
    LoggerPtr _logger;
    double _quaternionRate = 0.0;
    UnderConstrainedInitializationStrategyPtr _underConstrainedInitializationStrategy;
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
