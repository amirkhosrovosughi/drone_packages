#ifndef SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_
#define SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_

#include "base_association.hpp"
#include "motion/motion_model.hpp"
#include "measurement/measurement_model.hpp"
#include "measurement/measurement.hpp"
#include "association/under_constrained_initialization_strategy.hpp"
#include <cmath>
#include <mutex>
#include <unordered_map>
#include "common/slam_logger.hpp"

#ifdef STORE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif

/**
 * @brief Tentative landmark candidate tracked before EKF insertion.
 *
 * Stores running statistics and lifecycle counters used to decide
 * whether a candidate should be confirmed or rejected.
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

        TentativeLandmark()
            : candidateId(0), position(Position()), variance(Variance2D()), isUnderConstrained(false), consistentObservations(0),
                    missedFrames(0), seenInCurrentFrame(false), sampleCount(0), meanX(0.0), meanY(0.0),
                    meanZ(0.0), m2X(0.0), m2Y(0.0) {}
};

using UnderConstrainedInitializationStrategyPtr =
    std::shared_ptr<UnderConstrainedInitializationStrategy>;

/**
 * @brief Nearest Neighbor Association implementation.
 *
 * Compares incoming measurements to stored landmarks and assigns each measurement
 * to the nearest landmark (if within a threshold) or creates a new landmark.
 */
class NearestNeighborAssociation : public BaseAssociation {
public:
    /**
     * @brief Construct a new NearestNeighborAssociation object.
     */
    NearestNeighborAssociation();

    /**
     * @brief Receive new raw measurements for association.
     *
     * This function typically schedules processing (e.g. on a background thread)
     * and returns quickly.
     *
     * @param meas Vector of incoming measurements.
     */
    void onReceiveMeasurement(const Measurements& meas) override;

    /**
     * @brief Update internal map summary (landmarks + robot pose).
     *
     * Synchronizes internal landmark list and robot pose with the SLAM map.
     *
     * @param map Current map summary.
     */
    void handleUpdate(const MapSummary& map) override;

    /**
     * @brief Register callback to receive associated measurements.
     *
     * @param callback Function to call after association completes.
     */
    void registerCallback(std::function<void(AssignedMeasurements)> callback) override
    {
        _callback = callback;
    }

    /**
     * @brief Set logger instance used by this component.
     *
     * @param logger Shared logger pointer.
     */
    void setLogger(LoggerPtr logger) override
    {
        _logger = logger;
    }

    /**
     * @brief Set strategy used when a measurement model inverse is under-constrained.
     *
     * @param strategy Strategy instance. If nullptr, a no-op strategy is restored.
     */
    void setUnderConstrainedInitializationStrategy(
        UnderConstrainedInitializationStrategyPtr strategy)
    {
        if (strategy)
        {
            _underConstrainedInitializationStrategy = std::move(strategy);
        }
        else
        {
            _underConstrainedInitializationStrategy = std::make_shared<NoOpUnderConstrainedInitializationStrategy>();
        }
    }

private:
    /**
     * @brief Core measurement-processing routine (does the actual association).
     *
     * @param meas Measurements to process/associate.
     */
    void processMeasurement(const Measurements& meas) override;

    /**
     * @brief Compute Euclidean distance between two landmarks (positions).
     *
     * @param meas Landmark derived from measurement (candidate).
     * @param feature Existing map feature to compare against.
     * @return Euclidean distance.
     */

protected:
    double euclideanDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Compute Mahalanobis distance between two landmarks (placeholder).
     *
     * @param meas Landmark derived from measurement.
     * @param feature Existing map feature to compare against.
     * @return Mahalanobis distance.
     */
    double mahalanobisDistance(const Landmark& meas, Landmark feature);

    /**
     * @brief Convert a distance value into a matching score (0..1).
     *
     * @param distance Euclidean distance value.
     * @return Matching score in (0,1].
     */
    double matchingScore(double distance);

private:
    /**
     * @brief Initialize a landmark position from measurement.
     *
     * Uses model inverse when possible, otherwise falls back to the configured
     * under-constrained initialization strategy.
     *
     * @param measurement Input measurement.
     * @param landmark Output initialized landmark (position only).
     * @param isUnderConstrainedInitialization True when strategy fallback is used.
     * @return True if initialization succeeded.
     */
    bool tryInitializeLandmarkFromMeasurement(
        const Measurement& measurement,
        Landmark& landmark,
        bool& isUnderConstrainedInitialization) const;

    /**
     * @brief Append a newly initialized/confirmed landmark assignment.
     */
    void appendNewLandmarkAssignment(
        const Measurement& measurement,
        const Landmark& landmark,
        AssignedMeasurements& assignedMeasurements) const;

    /**
     * @brief Add fully constrained landmark directly.
     */
    void addDirectLandmark(
        const Measurement& measurement,
        Landmark& landmark,
        AssignedMeasurements& assignedMeasurements,
        int* landmarkId = nullptr);

    /**
     * @brief Track tentative under-constrained landmark and confirm when ready.
     */
    void trackOrConfirmTentativeLandmark(
        const Measurement& measurement,
        const Landmark& landmark,
        AssignedMeasurements& assignedMeasurements,
        int* landmarkId = nullptr);

    /**
        * @brief Mark all tentative candidates as not observed for the current frame.
        *
        * This prepares lifecycle bookkeeping before processing incoming measurements.
        */
        void markTentativeCandidatesUnseenForCurrentFrame();

        /**
     * @brief Update tentative landmark running statistics from a new observation.
     *
     * @param candidate Tentative candidate to update.
     * @param measurementPosition Position inferred from current measurement.
     */
    void updateTentativeLandmark(TentativeLandmark& candidate, const Position& measurementPosition);

    /**
     * @brief Check whether a tentative candidate is ready for confirmation.
     *
     * @param candidate Tentative candidate to evaluate.
     * @return True when confirmation criteria are satisfied.
     */
    bool shouldConfirmTentativeLandmark(const TentativeLandmark& candidate) const;

    /**
     * @brief Remove stale tentative candidates that were not re-observed.
     */
    void pruneTentativeLandmarks();

    /**
     * @brief Find nearest tentative candidate for a measurement-derived landmark.
     *
     * @param measurementLandmark Landmark estimated from current measurement.
     * @param isUnderConstrainedInitialization Match candidates of same measurement type.
     * @return Candidate id if within gating distance, otherwise -1.
     */
    int findNearestTentativeCandidate(
        const Landmark& measurementLandmark,
        bool isUnderConstrainedInitialization) const;

    std::function<void(AssignedMeasurements)> _callback;            ///< Callback invoked with associated measurements
    std::mutex _mutex;                                              ///< Mutex to protect internal state
    Landmarks _landmarks;                                           ///< Current list of landmarks
    std::unordered_map<int, TentativeLandmark> _tentativeLandmarks; ///< Tentative landmarks pending confirmation
    int _numberLandmarks = 0;                                       ///< Counter used to assign new landmark ids
    int _nextTentativeId = 0;                                       ///< Counter used to assign tentative candidate ids
    std::size_t _frameCounter = 0;                                  ///< Internal frame counter for lifecycle updates
    Pose _robotPose;                                                ///< Latest robot pose
    LoggerPtr _logger;                                              ///< Logger instance
    double _quaternionRate = 0.0;                                   ///< Rate of quaternion change (used for skipping)
    UnderConstrainedInitializationStrategyPtr _underConstrainedInitializationStrategy; ///< Under-constrained initialization strategy instance
};

#endif  // SLAM__NEAREST_NEIGHBOR_ASSOCIATION_HPP_