#ifndef SLAM__UNDER_CONSTRAINED_INITIALIZATION_STRATEGY_HPP_
#define SLAM__UNDER_CONSTRAINED_INITIALIZATION_STRATEGY_HPP_

#include <optional>

#include "common/def_slam.hpp"
#include "measurement/measurement.hpp"

/**
 * @brief Strategy interface for initializing landmarks from under-constrained measurements.
 *
 * This seam allows association to remain backend-agnostic while supporting
 * different initialization behaviors (e.g., EKF triangulation now, graph factors later).
 */
class UnderConstrainedInitializationStrategy {
public:
    virtual ~UnderConstrainedInitializationStrategy() = default;

    /**
     * @brief Attempt to produce a world-frame landmark hypothesis.
     *
     * @param measurement Raw measurement that could not be inverted directly.
     * @param robotPose Current robot pose.
     * @return Landmark position hypothesis when available; std::nullopt otherwise.
     */
    virtual std::optional<Position> initialize(
        const Measurement& measurement,
        const Pose& robotPose) const = 0;
};

/**
 * @brief Default no-op strategy.
 *
 * Preserves existing behavior: under-constrained measurements are ignored.
 */
class NoOpUnderConstrainedInitializationStrategy : public UnderConstrainedInitializationStrategy {
public:
    std::optional<Position> initialize(
        const Measurement&,
        const Pose&) const override
    {
        return std::nullopt;
    }
};

#endif  // SLAM__UNDER_CONSTRAINED_INITIALIZATION_STRATEGY_HPP_
