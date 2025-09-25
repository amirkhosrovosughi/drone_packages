#ifndef SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_
#define SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_

#include "base_association.hpp"

/**
 * @brief Joint Compatibility Association module.
 *
 * Implements a joint-compatibility-based data association algorithm.
 * This class receives measurements and attempts to associate them with
 * existing map landmarks using joint compatibility tests.
 */
class JointCompatibilityAssociation : public BaseAssociation {
public:
    /**
     * @brief Construct a new JointCompatibilityAssociation object.
     */
    JointCompatibilityAssociation();

    /**
     * @brief Handle incoming sensor measurements.
     *
     * This is the public entry point for delivering raw measurements to the
     * association module.
     *
     * @param meas Incoming measurements vector.
     */    
    void onReceiveMeasurement(const Measurements& meas) override;

    /**
     * @brief Handle map updates from the SLAM manager.
     *
     * Update internal landmark list and robot pose from the given map summary.
     *
     * @param map Current map summary.
     */
    void handleUpdate(const MapSummary& map) override;

    /**
     * @brief Register a callback invoked when measurements are associated.
     *
     * The provided callback will be called with associated measurements.
     *
     * @param callback Function to call with associated measurements.
     */
    void registerCallback(std::function<void(Measurements)> callback) override
    {
        _callback = callback;
    }

    /**
     * @brief Set the logger instance used by this component.
     *
     * @param logger Shared pointer to logger instance.
     */
    void setLogger(LoggerPtr logger) override {}

private:
    /**
     * @brief Internal processing of measurements (implementation detail).
     *
     * Subclasses implement the actual association algorithm here.
     *
     * @param meas Measurements to process.
     */
    void processMeasurement(const Measurements& meas) override;

private:
    std::function<void(Measurements)> _callback;  ///< Callback invoked with associated measurements
};

#endif  // SLAM__JOINT_COMPATIBILITY_ASSOCIATION_HPP_