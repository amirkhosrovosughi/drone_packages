#ifndef SLAM__ASSOCIATION__JOINT_COMPATIBILITY_ASSOCIATION_HPP_
#define SLAM__ASSOCIATION__JOINT_COMPATIBILITY_ASSOCIATION_HPP_

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
     */
    void onReceiveMeasurement(const Measurements& meas) override;

    /**
     * @brief Handle map updates from the SLAM manager.
     */
    void handleUpdate(const MapSummary& map) override;

    /**
     * @brief Register a callback invoked when measurements are associated.
     */
    void registerCallback(std::function<void(AssignedMeasurements)> callback) override
    {
        _callback = callback;
    }

    /**
     * @brief Set the logger instance used by this component.
     */
    void setLogger(LoggerPtr logger) override {}

private:
    /**
     * @brief Internal processing of measurements (implementation detail).
     */
    void processMeasurement(const Measurements& meas) override;

private:
    std::function<void(AssignedMeasurements)> _callback;  ///< Callback invoked with associated measurements
};

#endif  // SLAM__ASSOCIATION__JOINT_COMPATIBILITY_ASSOCIATION_HPP_