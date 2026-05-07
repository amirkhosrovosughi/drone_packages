#ifndef SLAM__COMMON__DEF_SLAM_GPS_HPP_
#define SLAM__COMMON__DEF_SLAM_GPS_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>

// ---------------------------------------------------------------------------
// Geodetic / GPS types
// ---------------------------------------------------------------------------

struct GeodeticCoordinate
{
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeM = 0.0;
};

struct GpsReference
{
  double latitudeDeg = 0.0;
  double longitudeDeg = 0.0;
  double altitudeM = 0.0;
};

struct LocalFrameAnchor
{
  GpsReference anchorReference;
  std::uint64_t anchorTimestampUs = 0;
  Eigen::Vector3d initialEnuPosition = Eigen::Vector3d::Zero();
};

// ---------------------------------------------------------------------------
// GPS constraint (pipeline-facing, sensor-agnostic)
//
// enuPosition carries the absolute position in the local ENU frame established
// by LocalFrameAnchor.  sigmaXyM / sigmaZM are 1-sigma metres derived from
// PX4 EPH / EPV before the constraint reaches the pipeline.  The raw sensor
// fields (fixType, eph, epv, satellitesUsed, velMps) are kept for diagnostics
// and logging; the pipeline must not gate fusion on them — that decision
// belongs to GpsManager.
// ---------------------------------------------------------------------------
struct GpsConstraint
{
  Eigen::Vector3d enuPosition = Eigen::Vector3d::Zero();
  double sigmaXyM = 1.0;
  double sigmaZM  = 2.0;
  std::uint64_t timestampUs = 0;
  std::uint8_t  fixType = 0;
  float         eph = 999.f;
  float         epv = 999.f;
  std::uint8_t  satellitesUsed = 0;
  float         velMps = 0.f;
  bool          hasVelocity = false;
};

// ---------------------------------------------------------------------------
// Sensor-agnostic absolute-position constraint in local ENU frame.
//
// Carries only the fields the estimator needs: position and 1-sigma
// uncertainties.  Sensor-specific metadata (fix type, EPH/EPV, satellite
// count) stays in GpsConstraint and never crosses the estimator boundary.
//
// When graph-SLAM GPS factors are added this struct will gain
// a full 3x3 covariance matrix so both EKF and pose-graph can share one type.
// ---------------------------------------------------------------------------
struct AbsolutePositionConstraint
{
  Eigen::Vector3d enuPosition = Eigen::Vector3d::Zero();
  double sigmaXyM = 1.0;  ///< 1-sigma horizontal uncertainty [m]
  double sigmaZM  = 2.0;  ///< 1-sigma vertical uncertainty [m]
};

// ---------------------------------------------------------------------------
// Runtime GPS gating policy and decision types
// ---------------------------------------------------------------------------

enum class GpsRuntimeGateRejectReason
{
    None = 0,
    FixTypeTooLow,
    EphTooHigh,
    EpvTooHigh,
    InnovationTooHigh,
    SpeedTooHigh,
    NonFiniteInput
};

struct GpsRuntimeGateConfig
{
    bool enabled = true;

    // Sensor-quality gates
    std::uint8_t minFixType = 3;
    double maxEphM = 2.5;
    double maxEpvM = 4.0;

    // Innovation gate in local ENU frame
    double maxInnovationM = 12.0;

    // Optional speed gate to suppress samples during known bad dynamics
    bool enableSpeedGate = false;
    double maxSpeedMps = 15.0;

    // Streak policy (used by pipeline behavior in later steps)
    std::size_t badStreakWarnThreshold = 3;
};

struct GpsRuntimeGateDecision
{
    bool accepted = false;
    GpsRuntimeGateRejectReason reason = GpsRuntimeGateRejectReason::None;
    double innovationM = std::numeric_limits<double>::quiet_NaN();
    std::size_t badStreakCount = 0;
};

/// Snapshot of gate health for observability and health reporting.
struct GpsMeasurementGateHealth
{
    std::size_t acceptedCount = 0;    ///< Total accepted measurements since last reset
    std::size_t rejectedCount = 0;    ///< Total rejected measurements since last reset
    std::size_t currentBadStreak = 0; ///< Consecutive rejections right now
    std::size_t maxBadStreak = 0;     ///< Worst streak seen since last reset
    bool inDegradedMode = false;      ///< True when streak >= badStreakWarnThreshold
};

#endif  // SLAM__COMMON__DEF_SLAM_GPS_HPP_
