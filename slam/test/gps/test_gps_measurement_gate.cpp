#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "common/def_slam_core.hpp"
#include "gps/gps_measurement_gate.hpp"

namespace slam
{

namespace
{

// Default sentinel values that match the unset GpsConstraint defaults.
constexpr float kDefaultEph = 999.0f;
constexpr float kDefaultEpv = 999.0f;

// A GPS constraint with quality metadata filled to pass all default gates.
GpsConstraint makeGoodConstraint(const Eigen::Vector3d& pos = Eigen::Vector3d::Zero())
{
  GpsConstraint g;
  g.enuPosition = pos;
  g.sigmaXyM = 0.5;
  g.sigmaZM  = 1.0;
  g.fixType        = 3;
  g.eph            = 1.0f;
  g.epv            = 2.0f;
  g.satellitesUsed = 8;
  return g;
}

// A GPS constraint whose metadata fields are still at default sentinel values
// (as produced by code that does not populate quality fields yet).
GpsConstraint makeDefaultMetadataConstraint(const Eigen::Vector3d& pos = Eigen::Vector3d::Zero())
{
  GpsConstraint g;
  g.enuPosition = pos;
  g.sigmaXyM = 0.5;
  g.sigmaZM  = 1.0;
  // fixType, eph, epv, satellitesUsed, hasVelocity stay at zero/999 defaults.
  return g;
}

// Logger stub that records every log call so tests can assert on it.
class RecordingLogger : public SlamLogger
{
public:
  struct Entry
  {
    LogLevel level;
    std::string message;
  };

  const std::vector<Entry>& entries() const { return _entries; }

  bool hasWarn() const
  {
    for (const auto& e : _entries)
    {
      if (e.level == LogLevel::WARN) return true;
    }
    return false;
  }

  bool containsInfo(const std::string& needle) const
  {
    for (const auto& e : _entries)
    {
      if (e.level == LogLevel::INFO && e.message.find(needle) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  bool containsWarn(const std::string& needle) const
  {
    for (const auto& e : _entries)
    {
      if (e.level == LogLevel::WARN && e.message.find(needle) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

protected:
  void write(LogLevel level, const std::string& message) override
  {
    _entries.push_back({level, message});
  }

private:
  std::vector<Entry> _entries;
};

}  // namespace

// ---------------------------------------------------------------------------
// Default-metadata bypass tests
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, DefaultMetadataConstraintIsAlwaysAccepted)
{
  GpsMeasurementGate gate;

  const GpsConstraint g = makeDefaultMetadataConstraint();
  const Eigen::Vector3d predicted = Eigen::Vector3d::Zero();

  EXPECT_TRUE(gate.shouldAccept(g, predicted));
}

TEST(GpsMeasurementGateTest, DefaultMetadataBypassIgnoresInnovation)
{
  // Even a huge innovation must not block a constraint with default metadata.
  GpsMeasurementGate gate;

  const GpsConstraint g = makeDefaultMetadataConstraint(Eigen::Vector3d(1000.0, 0.0, 0.0));
  const Eigen::Vector3d predicted = Eigen::Vector3d::Zero();

  EXPECT_TRUE(gate.shouldAccept(g, predicted));
}

// ---------------------------------------------------------------------------
// Gate-disabled passthrough
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, DisabledGateAcceptsEverything)
{
  GpsRuntimeGateConfig cfg;
  cfg.enabled = false;
  GpsMeasurementGate gate(cfg);

  // Bad fix type, high eph/epv — would normally be rejected.
  GpsConstraint g = makeGoodConstraint();
  g.fixType = 0;
  g.eph     = 999.0f;
  g.epv     = 999.0f;
  g.satellitesUsed = 1;

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Quality-gate acceptance and rejection
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, GoodConstraintIsAccepted)
{
  GpsMeasurementGate gate;
  const GpsConstraint g = makeGoodConstraint();
  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, LowFixTypeIsRejected)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.fixType = 1;  // below minFixType=3

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, HighEphIsRejected)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.eph = 3.0f;  // above maxEphM=2.5

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, HighEpvIsRejected)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.epv = 5.0f;  // above maxEpvM=4.0

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, EphAtExactThresholdIsAccepted)
{
  // The gate uses strict >; a value equal to maxEphM is not rejected.
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.eph = 2.5f;  // exactly maxEphM=2.5 — passes because condition is eph > maxEphM

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, EphJustBelowThresholdIsAccepted)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.eph = 2.49f;  // just below maxEphM=2.5

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Innovation gate
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, InnovationBelowThresholdIsAccepted)
{
  GpsMeasurementGate gate;
  const GpsConstraint g = makeGoodConstraint(Eigen::Vector3d(5.0, 0.0, 0.0));
  const Eigen::Vector3d predicted(5.5, 0.0, 0.0);  // 0.5 m away, well below 12 m

  EXPECT_TRUE(gate.shouldAccept(g, predicted));
}

TEST(GpsMeasurementGateTest, InnovationAboveThresholdIsRejected)
{
  GpsMeasurementGate gate;
  const GpsConstraint g = makeGoodConstraint(Eigen::Vector3d(100.0, 0.0, 0.0));
  const Eigen::Vector3d predicted = Eigen::Vector3d::Zero();  // 100 m innovation

  EXPECT_FALSE(gate.shouldAccept(g, predicted));
}

TEST(GpsMeasurementGateTest, InnovationAtExactThresholdIsAccepted)
{
  // The gate uses strict >; exactly maxInnovationM=12.0 is not rejected.
  GpsMeasurementGate gate;
  const GpsConstraint g = makeGoodConstraint(Eigen::Vector3d(12.0, 0.0, 0.0));
  const Eigen::Vector3d predicted = Eigen::Vector3d::Zero();

  EXPECT_TRUE(gate.shouldAccept(g, predicted));
}

// ---------------------------------------------------------------------------
// Non-finite input rejection
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, NanPositionIsRejected)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.enuPosition = Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, InfPositionIsRejected)
{
  GpsMeasurementGate gate;
  GpsConstraint g = makeGoodConstraint();
  g.enuPosition = Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0);

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Reset clears bad-streak counter
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, ResetClearsBadStreak)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 2;
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  // Reject once to push bad streak up.
  GpsConstraint bad = makeGoodConstraint(Eigen::Vector3d(100.0, 0.0, 0.0));
  gate.shouldAccept(bad, Eigen::Vector3d::Zero());

  gate.reset();

  // After reset a good constraint should be accepted with zero bad streak
  // and the logger should record acceptance, not a streak warning.
  auto logger2 = std::make_shared<RecordingLogger>();
  gate.setLogger(logger2);
  const GpsConstraint good = makeGoodConstraint();
  EXPECT_TRUE(gate.shouldAccept(good, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(logger2->containsWarn("badStreak"));
}

// ---------------------------------------------------------------------------
// Logger integration
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, AcceptanceIsLoggedAtInfo)
{
  GpsMeasurementGate gate;
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  const GpsConstraint g = makeGoodConstraint();
  ASSERT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
  EXPECT_TRUE(logger->containsInfo("accepted"));
}

TEST(GpsMeasurementGateTest, RejectionNotLoggedBelowThrottle)
{
  GpsMeasurementGate gate;
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  // First rejection: count==1, which is not a multiple of 10 and not a streak threshold hit.
  gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  EXPECT_TRUE(logger->entries().empty());
}

TEST(GpsMeasurementGateTest, RejectionLoggedAtStreakWarnThreshold)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 3;
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  // The bad streak (inside GpsRuntimeGate) increments per rejection.
  // After badStreakWarnThreshold consecutive rejections shouldLogRejection fires.
  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  for (int i = 0; i < static_cast<int>(cfg.badStreakWarnThreshold); ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  EXPECT_TRUE(logger->containsWarn("rejected"));
}

TEST(GpsMeasurementGateTest, RejectionLoggedEveryTenthReject)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 1000;  // keep streak log from firing
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  // Reject 10 times; the 10th rejection must produce a log entry.
  for (int i = 0; i < 10; ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  EXPECT_FALSE(logger->entries().empty());
}

TEST(GpsMeasurementGateTest, NoLoggerDoesNotCrash)
{
  GpsMeasurementGate gate;  // no logger set

  const GpsConstraint good = makeGoodConstraint();
  EXPECT_NO_THROW(gate.shouldAccept(good, Eigen::Vector3d::Zero()));

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;
  EXPECT_NO_THROW(gate.shouldAccept(bad, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Speed gate (optional)
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, SpeedGateDisabledDoesNotRejectHighSpeed)
{
  GpsRuntimeGateConfig cfg;
  cfg.enableSpeedGate = false;
  GpsMeasurementGate gate(cfg);

  GpsConstraint g = makeGoodConstraint();
  g.hasVelocity = true;
  g.velMps = 100.0f;  // far above default 15 m/s limit

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, SpeedGateEnabledRejectsHighSpeed)
{
  GpsRuntimeGateConfig cfg;
  cfg.enableSpeedGate = true;
  cfg.maxSpeedMps = 15.0;
  GpsMeasurementGate gate(cfg);

  GpsConstraint g = makeGoodConstraint();
  g.hasVelocity = true;
  g.velMps = 20.0f;  // above maxSpeedMps

  EXPECT_FALSE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

TEST(GpsMeasurementGateTest, SpeedGateEnabledAcceptsBelowLimit)
{
  GpsRuntimeGateConfig cfg;
  cfg.enableSpeedGate = true;
  cfg.maxSpeedMps = 15.0;
  GpsMeasurementGate gate(cfg);

  GpsConstraint g = makeGoodConstraint();
  g.hasVelocity = true;
  g.velMps = 10.0f;  // below maxSpeedMps

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Custom config
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, CustomConfigIsRespected)
{
  GpsRuntimeGateConfig cfg;
  cfg.minFixType = 2;
  cfg.maxEphM = 5.0;
  cfg.maxEpvM = 8.0;
  cfg.maxInnovationM = 50.0;
  GpsMeasurementGate gate(cfg);

  // fixType=2 would normally be rejected by default config but passes with minFixType=2.
  GpsConstraint g = makeGoodConstraint();
  g.fixType = 2;
  g.eph = 4.0f;
  g.epv = 7.0f;

  EXPECT_TRUE(gate.shouldAccept(g, Eigen::Vector3d::Zero()));
}

// ---------------------------------------------------------------------------
// Step 4: degraded-mode state transitions and health counters
// ---------------------------------------------------------------------------

TEST(GpsMeasurementGateTest, EnteringDegradedModeLogsWarnOnce)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 3;
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  // Reject badStreakWarnThreshold times to cross the threshold.
  for (std::size_t i = 0; i < cfg.badStreakWarnThreshold; ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  // Exactly one WARN entry for "entering degraded mode".
  int warnCount = 0;
  for (const auto& e : logger->entries())
  {
    if (e.level == LogLevel::WARN && e.message.find("degraded") != std::string::npos)
    {
      ++warnCount;
    }
  }
  EXPECT_EQ(warnCount, 1);
}

TEST(GpsMeasurementGateTest, DegradedModeWarnNotRepeatedAfterThreshold)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 2;
  cfg.badStreakWarnThreshold = 2;
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  // Reject well past the threshold.
  for (int i = 0; i < 10; ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  // Still only one "degraded" WARN.
  int warnCount = 0;
  for (const auto& e : logger->entries())
  {
    if (e.level == LogLevel::WARN && e.message.find("degraded") != std::string::npos)
    {
      ++warnCount;
    }
  }
  EXPECT_EQ(warnCount, 1);
}

TEST(GpsMeasurementGateTest, RecoveryFromDegradedModeIsLoggedOnce)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 2;
  GpsMeasurementGate gate(cfg);
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  // Enter degraded mode.
  for (std::size_t i = 0; i < cfg.badStreakWarnThreshold; ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  // Recover with a good sample.
  const GpsConstraint good = makeGoodConstraint();
  EXPECT_TRUE(gate.shouldAccept(good, Eigen::Vector3d::Zero()));

  // Exactly one INFO entry mentioning "recovered".
  int recoverCount = 0;
  for (const auto& e : logger->entries())
  {
    if (e.level == LogLevel::INFO && e.message.find("recovered") != std::string::npos)
    {
      ++recoverCount;
    }
  }
  EXPECT_EQ(recoverCount, 1);
}

TEST(GpsMeasurementGateTest, RecoveryNotLoggedWhenNoDegradedMode)
{
  // Accepting a good sample without prior streak must not log "recovered".
  GpsMeasurementGate gate;
  auto logger = std::make_shared<RecordingLogger>();
  gate.setLogger(logger);

  const GpsConstraint good = makeGoodConstraint();
  gate.shouldAccept(good, Eigen::Vector3d::Zero());

  for (const auto& e : logger->entries())
  {
    EXPECT_EQ(e.message.find("recovered"), std::string::npos);
  }
}

TEST(GpsMeasurementGateTest, HealthCountersAccurate)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 10;  // prevent degraded log noise
  GpsMeasurementGate gate(cfg);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;
  const GpsConstraint good = makeGoodConstraint();
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  gate.shouldAccept(bad, origin);
  gate.shouldAccept(bad, origin);
  gate.shouldAccept(good, origin);
  gate.shouldAccept(bad, origin);

  const GpsMeasurementGateHealth h = gate.health();
  EXPECT_EQ(h.acceptedCount, 1u);
  EXPECT_EQ(h.rejectedCount, 3u);
  EXPECT_EQ(h.currentBadStreak, 1u);
  EXPECT_EQ(h.inDegradedMode, false);
}

TEST(GpsMeasurementGateTest, MaxBadStreakTracked)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 100;  // suppress transition log
  GpsMeasurementGate gate(cfg);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;
  const GpsConstraint good = makeGoodConstraint();
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  // Streak of 4, then recover, then streak of 2.
  for (int i = 0; i < 4; ++i) gate.shouldAccept(bad, origin);
  gate.shouldAccept(good, origin);
  for (int i = 0; i < 2; ++i) gate.shouldAccept(bad, origin);

  EXPECT_EQ(gate.health().maxBadStreak, 4u);
}

TEST(GpsMeasurementGateTest, HealthResetOnReset)
{
  GpsMeasurementGate gate;

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;
  gate.shouldAccept(bad, Eigen::Vector3d::Zero());

  gate.reset();

  const GpsMeasurementGateHealth h = gate.health();
  EXPECT_EQ(h.acceptedCount, 0u);
  EXPECT_EQ(h.rejectedCount, 0u);
  EXPECT_EQ(h.currentBadStreak, 0u);
  EXPECT_EQ(h.maxBadStreak, 0u);
  EXPECT_EQ(h.inDegradedMode, false);
}

TEST(GpsMeasurementGateTest, DegradedModeHealthFlagSet)
{
  GpsRuntimeGateConfig cfg;
  cfg.badStreakWarnThreshold = 2;
  GpsMeasurementGate gate(cfg);

  GpsConstraint bad = makeGoodConstraint();
  bad.fixType = 0;

  for (std::size_t i = 0; i < cfg.badStreakWarnThreshold; ++i)
  {
    gate.shouldAccept(bad, Eigen::Vector3d::Zero());
  }

  EXPECT_TRUE(gate.health().inDegradedMode);

  // After acceptance the flag clears.
  EXPECT_TRUE(gate.shouldAccept(makeGoodConstraint(), Eigen::Vector3d::Zero()));
  EXPECT_FALSE(gate.health().inDegradedMode);
}

}  // namespace slam
