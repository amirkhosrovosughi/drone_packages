#include <gtest/gtest.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include "startup/slam_startup_gate.hpp"
#include "startup/slam_startup_contract.hpp"
#include "startup/gps_startup_session.hpp"
#include "startup/gps_startup_initializer.hpp"

namespace
{

// ---- constants --------------------------------------------------------------

static constexpr int kMinSamplesForReady = 2;
static constexpr double kPolicyMaxEph = 2.0;
static constexpr double kPolicyMaxEpv = 2.0;
static constexpr int kPolicyRequiredFixType = 3;
static constexpr double kPolicyMaxSpeedMps = 0.5;
static constexpr double kPolicyMaxDispersionM = 100.0;
static constexpr double kPolicyWindowSec = 60.0;

static constexpr double kGoodEph = 1.0;
static constexpr double kGoodEpv = 1.0;
static constexpr int kGoodFixType = 3;
static constexpr int kBadFixType = kGoodFixType - 1;
static constexpr double kZeroSpeed = 0.0;

static constexpr double kTimeoutSec = 5.0;
static constexpr double kBeforeTimeoutSec = 2.0;
static constexpr double kAfterTimeoutSec = 6.0;

static constexpr double kSampleT0 = 1.0;
static constexpr double kSampleT1 = 1.1;
static constexpr double kSampleLat = 48.1372;
static constexpr double kSampleLon = 11.5755;
static constexpr double kSampleAlt = 520.0;

// ---- helpers ----------------------------------------------------------------

rclcpp::Time makeTime(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

px4_msgs::msg::SensorGps makeGoodSample()
{
  px4_msgs::msg::SensorGps msg;
  msg.fix_type = kGoodFixType;
  msg.eph = kGoodEph;
  msg.epv = kGoodEpv;
  msg.vel_m_s = kZeroSpeed;
  msg.latitude_deg = kSampleLat;
  msg.longitude_deg = kSampleLon;
  msg.altitude_msl_m = kSampleAlt;
  return msg;
}

// Session that becomes Ready after kMinSamplesForReady accepted samples.
std::unique_ptr<GpsStartupSession> makeSession()
{
  GpsInitializationPolicy policy;
  policy.minSamples = kMinSamplesForReady;
  policy.maxWindowSec = kPolicyWindowSec;
  policy.requiredFixType = kPolicyRequiredFixType;
  policy.maxEph = kPolicyMaxEph;
  policy.maxEpv = kPolicyMaxEpv;
  policy.stationaryMaxSpeedMps = kPolicyMaxSpeedMps;
  policy.stationaryMaxDispersionM = kPolicyMaxDispersionM;
  policy.useAverage = true;
  return std::make_unique<GpsStartupSession>(
    std::make_unique<GpsStartupInitializer>(policy));
}

SlamStartupContractConfig makeGpsConfig(
  bool dropInput = true,
  bool allowDegraded = false,
  double timeoutSec = kTimeoutSec)
{
  SlamStartupContractConfig config;
  config.gpsRequiredInit = true;
  config.dropInputWhileWaitingGpsInit = dropInput;
  config.allowDegradedNoGps = allowDegraded;
  config.gpsInitTimeoutSec = timeoutSec;
  config.gpsStartupSession = makeSession();
  return config;
}

SlamStartupContractConfig makeNoGpsConfig()
{
  SlamStartupContractConfig config;
  config.gpsRequiredInit = false;
  return config;
}

// Drives the gate to Running by feeding two good samples.
void satisfyGpsInit(SlamStartupGate& gate)
{
  gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));
  gate.onGpsSample(makeGoodSample(), makeTime(kSampleT1));
}

// ---- tests: GPS disabled ----------------------------------------------------

TEST(SlamStartupGateTest, GpsDisabled_Configure_NoTransition)
{
  SlamStartupGate gate;
  const auto transition = gate.configure(makeNoGpsConfig(), makeTime(0.0));

  // Gate starts in Running, GPS disabled keeps it in Running — no change.
  EXPECT_FALSE(transition.changed);
}

TEST(SlamStartupGateTest, GpsDisabled_AllowsProcessingImmediately)
{
  SlamStartupGate gate;
  gate.configure(makeNoGpsConfig(), makeTime(0.0));

  EXPECT_TRUE(gate.canProcessSlamInput());
  EXPECT_FALSE(gate.shouldDropSlamInput());
}

TEST(SlamStartupGateTest, GpsDisabled_DoesNotRequireGpsSubscription)
{
  SlamStartupGate gate;
  gate.configure(makeNoGpsConfig(), makeTime(0.0));

  EXPECT_FALSE(gate.requiresGpsSubscription());
}

TEST(SlamStartupGateTest, GpsDisabled_GpsSampleIsIgnored)
{
  SlamStartupGate gate;
  gate.configure(makeNoGpsConfig(), makeTime(0.0));

  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Ignored);
  EXPECT_FALSE(result.transition.changed);
}

TEST(SlamStartupGateTest, GpsDisabled_WatchdogPastTimeout_DoesNothing)
{
  SlamStartupGate gate;
  gate.configure(makeNoGpsConfig(), makeTime(0.0));

  const auto result = gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  EXPECT_FALSE(result.shouldWarnBlocked);
  EXPECT_FALSE(result.transition.changed);
}

// ---- tests: GPS required init — state after configure -----------------------

TEST(SlamStartupGateTest, GpsRequired_Configure_TransitionsToWaitGpsInit)
{
  SlamStartupGate gate;
  const auto transition = gate.configure(makeGpsConfig(), makeTime(0.0));

  EXPECT_TRUE(transition.changed);
  EXPECT_EQ(transition.from, SlamStartupGate::RuntimeState::Running);
  EXPECT_EQ(transition.to, SlamStartupGate::RuntimeState::WaitGpsInit);
}

TEST(SlamStartupGateTest, GpsRequired_RequiresGpsSubscription)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  EXPECT_TRUE(gate.requiresGpsSubscription());
}

// ---- tests: drop-input policy -----------------------------------------------

TEST(SlamStartupGateTest, GpsRequired_DropInput_BlocksProcessingWhileWaiting)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(/*dropInput=*/true), makeTime(0.0));

  EXPECT_FALSE(gate.canProcessSlamInput());
  EXPECT_TRUE(gate.shouldDropSlamInput());
}

TEST(SlamStartupGateTest, GpsRequired_NoDropInput_AllowsProcessingWhileWaiting)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(/*dropInput=*/false), makeTime(0.0));

  EXPECT_TRUE(gate.canProcessSlamInput());
  EXPECT_FALSE(gate.shouldDropSlamInput());
}

// ---- tests: initializer/session metrics and reasons ------------------------

TEST(SlamStartupGateTest, Initializer_Pending_ReturnsMetricsAndPendingReason)
{
  GpsInitializationPolicy policy;
  policy.minSamples = 3;
  policy.maxWindowSec = kPolicyWindowSec;
  policy.requiredFixType = kPolicyRequiredFixType;
  policy.maxEph = kPolicyMaxEph;
  policy.maxEpv = kPolicyMaxEpv;
  policy.stationaryMaxSpeedMps = kPolicyMaxSpeedMps;
  policy.stationaryMaxDispersionM = kPolicyMaxDispersionM;
  policy.useAverage = true;

  GpsStartupInitializer initializer(policy);
  const GpsInitializationOutcome outcome =
    initializer.ingest(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_FALSE(outcome.ready);
  EXPECT_FALSE(outcome.rejected);
  EXPECT_EQ(outcome.acceptedSampleCount, 1u);
  EXPECT_EQ(outcome.reason, "GPS init collecting accepted samples.");
  EXPECT_EQ(outcome.metrics.acceptedSampleCount, 1u);
  EXPECT_EQ(outcome.metrics.minSamplesRequired, 3u);
  EXPECT_DOUBLE_EQ(outcome.metrics.ephThreshold, kPolicyMaxEph);
  EXPECT_DOUBLE_EQ(outcome.metrics.epvThreshold, kPolicyMaxEpv);
  EXPECT_DOUBLE_EQ(outcome.metrics.speedThresholdMps, kPolicyMaxSpeedMps);
}

TEST(SlamStartupGateTest, Initializer_RejectFixType_ReasonContainsActualAndThreshold)
{
  GpsInitializationPolicy policy;
  policy.minSamples = kMinSamplesForReady;
  policy.maxWindowSec = kPolicyWindowSec;
  policy.requiredFixType = kPolicyRequiredFixType;
  policy.maxEph = kPolicyMaxEph;
  policy.maxEpv = kPolicyMaxEpv;
  policy.stationaryMaxSpeedMps = kPolicyMaxSpeedMps;
  policy.stationaryMaxDispersionM = kPolicyMaxDispersionM;
  policy.useAverage = true;

  GpsStartupInitializer initializer(policy);
  px4_msgs::msg::SensorGps badSample = makeGoodSample();
  badSample.fix_type = kBadFixType;

  const GpsInitializationOutcome outcome =
    initializer.ingest(badSample, makeTime(kSampleT0));

  EXPECT_TRUE(outcome.rejected);
  EXPECT_NE(outcome.reason.find("fix type below required threshold"), std::string::npos);
  EXPECT_NE(outcome.reason.find("2 < 3"), std::string::npos);
}

TEST(SlamStartupGateTest, Session_PropagatesPendingReasonAndMetrics)
{
  GpsInitializationPolicy policy;
  policy.minSamples = 3;
  policy.maxWindowSec = kPolicyWindowSec;
  policy.requiredFixType = kPolicyRequiredFixType;
  policy.maxEph = kPolicyMaxEph;
  policy.maxEpv = kPolicyMaxEpv;
  policy.stationaryMaxSpeedMps = kPolicyMaxSpeedMps;
  policy.stationaryMaxDispersionM = kPolicyMaxDispersionM;
  policy.useAverage = true;

  GpsStartupSession session(
    std::make_unique<GpsStartupInitializer>(policy));

  const GpsStartupSession::Result result =
    session.ingestSample(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_EQ(result.status, GpsStartupSession::ResultStatus::Pending);
  EXPECT_EQ(result.reason, "GPS init collecting accepted samples.");
  ASSERT_TRUE(result.metrics.has_value());
  EXPECT_EQ(result.metrics->acceptedSampleCount, 1u);
  EXPECT_EQ(result.metrics->minSamplesRequired, 3u);
}

TEST(SlamStartupGateTest, Session_Ready_PropagatesComputedMetrics)
{
  GpsInitializationPolicy policy;
  policy.minSamples = 2;
  policy.maxWindowSec = kPolicyWindowSec;
  policy.requiredFixType = kPolicyRequiredFixType;
  policy.maxEph = kPolicyMaxEph;
  policy.maxEpv = kPolicyMaxEpv;
  policy.stationaryMaxSpeedMps = kPolicyMaxSpeedMps;
  policy.stationaryMaxDispersionM = kPolicyMaxDispersionM;
  policy.useAverage = true;

  GpsStartupSession session(
    std::make_unique<GpsStartupInitializer>(policy));

  (void)session.ingestSample(makeGoodSample(), makeTime(kSampleT0));
  const GpsStartupSession::Result ready =
    session.ingestSample(makeGoodSample(), makeTime(kSampleT1));

  EXPECT_EQ(ready.status, GpsStartupSession::ResultStatus::Ready);
  ASSERT_TRUE(ready.metrics.has_value());
  EXPECT_EQ(ready.metrics->acceptedSampleCount, 2u);
  EXPECT_GE(ready.metrics->windowDurationSec, 0.0);
  EXPECT_DOUBLE_EQ(ready.metrics->meanEph, kGoodEph);
  EXPECT_DOUBLE_EQ(ready.metrics->meanEpv, kGoodEpv);
  EXPECT_DOUBLE_EQ(ready.metrics->maxSpeedMps, kZeroSpeed);
}

// ---- tests: GPS sample ingestion --------------------------------------------

TEST(SlamStartupGateTest, GpsRequired_FirstSampleBeforeMinCount_IsPending)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Pending);
  EXPECT_EQ(result.acceptedSampleCount, 1u);
  EXPECT_FALSE(result.transition.changed);
}

TEST(SlamStartupGateTest, GpsRequired_SampleAtMinCount_BecomesReady)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));
  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT1));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Ready);
  EXPECT_TRUE(result.reference.has_value());
  EXPECT_DOUBLE_EQ(result.reference->latitudeDeg, kSampleLat);
  EXPECT_DOUBLE_EQ(result.reference->longitudeDeg, kSampleLon);
  EXPECT_DOUBLE_EQ(result.reference->altitudeM, kSampleAlt);
}

TEST(SlamStartupGateTest, GpsRequired_ReadySample_TransitionsToRunning)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));
  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT1));

  EXPECT_TRUE(result.transition.changed);
  EXPECT_EQ(result.transition.from, SlamStartupGate::RuntimeState::WaitGpsInit);
  EXPECT_EQ(result.transition.to, SlamStartupGate::RuntimeState::Running);
}

TEST(SlamStartupGateTest, GpsRequired_AfterReady_GateAllowsProcessing)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));
  satisfyGpsInit(gate);

  EXPECT_TRUE(gate.canProcessSlamInput());
  EXPECT_FALSE(gate.shouldDropSlamInput());
}

TEST(SlamStartupGateTest, GpsRequired_AfterReady_FurtherSamplesAreIgnored)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));
  satisfyGpsInit(gate);

  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(2.0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Ignored);
}

TEST(SlamStartupGateTest, GpsRequired_SampleWithBadFixType_IsRejected)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  px4_msgs::msg::SensorGps badSample = makeGoodSample();
  badSample.fix_type = kBadFixType;

  const auto result = gate.onGpsSample(badSample, makeTime(kSampleT0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Rejected);
  EXPECT_FALSE(result.reason.empty());
}

TEST(SlamStartupGateTest, GpsRequired_NullSession_SampleIsRejected)
{
  SlamStartupGate gate;
  SlamStartupContractConfig config;
  config.gpsRequiredInit = true;
  config.dropInputWhileWaitingGpsInit = true;
  config.gpsInitTimeoutSec = kTimeoutSec;
  config.gpsStartupSession = nullptr;
  gate.configure(std::move(config), makeTime(0.0));

  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Rejected);
  EXPECT_FALSE(result.reason.empty());
}

// ---- tests: watchdog --------------------------------------------------------

TEST(SlamStartupGateTest, Watchdog_BeforeTimeout_NoTransition)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  const auto result = gate.onWatchdogTick(makeTime(kBeforeTimeoutSec));

  EXPECT_FALSE(result.shouldWarnBlocked);
  EXPECT_FALSE(result.transition.changed);
  EXPECT_LT(result.elapsedSec, kTimeoutSec);
}

TEST(SlamStartupGateTest, Watchdog_ExactlyAtTimeout_WarnsWhenDegradedModeIsDisabled)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));

  // elapsed == timeout reaches the timeout boundary and triggers watchdog handling.
  const auto result = gate.onWatchdogTick(makeTime(kTimeoutSec));

  EXPECT_FALSE(result.transition.changed);
  EXPECT_TRUE(result.shouldWarnBlocked);
}

TEST(SlamStartupGateTest, Watchdog_PastTimeout_AllowDegraded_TransitionsToDegraded)
{
  SlamStartupGate gate;
  gate.configure(
    makeGpsConfig(/*dropInput=*/true, /*allowDegraded=*/true),
    makeTime(0.0));

  const auto result = gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  EXPECT_FALSE(result.shouldWarnBlocked);
  EXPECT_TRUE(result.transition.changed);
  EXPECT_EQ(result.transition.from, SlamStartupGate::RuntimeState::WaitGpsInit);
  EXPECT_EQ(result.transition.to, SlamStartupGate::RuntimeState::DegradedNoGps);
}

TEST(SlamStartupGateTest, Watchdog_PastTimeout_NoAllowDegraded_WarnsAndKeepsWaiting)
{
  SlamStartupGate gate;
  gate.configure(
    makeGpsConfig(/*dropInput=*/true, /*allowDegraded=*/false),
    makeTime(0.0));

  const auto result = gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  EXPECT_TRUE(result.shouldWarnBlocked);
  EXPECT_FALSE(result.transition.changed);
}

TEST(SlamStartupGateTest, Watchdog_AfterRunning_DoesNothing)
{
  SlamStartupGate gate;
  gate.configure(makeGpsConfig(), makeTime(0.0));
  satisfyGpsInit(gate);

  // Watchdog fires past "timeout" but gate is already Running.
  const auto result = gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  EXPECT_FALSE(result.shouldWarnBlocked);
  EXPECT_FALSE(result.transition.changed);
}

// ---- tests: DegradedNoGps state ---------------------------------------------

TEST(SlamStartupGateTest, DegradedNoGps_AllowsProcessing)
{
  SlamStartupGate gate;
  gate.configure(
    makeGpsConfig(/*dropInput=*/true, /*allowDegraded=*/true),
    makeTime(0.0));
  gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  EXPECT_TRUE(gate.canProcessSlamInput());
  EXPECT_FALSE(gate.shouldDropSlamInput());
}

TEST(SlamStartupGateTest, DegradedNoGps_FurtherGpsSamplesAreIgnored)
{
  SlamStartupGate gate;
  gate.configure(
    makeGpsConfig(/*dropInput=*/true, /*allowDegraded=*/true),
    makeTime(0.0));
  gate.onWatchdogTick(makeTime(kAfterTimeoutSec));

  const auto result = gate.onGpsSample(makeGoodSample(), makeTime(kSampleT0));

  EXPECT_EQ(result.status, SlamStartupGate::GpsSampleResult::Status::Ignored);
}

// ---- tests: runtimeStateToString --------------------------------------------

TEST(SlamStartupGateTest, RuntimeStateToString_WaitGpsInit)
{
  EXPECT_STREQ(
    SlamStartupGate::runtimeStateToString(SlamStartupGate::RuntimeState::WaitGpsInit),
    "WAIT_GPS_INIT");
}

TEST(SlamStartupGateTest, RuntimeStateToString_Running)
{
  EXPECT_STREQ(
    SlamStartupGate::runtimeStateToString(SlamStartupGate::RuntimeState::Running),
    "RUNNING");
}

TEST(SlamStartupGateTest, RuntimeStateToString_DegradedNoGps)
{
  EXPECT_STREQ(
    SlamStartupGate::runtimeStateToString(SlamStartupGate::RuntimeState::DegradedNoGps),
    "DEGRADED_NO_GPS");
}

}  // namespace
