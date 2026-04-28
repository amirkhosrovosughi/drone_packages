#include "pipeline/frontend_health_monitor.hpp"

namespace slam
{

constexpr std::size_t kAssociationDropWarnMinSamples = 30;
constexpr double kAssociationDropWarnRate = 0.45;
constexpr std::size_t kLoopRejectRollingWindowSize = 8;
constexpr std::size_t kLoopRejectSpikeMinCandidates = 4;
constexpr double kLoopRejectSpikeAbsoluteRatio = 0.80;
constexpr double kLoopRejectSpikeRelativeJump = 0.35;
constexpr int kLoopClosureCooldownKeyframes = 5;

namespace
{

double averageRatio(const std::deque<double>& values)
{
  if (values.empty())
  {
    return 0.0;
  }
  double sum = 0.0;
  for (double v : values)
  {
    sum += v;
  }
  return sum / static_cast<double>(values.size());
}

}  // namespace

FrontendHealthMonitor::FrontendHealthMonitor() = default;

void FrontendHealthMonitor::setLogger(LoggerPtr logger)
{
  _logger = logger;
}

void FrontendHealthMonitor::recordMeasurementBatch(std::size_t attempted, std::size_t accepted)
{
  const std::size_t dropped = attempted > accepted ? attempted - accepted : 0;
  FrontendHealthMetrics localMetrics;
  bool shouldWarnDropRate = false;

  {
    std::lock_guard<std::mutex> lock(_mutex);

    _metrics.totalMeasurementAttempts += attempted;
    _metrics.totalAssociatedMeasurements += accepted;
    _metrics.totalDroppedMeasurements += dropped;

    if (_metrics.totalMeasurementAttempts > 0)
    {
      const double totalAttempts = static_cast<double>(_metrics.totalMeasurementAttempts);
      _metrics.associationAcceptanceRate =
        static_cast<double>(_metrics.totalAssociatedMeasurements) / totalAttempts;
      _metrics.associationDropRate =
        static_cast<double>(_metrics.totalDroppedMeasurements) / totalAttempts;
    }

    shouldWarnDropRate =
      _metrics.totalMeasurementAttempts >= kAssociationDropWarnMinSamples &&
      _metrics.associationDropRate >= kAssociationDropWarnRate;
    localMetrics = _metrics;
  }

  if (_logger)
  {
    _logger->logDebug(
      "Association health attempted=", attempted,
      ", accepted=", accepted,
      ", dropped=", dropped,
      ", totalDropRate=", localMetrics.associationDropRate);

    if (shouldWarnDropRate)
    {
      _logger->logWarn(
        "Association drop rate high: dropRate=", localMetrics.associationDropRate,
        ", attempts=", localMetrics.totalMeasurementAttempts,
        ", accepted=", localMetrics.totalAssociatedMeasurements,
        ", dropped=", localMetrics.totalDroppedMeasurements);
    }
  }
}

void FrontendHealthMonitor::recordLoopClosureCycle(
  std::size_t totalCandidates, std::size_t rejectedByValidation)
{
  bool spikeDetected = false;
  double cycleRejectRatio = 0.0;
  double rollingBaseline = 0.0;
  FrontendHealthMetrics localMetrics;

  {
    std::lock_guard<std::mutex> lock(_mutex);

    _metrics.rejectionSpikeInLastCycle = false;
    _metrics.totalLoopCandidates += totalCandidates;
    _metrics.totalRejectedLoopCandidates += rejectedByValidation;

    if (_metrics.totalLoopCandidates > 0)
    {
      _metrics.loopConstraintRejectRate =
        static_cast<double>(_metrics.totalRejectedLoopCandidates) /
        static_cast<double>(_metrics.totalLoopCandidates);
    }

    if (totalCandidates > 0)
    {
      cycleRejectRatio =
        static_cast<double>(rejectedByValidation) / static_cast<double>(totalCandidates);
      rollingBaseline = averageRatio(_recentLoopRejectRatios);

      if (totalCandidates >= kLoopRejectSpikeMinCandidates && !_recentLoopRejectRatios.empty())
      {
        const double ratioJump = cycleRejectRatio - rollingBaseline;
        spikeDetected =
          cycleRejectRatio >= kLoopRejectSpikeAbsoluteRatio &&
          ratioJump >= kLoopRejectSpikeRelativeJump;
      }

      _recentLoopRejectRatios.push_back(cycleRejectRatio);
      if (_recentLoopRejectRatios.size() > kLoopRejectRollingWindowSize)
      {
        _recentLoopRejectRatios.pop_front();
      }
    }

    if (spikeDetected)
    {
      _metrics.rejectionSpikeCount++;
      _metrics.rejectionSpikeInLastCycle = true;
      _loopClosureCooldownRemaining = kLoopClosureCooldownKeyframes;
      _metrics.loopClosureCooldownRemaining = kLoopClosureCooldownKeyframes;
    }

    localMetrics = _metrics;
  }

  if (_logger && spikeDetected)
  {
    _logger->logWarn(
      "Loop rejection spike detected: candidates=", totalCandidates,
      ", rejected=", rejectedByValidation,
      ", cycleRejectRatio=", cycleRejectRatio,
      ", baseline=", rollingBaseline,
      ", totalSpikes=", localMetrics.rejectionSpikeCount);
  }
}

FrontendHealthMetrics FrontendHealthMonitor::metrics() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _metrics;
}

void FrontendHealthMonitor::onKeyframeAccepted()
{
  bool cooldownExpired = false;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_loopClosureCooldownRemaining > 0)
    {
      --_loopClosureCooldownRemaining;
      _metrics.loopClosureCooldownRemaining = _loopClosureCooldownRemaining;
      cooldownExpired = (_loopClosureCooldownRemaining == 0);
    }
  }

  if (_logger && cooldownExpired)
  {
    _logger->logInfo("Loop closure cooldown expired — re-enabling loop closures.");
  }
}

bool FrontendHealthMonitor::isLoopClosureEnabled() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _loopClosureCooldownRemaining == 0;
}

void FrontendHealthMonitor::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _metrics = FrontendHealthMetrics();
  _recentLoopRejectRatios.clear();
  _loopClosureCooldownRemaining = 0;
}

}  // namespace slam
