#ifndef SLAM__ASSOCIATION__ASSOCIATION_PROFILE_HPP_
#define SLAM__ASSOCIATION__ASSOCIATION_PROFILE_HPP_

#include <optional>

namespace slam
{

enum class AssociationProfileMode
{
  Baseline = 0,
  GpsEnabled,
};

// Only the parameters that are expected to diverge between GPS OFF/ON modes.
// Other thresholds remain in per-pipeline constants until tuning requires them.
struct AssociationConfirmationConfig
{
  int minConfirmationObservations = 0;
  double maxTentativeCovarianceTrace = 0.0;
  double underConstrainedMaxCovarianceTrace = 0.0;
};

struct AssociationConfirmationOverrides
{
  std::optional<int> minConfirmationObservations;
  std::optional<double> maxTentativeCovarianceTrace;
  std::optional<double> underConstrainedMaxCovarianceTrace;
};

AssociationConfirmationConfig applyOverrides(
  const AssociationConfirmationConfig& baseline,
  const AssociationConfirmationOverrides& overrides);

AssociationConfirmationConfig makeEkfAssociationConfirmationConfig(
  AssociationProfileMode mode);

AssociationConfirmationConfig makeGraphAssociationConfirmationConfig(
  AssociationProfileMode mode);

}  // namespace slam

#endif  // SLAM__ASSOCIATION__ASSOCIATION_PROFILE_HPP_
