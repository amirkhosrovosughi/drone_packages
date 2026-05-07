#include "association/association_profile.hpp"

namespace slam
{

AssociationConfirmationConfig applyOverrides(
  const AssociationConfirmationConfig& baseline,
  const AssociationConfirmationOverrides& overrides)
{
  AssociationConfirmationConfig cfg = baseline;
  if (overrides.minConfirmationObservations.has_value())
  {
    cfg.minConfirmationObservations = *overrides.minConfirmationObservations;
  }
  if (overrides.maxTentativeCovarianceTrace.has_value())
  {
    cfg.maxTentativeCovarianceTrace = *overrides.maxTentativeCovarianceTrace;
  }
  if (overrides.underConstrainedMaxCovarianceTrace.has_value())
  {
    cfg.underConstrainedMaxCovarianceTrace =
      *overrides.underConstrainedMaxCovarianceTrace;
  }
  return cfg;
}

AssociationConfirmationConfig makeEkfAssociationConfirmationConfig(
  AssociationProfileMode mode)
{
  const AssociationConfirmationConfig baseline{
    8,
    0.10,
    0.35,
  };

  if (mode == AssociationProfileMode::GpsEnabled)
  {
    // GPS-enabled mode: slightly less conservative confirmation policy.
    const AssociationConfirmationOverrides gpsEnabled{
      6,
      0.14,
      0.45,
    };
    return applyOverrides(baseline, gpsEnabled);
  }

  return baseline;
}

AssociationConfirmationConfig makeGraphAssociationConfirmationConfig(
  AssociationProfileMode mode)
{
  const AssociationConfirmationConfig baseline{
    2,
    0.30,
    1.20,
  };

  if (mode == AssociationProfileMode::GpsEnabled)
  {
    // Graph is already permissive; only a minor covariance relaxation for now.
    const AssociationConfirmationOverrides gpsEnabled{
      std::nullopt,
      0.35,
      1.30,
    };
    return applyOverrides(baseline, gpsEnabled);
  }

  return baseline;
}

}  // namespace slam
