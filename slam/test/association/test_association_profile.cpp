#include <gtest/gtest.h>

#include "association/association_profile.hpp"

namespace slam
{

TEST(AssociationProfileTest, EkfBaselineMatchesCurrentBehavior)
{
  const AssociationConfirmationConfig cfg =
    makeEkfAssociationConfirmationConfig(AssociationProfileMode::Baseline);

  EXPECT_EQ(cfg.minConfirmationObservations, 8);
  EXPECT_DOUBLE_EQ(cfg.maxTentativeCovarianceTrace, 0.10);
  EXPECT_DOUBLE_EQ(cfg.underConstrainedMaxCovarianceTrace, 0.35);
}

TEST(AssociationProfileTest, GraphBaselineMatchesCurrentBehavior)
{
  const AssociationConfirmationConfig cfg =
    makeGraphAssociationConfirmationConfig(AssociationProfileMode::Baseline);

  EXPECT_EQ(cfg.minConfirmationObservations, 2);
  EXPECT_DOUBLE_EQ(cfg.maxTentativeCovarianceTrace, 0.30);
  EXPECT_DOUBLE_EQ(cfg.underConstrainedMaxCovarianceTrace, 1.20);
}

TEST(AssociationProfileTest, EkfGpsEnabledChangesOnlySelectedFields)
{
  const AssociationConfirmationConfig baseline =
    makeEkfAssociationConfirmationConfig(AssociationProfileMode::Baseline);
  const AssociationConfirmationConfig gpsEnabled =
    makeEkfAssociationConfirmationConfig(AssociationProfileMode::GpsEnabled);

  EXPECT_NE(gpsEnabled.minConfirmationObservations, baseline.minConfirmationObservations);
  EXPECT_NE(gpsEnabled.maxTentativeCovarianceTrace, baseline.maxTentativeCovarianceTrace);
  EXPECT_NE(
    gpsEnabled.underConstrainedMaxCovarianceTrace,
    baseline.underConstrainedMaxCovarianceTrace);
}

TEST(AssociationProfileTest, ApplyOverridesLeavesUnsetFieldsUntouched)
{
  const AssociationConfirmationConfig baseline{8, 0.10, 0.35};
  const AssociationConfirmationOverrides overrides{
    std::nullopt,
    0.14,
    std::nullopt,
  };

  const AssociationConfirmationConfig cfg = applyOverrides(baseline, overrides);

  EXPECT_EQ(cfg.minConfirmationObservations, baseline.minConfirmationObservations);
  EXPECT_DOUBLE_EQ(cfg.maxTentativeCovarianceTrace, 0.14);
  EXPECT_DOUBLE_EQ(
    cfg.underConstrainedMaxCovarianceTrace,
    baseline.underConstrainedMaxCovarianceTrace);
}

}  // namespace slam
