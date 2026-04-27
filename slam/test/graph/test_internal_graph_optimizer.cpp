#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include "graph/internal_graph_optimizer.hpp"

namespace slam
{

static MotionConstraint makeMotion(
  const Eigen::Vector3d& delta,
  const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
{
  MotionConstraint m;
  m.delta_position = delta;
  m.orientation = orientation;
  return m;
}

static AssignedMeasurement makeNewLandmarkMeasurement(
  int id,
  const Eigen::Vector3d& position)
{
  AssignedMeasurement measurement;
  measurement.id = id;
  measurement.isNew = true;
  measurement.hasInitializedPosition = true;
  measurement.position = Position(position.x(), position.y(), position.z());
  return measurement;
}

static AssignedMeasurement makeRevisitMeasurement(int id)
{
  AssignedMeasurement measurement;
  measurement.id = id;
  measurement.isNew = false;
  measurement.hasInitializedPosition = false;
  return measurement;
}

static void seedSharedLandmarks(InternalGraphOptimizer* optimizer)
{
  ASSERT_NE(optimizer, nullptr);

  optimizer->applyObservation(AssignedMeasurements{
    makeNewLandmarkMeasurement(101, Eigen::Vector3d(0.2, 0.5, 0.1)),
    makeNewLandmarkMeasurement(102, Eigen::Vector3d(-0.4, 0.3, 0.2)),
    makeNewLandmarkMeasurement(103, Eigen::Vector3d(0.6, -0.1, 0.3))});

  optimizer->applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer->applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(101),
    makeRevisitMeasurement(102),
    makeRevisitMeasurement(103)});

  optimizer->applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer->applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(101),
    makeRevisitMeasurement(102),
    makeRevisitMeasurement(103)});

  optimizer->applyMotion(makeMotion(Eigen::Vector3d(-1.9, 0.0, 0.0))); // kf 3
  optimizer->applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(101),
    makeRevisitMeasurement(102),
    makeRevisitMeasurement(103)});
}

TEST(InternalGraphOptimizerTest, KeyframeAndOdometryEdgeInsertion)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  auto initial = optimizer.getGraphState();
  ASSERT_EQ(initial.keyframes.size(), 1u);
  EXPECT_EQ(initial.activeKeyframeId, 0);
  EXPECT_TRUE(initial.odometryEdges.empty());

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(0.0, 2.0, 0.0)));

  auto graph = optimizer.getGraphState();

  ASSERT_EQ(graph.keyframes.size(), 3u);
  ASSERT_EQ(graph.odometryEdges.size(), 2u);

  EXPECT_EQ(graph.activeKeyframeId, 2);
  EXPECT_EQ(graph.odometryEdges[0].fromKeyframeId, 0);
  EXPECT_EQ(graph.odometryEdges[0].toKeyframeId, 1);
  EXPECT_EQ(graph.odometryEdges[1].fromKeyframeId, 1);
  EXPECT_EQ(graph.odometryEdges[1].toKeyframeId, 2);

  EXPECT_DOUBLE_EQ(graph.robot.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(graph.robot.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(graph.robot.pose.position.z, 0.0);
}

TEST(InternalGraphOptimizerTest, LandmarkInsertionAndUpdateBehavior)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(0.5, 0.0, 0.0)));

  AssignedMeasurement newLandmark;
  newLandmark.id = 11;
  newLandmark.isNew = true;
  newLandmark.hasInitializedPosition = true;
  newLandmark.position = Position(3.0, -1.0, 2.0);

  optimizer.applyObservation(AssignedMeasurements{newLandmark});

  auto afterInsert = optimizer.getGraphState();
  ASSERT_EQ(afterInsert.landmarks.size(), 1u);
  EXPECT_EQ(afterInsert.landmarks[0].id, 11);
  EXPECT_EQ(afterInsert.landmarks[0].observeRepeat, 1);
  ASSERT_EQ(afterInsert.observationEdges.size(), 1u);
  EXPECT_EQ(afterInsert.observationEdges[0].keyframeId, afterInsert.activeKeyframeId);
  EXPECT_EQ(afterInsert.observationEdges[0].landmarkId, 11);

  AssignedMeasurement revisit;
  revisit.id = 11;
  revisit.isNew = false;
  revisit.hasInitializedPosition = false;

  optimizer.applyObservation(AssignedMeasurements{revisit});

  auto afterUpdate = optimizer.getGraphState();
  ASSERT_EQ(afterUpdate.landmarks.size(), 1u);
  EXPECT_EQ(afterUpdate.landmarks[0].observeRepeat, 2);
  ASSERT_EQ(afterUpdate.observationEdges.size(), 2u);
  EXPECT_EQ(afterUpdate.observationEdges[1].keyframeId, afterUpdate.activeKeyframeId);
  EXPECT_EQ(afterUpdate.observationEdges[1].landmarkId, 11);
}

TEST(InternalGraphOptimizerTest, GraphToMapAdapterConsistency)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 2.0, 0.0)));

  AssignedMeasurement newLandmark;
  newLandmark.id = 21;
  newLandmark.isNew = true;
  newLandmark.hasInitializedPosition = true;
  newLandmark.position = Position(4.0, 5.0, 6.0);

  optimizer.applyObservation(AssignedMeasurements{newLandmark});

  const auto graph = optimizer.getGraphState();
  const auto mapFromFreeAdapter = graphStateToMapSummary(graph);
  const auto mapFromOptimizer = optimizer.getMap();

  EXPECT_DOUBLE_EQ(mapFromOptimizer.robot.pose.position.x, graph.robot.pose.position.x);
  EXPECT_DOUBLE_EQ(mapFromOptimizer.robot.pose.position.y, graph.robot.pose.position.y);
  EXPECT_DOUBLE_EQ(mapFromOptimizer.robot.pose.position.z, graph.robot.pose.position.z);

  ASSERT_EQ(mapFromOptimizer.landmarks.size(), graph.landmarks.size());
  ASSERT_EQ(mapFromFreeAdapter.landmarks.size(), graph.landmarks.size());

  ASSERT_EQ(mapFromOptimizer.landmarks.size(), 1u);
  EXPECT_EQ(mapFromOptimizer.landmarks[0].id, 21);
  EXPECT_DOUBLE_EQ(mapFromOptimizer.landmarks[0].position.x, 4.0);
  EXPECT_DOUBLE_EQ(mapFromOptimizer.landmarks[0].position.y, 5.0);
  EXPECT_DOUBLE_EQ(mapFromOptimizer.landmarks[0].position.z, 6.0);

  EXPECT_EQ(mapFromOptimizer.landmarks[0].id, mapFromFreeAdapter.landmarks[0].id);
  EXPECT_EQ(mapFromOptimizer.landmarks[0].observeRepeat, mapFromFreeAdapter.landmarks[0].observeRepeat);
}

TEST(InternalGraphOptimizerTest, SpatialLoopCandidatesUseDistanceAndSeparation)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  seedSharedLandmarks(&optimizer);

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(
    0.5,
    2);

  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_EQ(candidates.front().sourceKeyframeId, 3);
  EXPECT_EQ(candidates.front().targetKeyframeId, 0);
  EXPECT_LE(candidates.front().spatialDistanceMeters, 0.5);
}

TEST(InternalGraphOptimizerTest, GeometricValidationAcceptsConsistentCandidate)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  seedSharedLandmarks(&optimizer);

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(
    0.5,
    2);
  ASSERT_EQ(candidates.size(), 1u);

  const LoopClosureValidationResult validation =
    optimizer.validateLoopClosureCandidate(candidates.front());
  EXPECT_TRUE(validation.accepted);
  EXPECT_GE(validation.supportCount, 3);
  EXPECT_GE(validation.inlierCount, 2);
  EXPECT_GE(validation.inlierRatio, 0.6);

  const bool committed = optimizer.commitLoopClosure(candidates.front(), validation);
  EXPECT_TRUE(committed);

  const auto graph = optimizer.getGraphState();
  ASSERT_EQ(graph.loopClosureEdges.size(), 1u);
  EXPECT_EQ(graph.loopClosureEdges.front().fromKeyframeId, candidates.front().sourceKeyframeId);
  EXPECT_EQ(graph.loopClosureEdges.front().toKeyframeId, candidates.front().targetKeyframeId);
  EXPECT_GE(graph.loopClosureEdges.front().inlierRatio, 0.6);
}

TEST(InternalGraphOptimizerTest, ExistingLoopEdgeIsSuppressedFromNewCandidates)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  seedSharedLandmarks(&optimizer);

  const auto firstCandidates = optimizer.findSpatialLoopClosureCandidates(
    0.5,
    2);
  ASSERT_EQ(firstCandidates.size(), 1u);

  const LoopClosureValidationResult validation =
    optimizer.validateLoopClosureCandidate(firstCandidates.front());
  ASSERT_TRUE(validation.accepted);
  ASSERT_TRUE(optimizer.commitLoopClosure(firstCandidates.front(), validation));

  const auto secondCandidates = optimizer.findSpatialLoopClosureCandidates(
    0.5,
    2);
  EXPECT_TRUE(secondCandidates.empty());
}

TEST(InternalGraphOptimizerTest, RejectsWhenSharedCorrespondenceSupportTooLow)
{
  constexpr int kMinRequiredCorrespondences = 2;

  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyObservation(AssignedMeasurements{
    makeNewLandmarkMeasurement(201, Eigen::Vector3d(0.1, 0.1, 0.0))});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201)});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201)});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(-1.9, 0.0, 0.0))); // kf 3
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201)});

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(0.5, 2);
  ASSERT_EQ(candidates.size(), 1u);

  const auto validation = optimizer.validateLoopClosureCandidate(candidates.front());
  EXPECT_FALSE(validation.accepted);
  EXPECT_LT(validation.supportCount, kMinRequiredCorrespondences);
  EXPECT_EQ(validation.reason, "Insufficient shared landmark correspondences");
}

TEST(InternalGraphOptimizerTest, RejectValidationDoesNotMutateLoopClosureEdges)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2

  const LoopClosureCandidate invalidCandidate(2, 1, 0.1, false, 0.0);
  const LoopClosureValidationResult validation =
    optimizer.validateLoopClosureCandidate(invalidCandidate);
  EXPECT_FALSE(validation.accepted);

  const bool committed = optimizer.commitLoopClosure(invalidCandidate, validation);
  EXPECT_FALSE(committed);

  const auto graph = optimizer.getGraphState();
  EXPECT_TRUE(graph.loopClosureEdges.empty());
}

TEST(InternalGraphOptimizerTest, CandidatesExcludedWhenPriorKeyframesAreTooFar)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  // Move far from origin so no prior keyframe falls within maxDistance.
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(5.0, 0.0, 0.0)));  // kf 1
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(5.0, 0.0, 0.0)));  // kf 2
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(5.0, 0.0, 0.0)));  // kf 3

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(0.5, 2);
  EXPECT_TRUE(candidates.empty());
}

TEST(InternalGraphOptimizerTest, CandidatesBelowMinKeyframeSeparationAreExcluded)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  // kf1 is only 1 frame away from kf0 — separation = 1 < minSeparation = 2.
  // Distance is within range but the pair must not appear as a candidate.
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(0.1, 0.0, 0.0)));  // kf 1

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(1.0, 2);
  EXPECT_TRUE(candidates.empty());
}

TEST(InternalGraphOptimizerTest, CommittedEdgeFieldsMatchValidationResult)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  seedSharedLandmarks(&optimizer);

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(0.5, 2);
  ASSERT_EQ(candidates.size(), 1u);

  const LoopClosureValidationResult validation =
    optimizer.validateLoopClosureCandidate(candidates.front());
  ASSERT_TRUE(validation.accepted);

  optimizer.commitLoopClosure(candidates.front(), validation);

  const auto graph = optimizer.getGraphState();
  ASSERT_EQ(graph.loopClosureEdges.size(), 1u);
  const GraphLoopClosureEdge& edge = graph.loopClosureEdges.front();
  EXPECT_EQ(edge.fromKeyframeId, candidates.front().sourceKeyframeId);
  EXPECT_EQ(edge.toKeyframeId, candidates.front().targetKeyframeId);
  EXPECT_EQ(edge.inlierCount, validation.inlierCount);
  EXPECT_EQ(edge.supportCount, validation.supportCount);
  EXPECT_DOUBLE_EQ(edge.inlierRatio, validation.inlierRatio);
}

TEST(InternalGraphOptimizerTest, RefineActiveKeyframeMovesPoseAfterObservation)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyObservation(AssignedMeasurements{
    makeNewLandmarkMeasurement(301, Eigen::Vector3d(5.0, 1.0, 0.0)),
    makeNewLandmarkMeasurement(302, Eigen::Vector3d(4.5, -1.0, 0.0))});

  const GraphState before = optimizer.getGraphState();
  const GraphKeyframeNode beforeActive = before.keyframes.back();

  OptimizationConfig config;
  config.strategy = RefinementStrategy::GaussNewton;
  config.maxIterations = 3;
  config.convergeThreshold = 1e-6;

  optimizer.refineActiveKeyframe(config);

  const GraphState after = optimizer.getGraphState();
  const GraphKeyframeNode afterActive = after.keyframes.back();

  const double dx = afterActive.robot.pose.position.x - beforeActive.robot.pose.position.x;
  const double dy = afterActive.robot.pose.position.y - beforeActive.robot.pose.position.y;
  const double dz = afterActive.robot.pose.position.z - beforeActive.robot.pose.position.z;
  const double deltaNorm = std::sqrt(dx * dx + dy * dy + dz * dz);

  EXPECT_GT(deltaNorm, 1e-9);
}

TEST(InternalGraphOptimizerTest, RefineActiveKeyframeDeltaIsFiniteAndBounded)
{
  constexpr double kMaxLocalRefinementStepMeters = 0.15;

  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(0.5, 0.0, 0.0)));  // kf 1
  optimizer.applyObservation(AssignedMeasurements{
    makeNewLandmarkMeasurement(401, Eigen::Vector3d(6.0, 0.0, 0.0)),
    makeNewLandmarkMeasurement(402, Eigen::Vector3d(5.5, 0.5, 0.0)),
    makeNewLandmarkMeasurement(403, Eigen::Vector3d(5.5, -0.5, 0.0))});

  const GraphState before = optimizer.getGraphState();
  const GraphKeyframeNode beforeActive = before.keyframes.back();

  OptimizationConfig config;
  config.strategy = RefinementStrategy::GaussNewton;
  config.maxIterations = 4;
  config.convergeThreshold = 1e-8;

  optimizer.refineActiveKeyframe(config);

  const GraphState after = optimizer.getGraphState();
  const GraphKeyframeNode afterActive = after.keyframes.back();

  const double dx = afterActive.robot.pose.position.x - beforeActive.robot.pose.position.x;
  const double dy = afterActive.robot.pose.position.y - beforeActive.robot.pose.position.y;
  const double dz = afterActive.robot.pose.position.z - beforeActive.robot.pose.position.z;
  const double deltaNorm = std::sqrt(dx * dx + dy * dy + dz * dz);

  EXPECT_TRUE(std::isfinite(dx));
  EXPECT_TRUE(std::isfinite(dy));
  EXPECT_TRUE(std::isfinite(dz));
  EXPECT_TRUE(std::isfinite(deltaNorm));

  // Overall refinement is bounded by per-iteration clamp * iteration count.
  const double maxExpectedNorm =
    static_cast<double>(config.maxIterations) * kMaxLocalRefinementStepMeters + 1e-9;
  EXPECT_LE(deltaNorm, maxExpectedNorm);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphAdjustsGlobalKeyframePoses)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 3

  const GraphState before = optimizer.getGraphState();
  ASSERT_EQ(before.keyframes.size(), 4u);

  LoopClosureCandidate closureCandidate(3, 0, 0.0, false, 0.0);
  MotionConstraint closureMotion;
  closureMotion.delta_position = Eigen::Vector3d::Zero();
  closureMotion.orientation = Eigen::Quaterniond::Identity();
  LoopClosureValidationResult closureValidation(
    true, 4, 4, "", closureMotion, 1.0);
  ASSERT_TRUE(optimizer.commitLoopClosure(closureCandidate, closureValidation));

  OptimizationConfig config;
  config.maxIterations = 10;
  config.convergeThreshold = 1e-6;

  OptimizationResult result;
  const bool optimizeOk = optimizer.optimizeGraph(config, &result);
  EXPECT_TRUE(optimizeOk);
  EXPECT_TRUE(result.success);
  EXPECT_GT(result.numPosesRefined, 0);

  const GraphState after = optimizer.getGraphState();
  ASSERT_EQ(after.keyframes.size(), before.keyframes.size());

  const double beforeLoopGap = std::abs(
    before.keyframes[3].robot.pose.position.x - before.keyframes[0].robot.pose.position.x);
  const double afterLoopGap = std::abs(
    after.keyframes[3].robot.pose.position.x - after.keyframes[0].robot.pose.position.x);

  EXPECT_LT(afterLoopGap, beforeLoopGap);
}

// --- optimizeGraph unit tests ---

TEST(InternalGraphOptimizerTest, OptimizeGraphReturnsFalseWithSingleKeyframe)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();  // only kf0, no motion applied

  OptimizationResult result;
  const bool ok = optimizer.optimizeGraph(OptimizationConfig{}, &result);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.failureReason.empty());
}

TEST(InternalGraphOptimizerTest, OptimizeGraphSucceedsWithMinimalConsistentGraph)
{
  // 2 keyframes, 1 odometry edge, no tension → should converge to success.
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1

  OptimizationConfig config;
  config.maxIterations = 5;
  config.convergeThreshold = 1e-6;

  OptimizationResult result;
  const bool ok = optimizer.optimizeGraph(config, &result);

  EXPECT_TRUE(ok);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.numPosesRefined, 2);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphResultFieldsArePopulatedOnSuccess)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));

  OptimizationConfig config;
  config.maxIterations = 5;

  OptimizationResult result;
  optimizer.optimizeGraph(config, &result);

  EXPECT_GT(result.numPosesRefined, 0);
  EXPECT_GT(result.numIterations, 0);
  EXPECT_GE(result.solveTimeMs, 0);
  EXPECT_GE(result.finalError, 0.0);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphNumPosesRefinedMatchesKeyframeCount)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  for (int i = 0; i < 4; ++i)
  {
    optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  }
  // 5 keyframes total (kf0 … kf4)
  OptimizationResult result;
  optimizer.optimizeGraph(OptimizationConfig{}, &result);

  EXPECT_EQ(result.numPosesRefined, 5);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphAnchorKeyframeIsPreservedUnderLoopTension)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 3

  const double anchorX = optimizer.getGraphState().keyframes.front().robot.pose.position.x;

  MotionConstraint loopMotion;
  loopMotion.delta_position = Eigen::Vector3d::Zero();
  loopMotion.orientation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(optimizer.commitLoopClosure(
    LoopClosureCandidate(3, 0, 0.0, false, 0.0),
    LoopClosureValidationResult(true, 5, 5, "", loopMotion, 1.0)));

  OptimizationConfig config;
  config.maxIterations = 20;
  config.convergeThreshold = 1e-8;
  optimizer.optimizeGraph(config, nullptr);

  const double anchorXAfter =
    optimizer.getGraphState().keyframes.front().robot.pose.position.x;

  // The first keyframe should barely drift compared to the 3-meter loop closure correction.
  EXPECT_NEAR(anchorXAfter, anchorX, 0.1);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphActiveRobotStateSyncedAfterSolve)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));

  optimizer.optimizeGraph(OptimizationConfig{}, nullptr);

  const GraphState state = optimizer.getGraphState();
  const GraphKeyframeNode* activeKf =
    [&]() -> const GraphKeyframeNode*
    {
      for (const auto& kf : state.keyframes)
      {
        if (kf.id == state.activeKeyframeId)
        {
          return &kf;
        }
      }
      return nullptr;
    }();

  ASSERT_NE(activeKf, nullptr);
  EXPECT_DOUBLE_EQ(state.robot.pose.position.x, activeKf->robot.pose.position.x);
  EXPECT_DOUBLE_EQ(state.robot.pose.position.y, activeKf->robot.pose.position.y);
  EXPECT_DOUBLE_EQ(state.robot.pose.position.z, activeKf->robot.pose.position.z);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphLoopClosureReducesWeightedResidual)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));

  MotionConstraint loopMotion;
  loopMotion.delta_position = Eigen::Vector3d::Zero();
  loopMotion.orientation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(optimizer.commitLoopClosure(
    LoopClosureCandidate(3, 0, 0.0, false, 0.0),
    LoopClosureValidationResult(true, 5, 5, "", loopMotion, 1.0)));

  // First pass: one iteration gives us post-solve residual.
  OptimizationConfig singleIter;
  singleIter.maxIterations = 1;
  OptimizationResult firstResult;
  ASSERT_TRUE(optimizer.optimizeGraph(singleIter, &firstResult));

  // Second full solve: should converge to a lower residual.
  OptimizationConfig fullConfig;
  fullConfig.maxIterations = 20;
  fullConfig.convergeThreshold = 1e-8;
  OptimizationResult fullResult;
  ASSERT_TRUE(optimizer.optimizeGraph(fullConfig, &fullResult));

  EXPECT_LE(fullResult.finalError, firstResult.finalError + 1e-9);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphSymmetricCorrectionDistributedAcrossChain)
{
  // 4 kf at x = 0,1,2,3.  Loop closure kf3→kf0 with zero relative motion
  // should compress the chain so intermediate poses move toward the anchor.
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 3

  MotionConstraint loopMotion;
  loopMotion.delta_position = Eigen::Vector3d::Zero();
  loopMotion.orientation = Eigen::Quaterniond::Identity();
  ASSERT_TRUE(optimizer.commitLoopClosure(
    LoopClosureCandidate(3, 0, 0.0, false, 0.0),
    LoopClosureValidationResult(true, 5, 5, "", loopMotion, 1.0)));

  OptimizationConfig config;
  config.maxIterations = 50;
  config.convergeThreshold = 1e-8;
  ASSERT_TRUE(optimizer.optimizeGraph(config, nullptr));

  const auto& kfs = optimizer.getGraphState().keyframes;
  ASSERT_EQ(kfs.size(), 4u);

  // kf1 and kf2 should have shifted from their original odometry positions.
  EXPECT_LT(kfs[1].robot.pose.position.x, 1.0);
  EXPECT_LT(kfs[2].robot.pose.position.x, 2.0);
  // All poses remain finite.
  for (const auto& kf : kfs)
  {
    EXPECT_TRUE(std::isfinite(kf.robot.pose.position.x));
    EXPECT_TRUE(std::isfinite(kf.robot.pose.position.y));
    EXPECT_TRUE(std::isfinite(kf.robot.pose.position.z));
  }
}

TEST(InternalGraphOptimizerTest, OptimizeGraphSingleIterationStillSucceeds)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));

  OptimizationConfig config;
  config.maxIterations = 1;

  OptimizationResult result;
  const bool ok = optimizer.optimizeGraph(config, &result);

  EXPECT_TRUE(ok);
  EXPECT_EQ(result.numIterations, 1);
}

TEST(InternalGraphOptimizerTest, OptimizeGraphSecondCallOnConvergedGraphChangesNothingSignificant)
{
  InternalGraphOptimizer optimizer;
  optimizer.initialize();
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));

  OptimizationConfig config;
  config.maxIterations = 20;
  config.convergeThreshold = 1e-8;

  OptimizationResult first, second;
  ASSERT_TRUE(optimizer.optimizeGraph(config, &first));

  const double x1 = optimizer.getGraphState().keyframes.back().robot.pose.position.x;
  ASSERT_TRUE(optimizer.optimizeGraph(config, &second));
  const double x2 = optimizer.getGraphState().keyframes.back().robot.pose.position.x;

  EXPECT_NEAR(x1, x2, 1e-6);
  EXPECT_TRUE(second.success);
}

}  // namespace slam
