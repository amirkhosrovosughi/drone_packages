#include <gtest/gtest.h>

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
  InternalGraphOptimizer optimizer;
  optimizer.initialize();

  optimizer.applyObservation(AssignedMeasurements{
    makeNewLandmarkMeasurement(201, Eigen::Vector3d(0.1, 0.1, 0.0)),
    makeNewLandmarkMeasurement(202, Eigen::Vector3d(-0.2, 0.4, 0.0))});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 1
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201),
    makeRevisitMeasurement(202)});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));  // kf 2
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201),
    makeRevisitMeasurement(202)});

  optimizer.applyMotion(makeMotion(Eigen::Vector3d(-1.9, 0.0, 0.0))); // kf 3
  optimizer.applyObservation(AssignedMeasurements{
    makeRevisitMeasurement(201),
    makeRevisitMeasurement(202)});

  const auto candidates = optimizer.findSpatialLoopClosureCandidates(0.5, 2);
  ASSERT_EQ(candidates.size(), 1u);

  const auto validation = optimizer.validateLoopClosureCandidate(candidates.front());
  EXPECT_FALSE(validation.accepted);
  EXPECT_LT(validation.supportCount, 3);
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

}  // namespace slam
