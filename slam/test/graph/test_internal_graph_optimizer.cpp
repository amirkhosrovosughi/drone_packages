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

}  // namespace slam
