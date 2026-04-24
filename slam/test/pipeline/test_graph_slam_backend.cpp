#include <gtest/gtest.h>

#include <algorithm>
#include <memory>

#include "pipeline/graph_slam_backend.hpp"

namespace slam
{

class FakeGraphOptimizer : public GraphOptimizer
{
public:
  void initialize() override
  {
    initializeCalled = true;
    graph = GraphState();
    graph.activeKeyframeId = 0;
    graph.keyframes.emplace_back(0, 0.0, graph.robot);
  }

  void reset() override
  {
    resetCalled = true;
    graph = GraphState();
    graph.activeKeyframeId = 0;
    graph.keyframes.emplace_back(0, 0.0, graph.robot);
  }

  void applyMotion(const MotionConstraint& motion) override
  {
    applyMotionCalled = true;

    if (graph.keyframes.empty())
    {
      graph.activeKeyframeId = 0;
      graph.keyframes.emplace_back(0, 0.0, graph.robot);
    }

    const int fromId = graph.activeKeyframeId;

    graph.robot.pose.position.x += motion.delta_position.x();
    graph.robot.pose.position.y += motion.delta_position.y();
    graph.robot.pose.position.z += motion.delta_position.z();
    graph.robot.pose.quaternion.w = motion.orientation.w();
    graph.robot.pose.quaternion.x = motion.orientation.x();
    graph.robot.pose.quaternion.y = motion.orientation.y();
    graph.robot.pose.quaternion.z = motion.orientation.z();

    const int toId = fromId + 1;
    graph.keyframes.emplace_back(toId, 0.0, graph.robot);
    graph.odometryEdges.emplace_back(fromId, toId, motion);
    graph.activeKeyframeId = toId;
  }

  void applyObservation(const AssignedMeasurements& measurements) override
  {
    applyMeasurementsCalled = true;

    for (const auto& assigned : measurements)
    {
      graph.observationEdges.emplace_back(graph.activeKeyframeId, assigned.id);

      auto it = std::find_if(
        graph.landmarks.begin(),
        graph.landmarks.end(),
        [&](const GraphLandmarkNode& lm) { return lm.id == assigned.id; });

      if (it == graph.landmarks.end())
      {
        if (assigned.isNew && assigned.hasInitializedPosition)
        {
          GraphLandmarkNode lm;
          lm.id = assigned.id;
          lm.position = assigned.position;
          lm.observeRepeat = 1;
          graph.landmarks.push_back(lm);
        }
      }
      else
      {
        it->observeRepeat += 1;
      }
    }
  }

  std::vector<LoopClosureCandidate> findSpatialLoopClosureCandidates(
    double,
    int) const override
  {
    return candidates;
  }

  LoopClosureValidationResult validateLoopClosureCandidate(
    const LoopClosureCandidate& candidate) const override
  {
    lastValidatedCandidate = candidate;
    return validationToReturn;
  }

  bool commitLoopClosure(
    const LoopClosureCandidate& candidate,
    const LoopClosureValidationResult& validation) override
  {
    commitCalled = true;
    lastCommittedCandidate = candidate;
    lastCommitValidation = validation;
    return commitReturnValue;
  }

  GraphState getGraphState() const override
  {
    return graph;
  }

  void setLogger(LoggerPtr logger) override
  {
    loggerSet = logger;
  }

  bool initializeCalled = false;
  bool resetCalled = false;
  bool applyMotionCalled = false;
  bool applyMeasurementsCalled = false;
  bool commitCalled = false;
  LoggerPtr loggerSet;
  GraphState graph;
  std::vector<LoopClosureCandidate> candidates;
  mutable LoopClosureCandidate lastValidatedCandidate;
  mutable LoopClosureCandidate lastCommittedCandidate;
  mutable LoopClosureValidationResult lastCommitValidation;
  LoopClosureValidationResult validationToReturn;
  bool commitReturnValue = false;
};

static MotionConstraint makeMotion(
  const Eigen::Vector3d& delta,
  const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity())
{
  MotionConstraint m;
  m.delta_position = delta;
  m.orientation = orientation;
  return m;
}

TEST(GraphSlamBackendTest, InitializeThrowsWhenOptimizerMissing)
{
  GraphSlamBackend backend(nullptr);
  EXPECT_THROW(backend.initialize(), std::runtime_error);
}

TEST(GraphSlamBackendTest, InitializeAndResetContracts)
{
  auto optimizer = std::make_shared<FakeGraphOptimizer>();
  GraphSlamBackend backend(optimizer);

  backend.initialize();
  EXPECT_TRUE(optimizer->initializeCalled);
  ASSERT_EQ(optimizer->graph.keyframes.size(), 1u);
  EXPECT_EQ(optimizer->graph.activeKeyframeId, 0);

  backend.applyMotionConstraint(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  ASSERT_EQ(optimizer->graph.keyframes.size(), 2u);

  backend.reset();
  EXPECT_TRUE(optimizer->resetCalled);
  ASSERT_EQ(optimizer->graph.keyframes.size(), 1u);
  EXPECT_EQ(optimizer->graph.activeKeyframeId, 0);
  EXPECT_TRUE(optimizer->graph.odometryEdges.empty());
  EXPECT_TRUE(optimizer->graph.landmarks.empty());
  EXPECT_TRUE(optimizer->graph.observationEdges.empty());
}

TEST(GraphSlamBackendTest, ApplyMotionConstraintCreatesGraphProgression)
{
  auto optimizer = std::make_shared<FakeGraphOptimizer>();
  GraphSlamBackend backend(optimizer);
  backend.initialize();

  backend.applyMotionConstraint(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  backend.applyMotionConstraint(makeMotion(Eigen::Vector3d(0.0, 2.0, 0.0)));

  EXPECT_TRUE(optimizer->applyMotionCalled);
  ASSERT_EQ(optimizer->graph.keyframes.size(), 3u);
  ASSERT_EQ(optimizer->graph.odometryEdges.size(), 2u);

  EXPECT_EQ(optimizer->graph.odometryEdges[0].fromKeyframeId, 0);
  EXPECT_EQ(optimizer->graph.odometryEdges[0].toKeyframeId, 1);
  EXPECT_EQ(optimizer->graph.odometryEdges[1].fromKeyframeId, 1);
  EXPECT_EQ(optimizer->graph.odometryEdges[1].toKeyframeId, 2);

  EXPECT_EQ(optimizer->graph.activeKeyframeId, 2);

  auto map = backend.getMap();
  EXPECT_DOUBLE_EQ(map.robot.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(map.robot.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(map.robot.pose.position.z, 0.0);
}

TEST(GraphSlamBackendTest, ApplyObservationConstraintUpdatesLandmarksAndEdgesConsistently)
{
  auto optimizer = std::make_shared<FakeGraphOptimizer>();
  GraphSlamBackend backend(optimizer);
  backend.initialize();

  backend.applyMotionConstraint(makeMotion(Eigen::Vector3d(0.5, 0.0, 0.0)));

  AssignedMeasurement newLandmark;
  newLandmark.id = 7;
  newLandmark.isNew = true;
  newLandmark.hasInitializedPosition = true;
  newLandmark.position = Position(3.0, -1.0, 2.0);

  backend.applyObservationConstraint(AssignedMeasurements{newLandmark});

  ASSERT_EQ(optimizer->graph.landmarks.size(), 1u);
  EXPECT_EQ(optimizer->graph.landmarks[0].id, 7);
  EXPECT_EQ(optimizer->graph.landmarks[0].observeRepeat, 1);
  ASSERT_EQ(optimizer->graph.observationEdges.size(), 1u);
  EXPECT_EQ(optimizer->graph.observationEdges[0].keyframeId, optimizer->graph.activeKeyframeId);
  EXPECT_EQ(optimizer->graph.observationEdges[0].landmarkId, 7);

  AssignedMeasurement revisit;
  revisit.id = 7;
  revisit.isNew = false;
  revisit.hasInitializedPosition = false;

  backend.applyObservationConstraint(AssignedMeasurements{revisit});

  EXPECT_TRUE(optimizer->applyMeasurementsCalled);
  ASSERT_EQ(optimizer->graph.landmarks.size(), 1u);
  EXPECT_EQ(optimizer->graph.landmarks[0].observeRepeat, 2);
  ASSERT_EQ(optimizer->graph.observationEdges.size(), 2u);

  auto map = backend.getMap();
  ASSERT_EQ(map.landmarks.size(), 1u);
  EXPECT_EQ(map.landmarks[0].id, 7);
  EXPECT_EQ(map.landmarks[0].observeRepeat, 2);
}

TEST(GraphSlamBackendTest, ValidateAndCommitFlowRejectsWithoutCommit)
{
  auto optimizer = std::make_shared<FakeGraphOptimizer>();
  GraphSlamBackend backend(optimizer);
  backend.initialize();

  optimizer->validationToReturn =
    LoopClosureValidationResult(false, 0, 0, "rejected");

  const LoopClosureCandidate candidate(5, 1, 0.3, false, 0.0);
  LoopClosureValidationResult validation;
  const bool accepted = backend.validateAndCommitLoopClosure(candidate, &validation);

  EXPECT_FALSE(accepted);
  EXPECT_FALSE(optimizer->commitCalled);
  EXPECT_FALSE(validation.accepted);
}

TEST(GraphSlamBackendTest, ValidateAndCommitFlowCommitsWhenAccepted)
{
  auto optimizer = std::make_shared<FakeGraphOptimizer>();
  GraphSlamBackend backend(optimizer);
  backend.initialize();

  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 12, 15, "");
  optimizer->commitReturnValue = true;

  const LoopClosureCandidate candidate(8, 2, 0.4, false, 0.0);
  LoopClosureValidationResult validation;
  const bool accepted = backend.validateAndCommitLoopClosure(candidate, &validation);

  EXPECT_TRUE(accepted);
  EXPECT_TRUE(optimizer->commitCalled);
  EXPECT_TRUE(validation.accepted);
  EXPECT_EQ(optimizer->lastCommittedCandidate.sourceKeyframeId, 8);
  EXPECT_EQ(optimizer->lastCommittedCandidate.targetKeyframeId, 2);
}

}  // namespace slam
