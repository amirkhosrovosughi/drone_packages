#include <gtest/gtest.h>

#include <memory>
#include <thread>

#include <Eigen/Geometry>

#include "association/base_association.hpp"
#include "graph/graph_optimizer.hpp"
#include "graph/internal_graph_optimizer.hpp"
#include "pipeline/graph_slam_backend.hpp"
#include "pipeline/graph_slam_frontend.hpp"
#include "pipeline/graph_slam_pipeline.hpp"

namespace slam
{

class PipelineFakeAssociation : public BaseAssociation
{
public:
  void onReceiveMeasurement(const Measurements& meas) override
  {
    lastMeasurements = meas;
    if (storedCallback)
    {
      storedCallback(assignedToEmit);
    }
  }

  void handleUpdate(const MapSummary& map) override
  {
    lastMap = map;
  }

  void registerCallback(std::function<void(AssignedMeasurements)> callback) override
  {
    storedCallback = std::move(callback);
  }

  void setLogger(LoggerPtr logger) override
  {
    loggerSet = logger;
  }

  AssignedMeasurements assignedToEmit;
  Measurements lastMeasurements;
  MapSummary lastMap;
  LoggerPtr loggerSet;

private:
  void processMeasurement(const Measurements&) override {}
  std::function<void(AssignedMeasurements)> storedCallback;
};

class PipelineFakeGraphOptimizer : public GraphOptimizer
{
public:
  void initialize() override
  {
    graph = GraphState();
    graph.activeKeyframeId = 0;
    graph.keyframes.emplace_back(0, 0.0, graph.robot);
  }

  void reset() override
  {
    graph = GraphState();
    graph.activeKeyframeId = 0;
    graph.keyframes.emplace_back(0, 0.0, graph.robot);
  }

  void applyMotion(const MotionConstraint& motion) override
  {
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
    for (const auto& assigned : measurements)
    {
      graph.observationEdges.emplace_back(graph.activeKeyframeId, assigned.id);
    }
  }

  std::vector<LoopClosureCandidate> findSpatialLoopClosureCandidates(
    double,
    int) const override
  {
    return candidatesToReturn;
  }

  LoopClosureValidationResult validateLoopClosureCandidate(
    const LoopClosureCandidate& candidate) const override
  {
    validateCallCount += 1;
    lastValidatedCandidate = candidate;
    return validationToReturn;
  }

  bool commitLoopClosure(
    const LoopClosureCandidate& candidate,
    const LoopClosureValidationResult& validation) override
  {
    commitCallCount += 1;
    lastCommittedCandidate = candidate;
    lastCommitValidation = validation;
    if (commitReturnValue)
    {
      graph.loopClosureEdges.emplace_back(
        candidate.sourceKeyframeId,
        candidate.targetKeyframeId,
        validation.estimatedRelativeMotion,
        validation.inlierCount,
        validation.supportCount,
        validation.inlierRatio);
    }
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

    void refineActiveKeyframe(const OptimizationConfig& config = OptimizationConfig()) override
    {
      // Stub: do nothing
    }

    bool optimizeGraph(
      const OptimizationConfig& config,
      OptimizationResult* resultOut = nullptr) override
    {
      ++optimizeCallCount;
      if (resultOut)
      {
        resultOut->success = false;
        resultOut->failureReason = "Test stub";
      }
      return false;
    }

  mutable int validateCallCount = 0;
  mutable int commitCallCount = 0;
  int optimizeCallCount = 0;
  mutable LoopClosureCandidate lastValidatedCandidate;
  mutable LoopClosureCandidate lastCommittedCandidate;
  mutable LoopClosureValidationResult lastCommitValidation;
  std::vector<LoopClosureCandidate> candidatesToReturn;
  LoopClosureValidationResult validationToReturn;
  bool commitReturnValue = false;
  GraphState graph;
  LoggerPtr loggerSet;
};

static MotionConstraint makeMotion(const Eigen::Vector3d& delta)
{
  MotionConstraint motion;
  motion.delta_position = delta;
  motion.orientation = Eigen::Quaterniond::Identity();
  return motion;
}

static AssignedMeasurement makeAssignedMeasurement(int id)
{
  AssignedMeasurement measurement;
  measurement.id = id;
  measurement.isNew = false;
  measurement.hasInitializedPosition = false;
  return measurement;
}

static AssignedMeasurement makeNewLandmarkAssignment(int id, const Eigen::Vector3d& worldPos)
{
  AssignedMeasurement measurement;
  measurement.id = id;
  measurement.isNew = true;
  measurement.hasInitializedPosition = true;
  measurement.position = Position(worldPos.x(), worldPos.y(), worldPos.z());
  return measurement;
}

TEST(GraphSlamPipelineTest, ObservationTriggeredLoopClosureUsesAtomicBackendPath)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));

  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(41)};
  optimizer->candidatesToReturn = {LoopClosureCandidate(1, 0, 0.2, false, 0.0)};
  optimizer->validationToReturn = LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;

  Observation observation(1.0, Point3D{Eigen::Vector3d(1.0, 2.0, 3.0)});
  pipeline.processObservation(Observations{observation});

  EXPECT_EQ(optimizer->validateCallCount, 1);
  EXPECT_EQ(optimizer->commitCallCount, 1);
  ASSERT_EQ(optimizer->graph.loopClosureEdges.size(), 1u);
  EXPECT_EQ(optimizer->graph.loopClosureEdges.front().fromKeyframeId, 1);
  EXPECT_EQ(optimizer->graph.loopClosureEdges.front().toKeyframeId, 0);
}

TEST(GraphSlamPipelineTest, FailedLoopValidationDoesNotMutateGraph)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));

  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(41)};
  optimizer->candidatesToReturn = {LoopClosureCandidate(1, 0, 0.2, false, 0.0)};
  optimizer->validationToReturn = LoopClosureValidationResult(false, 1, 4, "rejected", MotionConstraint(), 0.25);
  optimizer->commitReturnValue = true;

  Observation observation(1.0, Point3D{Eigen::Vector3d(1.0, 2.0, 3.0)});
  pipeline.processObservation(Observations{observation});

  EXPECT_EQ(optimizer->validateCallCount, 1);
  EXPECT_EQ(optimizer->commitCallCount, 0);
  EXPECT_TRUE(optimizer->graph.loopClosureEdges.empty());
}

TEST(GraphSlamPipelineTest, ResetClearsLoopClosureEdgesAndGetMapSucceeds)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  // Inject a committed loop closure edge directly into the fake graph.
  optimizer->graph.loopClosureEdges.emplace_back(2, 0, MotionConstraint(), 3, 4, 0.75);
  ASSERT_EQ(optimizer->graph.loopClosureEdges.size(), 1u);

  pipeline.reset();

  EXPECT_TRUE(optimizer->graph.loopClosureEdges.empty());
  EXPECT_NO_THROW(pipeline.getMap());
}

TEST(GraphSlamPipelineTest, MotionAndObservationPathsUnaffectedByLoopClosureAdditions)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  optimizer->candidatesToReturn = {};  // no loop candidates

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));

  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(55)};
  pipeline.processObservation(Observations{Observation(1.0, Point3D{Eigen::Vector3d(1.0, 0.0, 0.0)})});

  ASSERT_EQ(optimizer->graph.keyframes.size(), 3u);
  ASSERT_EQ(optimizer->graph.odometryEdges.size(), 2u);
  ASSERT_EQ(optimizer->graph.observationEdges.size(), 1u);
  EXPECT_TRUE(optimizer->graph.loopClosureEdges.empty());
  EXPECT_EQ(optimizer->validateCallCount, 0);
}

TEST(GraphSlamPipelineTest, MultipleCandidatesAreAllValidatedInSingleCycle)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.3, 0.0, 0.0)));

  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(70)};
  optimizer->candidatesToReturn = {
    LoopClosureCandidate(2, 0, 0.1, false, 0.0),
    LoopClosureCandidate(2, 1, 0.2, false, 0.0)};
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;

  pipeline.processObservation(
    Observations{Observation(1.0, Point3D{Eigen::Vector3d(1.0, 0.0, 0.0)})});

  EXPECT_EQ(optimizer->validateCallCount, 2);
  EXPECT_EQ(optimizer->commitCallCount, 2);
}

TEST(GraphSlamPipelineTest, EndToEndWithRealOptimizerCommitsLoopClosure)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<InternalGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  const Observation dummyObs(1.0, Point3D{Eigen::Vector3d(0.1, 0.3, 0.0)});

  // kf1 at (1.0, 0, 0): introduce three shared landmarks.
  association->assignedToEmit = AssignedMeasurements{
    makeNewLandmarkAssignment(301, Eigen::Vector3d(0.1, 0.3, 0.0)),
    makeNewLandmarkAssignment(302, Eigen::Vector3d(-0.2, 0.4, 0.0)),
    makeNewLandmarkAssignment(303, Eigen::Vector3d(0.3, -0.1, 0.0))};
  pipeline.processMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  pipeline.processObservation(Observations{dummyObs});

  // kf2 at (2.0, 0, 0): revisit landmarks.
  association->assignedToEmit = AssignedMeasurements{
    makeAssignedMeasurement(301),
    makeAssignedMeasurement(302),
    makeAssignedMeasurement(303)};
  pipeline.processMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  pipeline.processObservation(Observations{dummyObs});

  // kf3 at (3.0, 0, 0): revisit landmarks.
  association->assignedToEmit = AssignedMeasurements{
    makeAssignedMeasurement(301),
    makeAssignedMeasurement(302),
    makeAssignedMeasurement(303)};
  pipeline.processMotion(makeMotion(Eigen::Vector3d(1.0, 0.0, 0.0)));
  pipeline.processObservation(Observations{dummyObs});

  // kf4 at (1.1, 0, 0): revisit landmarks.
  // kf4 is 0.1 m from kf1, separation = 3 — spatial candidate expected.
  association->assignedToEmit = AssignedMeasurements{
    makeAssignedMeasurement(301),
    makeAssignedMeasurement(302),
    makeAssignedMeasurement(303)};
  pipeline.processMotion(makeMotion(Eigen::Vector3d(-1.9, 0.0, 0.0)));
  pipeline.processObservation(Observations{dummyObs});

  const GraphState graph = optimizer->getGraphState();
  ASSERT_FALSE(graph.loopClosureEdges.empty())
    << "Expected pipeline to commit a loop closure between kf4 and kf1";
  EXPECT_EQ(graph.loopClosureEdges.front().fromKeyframeId, 4);
  EXPECT_EQ(graph.loopClosureEdges.front().toKeyframeId, 1);
  EXPECT_GE(graph.loopClosureEdges.front().inlierRatio, 0.6);
}

TEST(GraphSlamPipelineTest, FrontendHealthMetricsTrackAssociationAcceptanceAndDrop)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));

  association->assignedToEmit = {};
  pipeline.processObservation(
    Observations{Observation(1.0, Point3D{Eigen::Vector3d(1.0, 0.0, 0.0)})});

  FrontendHealthMetrics first = pipeline.frontendHealthMetrics();
  EXPECT_EQ(first.totalMeasurementAttempts, 1u);
  EXPECT_EQ(first.totalAssociatedMeasurements, 0u);
  EXPECT_EQ(first.totalDroppedMeasurements, 1u);
  EXPECT_DOUBLE_EQ(first.associationAcceptanceRate, 0.0);
  EXPECT_DOUBLE_EQ(first.associationDropRate, 1.0);

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));

  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(77)};
  pipeline.processObservation(
    Observations{Observation(2.0, Point3D{Eigen::Vector3d(1.2, 0.1, 0.0)})});

  FrontendHealthMetrics second = pipeline.frontendHealthMetrics();
  EXPECT_EQ(second.totalMeasurementAttempts, 2u);
  EXPECT_EQ(second.totalAssociatedMeasurements, 1u);
  EXPECT_EQ(second.totalDroppedMeasurements, 1u);
  EXPECT_DOUBLE_EQ(second.associationAcceptanceRate, 0.5);
  EXPECT_DOUBLE_EQ(second.associationDropRate, 0.5);
}

TEST(GraphSlamPipelineTest, RejectedLoopConstraintSpikeDetectedOnSuddenBurst)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  std::vector<LoopClosureCandidate> batch = {
    LoopClosureCandidate(2, 0, 0.1, false, 0.0),
    LoopClosureCandidate(2, 1, 0.1, false, 0.0),
    LoopClosureCandidate(3, 0, 0.1, false, 0.0),
    LoopClosureCandidate(3, 1, 0.1, false, 0.0)};

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(91)};
  optimizer->candidatesToReturn = batch;
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;
  pipeline.processObservation(
    Observations{Observation(1.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.0)})});

  FrontendHealthMetrics baseline = pipeline.frontendHealthMetrics();
  EXPECT_EQ(baseline.rejectionSpikeCount, 0u);
  EXPECT_FALSE(baseline.rejectionSpikeInLastCycle);

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(92)};
  optimizer->candidatesToReturn = batch;
  optimizer->validationToReturn =
    LoopClosureValidationResult(false, 0, 4, "rejected", MotionConstraint(), 0.0);
  pipeline.processObservation(
    Observations{Observation(2.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.2)})});

  FrontendHealthMetrics burst = pipeline.frontendHealthMetrics();
  EXPECT_EQ(burst.totalLoopCandidates, 8u);
  EXPECT_EQ(burst.totalRejectedLoopCandidates, 4u);
  EXPECT_DOUBLE_EQ(burst.loopConstraintRejectRate, 0.5);
  EXPECT_EQ(burst.rejectionSpikeCount, 1u);
  EXPECT_TRUE(burst.rejectionSpikeInLastCycle);
}

TEST(GraphSlamPipelineTest, RejectedLoopConstraintSpikeIgnoredWhenCandidateCountTooLow)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  std::vector<LoopClosureCandidate> smallBatch = {
    LoopClosureCandidate(2, 0, 0.1, false, 0.0)};

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(101)};
  optimizer->candidatesToReturn = smallBatch;
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;
  pipeline.processObservation(
    Observations{Observation(1.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.0)})});

  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(102)};
  optimizer->candidatesToReturn = smallBatch;
  optimizer->validationToReturn =
    LoopClosureValidationResult(false, 0, 4, "rejected", MotionConstraint(), 0.0);
  pipeline.processObservation(
    Observations{Observation(2.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.2)})});

  FrontendHealthMetrics metrics = pipeline.frontendHealthMetrics();
  EXPECT_EQ(metrics.rejectionSpikeCount, 0u);
  EXPECT_FALSE(metrics.rejectionSpikeInLastCycle);
}

TEST(GraphSlamPipelineTest, WatchdogMetricsRecordedWhenOptimizationFires)
{
  // Use a fallback-every-1-keyframe policy so optimization fires on every
  // keyframe processed, making it easy to verify watchdog is wired in.
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  // kOptimizationFallbackEveryNKeyframes is 5.  Drive 5 keyframes to fire it.
  for (int i = 0; i < 5; ++i)
  {
    pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  }

  const OptimizationMetrics m = pipeline.watchdogMetrics();
  EXPECT_GE(m.totalSolves, 1);
  // Fake optimizer returns immediately — no timeouts expected.
  EXPECT_EQ(m.timedOutSolves, 0);
  EXPECT_GE(optimizer->optimizeCallCount, 1);
}

// Helper: drive enough rejected-candidate cycles to trigger a rejection spike.
// The spike detector requires >=4 candidates, >=80% rejected, and a relative
// jump of >=0.35 above the rolling baseline.  We seed one accepted cycle to
// establish the baseline, then fire one fully-rejected cycle with 4 candidates.
static void triggerRejectionSpike(
  GraphSlamPipeline& pipeline,
  PipelineFakeAssociation* association,
  PipelineFakeGraphOptimizer* optimizer)
{
  const std::vector<LoopClosureCandidate> candidates = {
    LoopClosureCandidate(2, 0, 0.1, false, 0.0),
    LoopClosureCandidate(2, 1, 0.1, false, 0.0),
    LoopClosureCandidate(3, 0, 0.1, false, 0.0),
    LoopClosureCandidate(3, 1, 0.1, false, 0.0)};

  // Seed cycle: all accepted — establishes rolling baseline near 0.
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(201)};
  optimizer->candidatesToReturn = candidates;
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;
  pipeline.processObservation(
    Observations{Observation(1.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.0)})});

  // Spike cycle: all rejected — ratio 1.0 against baseline ~0.0.
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(202)};
  optimizer->candidatesToReturn = candidates;
  optimizer->validationToReturn =
    LoopClosureValidationResult(false, 0, 4, "rejected", MotionConstraint(), 0.0);
  optimizer->commitReturnValue = false;
  pipeline.processObservation(
    Observations{Observation(2.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.2)})});
}

TEST(GraphSlamPipelineTest, LoopClosureCooldownEngagesAfterRejectionSpike)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  triggerRejectionSpike(pipeline, association.get(), optimizer.get());

  ASSERT_TRUE(pipeline.frontendHealthMetrics().rejectionSpikeInLastCycle)
    << "Spike must have fired before testing cooldown";
  EXPECT_GT(pipeline.frontendHealthMetrics().loopClosureCooldownRemaining, 0);

  // Reset call count and fire another observation cycle immediately after spike.
  const int validateBefore = optimizer->validateCallCount;
  pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(203)};
  optimizer->candidatesToReturn = {LoopClosureCandidate(3, 0, 0.1, false, 0.0)};
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;
  pipeline.processObservation(
    Observations{Observation(3.0, Point3D{Eigen::Vector3d(0.0, 0.0, 1.5)})});

  EXPECT_EQ(optimizer->validateCallCount, validateBefore)
    << "No loop closure validation should occur while cooldown is active";
}

TEST(GraphSlamPipelineTest, LoopClosureCooldownExpiresAfterNKeyframes)
{
  // kLoopClosureCooldownKeyframes == 5 (internal constant in frontend_health_monitor.cpp)
  constexpr int COOLDOWN_KEYFRAMES = 5;

  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  triggerRejectionSpike(pipeline, association.get(), optimizer.get());

  ASSERT_GT(pipeline.frontendHealthMetrics().loopClosureCooldownRemaining, 0);

  // Drive exactly COOLDOWN_KEYFRAMES motions — no observations needed.
  for (int i = 0; i < COOLDOWN_KEYFRAMES; ++i)
  {
    pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  }

  EXPECT_EQ(pipeline.frontendHealthMetrics().loopClosureCooldownRemaining, 0);

  // A subsequent observation cycle should now attempt loop closures again.
  const int validateBefore = optimizer->validateCallCount;
  association->assignedToEmit = AssignedMeasurements{makeAssignedMeasurement(210)};
  optimizer->candidatesToReturn = {LoopClosureCandidate(5, 0, 0.1, false, 0.0)};
  optimizer->validationToReturn =
    LoopClosureValidationResult(true, 3, 4, "", MotionConstraint(), 0.75);
  optimizer->commitReturnValue = true;
  pipeline.processObservation(
    Observations{Observation(10.0, Point3D{Eigen::Vector3d(0.0, 0.0, 2.0)})});

  EXPECT_GT(optimizer->validateCallCount, validateBefore)
    << "Loop closure validation should resume after cooldown expires";
}

TEST(GraphSlamPipelineTest, OdometryAndRefinementContinueDuringCooldown)
{
  auto association = std::make_shared<PipelineFakeAssociation>();
  auto factory = std::make_shared<MeasurementFactory>();
  auto optimizer = std::make_shared<PipelineFakeGraphOptimizer>();
  auto frontend = std::make_shared<GraphSlamFrontend>(association, factory);
  auto backend = std::make_shared<GraphSlamBackend>(optimizer);

  GraphSlamPipeline pipeline(frontend, backend);
  pipeline.initialize();

  triggerRejectionSpike(pipeline, association.get(), optimizer.get());

  ASSERT_GT(pipeline.frontendHealthMetrics().loopClosureCooldownRemaining, 0);

  const std::size_t keyframesBefore = optimizer->graph.keyframes.size();
  const std::size_t odometryEdgesBefore = optimizer->graph.odometryEdges.size();
  const std::size_t loopEdgesBefore = optimizer->graph.loopClosureEdges.size();

  // Drive 3 more keyframes while in cooldown.
  for (int i = 0; i < 3; ++i)
  {
    pipeline.processMotion(makeMotion(Eigen::Vector3d(0.6, 0.0, 0.0)));
  }

  EXPECT_EQ(optimizer->graph.keyframes.size(), keyframesBefore + 3)
    << "Keyframes must continue to be committed during cooldown";
  EXPECT_EQ(optimizer->graph.odometryEdges.size(), odometryEdgesBefore + 3)
    << "Odometry edges must continue to be added during cooldown";
  EXPECT_EQ(optimizer->graph.loopClosureEdges.size(), loopEdgesBefore)
    << "No additional loop closures should be committed during cooldown";
}

}  // namespace slam
