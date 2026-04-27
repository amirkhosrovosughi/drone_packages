#include "graph/internal_graph_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace
{
constexpr int kMinSequentialOdometrySupportEdges = 2;
constexpr int kMaxLocalRefinementSupportEdges = 5;
constexpr int kMinLoopSupportCorrespondences = 2;
constexpr int kMinLoopInlierCount = 2;
constexpr double kMinLoopInlierRatio = 0.6;
constexpr double kMaxLoopTranslationResidualMeters = 0.35;
constexpr double kMaxLandmarkCorrespondenceResidualMeters = 0.5;
constexpr double kOdometryConstraintWeight = 1.0;
constexpr double kObservationConstraintWeight = 0.08;
constexpr double kMaxLocalRefinementStepMeters = 0.15;
constexpr double kGlobalAnchorWeight = 100.0;
constexpr double kGlobalLoopClosureBaseWeight = 3.0;

Eigen::Quaterniond poseQuaternionToEigen(const Pose& pose)
{
  return Eigen::Quaterniond(
    pose.quaternion.w,
    pose.quaternion.x,
    pose.quaternion.y,
    pose.quaternion.z);
}
}  // namespace

namespace slam
{

void InternalGraphOptimizer::initialize()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _graph = GraphState();
  _graph.activeKeyframeId = 0;
  _graph.keyframes.emplace_back(_graph.activeKeyframeId, 0.0, _graph.robot);
  _lastRefinementTime = std::chrono::steady_clock::now();
}

void InternalGraphOptimizer::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _graph = GraphState();
  _graph.activeKeyframeId = 0;
  _graph.keyframes.emplace_back(_graph.activeKeyframeId, 0.0, _graph.robot);
  _lastRefinementTime = std::chrono::steady_clock::now();
}

void InternalGraphOptimizer::applyMotion(const MotionConstraint& motion)
{
  std::lock_guard<std::mutex> lock(_mutex);

  if (_graph.keyframes.empty())
  {
    _graph.activeKeyframeId = 0;
    _graph.keyframes.emplace_back(_graph.activeKeyframeId, 0.0, _graph.robot);
  }

  const int fromKeyframeId = _graph.activeKeyframeId;

  _graph.robot.pose.position.x += motion.delta_position.x();
  _graph.robot.pose.position.y += motion.delta_position.y();
  _graph.robot.pose.position.z += motion.delta_position.z();
  _graph.robot.pose.quaternion.w = motion.orientation.w();
  _graph.robot.pose.quaternion.x = motion.orientation.x();
  _graph.robot.pose.quaternion.y = motion.orientation.y();
  _graph.robot.pose.quaternion.z = motion.orientation.z();

  const int toKeyframeId = fromKeyframeId + 1;
  _graph.keyframes.emplace_back(toKeyframeId, 0.0, _graph.robot);
  _graph.odometryEdges.emplace_back(fromKeyframeId, toKeyframeId, motion);
  _graph.activeKeyframeId = toKeyframeId;
}

void InternalGraphOptimizer::applyObservation(
  const AssignedMeasurements& measurements)
{
  std::lock_guard<std::mutex> lock(_mutex);

  for (const auto& assigned : measurements)
  {
    _graph.observationEdges.emplace_back(_graph.activeKeyframeId, assigned.id);

    if (assigned.isNew && assigned.hasInitializedPosition)
    {
      const auto it = std::find_if(
        _graph.landmarks.begin(),
        _graph.landmarks.end(),
        [&](const GraphLandmarkNode& lm) { return lm.id == assigned.id; });

      if (it == _graph.landmarks.end())
      {
        GraphLandmarkNode lm;
        lm.id = assigned.id;
        lm.position = assigned.position;
        lm.observeRepeat = 1;
        _graph.landmarks.push_back(lm);
      }
      else
      {
        it->observeRepeat += 1;
      }
      continue;
    }

    auto it = std::find_if(
      _graph.landmarks.begin(),
      _graph.landmarks.end(),
      [&](const GraphLandmarkNode& lm) { return lm.id == assigned.id; });
    if (it != _graph.landmarks.end())
    {
      it->observeRepeat += 1;
    }
  }
}

std::vector<LoopClosureCandidate> InternalGraphOptimizer::findSpatialLoopClosureCandidates(
  double maxDistanceMeters,
  int minKeyframeSeparation) const
{
  std::lock_guard<std::mutex> lock(_mutex);

  std::vector<LoopClosureCandidate> candidates;
  if (maxDistanceMeters <= 0.0 || _graph.keyframes.size() < 2u)
  {
    return candidates;
  }

  const int sourceKeyframeId = _graph.activeKeyframeId;
  const GraphKeyframeNode* sourceKeyframe = findKeyframeById(sourceKeyframeId);
  if (!sourceKeyframe)
  {
    return candidates;
  }

  const Eigen::Vector3d sourcePosition = sourceKeyframe->robot.pose.position.getPositionVector();

  for (const auto& targetKeyframe : _graph.keyframes)
  {
    if (targetKeyframe.id == sourceKeyframeId)
    {
      continue;
    }

    if (std::abs(sourceKeyframeId - targetKeyframe.id) < std::max(1, minKeyframeSeparation))
    {
      continue;
    }

    if (loopClosureExists(sourceKeyframeId, targetKeyframe.id))
    {
      continue;
    }

    const double spatialDistance =
      (sourcePosition - targetKeyframe.robot.pose.position.getPositionVector()).norm();
    if (spatialDistance <= maxDistanceMeters)
    {
      candidates.emplace_back(
        sourceKeyframeId,
        targetKeyframe.id,
        spatialDistance,
        false,
        0.0);
    }
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const LoopClosureCandidate& a, const LoopClosureCandidate& b)
    {
      return a.spatialDistanceMeters < b.spatialDistanceMeters;
    });

  return candidates;
}


    void InternalGraphOptimizer::refineActiveKeyframe(const OptimizationConfig& config)
    {
      std::lock_guard<std::mutex> lock(_mutex);

      if (_graph.keyframes.empty())
      {
        if (_logger) _logger->logWarn("refineActiveKeyframe: No keyframes in graph");
        return;
      }

      // Dispatch based on refinement strategy
      if (config.strategy == RefinementStrategy::GaussNewton)
      {
        refineActiveKeyframeGaussNewton(config);
      }
      else if (config.strategy == RefinementStrategy::ClosedForm)
      {
        refineActiveKeyframeClosedForm(config);
      }
      else
      {
        if (_logger) _logger->logWarn("refineActiveKeyframe: Unknown refinement strategy");
      }
    }

void InternalGraphOptimizer::refineActiveKeyframeGaussNewton(const OptimizationConfig& config)
    {
      // Find active keyframe (mutable)
      auto* activeKf = findKeyframeByIdMutable(_graph.activeKeyframeId);
      if (!activeKf)
      {
        if (_logger) _logger->logWarn("refineActiveKeyframe: Active keyframe not found");
        return;
      }

      std::vector<GraphObservationEdge> observations;
      for (const auto& edge : _graph.observationEdges)
      {
        if (edge.keyframeId == _graph.activeKeyframeId)
        {
          observations.push_back(edge);
        }
      }

      if (observations.empty())
      {
        if (_logger)
          _logger->logDebug("refineActiveKeyframe: No observations for active keyframe; skipping refinement");
        return;
      }

      Eigen::VectorXd accumulatedDelta = Eigen::VectorXd::Zero(6);
      for (int iter = 0; iter < config.maxIterations; ++iter)
      {
        Eigen::VectorXd delta = computeLocalPoseRefinement(_graph.activeKeyframeId);
        if (delta.size() != 6)
        {
          break;
        }

        if (!delta.allFinite())
        {
          if (_logger)
          {
            _logger->logWarn("refineActiveKeyframe: Non-finite delta detected; aborting iteration");
          }
          break;
        }

        if (delta.norm() < config.convergeThreshold)
        {
          if (_logger)
          {
            _logger->logDebug("refineActiveKeyframe: Converged at iteration " +
                           std::to_string(iter) + ", delta_norm=" + std::to_string(delta.norm()));
          }
          break;
        }

        accumulatedDelta += delta;

        // Update active keyframe translation after each local step.
        activeKf->robot.pose.position.x += delta(0);
        activeKf->robot.pose.position.y += delta(1);
        activeKf->robot.pose.position.z += delta(2);

        // Keep graph.robot synchronized with active keyframe state.
        _graph.robot = activeKf->robot;
      }

      // Apply refinement: orientation update (small-angle approximation)
      Eigen::Vector3d angleAxis(accumulatedDelta(3), accumulatedDelta(4), accumulatedDelta(5));
      if (angleAxis.norm() > 1e-10)  // Avoid singular case
      {
        Eigen::Quaterniond deltaQuat(Eigen::AngleAxisd(angleAxis.norm(), angleAxis.normalized()));
        Eigen::Quaterniond currentQuat(activeKf->robot.pose.quaternion.w,
                                       activeKf->robot.pose.quaternion.x,
                                       activeKf->robot.pose.quaternion.y,
                                       activeKf->robot.pose.quaternion.z);
        Eigen::Quaterniond refinedQuat = deltaQuat * currentQuat;
        refinedQuat.normalize();

        activeKf->robot.pose.quaternion = Quaternion(refinedQuat.w(), refinedQuat.x(),
                                                     refinedQuat.y(), refinedQuat.z());
        _graph.robot.pose.quaternion = activeKf->robot.pose.quaternion;
      }

      _lastRefinementTime = std::chrono::steady_clock::now();

      if (_logger)
      {
        _logger->logDebug("refineActiveKeyframe: Gauss-Newton complete, delta_norm=" +
                       std::to_string(accumulatedDelta.norm()));
      }
    }

    void InternalGraphOptimizer::refineActiveKeyframeClosedForm(const OptimizationConfig& config)
    {
      // Placeholder for closed-form refinement
      // Currently just logs that it's not implemented
      (void)config;  // Silence unused parameter warning
      if (_logger)
      {
        _logger->logDebug("refineActiveKeyframe: Closed-form strategy not yet implemented; skipping");
      }
    }

Eigen::VectorXd InternalGraphOptimizer::computeLocalPoseRefinement(int keyframeId)
    {
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(6);
      GraphKeyframeNode* activeKf = findKeyframeByIdMutable(keyframeId);
      if (!activeKf)
      {
        return delta;
      }

      const Eigen::Vector3d activePosition = activeKf->robot.pose.position.getPositionVector();

      // Build local normal equations in translation only (3x3).
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Identity() * 1e-6;
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();

      // Odometry anchors from recent predecessors to active keyframe.
      int supportUsed = 0;
      for (auto keyframeIt = _graph.keyframes.rbegin();
        keyframeIt != _graph.keyframes.rend() && supportUsed < kMaxLocalRefinementSupportEdges;
        ++keyframeIt)
      {
        const int anchorId = keyframeIt->id;
        if (anchorId == keyframeId)
        {
          continue;
        }

        Eigen::Vector3d cumulativeDelta = Eigen::Vector3d::Zero();
        int edgeCount = 0;
        if (!accumulateSequentialOdometryDelta(anchorId, keyframeId, &cumulativeDelta, &edgeCount))
        {
          continue;
        }

        const GraphKeyframeNode* anchor = findKeyframeById(anchorId);
        if (!anchor)
        {
          continue;
        }

        const Eigen::Vector3d predictedActive =
          anchor->robot.pose.position.getPositionVector() + cumulativeDelta;
        const Eigen::Vector3d residual = activePosition - predictedActive;

        hessian += kOdometryConstraintWeight * Eigen::Matrix3d::Identity();
        rhs += kOdometryConstraintWeight * residual;
        ++supportUsed;
      }

      // Observation anchors from landmarks observed at this keyframe.
      int observationSupport = 0;
      for (const auto& observation : _graph.observationEdges)
      {
        if (observation.keyframeId != keyframeId)
        {
          continue;
        }

        const GraphLandmarkNode* landmark = findLandmarkById(observation.landmarkId);
        if (!landmark)
        {
          continue;
        }

        const Eigen::Vector3d landmarkPos = landmark->position.getPositionVector();
        const Eigen::Vector3d residual = activePosition - landmarkPos;

        hessian += kObservationConstraintWeight * Eigen::Matrix3d::Identity();
        rhs += kObservationConstraintWeight * residual;
        ++observationSupport;
      }

      if (supportUsed == 0 && observationSupport == 0)
      {
        return delta;
      }

      Eigen::Vector3d translationStep =
        -hessian.ldlt().solve(rhs);

      if (!translationStep.allFinite())
      {
        return delta;
      }

      // Clamp local translation step for stability.
      const double stepNorm = translationStep.norm();
      if (stepNorm > kMaxLocalRefinementStepMeters)
      {
        translationStep = translationStep * (kMaxLocalRefinementStepMeters / stepNorm);
      }

      delta(0) = translationStep.x();
      delta(1) = translationStep.y();
      delta(2) = translationStep.z();
      return delta;
    }

    GraphKeyframeNode* InternalGraphOptimizer::findKeyframeByIdMutable(int keyframeId)
    {
      auto it = std::find_if(
        _graph.keyframes.begin(),
        _graph.keyframes.end(),
        [keyframeId](const GraphKeyframeNode& keyframe)
        {
          return keyframe.id == keyframeId;
        });
      return it != _graph.keyframes.end() ? &(*it) : nullptr;
    }

// ---------------------------------------------------------------------------
// Global optimisation helpers (all called while _mutex is already held)
// ---------------------------------------------------------------------------

double InternalGraphOptimizer::computeMaxNodeStep(
  const Eigen::VectorXd& delta, int numKeyframes)
{
  double maxStep = 0.0;
  for (int i = 0; i < numKeyframes; ++i)
  {
    maxStep = std::max(maxStep, delta.segment<3>(i * 3).norm());
  }
  return maxStep;
}

std::unordered_map<int, int> InternalGraphOptimizer::buildKeyframeIdToIndex() const
{
  std::unordered_map<int, int> idToIndex;
  idToIndex.reserve(_graph.keyframes.size());
  for (int i = 0; i < static_cast<int>(_graph.keyframes.size()); ++i)
  {
    idToIndex[_graph.keyframes[i].id] = i;
  }
  return idToIndex;
}

void InternalGraphOptimizer::buildSystemMatrices(
  const std::unordered_map<int, int>& idToIndex,
  const Eigen::Vector3d& anchorPosition,
  Eigen::MatrixXd& hessian,
  Eigen::VectorXd& gradient,
  int& constraintCount) const
{
  // Gauge-freedom anchor: pin the first keyframe to its initial pose.
  {
    const Eigen::Vector3d currentAnchor =
      _graph.keyframes.front().robot.pose.position.getPositionVector();
    const Eigen::Vector3d residual = currentAnchor - anchorPosition;
    hessian.block<3, 3>(0, 0).diagonal().array() += kGlobalAnchorWeight;
    gradient.segment<3>(0) += kGlobalAnchorWeight * residual;
    ++constraintCount;
  }

  // Odometry constraints.
  for (const auto& edge : _graph.odometryEdges)
  {
    const auto fromIt = idToIndex.find(edge.fromKeyframeId);
    const auto toIt   = idToIndex.find(edge.toKeyframeId);
    if (fromIt == idToIndex.end() || toIt == idToIndex.end())
    {
      continue;
    }

    const int fromRow = fromIt->second * 3;
    const int toRow   = toIt->second * 3;

    const Eigen::Vector3d fromPos =
      _graph.keyframes[fromIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d toPos =
      _graph.keyframes[toIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d residual = (toPos - fromPos) - edge.motion.delta_position;

    hessian.block<3, 3>(fromRow, fromRow).diagonal().array() += kOdometryConstraintWeight;
    hessian.block<3, 3>(toRow,   toRow  ).diagonal().array() += kOdometryConstraintWeight;
    hessian.block<3, 3>(fromRow, toRow  ).diagonal().array() -= kOdometryConstraintWeight;
    hessian.block<3, 3>(toRow,   fromRow).diagonal().array() -= kOdometryConstraintWeight;

    gradient.segment<3>(fromRow) += -kOdometryConstraintWeight * residual;
    gradient.segment<3>(toRow)   +=  kOdometryConstraintWeight * residual;
    ++constraintCount;
  }

  // Loop-closure constraints (weighted by inlier quality).
  for (const auto& edge : _graph.loopClosureEdges)
  {
    const auto fromIt = idToIndex.find(edge.fromKeyframeId);
    const auto toIt   = idToIndex.find(edge.toKeyframeId);
    if (fromIt == idToIndex.end() || toIt == idToIndex.end())
    {
      continue;
    }

    const int fromRow = fromIt->second * 3;
    const int toRow   = toIt->second * 3;

    const Eigen::Vector3d fromPos =
      _graph.keyframes[fromIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d toPos =
      _graph.keyframes[toIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d residual =
      (toPos - fromPos) - edge.relativeMotion.delta_position;

    const double supportScale =
      std::min(1.0, std::max(0.0, static_cast<double>(edge.supportCount) / 5.0));
    const double ratioScale = std::max(0.1, edge.inlierRatio);
    const double loopWeight = kGlobalLoopClosureBaseWeight * supportScale * ratioScale;

    hessian.block<3, 3>(fromRow, fromRow).diagonal().array() += loopWeight;
    hessian.block<3, 3>(toRow,   toRow  ).diagonal().array() += loopWeight;
    hessian.block<3, 3>(fromRow, toRow  ).diagonal().array() -= loopWeight;
    hessian.block<3, 3>(toRow,   fromRow).diagonal().array() -= loopWeight;

    gradient.segment<3>(fromRow) += -loopWeight * residual;
    gradient.segment<3>(toRow)   +=  loopWeight * residual;
    ++constraintCount;
  }
}

void InternalGraphOptimizer::applyKeyframeDelta(const Eigen::VectorXd& delta)
{
  const int numKeyframes = static_cast<int>(_graph.keyframes.size());
  for (int i = 0; i < numKeyframes; ++i)
  {
    const int row = i * 3;
    _graph.keyframes[i].robot.pose.position.x += delta(row + 0);
    _graph.keyframes[i].robot.pose.position.y += delta(row + 1);
    _graph.keyframes[i].robot.pose.position.z += delta(row + 2);
  }
}

double InternalGraphOptimizer::computeWeightedResidual(
  const std::unordered_map<int, int>& idToIndex) const
{
  double totalError = 0.0;

  for (const auto& edge : _graph.odometryEdges)
  {
    const auto fromIt = idToIndex.find(edge.fromKeyframeId);
    const auto toIt   = idToIndex.find(edge.toKeyframeId);
    if (fromIt == idToIndex.end() || toIt == idToIndex.end())
    {
      continue;
    }
    const Eigen::Vector3d fromPos =
      _graph.keyframes[fromIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d toPos =
      _graph.keyframes[toIt->second].robot.pose.position.getPositionVector();
    totalError +=
      kOdometryConstraintWeight * ((toPos - fromPos) - edge.motion.delta_position).squaredNorm();
  }

  for (const auto& edge : _graph.loopClosureEdges)
  {
    const auto fromIt = idToIndex.find(edge.fromKeyframeId);
    const auto toIt   = idToIndex.find(edge.toKeyframeId);
    if (fromIt == idToIndex.end() || toIt == idToIndex.end())
    {
      continue;
    }
    const Eigen::Vector3d fromPos =
      _graph.keyframes[fromIt->second].robot.pose.position.getPositionVector();
    const Eigen::Vector3d toPos =
      _graph.keyframes[toIt->second].robot.pose.position.getPositionVector();
    const double supportScale =
      std::min(1.0, std::max(0.0, static_cast<double>(edge.supportCount) / 5.0));
    const double loopWeight =
      kGlobalLoopClosureBaseWeight * supportScale * std::max(0.1, edge.inlierRatio);
    totalError +=
      loopWeight * ((toPos - fromPos) - edge.relativeMotion.delta_position).squaredNorm();
  }

  return totalError;
}

bool InternalGraphOptimizer::optimizeGraph(
  const OptimizationConfig& config,
  OptimizationResult* resultOut)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto solveStart = std::chrono::steady_clock::now();
  OptimizationResult result;

  const int numKeyframes = static_cast<int>(_graph.keyframes.size());
  if (numKeyframes < 2)
  {
    result.failureReason = "Insufficient keyframes for global optimization";
    result.numPosesRefined = numKeyframes;
    if (_logger) _logger->logDebug("optimizeGraph: " + result.failureReason);
    if (resultOut) *resultOut = result;
    return false;
  }

  const auto idToIndex       = buildKeyframeIdToIndex();
  const int dof              = numKeyframes * 3;
  const Eigen::Vector3d anchorPosition =
    _graph.keyframes.front().robot.pose.position.getPositionVector();

  bool solved = false;
  int  performedIterations = 0;

  for (int iter = 0; iter < std::max(1, config.maxIterations); ++iter)
  {
    Eigen::MatrixXd hessian  = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dof);
    int constraintCount = 0;

    buildSystemMatrices(idToIndex, anchorPosition, hessian, gradient, constraintCount);

    if (constraintCount <= 1)
    {
      result.failureReason = "Insufficient graph constraints for global optimization";
      result.numIterations = iter;
      break;
    }

    const Eigen::VectorXd delta = hessian.ldlt().solve(-gradient);
    if (!delta.allFinite())
    {
      result.failureReason = "Global solver produced non-finite update";
      result.numIterations = iter;
      break;
    }

    applyKeyframeDelta(delta);
    solved               = true;
    performedIterations  = iter + 1;

    if (computeMaxNodeStep(delta, numKeyframes) < config.convergeThreshold)
    {
      break;
    }
  }

  if (solved)
  {
    const auto activeIt = idToIndex.find(_graph.activeKeyframeId);
    if (activeIt != idToIndex.end())
    {
      _graph.robot = _graph.keyframes[activeIt->second].robot;
    }

    result.success        = true;
    result.numIterations  = performedIterations;
    result.numPosesRefined = numKeyframes;
    result.finalError     = computeWeightedResidual(idToIndex);
  }

  const auto solveEnd = std::chrono::steady_clock::now();
  result.solveTimeMs = static_cast<int>(
    std::chrono::duration_cast<std::chrono::milliseconds>(solveEnd - solveStart).count());

  if (_logger)
  {
    if (result.success)
    {
      _logger->logDebug(
        "optimizeGraph: success=true posesRefined=" + std::to_string(result.numPosesRefined) +
        " iterations=" + std::to_string(result.numIterations) +
        " finalError=" + std::to_string(result.finalError));
    }
    else
    {
      _logger->logDebug("optimizeGraph: " + result.failureReason);
    }
  }

  if (resultOut) *resultOut = result;
  return result.success;
}

LoopClosureValidationResult InternalGraphOptimizer::validateLoopClosureCandidate(
  const LoopClosureCandidate& candidate) const
{
  std::lock_guard<std::mutex> lock(_mutex);

  if (candidate.sourceKeyframeId == candidate.targetKeyframeId)
  {
    return LoopClosureValidationResult(false, 0, 0, "Self loop closure is invalid");
  }

  if (std::abs(candidate.sourceKeyframeId - candidate.targetKeyframeId) <= 1)
  {
    return LoopClosureValidationResult(false, 0, 0, "Consecutive keyframes are not loop closures");
  }

  if (!keyframeIdExists(candidate.sourceKeyframeId) ||
      !keyframeIdExists(candidate.targetKeyframeId))
  {
    return LoopClosureValidationResult(false, 0, 0, "Loop-closure candidate references unknown keyframe");
  }

  const GraphKeyframeNode* sourceKeyframe = findKeyframeById(candidate.sourceKeyframeId);
  const GraphKeyframeNode* targetKeyframe = findKeyframeById(candidate.targetKeyframeId);
  if (!sourceKeyframe || !targetKeyframe)
  {
    return LoopClosureValidationResult(false, 0, 0, "Loop-closure keyframes not found in graph");
  }

  Eigen::Vector3d integratedOdometryDelta = Eigen::Vector3d::Zero();
  int supportCount = 0;
  if (!accumulateSequentialOdometryDelta(
        candidate.targetKeyframeId,
        candidate.sourceKeyframeId,
        &integratedOdometryDelta,
        &supportCount))
  {
    return LoopClosureValidationResult(false, 0, 0, "Missing sequential odometry support for candidate");
  }

  const Eigen::Vector3d sourcePosition = sourceKeyframe->robot.pose.position.getPositionVector();
  const Eigen::Vector3d targetPosition = targetKeyframe->robot.pose.position.getPositionVector();
  const Eigen::Vector3d geometricRelativeDelta = sourcePosition - targetPosition;

  const double translationResidual =
    (geometricRelativeDelta - integratedOdometryDelta).norm();
  MotionConstraint estimatedRelativeMotion;
  estimatedRelativeMotion.delta_position = geometricRelativeDelta;
  estimatedRelativeMotion.orientation = poseQuaternionToEigen(sourceKeyframe->robot.pose);

  if (supportCount < kMinSequentialOdometrySupportEdges)
  {
    return LoopClosureValidationResult(
      false,
      0,
      supportCount,
      "Insufficient odometry support for loop candidate",
      estimatedRelativeMotion);
  }

  if (translationResidual > kMaxLoopTranslationResidualMeters)
  {
    return LoopClosureValidationResult(
      false,
      0,
      supportCount,
      "Relative transform consistency check failed",
      estimatedRelativeMotion);
  }

  const std::vector<int> commonLandmarkIds =
    collectCommonObservedLandmarkIds(candidate.sourceKeyframeId, candidate.targetKeyframeId);
  const int correspondenceSupportCount = static_cast<int>(commonLandmarkIds.size());
  if (correspondenceSupportCount < kMinLoopSupportCorrespondences)
  {
    return LoopClosureValidationResult(
      false,
      0,
      correspondenceSupportCount,
      "Insufficient shared landmark correspondences",
      estimatedRelativeMotion);
  }

  Eigen::Quaterniond qWorldSource = poseQuaternionToEigen(sourceKeyframe->robot.pose);
  Eigen::Quaterniond qWorldTarget = poseQuaternionToEigen(targetKeyframe->robot.pose);
  qWorldSource.normalize();
  qWorldTarget.normalize();

  const Eigen::Matrix3d RWorldSource = qWorldSource.toRotationMatrix();
  const Eigen::Matrix3d RWorldTarget = qWorldTarget.toRotationMatrix();
  const Eigen::Matrix3d RSourceWorld = RWorldSource.transpose();
  const Eigen::Matrix3d RSourceTarget = RSourceWorld * RWorldTarget;
  const Eigen::Vector3d tSourceTarget = RSourceWorld * (targetPosition - sourcePosition);

  int inlierCount = 0;
  for (const int landmarkId : commonLandmarkIds)
  {
    const GraphLandmarkNode* landmark = findLandmarkById(landmarkId);
    if (!landmark)
    {
      continue;
    }

    const Eigen::Vector3d landmarkWorld = landmark->position.getPositionVector();
    const Eigen::Vector3d landmarkInSource =
      RSourceWorld * (landmarkWorld - sourcePosition);
    const Eigen::Vector3d landmarkInTarget =
      RWorldTarget.transpose() * (landmarkWorld - targetPosition);

    const Eigen::Vector3d predictedSourceFromTarget =
      RSourceTarget * landmarkInTarget + tSourceTarget;
    const double correspondenceResidual =
      (predictedSourceFromTarget - landmarkInSource).norm();

    if (correspondenceResidual <= kMaxLandmarkCorrespondenceResidualMeters)
    {
      inlierCount += 1;
    }
  }

  const double inlierRatio =
    static_cast<double>(inlierCount) / static_cast<double>(correspondenceSupportCount);

  if (inlierCount < kMinLoopInlierCount)
  {
    return LoopClosureValidationResult(
      false,
      inlierCount,
      correspondenceSupportCount,
      "Minimum inlier count not met",
      estimatedRelativeMotion,
      inlierRatio);
  }

  if (inlierRatio < kMinLoopInlierRatio)
  {
    return LoopClosureValidationResult(
      false,
      inlierCount,
      correspondenceSupportCount,
      "Inlier ratio threshold not met",
      estimatedRelativeMotion,
      inlierRatio);
  }

  return LoopClosureValidationResult(
    true,
    inlierCount,
    correspondenceSupportCount,
    "",
    estimatedRelativeMotion,
    inlierRatio);
}

bool InternalGraphOptimizer::commitLoopClosure(
  const LoopClosureCandidate& candidate,
  const LoopClosureValidationResult& validation)
{
  std::lock_guard<std::mutex> lock(_mutex);

  if (!validation.accepted)
  {
    return false;
  }

  if (!keyframeIdExists(candidate.sourceKeyframeId) ||
      !keyframeIdExists(candidate.targetKeyframeId))
  {
    return false;
  }

  if (loopClosureExists(candidate.sourceKeyframeId, candidate.targetKeyframeId))
  {
    return false;
  }

  _graph.loopClosureEdges.emplace_back(
    candidate.sourceKeyframeId,
    candidate.targetKeyframeId,
    validation.estimatedRelativeMotion,
    validation.inlierCount,
    validation.supportCount,
    validation.inlierRatio);

  return true;
}

GraphState InternalGraphOptimizer::getGraphState() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _graph;
}

void InternalGraphOptimizer::setLogger(LoggerPtr logger)
{
  _logger = logger;
}

bool InternalGraphOptimizer::keyframeIdExists(int keyframeId) const
{
  const auto it = std::find_if(
    _graph.keyframes.begin(),
    _graph.keyframes.end(),
    [keyframeId](const GraphKeyframeNode& keyframe)
    {
      return keyframe.id == keyframeId;
    });
  return it != _graph.keyframes.end();
}

const GraphKeyframeNode* InternalGraphOptimizer::findKeyframeById(int keyframeId) const
{
  const auto it = std::find_if(
    _graph.keyframes.begin(),
    _graph.keyframes.end(),
    [keyframeId](const GraphKeyframeNode& keyframe)
    {
      return keyframe.id == keyframeId;
    });
  return it != _graph.keyframes.end() ? &(*it) : nullptr;
}

bool InternalGraphOptimizer::loopClosureExists(int keyframeAId, int keyframeBId) const
{
  const auto it = std::find_if(
    _graph.loopClosureEdges.begin(),
    _graph.loopClosureEdges.end(),
    [keyframeAId, keyframeBId](const GraphLoopClosureEdge& edge)
    {
      return (edge.fromKeyframeId == keyframeAId && edge.toKeyframeId == keyframeBId) ||
             (edge.fromKeyframeId == keyframeBId && edge.toKeyframeId == keyframeAId);
    });
  return it != _graph.loopClosureEdges.end();
}

bool InternalGraphOptimizer::accumulateSequentialOdometryDelta(
  int fromKeyframeId,
  int toKeyframeId,
  Eigen::Vector3d* deltaOut,
  int* edgeCountOut) const
{
  if (!deltaOut || !edgeCountOut)
  {
    return false;
  }

  *deltaOut = Eigen::Vector3d::Zero();
  *edgeCountOut = 0;

  if (fromKeyframeId == toKeyframeId)
  {
    return false;
  }

  const int lowerId = std::min(fromKeyframeId, toKeyframeId);
  const int upperId = std::max(fromKeyframeId, toKeyframeId);

  Eigen::Vector3d accumulatedForward = Eigen::Vector3d::Zero();
  for (int keyframeId = lowerId + 1; keyframeId <= upperId; ++keyframeId)
  {
    const auto edgeIt = std::find_if(
      _graph.odometryEdges.begin(),
      _graph.odometryEdges.end(),
      [keyframeId](const GraphOdometryEdge& edge)
      {
        return edge.fromKeyframeId == (keyframeId - 1) &&
               edge.toKeyframeId == keyframeId;
      });

    if (edgeIt == _graph.odometryEdges.end())
    {
      *deltaOut = Eigen::Vector3d::Zero();
      *edgeCountOut = 0;
      return false;
    }

    accumulatedForward += edgeIt->motion.delta_position;
    *edgeCountOut += 1;
  }

  *deltaOut = (fromKeyframeId <= toKeyframeId)
    ? accumulatedForward
    : -accumulatedForward;
  return true;
}

const GraphLandmarkNode* InternalGraphOptimizer::findLandmarkById(int landmarkId) const
{
  const auto it = std::find_if(
    _graph.landmarks.begin(),
    _graph.landmarks.end(),
    [landmarkId](const GraphLandmarkNode& landmark)
    {
      return landmark.id == landmarkId;
    });
  return it != _graph.landmarks.end() ? &(*it) : nullptr;
}

std::vector<int> InternalGraphOptimizer::collectCommonObservedLandmarkIds(
  int keyframeAId,
  int keyframeBId) const
{
  std::unordered_set<int> observedInA;
  std::unordered_set<int> observedInB;

  for (const auto& edge : _graph.observationEdges)
  {
    if (edge.keyframeId == keyframeAId)
    {
      observedInA.insert(edge.landmarkId);
    }
    else if (edge.keyframeId == keyframeBId)
    {
      observedInB.insert(edge.landmarkId);
    }
  }

  std::vector<int> common;
  common.reserve(std::min(observedInA.size(), observedInB.size()));
  for (const int landmarkId : observedInA)
  {
    if (observedInB.find(landmarkId) != observedInB.end())
    {
      common.push_back(landmarkId);
    }
  }
  return common;
}

}  // namespace slam
