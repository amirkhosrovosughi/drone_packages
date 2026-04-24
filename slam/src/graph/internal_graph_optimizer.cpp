#include "graph/internal_graph_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace
{
constexpr int kMinSequentialOdometrySupportEdges = 2;
constexpr int kMinLoopSupportCorrespondences = 3;
constexpr int kMinLoopInlierCount = 2;
constexpr double kMinLoopInlierRatio = 0.6;
constexpr double kMaxLoopTranslationResidualMeters = 0.35;
constexpr double kMaxLandmarkCorrespondenceResidualMeters = 0.5;

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
}

void InternalGraphOptimizer::reset()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _graph = GraphState();
  _graph.activeKeyframeId = 0;
  _graph.keyframes.emplace_back(_graph.activeKeyframeId, 0.0, _graph.robot);
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
