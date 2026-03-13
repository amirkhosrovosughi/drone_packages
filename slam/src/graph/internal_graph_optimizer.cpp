#include "graph/internal_graph_optimizer.hpp"

#include <algorithm>

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

GraphState InternalGraphOptimizer::getGraphState() const
{
  std::lock_guard<std::mutex> lock(_mutex);
  return _graph;
}

void InternalGraphOptimizer::setLogger(LoggerPtr logger)
{
  _logger = logger;
}

}  // namespace slam
