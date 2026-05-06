#ifndef SLAM__COMMON__DEF_SLAM_GRAPH_HPP_
#define SLAM__COMMON__DEF_SLAM_GRAPH_HPP_

#include <string>
#include <vector>

#include "common/def_slam_core.hpp"
#include "common/def_slam_optimization.hpp"

struct GraphKeyframeNode {
    int id;
    double timeTag;
    RobotState robot;

    GraphKeyframeNode(int id = 0, double timeTag = 0.0, const RobotState& robot = RobotState())
        : id(id), timeTag(timeTag), robot(robot) {}
};

struct GraphLandmarkNode {
    int id;
    Position position;
    Variance2D variance;
    int observeRepeat;

    GraphLandmarkNode(int id = 0,
                      const Position& position = Position(),
                      const Variance2D& variance = Variance2D(),
                      int observeRepeat = 0)
        : id(id), position(position), variance(variance), observeRepeat(observeRepeat) {}
};

struct GraphOdometryEdge {
    int fromKeyframeId;
    int toKeyframeId;
    MotionConstraint motion;

    GraphOdometryEdge(int fromKeyframeId = 0,
                      int toKeyframeId = 0,
                      const MotionConstraint& motion = MotionConstraint())
        : fromKeyframeId(fromKeyframeId), toKeyframeId(toKeyframeId), motion(motion) {}
};

struct GraphObservationEdge {
    int keyframeId;
    int landmarkId;

    GraphObservationEdge(int keyframeId = 0, int landmarkId = 0)
        : keyframeId(keyframeId), landmarkId(landmarkId) {}
};

struct LoopClosureCandidate {
    int sourceKeyframeId;
    int targetKeyframeId;
    double spatialDistanceMeters;
    bool hasAppearanceScore;
    double appearanceScore;

    LoopClosureCandidate(
        int sourceKeyframeId = 0,
        int targetKeyframeId = 0,
        double spatialDistanceMeters = 0.0,
        bool hasAppearanceScore = false,
        double appearanceScore = 0.0)
        : sourceKeyframeId(sourceKeyframeId),
          targetKeyframeId(targetKeyframeId),
          spatialDistanceMeters(spatialDistanceMeters),
          hasAppearanceScore(hasAppearanceScore),
          appearanceScore(appearanceScore) {}
};

struct LoopClosureValidationResult {
    bool accepted;
    int inlierCount;
    int supportCount;
    double inlierRatio;
    std::string reason;
    MotionConstraint estimatedRelativeMotion;

    LoopClosureValidationResult(
        bool accepted = false,
        int inlierCount = 0,
        int supportCount = 0,
        const std::string& reason = "",
                const MotionConstraint& estimatedRelativeMotion = MotionConstraint(),
                double inlierRatio = 0.0)
        : accepted(accepted),
          inlierCount(inlierCount),
          supportCount(supportCount),
          inlierRatio(inlierRatio),
          reason(reason),
          estimatedRelativeMotion(estimatedRelativeMotion) {}
};

struct GraphLoopClosureEdge {
    int fromKeyframeId;
    int toKeyframeId;
    MotionConstraint relativeMotion;
    int inlierCount;
    int supportCount;
    double inlierRatio;

    GraphLoopClosureEdge(
        int fromKeyframeId = 0,
        int toKeyframeId = 0,
        const MotionConstraint& relativeMotion = MotionConstraint(),
        int inlierCount = 0,
        int supportCount = 0,
        double inlierRatio = 0.0)
        : fromKeyframeId(fromKeyframeId),
          toKeyframeId(toKeyframeId),
          relativeMotion(relativeMotion),
          inlierCount(inlierCount),
          supportCount(supportCount),
          inlierRatio(inlierRatio) {}
};

/// Absolute position prior tied to a single keyframe.
/// Sensor-agnostic: produced by GPS, UWB, or any absolute localization source.
struct GraphGpsPriorEdge {
    int keyframeId = 0;
    AbsolutePositionConstraint constraint;

    GraphGpsPriorEdge() = default;
    GraphGpsPriorEdge(int keyframeId, const AbsolutePositionConstraint& constraint)
        : keyframeId(keyframeId), constraint(constraint) {}
};

struct GraphState {
    RobotState robot;
    std::vector<GraphKeyframeNode> keyframes;
    std::vector<GraphLandmarkNode> landmarks;
    std::vector<GraphOdometryEdge> odometryEdges;
    std::vector<GraphObservationEdge> observationEdges;
    std::vector<GraphLoopClosureEdge> loopClosureEdges;
    std::vector<GraphGpsPriorEdge> gpsPriorEdges;
    int activeKeyframeId = 0;

    GraphState() {}

    bool isValid() const { return true; }
};

inline MapSummary graphStateToMapSummary(const GraphState& graph)
{
    MapSummary summary;
    summary.robot = graph.robot;

    summary.landmarks.reserve(graph.landmarks.size());
    for (const auto& lm : graph.landmarks)
    {
        Landmark summaryLandmark;
        summaryLandmark.id = lm.id;
        summaryLandmark.position = lm.position;
        summaryLandmark.variance = lm.variance;
        summaryLandmark.observeRepeat = lm.observeRepeat;
        summary.landmarks.push_back(summaryLandmark);
    }

    return summary;
}

#endif  // SLAM__COMMON__DEF_SLAM_GRAPH_HPP_
