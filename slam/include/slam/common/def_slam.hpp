#ifndef DEF_SLAM_HPP_
#define DEF_SLAM_HPP_

#include <string>
#include <vector>
#include <Eigen/Dense>

// Structure for Position
struct Position {
    double x;
    double y;
    double z;

    Position(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
    Position(Eigen::Vector3d positionVector) : x(positionVector[0]), y(positionVector[1]), z(positionVector[2]) {}
    Eigen::Vector3d getPositionVector() const
    {
        return Eigen::Vector3d(x, y, z);
    }
};

// Structure for Orientation
struct Orientation {
    double roll;
    double pitch;
    double yaw;

    Orientation(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
        : roll(roll), pitch(pitch), yaw(yaw) {}
};

struct Quaternion {
    double w;
    double x;
    double y;
    double z;

    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}
    Quaternion(Eigen::Vector4d quaternionVector) : w(quaternionVector[0]), x(quaternionVector[1]), y(quaternionVector[2]), z(quaternionVector[3]) {}
    Eigen::Vector4d getVector() const
    {
        return Eigen::Vector4d(w, x, y, z);
    }
};

// Structure for Pose
struct Pose {
    Position position;
    Quaternion quaternion;

    Pose(): position(Position()), quaternion(Quaternion()) {}
    Pose(Position position, Quaternion quaternion)
        : position(position), quaternion(quaternion) {}
    Eigen::Matrix4d getTransformationMatrix() const
    {
        Eigen::Vector3d translation(position.x,
                                    position.y,
                                    position.z);
        Eigen::Quaterniond rotation(quaternion.w,
                                    quaternion.x,
                                    quaternion.y,
                                    quaternion.z);

        // Construct the transformation matrix
        Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
        transformationMatrix.block<3, 3>(0, 0) = rotation_matrix;
        transformationMatrix.block<3, 1>(0, 3) = translation;
        return transformationMatrix;
    }
};

// Structure for Variance2D
struct Variance2D {
    double xx;
    double xy;
    double yy;

    Variance2D(double xx = 0.0, double xy = 0.0, double yy = 0.0)
        : xx(xx), xy(xy), yy(yy) {}
};
// Structure for Landmark
struct Landmark {
    int id;
    Position position;
    Variance2D variance;
    int observeRepeat;

    Landmark(const int id, const Position& position, const Variance2D& variance)
        : id(id) ,position(position), variance(variance), observeRepeat(0) {}
    Landmark()
        : id(0) ,position(Position()), variance(Variance2D()), observeRepeat(0) {}
};

// Vector of landmarks
using Landmarks = std::vector<Landmark>;

// Structure for RobotPose
struct RobotState {
    Pose pose;
    Variance2D variance;

    RobotState(const Pose& pose = Pose(), const Variance2D& variance = Variance2D())
        : pose(pose), variance(variance) {}
};

// Structure for Map information summary
struct MapSummary {
    RobotState robot;
    Landmarks landmarks;

    MapSummary() {}

    bool is_valid() const { return true; }
};

struct LinearVelocity {
    double x;
    double y;
    double z;

    LinearVelocity(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
};

struct AngularVelocity {
    double roll;
    double pitch;
    double yaw;

    AngularVelocity(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
        : roll(roll), pitch(pitch), yaw(yaw) {}
};

struct Velocity {
    LinearVelocity linear;
    AngularVelocity angular;

    Velocity() {}
};

struct MotionConstraint
{
  // Relative translation in world or local frame (meters)
  Eigen::Vector3d delta_position = Eigen::Vector3d::Zero();

  // Absolute orientation at the end of the motion (trusted input)
  // Used only for frame transformations, not estimated.
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  MotionConstraint() = default;
};

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

struct GraphState {
    RobotState robot;
    std::vector<GraphKeyframeNode> keyframes;
    std::vector<GraphLandmarkNode> landmarks;
    std::vector<GraphOdometryEdge> odometryEdges;
    std::vector<GraphObservationEdge> observationEdges;
    std::vector<GraphLoopClosureEdge> loopClosureEdges;
    int activeKeyframeId = 0;

    GraphState() {}

    bool is_valid() const { return true; }
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

struct PredictionInput
{

    Eigen::Vector3d delta_position;  // or velocity * dt

    // External attitude reference (not estimated)
    Eigen::Quaterniond orientation;

    PredictionInput() = default;
    PredictionInput(MotionConstraint m) {
        this->delta_position = m.delta_position;
        this->orientation = m.orientation;
    }
};

using CameraExtrinsics = Eigen::Matrix4d;
struct CameraIntrinsic
{
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;

    CameraIntrinsic(int width = 0, int height = 0, double fx = 0.0, double fy = 0.0, double cx = 0.0, double cy = 0.0)
        : width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy) {}
};

struct CameraInfo
{
    CameraExtrinsics extrinsics;
    CameraIntrinsic intrinsic;

    CameraInfo() : extrinsics(CameraExtrinsics::Identity()), intrinsic(CameraIntrinsic()) {}
};

/**
 * @brief Refinement strategy for pose optimization.
 * Allows future flexibility between different optimization approaches.
 */
enum class RefinementStrategy {
  GaussNewton,    // Iterative Gauss-Newton for accuracy
  ClosedForm      // Single-step closed-form for speed
};

/**
 * @brief Configuration for optimization/refinement operations.
 */
struct OptimizationConfig {
  int maxIterations = 5;           // Max iterations for Gauss-Newton
  int maxSolveTimeMs = 100;        // Max time budget for solve (ms)
  double convergeThreshold = 1e-4; // Convergence criterion (pose delta norm)
  RefinementStrategy strategy = RefinementStrategy::GaussNewton;

  OptimizationConfig() = default;
};

/**
 * @brief Result from optimization/refinement operations.
 */
struct OptimizationResult {
  bool success = false;                   // Whether optimization succeeded
  int solveTimeMs = 0;                    // Actual time taken (ms)
  double finalError = 0.0;                // Final residual error
  int numPosesRefined = 0;                // Number of poses updated
  int numIterations = 0;                  // Actual iterations run
  std::string failureReason = "";         // Reason if success=false

  OptimizationResult() = default;
};

#endif // DEF_SLAM_HPP_
