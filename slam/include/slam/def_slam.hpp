#ifndef DEF_SLAM_HPP_
#define DEF_SLAM_HPP_

#include <vector>

// Structure for Position
struct Position {
    double x;
    double y;
    double z;

    Position(double x, double y, double z = 0.0) : x(x), y(y), z(z) {}
};

// Structure for Orientation
struct Orientation {
    double roll;
    double pitch;
    double yaw;

    Orientation(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
        : roll(roll), pitch(pitch), yaw(yaw) {}
};

// Structure for Orientation
struct Pose {
    Position position;
    Orientation orientation;

    Pose(Position position, Orientation orientation)
        : position(position), orientation(orientation) {}
};
// Structure for Measurement
struct Measurement {
    int id;
    Position position;

    Measurement(int id, const Position& position) : id(id), position(position) {}
};

// Vector of Measurements
using Measurements = std::vector<Measurement>;



// Structure for RobotPose
struct RobotPose {
    Pose pose;
    Orientation orientation;

    RobotPose(const Pose& pose, const Orientation& orientation)
        : pose(pose), orientation(orientation) {}
};

// Structure for RobotPose
struct RobotVariance {
    double xx;
    double xy;
    double yy; // for now, later see what we need to report outside

    RobotVariance(double xx = 0.0, double xy = 0.0, double yy = 0.0)
        : xx(xx), xy(xy), yy(yy) {}
};

// Structure for RobotPose
struct RobotStatic {
    RobotPose pose;
    RobotVariance variance;

    RobotStatic(const RobotPose& pose, const RobotVariance& variance)
        : pose(pose), variance(variance) {}
};

// Structure for Variance2D
struct Variance2D {
    double xx;
    double xy;
    double yy;

    Variance2D(double xx = 0.0, double xy = 0.0, double yy = 0.0)
        : xx(xx), xy(xy), yy(yy) {}
};

// Structure for Feature
struct Feature {
    Position position;
    Variance2D variance2D;

    Feature(const Position& position, const Variance2D& variance2D)
        : position(position), variance2D(variance2D) {}
};

// Structure for Map
struct Map {
    RobotStatic robot;
    std::vector<Feature> features;

    Map();
};

#endif // DEF_SLAM_HPP_
