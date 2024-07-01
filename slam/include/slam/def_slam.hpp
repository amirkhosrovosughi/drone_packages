#ifndef DEF_SLAM_HPP_
#define DEF_SLAM_HPP_

#include <vector>

// Structure for Position
struct Position {
    double x;
    double y;
    double z;

    Position(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
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
    double x;
    double y;
    double z;
    double w;

    Quaternion(double x = 0.0, double y = 0.0, double z = 0.0, double w = 1.0)
        : x(x), y(y), z(z), w(w) {}
};

// Structure for Pose
struct Pose {
    Position position;
    Quaternion quaternion;

    Pose(): position(Position()), quaternion(Quaternion()) {}
    Pose(Position position, Quaternion quaternion)
        : position(position), quaternion(quaternion) {}
};

// Structure for Measurement
struct Measurement {
    int id;
    Position position;
    int observeRepeat;

    Measurement(int id, const Position& position) : id(id), position(position), observeRepeat(0) {}
};

// Vector of Measurements
using Measurements = std::vector<Measurement>;


// Structure for Variance2D
struct Variance2D {
    double xx;
    double xy;
    double yy;

    Variance2D(double xx = 0.0, double xy = 0.0, double yy = 0.0)
        : xx(xx), xy(xy), yy(yy) {}
};

// Structure for RobotPose
struct RobotStatic {
    Pose pose;
    Variance2D variance;

    RobotStatic(const Pose& pose = Pose(), const Variance2D& variance = Variance2D())
        : pose(pose), variance(variance) {}
};

// Structure for Feature
struct Feature {
    int id;
    Position position;
    Variance2D variance2D;

    Feature(const int id, const Position& position, const Variance2D& variance2D)
        : id(id) ,position(position), variance2D(variance2D) {}
};

// Structure for Map
struct Map {
    RobotStatic robot;
    std::vector<Feature> features;

    Map() {}
    Measurements getFeatures() const
    {
        Measurements meas;
        meas.reserve(features.size());
        for (auto feature : features)
        {
            meas.emplace_back(feature.id, feature.position);
        }
        return meas;
    }
};

struct LinearVelocity {
    double x;
    double y;
    double z;

    LinearVelocity(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
};

struct AngularVelicity {
    double roll;
    double pitch;
    double yaw;

    AngularVelicity(double roll = 0.0, double pitch = 0.0, double yaw = 0.0)
        : roll(roll), pitch(pitch), yaw(yaw) {}
};

struct Velocity {
    LinearVelocity linear;
    AngularVelicity angular;

    Velocity() {}
};

struct OdometryInfo {
    Velocity NedVelocity;
    Velocity EnuVelocity;
    Quaternion orientation;

    OdometryInfo() {}
};

#endif // DEF_SLAM_HPP_
