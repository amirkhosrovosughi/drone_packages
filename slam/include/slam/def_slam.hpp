#ifndef DEF_SLAM_HPP_
#define DEF_SLAM_HPP_

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
    Quaternion(Eigen::Vector4d quaternionVector) : w(quaternionVector[3]), x(quaternionVector[0]), y(quaternionVector[1]), z(quaternionVector[2]) {}
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
        Eigen::Matrix4d tranformation_matrix = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
        tranformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
        tranformation_matrix.block<3, 1>(0, 3) = translation;
        return tranformation_matrix;
    }
};

// Structure for Measurement
struct Measurement {
    int id;
    Position position;
    bool isNew;

    Measurement() : id(0), position(Position()), isNew(false) {}
    Measurement(int id, const Position& position) : id(id), position(position), isNew(false) {}
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
struct RobotStatic {
    Pose pose;
    Variance2D variance;

    RobotStatic(const Pose& pose = Pose(), const Variance2D& variance = Variance2D())
        : pose(pose), variance(variance) {}
};

// Structure for Map information summary
struct MapSummary {
    RobotStatic robot;
    Landmarks landmarks;

    MapSummary() {}
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
    double timeTag;

    OdometryInfo() {}
};

#endif // DEF_SLAM_HPP_
