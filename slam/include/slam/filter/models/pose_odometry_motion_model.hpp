#ifndef POSE_ODOMETRY_MOTION__MODEL_HPP_
#define POSE_ODOMETRY_MOTION__MODEL_HPP_

#include "filter/models/motion_model.hpp"

#include <Eigen/Dense>
#include "def_slam.hpp"

class PoseOdometryMotionModel : public MotionModel {
public:
    PoseOdometryMotionModel(){}
    virtual int getDimension() override;
    virtual Eigen::VectorXd stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt) override;
    virtual Eigen::MatrixXd corrolationUpdate(const Eigen::MatrixXd& state) override;

private:
    Eigen::Vector3f bodyToInertial(const Eigen::Vector3f& linear_velocity_body, const Eigen::Quaternionf& orientation);
    
    Eigen::Quaternionf updateOrientation(const Eigen::Quaternionf& orientation, const Eigen::Vector3f& angular_velocity, float dt);

};

#endif  // POSE_ODOMETRY_MOTION__MODEL_HPP_