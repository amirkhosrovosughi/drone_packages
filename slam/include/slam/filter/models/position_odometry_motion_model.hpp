#ifndef POSITION_ODOMETRY_MOTION__MODEL_HPP_
#define POSITION_ODOMETRY_MOTION__MODEL_HPP_

#include "filter/models/motion_model.hpp"

#include <Eigen/Dense>
#include "def_slam.hpp"

class PositionOdometryMotionModel : public MotionModel {
public:
    virtual int getDimension() override;
    virtual Eigen::VectorXd stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt) override;
    virtual Eigen::MatrixXd corrolationUpdate(const Eigen::MatrixXd& state) override;

};

#endif  // POSITION_ODOMETRY_MOTION__MODEL_HPP_