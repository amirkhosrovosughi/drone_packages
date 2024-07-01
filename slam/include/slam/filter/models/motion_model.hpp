#ifndef MOTION__MODEL_HPP_
#define MOTION__MODEL_HPP_

#include <Eigen/Dense>
#include "def_slam.hpp"

class MotionModel {
public:
    virtual ~MotionModel() = default;
    virtual int getDimension() = 0;
    virtual Eigen::VectorXd stateUpdate(const OdometryInfo& odom, const Eigen::VectorXd& state, double dt) = 0;
    virtual Eigen::MatrixXd corrolationUpdate(const Eigen::MatrixXd& state) = 0;
};

#endif  // MOTION__MODEL_HPP_