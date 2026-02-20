#include <gtest/gtest.h>
#include "motion/pose_motion_model.hpp"

TEST(PoseMotionModelTest, StateDimension)
{
    PoseMotionModel m;
    EXPECT_EQ(m.getStateDimension(), 6);
}

TEST(PoseMotionModelTest, JacobianIdentity)
{
    PoseMotionModel m;
    Eigen::MatrixXd J = m.computeStateJacobian(MotionModel::State::Zero(6), Eigen::VectorXd::Zero(6));
    EXPECT_TRUE(J.isApprox(Eigen::MatrixXd::Identity(6,6)));
}

TEST(PoseMotionModelTest, ProcessNoiseDiagonal)
{
    PoseMotionModel m;
    Eigen::MatrixXd Q = m.getProcessNoise();
    EXPECT_EQ(Q.rows(), 6);
    EXPECT_EQ(Q.cols(), 6);
    EXPECT_DOUBLE_EQ(Q(0,0), 0.05);
    EXPECT_DOUBLE_EQ(Q(3,3), 0.01);
}
