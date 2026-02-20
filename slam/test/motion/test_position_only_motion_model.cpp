#include <gtest/gtest.h>

#include "motion/position_only_motion_model.hpp"

using namespace std;

TEST(PositionOnlyMotionModelTest, StateDimensionIsThree)
{
    PositionOnlyMotionModel model;
    EXPECT_EQ(model.getStateDimension(), 3);
}

TEST(PositionOnlyMotionModelTest, PropagateAddsDisplacement)
{
    PositionOnlyMotionModel model;
    MotionModel::State state = MotionModel::State::Zero(3);
    Eigen::VectorXd delta(3);
    delta << 1.0, 2.0, -0.5;

    MotionModel::State next = model.propagate(state, delta);
    EXPECT_DOUBLE_EQ(next(0), 1.0);
    EXPECT_DOUBLE_EQ(next(1), 2.0);
    EXPECT_DOUBLE_EQ(next(2), -0.5);
}

TEST(PositionOnlyMotionModelTest, JacobianIsIdentity)
{
    PositionOnlyMotionModel model;
    MotionModel::State state = MotionModel::State::Zero(3);
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(3);

    Eigen::MatrixXd J = model.computeStateJacobian(state, delta);
    EXPECT_EQ(J.rows(), 3);
    EXPECT_EQ(J.cols(), 3);
    EXPECT_TRUE(J.isApprox(Eigen::MatrixXd::Identity(3,3)));
}

TEST(PositionOnlyMotionModelTest, ProcessNoiseDiagonal)
{
    PositionOnlyMotionModel model;
    Eigen::MatrixXd Q = model.getProcessNoise();
    EXPECT_EQ(Q.rows(), 3);
    EXPECT_EQ(Q.cols(), 3);
    EXPECT_DOUBLE_EQ(Q(0,0), 0.05);
    EXPECT_DOUBLE_EQ(Q(1,1), 0.05);
    EXPECT_DOUBLE_EQ(Q(2,2), 0.05);
}
