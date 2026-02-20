#include <gtest/gtest.h>

#include "map/slam_map.hpp"

TEST(SlamMapTest, ConstructorInitializesRobotState)
{
    SlamMap map(3, 3);
    EXPECT_EQ(map.getLandmarkCount(), 0);
    EXPECT_EQ(map.getRobotMean().size(), 3);
    EXPECT_TRUE(map.getRobotMean().isApprox(Eigen::VectorXd::Zero(3)));

    Eigen::MatrixXd prr = map.getRobotCorrelation();
    EXPECT_EQ(prr.rows(), 3);
    EXPECT_EQ(prr.cols(), 3);
    EXPECT_DOUBLE_EQ(prr(0, 0), 0.1);
}

TEST(SlamMapTest, AddLandmarkExpandsStateAndCovariance)
{
    SlamMap map(3, 3);
    Eigen::Vector3d lm1(1.0, 2.0, 3.0);
    map.addLandmark(lm1);

    EXPECT_EQ(map.getLandmarkCount(), 1);
    EXPECT_TRUE(map.getLandmarkMean(0).isApprox(lm1));
    EXPECT_EQ(map.getMapCorrelation().rows(), 6);
    EXPECT_EQ(map.getMapCorrelation().cols(), 6);
}

TEST(SlamMapTest, AddLandmarkWrongDimensionThrows)
{
    SlamMap map(3, 3);
    Eigen::VectorXd bad(2);
    bad << 1.0, 2.0;
    EXPECT_THROW(map.addLandmark(bad), std::invalid_argument);
}

TEST(SlamMapTest, SetAndGetRobotMean)
{
    SlamMap map(3, 3);
    Eigen::Vector3d x(4.0, -1.0, 0.5);
    EXPECT_TRUE(map.setRobotMean(x));
    EXPECT_TRUE(map.getRobotMean().isApprox(x));
}

TEST(SlamMapTest, LandmarkCrossCorrelationUsesBothIndices)
{
    SlamMap map(3, 3);
    map.addLandmark(Eigen::Vector3d(1.0, 0.0, 0.0));
    map.addLandmark(Eigen::Vector3d(2.0, 0.0, 0.0));

    Eigen::MatrixXd cross = 0.7 * Eigen::MatrixXd::Identity(3, 3);
    EXPECT_TRUE(map.setLandmarkCrossCorrelation(cross, 0, 1));

    Eigen::MatrixXd got01 = map.getLandmarkCrossCorrelation(0, 1);
    EXPECT_TRUE(got01.isApprox(cross));
}

TEST(SlamMapTest, InvalidLandmarkIndexThrows)
{
    SlamMap map(3, 3);
    EXPECT_THROW(map.getLandmarkMean(0), std::invalid_argument);
    EXPECT_THROW(map.getLandmarkSelfCorrelation(0), std::invalid_argument);
}
