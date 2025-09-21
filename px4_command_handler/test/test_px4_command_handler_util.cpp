#include <gtest/gtest.h>
#include "px4_command_handler_util.hpp"
#include <Eigen/Dense>

TEST(Px4CommandHandlerUtilTest, SafeParseFloat_ValidInput) {
    float result;
    EXPECT_TRUE(Px4CommandHandlerUtil::safeParseFloat("3.14", result));
    EXPECT_FLOAT_EQ(result, 3.14f);
}

TEST(Px4CommandHandlerUtilTest, SafeParseFloat_InvalidInput) {
    float result;
    EXPECT_FALSE(Px4CommandHandlerUtil::safeParseFloat("abc", result));
}

TEST(Px4CommandHandlerUtilTest, SafeParseFloat_PartialParse) {
    float result;
    EXPECT_FALSE(Px4CommandHandlerUtil::safeParseFloat("123abc", result));
}

TEST(Px4CommandHandlerUtilTest, SafeParseFloat_OutOfRange) {
    float result;
    // Very large number that overflows float
    EXPECT_FALSE(Px4CommandHandlerUtil::safeParseFloat("1e1000", result));
}

TEST(Px4CommandHandlerUtilTest, SafeParseVector4f_ValidInput) {
    Eigen::Vector4f vec;
    EXPECT_TRUE(Px4CommandHandlerUtil::safeParseVector4f("1.0 2.0 3.0 4.0", vec));
    EXPECT_FLOAT_EQ(vec[0], 1.0f);
    EXPECT_FLOAT_EQ(vec[1], 2.0f);
    EXPECT_FLOAT_EQ(vec[2], 3.0f);
    EXPECT_FLOAT_EQ(vec[3], 4.0f);
}

TEST(Px4CommandHandlerUtilTest, SafeParseVector4f_InvalidInput) {
    Eigen::Vector4f vec;
    EXPECT_FALSE(Px4CommandHandlerUtil::safeParseVector4f("1.0 two 3.0 4.0", vec));
}

TEST(Px4CommandHandlerUtilTest, SafeParseVector4f_WrongSize) {
    Eigen::Vector4f vec;
    // Only 3 elements
    EXPECT_FALSE(Px4CommandHandlerUtil::safeParseVector4f("1.0 2.0 3.0", vec));
}
