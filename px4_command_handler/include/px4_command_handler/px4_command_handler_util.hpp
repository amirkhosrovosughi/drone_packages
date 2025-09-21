#ifndef PX4_COMMAND_HANDLER_UTIL_HPP_
#define PX4_COMMAND_HANDLER_UTIL_HPP_

/**
 * @file px4_command_handler_util.hpp
 * @brief Utility functions for PX4 command handling (pure logic, testable).
 */

#include <Eigen/Dense>

class Px4CommandHandlerUtil
{
public:
      /**
     * @brief Safely parse a float from a string.
     *
     * @param str Input string.
     * @param result Output parsed float.
     * @return True if successful, false otherwise.
     */
    static bool safeParseFloat(const std::string& str, float& result);

    /**
     * @brief Safely parse a space-separated 4D vector from a string.
     *
     * @param str Input string.
     * @param result Output Eigen::Vector4f.
     * @return True if successful, false otherwise.
     */
    static bool safeParseVector4f(const std::string& str, Eigen::Vector4f& result);

    /**
     * @brief Convert an Eigen vector to a space-separated string.
     *
     * @tparam Derived Eigen type.
     * @param vector Input Eigen vector.
     * @return String representation of the vector.
     */
    template<typename Derived>
    static std::string getStringFromVector(const Eigen::MatrixBase<Derived>& vector)
    {
        std::stringstream ss;
        ss << vector.transpose();
        return ss.str();
    }

};

#endif  // PX4_COMMAND_HANDLER_UTIL_HPP_
