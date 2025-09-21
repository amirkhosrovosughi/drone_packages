// px4_command_handler_util.cpp

#include "px4_command_handler_util.hpp"

bool Px4CommandHandlerUtil::safeParseFloat(const std::string& str, float& result)
{
    try {
        size_t idx;
        result = std::stof(str, &idx);

        // Check if the whole string was parsed
        if (idx != str.size()) {
            return false;
        }
        return true;
    } catch (const std::invalid_argument& e) {
        // No conversion could be performed
        return false;
    } catch (const std::out_of_range& e) {
        // The converted value would fall out of the range of the result type
        return false;
    }
}

bool Px4CommandHandlerUtil::safeParseVector4f(const std::string& str, Eigen::Vector4f& result)
{
    std::istringstream iss(str);
    std::string item;
    std::vector<float> values;
    while (std::getline(iss, item, ' ')) {
        float value;
        if (!safeParseFloat(item, value)) {
            return false;
        }
        values.push_back(value);
    }

    if (values.size() != 4) {
        return false;
    }

    result << values[0], values[1], values[2], values[3];
    return true;
}
