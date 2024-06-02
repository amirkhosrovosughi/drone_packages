#ifndef SLAM__FILTER_HPP_
#define SLAM__FILTER_HPP_

#include <functional>

class BaseFilter {
public:
    virtual ~BaseFilter() = default;
    virtual void prediction() = 0;
    virtual void correction() = 0;
    virtual void registerCallback(std::function<void()> callback) = 0;
};

#endif  // SLAM__FILTER_HPP_