#ifndef SLAM__FAST_SLAM_HPP_
#define SLAM__FAST_SLAM_HPP_

#include "base_filter.hpp"

class FastSlam : public BaseFilter {
public:
    FastSlam();
    void prediction() override;
    void correction() override;
    void registerCallback(std::function<void()> callback) override;

private:
    std::function<void()> _callback;
};

#endif  // SLAM__FAST_SLAM_HPP_