#ifndef SLAM__FAST_SLAM_HPP_
#define SLAM__FAST_SLAM_HPP_

#include "base_filter.hpp"

class FastSlam : public BaseFilter {
public:
    FastSlam();
    void prediction() override;
    void correction(const Measurements& meas) override;
    void registerCallback(std::function<void(const Map& map)> callback) override;

private:
    std::function<void(const Map& map)> _callback;
};

#endif  // SLAM__FAST_SLAM_HPP_