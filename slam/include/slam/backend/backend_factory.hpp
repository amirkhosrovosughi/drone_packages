#ifndef SLAM__BACKEND__BACKEND_FACTORY_HPP_
#define SLAM__BACKEND__BACKEND_FACTORY_HPP_

#include <memory>

#include "backend/slam_backend.hpp"
#include "common/slam_logger.hpp"
#include "measurement/measurement_factory.hpp"

namespace slam
{

    /**
     * @brief Create the active SLAM backend based on compile-time selection flags.
     *
     * The selected backend is controlled by compile definitions from CMake:
     * EKF, GRAPH, FAST_SLAM.
     */
    std::shared_ptr<SlamBackend> createBackend(
        LoggerPtr logger,
        std::shared_ptr<MeasurementFactory> measurementFactory);

} // namespace slam

#endif  // SLAM__BACKEND__BACKEND_FACTORY_HPP_
