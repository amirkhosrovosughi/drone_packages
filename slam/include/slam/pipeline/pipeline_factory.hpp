#ifndef SLAM__PIPELINE__PIPELINE_FACTORY_HPP_
#define SLAM__PIPELINE__PIPELINE_FACTORY_HPP_

#include <memory>

#include "pipeline/slam_pipeline.hpp"
#include "common/slam_logger.hpp"
#include "measurement/measurement_factory.hpp"

namespace slam
{

    /**
     * @brief Create the active SLAM pipeline based on compile-time selection flags.
     *
     * The selected pipeline is controlled by compile definitions from CMake:
     * EKF, GRAPH, FAST_SLAM.
     */
    std::shared_ptr<SlamPipeline> createPipeline(
        LoggerPtr logger,
        std::shared_ptr<MeasurementFactory> measurementFactory);

} // namespace slam

#endif  // SLAM__PIPELINE__PIPELINE_FACTORY_HPP_
