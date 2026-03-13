#include "pipeline/pipeline_factory.hpp"

#include <stdexcept>

#if defined(EKF)
#include "association/ekf_bearing_initialization_strategy.hpp"
#include "pipeline/ekf_slam_pipeline.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "motion/position_only_motion_model.hpp"
#elif defined(GRAPH)
#include "association/graph_bearing_initialization_strategy.hpp"
#include "pipeline/graph_slam_backend.hpp"
#include "pipeline/graph_slam_frontend.hpp"
#include "pipeline/graph_slam_pipeline.hpp"
#include "graph/internal_graph_optimizer.hpp"
#endif
#include "association/nearest_neighbor_association.hpp"

namespace slam
{

    std::shared_ptr<SlamPipeline> createPipeline(
        LoggerPtr logger,
        std::shared_ptr<MeasurementFactory> measurementFactory)
    {
        if (!measurementFactory)
        {
            throw std::invalid_argument("MeasurementFactory must not be null.");
        }

#if defined(EKF)
        auto motionModel = std::make_shared<PositionOnlyMotionModel>();
        auto ekf = std::make_shared<ExtendedKalmanFilter>(motionModel);
        auto association = std::make_shared<NearestNeighborAssociation>();

        association->setLogger(logger);
        association->setUnderConstrainedInitializationStrategy(
            std::make_shared<EkfBearingInitializationStrategy>());
        ekf->setLogger(logger);

        return std::make_shared<EkfSlamPipeline>(ekf, association, measurementFactory);
#elif defined(GRAPH)
        auto association = std::make_shared<NearestNeighborAssociation>();
        auto optimizer = std::make_shared<InternalGraphOptimizer>();

        association->setLogger(logger);
        association->setUnderConstrainedInitializationStrategy(
            std::make_shared<GraphBearingInitializationStrategy>());
        optimizer->setLogger(logger);

        auto frontend = std::make_shared<GraphSlamFrontend>(association, measurementFactory);
        auto backend = std::make_shared<GraphSlamBackend>(optimizer);

        return std::make_shared<GraphSlamPipeline>(frontend, backend);
#elif defined(FAST_SLAM)
        throw std::runtime_error(
            "FAST_SLAM pipeline selected at compile time, but pipeline factory path is not implemented yet.");
#else
#error "No pipeline compile definition found. Expected EKF, GRAPH, or FAST_SLAM."
#endif
    }

} // namespace slam
