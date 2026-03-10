#include "backend/backend_factory.hpp"

#include <stdexcept>

#if defined(EKF)
#include "association/ekf_bearing_initialization_strategy.hpp"
#include "association/nearest_neighbor_association.hpp"
#include "backend/ekf_slam_backend.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "motion/position_only_motion_model.hpp"
#endif

namespace slam
{

    std::shared_ptr<SlamBackend> createBackend(
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

        return std::make_shared<EkfSlamBackend>(ekf, association, measurementFactory);
#elif defined(GRAPH)
        throw std::runtime_error(
            "GRAPH backend selected at compile time, but backend factory path is not implemented yet.");
#elif defined(FAST_SLAM)
        throw std::runtime_error(
            "FAST_SLAM backend selected at compile time, but backend factory path is not implemented yet.");
#else
#error "No backend compile definition found. Expected EKF, GRAPH, or FAST_SLAM."
#endif
    }

} // namespace slam
