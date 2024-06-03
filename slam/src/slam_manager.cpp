#include "slam_manager.hpp"

SlamManager::SlamManager()
    : Node("slam_manager")
{
  // Initialization code here

  //create correct filter based on flag
#ifdef EKF
    _filter = std::make_shared<ExtendedKalmanFilter>();
#elif UKF
    _filter = std::make_shared<UnscentedKalmanFilter>();
#elif FAST_SLAM
    _filter = std::make_shared<FastSlam>();
#else
    _filter = std::make_shared<ExtendedKalmanFilter>();
#endif


  //create the current association based on flag
#ifdef NEAREST_NEIGHBOR
    _associantion = std::make_shared<NearestNeighborAssociation>();
#elif  JOINT_COMPATIBILITY
    _associantion = std::make_shared<JointCompatibilityAssociation>();
#else
    _associantion = std::make_shared<NearestNeighborAssociation>();
#endif

  initialize();
  createSubscribers();
  createPublishers();
}

void SlamManager::initialize()
{
    // _filter->registerCallback([this]() {
    //     // Assuming you have a way to obtain a Map object
    //     Map map;
    //     this->filterCallback(map);
    // });

    // _associantion->registerCallback([this]() {
    //     // Assuming you have a way to obtain a Map object
    //     Meas meas;
    //     this->associationCallback(map);
    // });
}

void SlamManager::createSubscribers()
{

}

void SlamManager::createPublishers()
{

}

void SlamManager::filterCallback(const Map& map)
{
    std::cout << "filterCallback called with map" << std::endl;
    // publish result
    // notify association   
}

void SlamManager::associationCallback(const Measurements& Meas)
{
    std::cout << "associationCallback called with measurements" << std::endl; 
    // send result to fllter  
}