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
        // pass filter pointer to association to call after association/gating

  
  createSubscribers();
  createPublishers();
}

void SlamManager::createSubscribers()
{

}

void SlamManager::createPublishers()
{

}