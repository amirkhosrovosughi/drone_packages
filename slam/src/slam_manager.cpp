#include "slam_manager.hpp"
#include <mutex>

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
    _filter->registerCallback([this](const Map& map) {
        this->filterCallback(map);
    });

    _associantion->registerCallback([this](const Measurements& meas) {
        this->associationCallback(meas);
    });
}

void SlamManager::createSubscribers()
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    _droneOdometrySubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
              "/fmu/out/vehicle_odometry", qos_profile, std::bind(&SlamManager::droneOdometryCallback, this, std::placeholders::_1));
    _feature3DcoordinatSubscriber = this->create_subscription<drone_msgs::msg::PointList>(
              "/feature/coordinate/baseLink", 10, std::bind(&SlamManager::featureDetectionCallback, this, std::placeholders::_1));
}

void SlamManager::createPublishers()
{

}

void SlamManager::filterCallback(const Map& map)
{
    std::cout << "filterCallback called with map" << std::endl;
    // publish result
    publishMap(map);
    // notify association
    Measurements meas = map.getFeatures();
    _associantion->handleUpdate(meas);
}

void SlamManager::associationCallback(const Measurements& meas)
{
    std::cout << "associationCallback called with measurements" << std::endl; 
    // send result to fllter 
    _filter->correction(meas);
}

void SlamManager::droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry)
{
    std::cout << "---- receive odometry ----" << std::endl;
    // collect need info from odometry and send to prediction
    
    // NOTE: with assumption that no rotation is needed, to be verified (making sure frame orientation/order match that of features)
    Velocity velocity;
    velocity.linear.x = odometry.velocity[0];
    velocity.linear.y = odometry.velocity[1];
    velocity.linear.z = odometry.velocity[2];

    velocity.angular.roll = odometry.angular_velocity[0];
    velocity.angular.pitch = odometry.angular_velocity[1];
    velocity.angular.yaw = odometry.angular_velocity[2];

    _filter->prediction(velocity);
}

void SlamManager::featureDetectionCallback(const drone_msgs::msg::PointList features)
{
    std::cout << "---- receive feature ----" << std::endl;
    Measurements meas;
    meas.reserve(features.points.size());
    for (auto point : features.points)
    {
        meas.emplace_back(1,Position(point.x, point.y, point.z));
    }
    //put it is a approperiae interal structure
    _associantion->onReceiveMeasurement(meas);
}

void SlamManager::publishMap(const Map& map)
{
    //publish map
}