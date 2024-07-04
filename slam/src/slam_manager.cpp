#include "slam_manager.hpp"
#include <mutex>

const std::string FROM_FRAME = "base_link";
const std::string TO_FRAME = "camera_frame";

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
    _filter->registerCallback([this](const MapSummary& map) {
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

    // need to subscribe to tf camera relative position from drone base
    _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tflistener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

    //TODO: later change it in way that stops listening when it gets the valiue
    _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SlamManager::updateTransform, this));
}

void SlamManager::createPublishers()
{

}

void SlamManager::filterCallback(const MapSummary& map)
{
    RCLCPP_INFO(rclcpp::get_logger("slam"), "filterCallback called with map");
    // publish result
    publishMap(map);
    // notify association
    Measurements meas = map.getLandmarks();
    _associantion->handleUpdate(meas);
}

void SlamManager::associationCallback(const Measurements& meas)
{ 
    RCLCPP_INFO(rclcpp::get_logger("slam"), "---- associationCallback called with measurements ----");
    // send result to fllter 
    _filter->correction(meas);
}

void SlamManager::droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry)
{
    RCLCPP_INFO(rclcpp::get_logger("slam"), "---- receive odometry ----");
    
    
    Eigen::Vector3f linearVelocityIntertiaNED{odometry.velocity[0], odometry.velocity[1], odometry.velocity[2]};
    RCLCPP_INFO(rclcpp::get_logger("slam"), "linearVelocityIntertiaNED is:\n%s", TransformUtil::matrixToString(linearVelocityIntertiaNED).c_str());

    Eigen::Vector3f linearVelocityIntertiaENU = TransformUtil::nedToEnu(linearVelocityIntertiaNED);
    RCLCPP_INFO(rclcpp::get_logger("slam"), "linearVelocityBodyENU is:\n%s", TransformUtil::matrixToString(linearVelocityIntertiaENU).c_str());
    
    // do no need Bdoy inertia for prediction, keep comments them for confirmation later
    // Eigen::Quaterniond rotation(odometry.q[0],
    //                           odometry.q[1],
    //                           odometry.q[2],
    //                           odometry.q[3]);
    // Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
    // RCLCPP_INFO(rclcpp::get_logger("slam"), "Rotation matrix is:\n%s", TransformUtil::matrixToString(rotation_matrix).c_str());

    // Eigen::Matrix3f mati = rotation_matrix.cast<float>();

    //   Eigen::Vector3f linearVelocityBodyNED = mati.transpose() * linearVelocityIntertiaNED;
    //   RCLCPP_INFO(rclcpp::get_logger("slam"), "linearVelocityBodyNED is:\n%s", TransformUtil::matrixToString(linearVelocityBodyNED).c_str());
    //   Eigen::Vector3f linearVelocityBodyENU = TransformUtil::nedToEnu(linearVelocityBodyNED);
    //   RCLCPP_INFO(rclcpp::get_logger("slam"), "linearVelocityBodyENU is:\n%s", TransformUtil::matrixToString(linearVelocityBodyENU).c_str());

    Eigen::Vector3f angularVelocityIntertiaNED{odometry.angular_velocity[0], odometry.angular_velocity[1], odometry.angular_velocity[2]};
    RCLCPP_INFO(rclcpp::get_logger("slam"), "angularVelocityIntertiaNED is:\n%s", TransformUtil::matrixToString(angularVelocityIntertiaNED).c_str());
    Eigen::Vector3f angularVelocityIntertiaENU = TransformUtil::nedToEnu(angularVelocityIntertiaNED);
    RCLCPP_INFO(rclcpp::get_logger("slam"), "angularVelocityIntertiaENU is:\n%s", TransformUtil::matrixToString(angularVelocityIntertiaENU).c_str());

    OdometryInfo odomInfo;

    Velocity velocityEnu;
    velocityEnu.linear.x = linearVelocityIntertiaENU[0];
    velocityEnu.linear.y = linearVelocityIntertiaENU[1];
    velocityEnu.linear.z = linearVelocityIntertiaENU[2];

    velocityEnu.angular.roll = angularVelocityIntertiaENU[0];
    velocityEnu.angular.pitch = angularVelocityIntertiaENU[1];
    velocityEnu.angular.yaw = angularVelocityIntertiaENU[2];
    odomInfo.EnuVelocity = velocityEnu;

    Velocity velocityNed;
    velocityNed.linear.x = linearVelocityIntertiaNED[0];
    velocityNed.linear.y = linearVelocityIntertiaNED[1];
    velocityNed.linear.z = linearVelocityIntertiaNED[2];

    Quaternion quaternion;
    quaternion.w = odometry.q[0];
    quaternion.x = odometry.q[1];
    quaternion.y = odometry.q[2];
    quaternion.z = odometry.q[3];

    odomInfo.NedVelocity = velocityNed;  
    odomInfo.orientation = quaternion; // in this case we assume that orientation is knows, using other
                                       // sensor or odometry and only use position for robot position calculation

    odomInfo.timeTag = double(odometry.timestamp)/1000000.0f; //TODO: to be verified
    _filter->prediction(odomInfo);
}

void SlamManager::featureDetectionCallback(const drone_msgs::msg::PointList features)
{
    RCLCPP_INFO(rclcpp::get_logger("slam"), "---- receive feature ----");

    Measurements meas;
    meas.reserve(features.points.size());

    //put it is a approperiae interal structure
    for (auto point : features.points)
    {
        // feature are in FLU (local ENU coordinate)
        meas.emplace_back(1,Position(point.x, point.y, point.z));
    }
    _associantion->onReceiveMeasurement(meas);
}

void SlamManager::publishMap(const MapSummary& map)
{
    //publish map
}

void SlamManager::updateTransform()
{
  if (!_cameraTransformLoaded)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      geometry_msgs::msg::TransformStamped transform = _tfBuffer->lookupTransform(FROM_FRAME, TO_FRAME, tf2::TimePointZero);
      // Extract translation and rotation from the transform message
      Eigen::Vector3d translation(transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z);
      Eigen::Quaterniond rotation(transform.transform.rotation.w,
                                  transform.transform.rotation.x,
                                  transform.transform.rotation.y,
                                  transform.transform.rotation.z);

      // Construct the transformation matrix
      Eigen::Matrix4d base2Camera;
      Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
      base2Camera.block<3, 3>(0, 0) = rotation_matrix;
      base2Camera.block<3, 1>(0, 3) = translation;
      
      _filter->setSensorInfo(base2Camera);
      _cameraTransformLoaded = true;
      RCLCPP_INFO(this->get_logger(), "Camera relative coordinate in loaded successfully");
  
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());
      return;
    }
  }
}