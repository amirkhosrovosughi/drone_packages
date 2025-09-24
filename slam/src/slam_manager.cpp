#include "slam_manager.hpp"
#include <mutex>
#include <chrono>

const std::string FROM_FRAME = "base_link";
const std::string TO_FRAME = "camera_frame";
const static int ROBOT_ID = 1;

SlamManager::SlamManager()
    : Node("slam_manager")
{
  // Initialization code here

    _logger = std::make_shared<SlamLogger>(this->get_logger());

  //TODO: determine motion_measurement_model here and pass it to
  // constructor/setter to filter and association

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
    _association = std::make_shared<NearestNeighborAssociation>();
#elif  JOINT_COMPATIBILITY
    _association = std::make_shared<JointCompatibilityAssociation>();
#else
    _association = std::make_shared<NearestNeighborAssociation>();
#endif

    _association->setLogger(_logger);
    _filter->setLogger(_logger);

  initialize();
  createSubscribers();
  createPublishers();
}

void SlamManager::initialize()
{
    _logger->log(LogLevel::INFO, "Initializing Slam.");
    _previousRobotPosition << 0, 0, 0;
    _lastOdomTime = getCurrentTimeInSeconds();

    _filter->registerCallback([this](const MapSummary& map) {
        this->filterCallback(map);
    });

    _association->registerCallback([this](const Measurements& meas) {
        this->associationCallback(meas);
    });
}

void SlamManager::createSubscribers()
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    _droneOdometrySubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
              "/fmu/out/vehicle_odometry", qos_profile, std::bind(&SlamManager::droneOdometryCallback, this, std::placeholders::_1));
    _feature3DCoordinateSubscriber = this->create_subscription<drone_msgs::msg::PointList>(
              "/feature/coordinate/baseLink", 10, std::bind(&SlamManager::featureDetectionCallback, this, std::placeholders::_1));

    // need to subscribe to tf camera relative position from drone base
    _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    _tfListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

    //TODO: later change it in way that stops listening when it gets the value
    _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SlamManager::updateTransform, this));
}

void SlamManager::createPublishers()
{
     _mapPublisher = this->create_publisher<drone_msgs::msg::MapSummary>("/slam/map", 10);
}

void SlamManager::filterCallback(const MapSummary& map)
{
    _logger->log(LogLevel::DEBUG, "filterCallback called with map");
    // publish result
    publishMap(map);
    // notify association
    _association->handleUpdate(map);
}

void SlamManager::associationCallback(const Measurements& meas)
{ 
    RCLCPP_INFO(rclcpp::get_logger("slam"), "---- associationCallback called with measurements ----");
    _logger->log(LogLevel::DEBUG, "---- associationCallback called with measurements ----");
    // send result to filter 
    _filter->correction(meas);
}

void SlamManager::droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry)
{
    _logger->log(LogLevel::INFO, "---- receive odometry ----");
    
    
    Eigen::Vector3f linearVelocityIntertiaNED{odometry.velocity[0], odometry.velocity[1], odometry.velocity[2]}; // do not use this, because it has big accumulative error for odometry
    _logger->log(LogLevel::DEBUG, "linearVelocityIntertiaNED is:\n", TransformUtil::matrixToString(linearVelocityIntertiaNED).c_str()); // TODO: use linearVelocityIntertiaNED 
    
    Eigen::Vector3f newRobotPosition(odometry.position[0], odometry.position[1], odometry.position[2]);

    Eigen::Vector3f directEnuPosition = TransformUtil::nedToEnu(newRobotPosition);

    #ifdef STORE_DEBUG_DATA
    std::map<std::string, double> mapLog;  // add plotting
    mapLog["odometry.position[0]"] = odometry.position[0];
    mapLog["odometry.position[1]"] = odometry.position[1];
    mapLog["odometry.position[2]"] = odometry.position[2];

    mapLog["directEnuPosition[0]"] = directEnuPosition[0];
    mapLog["directEnuPosition[1]"] = directEnuPosition[1];
    mapLog["directEnuPosition[2]"] = directEnuPosition[2];
    #endif


    // convert newRobotPosition directly to ENU here and compare is to the final result
    Eigen::Vector3f estimatedNEDSpeed = estimateLinearSpeed(newRobotPosition);
    _logger->log(LogLevel::DEBUG, "estimatedNEDSpeed: \n", estimatedNEDSpeed);

    Eigen::Vector3f linearVelocityIntertiaENU = TransformUtil::nedToEnu(estimatedNEDSpeed);
    _logger->log(LogLevel::DEBUG, "linearVelocityBodyENU is:\n", TransformUtil::matrixToString(linearVelocityIntertiaENU).c_str());

    Eigen::Vector3f angularVelocityIntertiaNED{odometry.angular_velocity[0], odometry.angular_velocity[1], odometry.angular_velocity[2]};
    _logger->log(LogLevel::DEBUG, "angularVelocityIntertiaNED is:\n", TransformUtil::matrixToString(angularVelocityIntertiaNED).c_str());
    Eigen::Vector3f angularVelocityIntertiaENU = TransformUtil::nedToEnu(angularVelocityIntertiaNED);
    _logger->log(LogLevel::DEBUG, "angularVelocityIntertiaENU is:\n", TransformUtil::matrixToString(angularVelocityIntertiaENU).c_str());
    float normAngularVel = std::sqrt(angularVelocityIntertiaENU.transpose()*angularVelocityIntertiaENU);
    _logger->log(LogLevel::DEBUG, "angularVelocity norm is:", normAngularVel);
    float normLinearVel = std::sqrt(estimatedNEDSpeed.transpose()*estimatedNEDSpeed);
    _logger->log(LogLevel::DEBUG, "angularVelocity norm is:", normLinearVel);

    OdometryInfo odomInfo;

    Velocity velocityEnu;
    #ifdef STORE_DEBUG_DATA
    mapLog["linearVelocityIntertiaENU[0]"] = linearVelocityIntertiaENU[0];
    mapLog["linearVelocityIntertiaENU[1]"] = linearVelocityIntertiaENU[1];
    mapLog["linearVelocityIntertiaENU[2]"] = linearVelocityIntertiaENU[2];
    #endif

    velocityEnu.linear.x = linearVelocityIntertiaENU[0];
    velocityEnu.linear.y = linearVelocityIntertiaENU[1];
    velocityEnu.linear.z = linearVelocityIntertiaENU[2];

    velocityEnu.angular.roll = angularVelocityIntertiaENU[0];
    velocityEnu.angular.pitch = angularVelocityIntertiaENU[1];
    velocityEnu.angular.yaw = angularVelocityIntertiaENU[2];
    odomInfo.enuVelocity = velocityEnu;

    Velocity velocityNed;
    velocityNed.linear.x = estimatedNEDSpeed[0];
    velocityNed.linear.y = estimatedNEDSpeed[1];
    velocityNed.linear.z = estimatedNEDSpeed[2];

    Quaternion quaternion;
    quaternion.w = odometry.q[0];
    quaternion.x = odometry.q[1];
    quaternion.y = odometry.q[2];
    quaternion.z = odometry.q[3];

    Eigen::Vector4d quaternionEnuVector = TransformUtil::nedToEnuQuaternion(quaternion.getVector());
    Quaternion quaternionEnu(quaternionEnuVector);

    #ifdef STORE_DEBUG_DATA
    mapLog["NED.quaternion.w"] = quaternion.w;
    mapLog["NED.quaternion.x"] = quaternion.x;
    mapLog["NED.quaternion.y"] = quaternion.y;
    mapLog["NED.quaternion.z"] = quaternion.z;

    mapLog["ENU.quaternionEnu.w"] = quaternionEnu.w;
    mapLog["ENU.quaternionEnu.x"] = quaternionEnu.x;
    mapLog["ENU.quaternionEnu.y"] = quaternionEnu.y;
    mapLog["ENU.quaternionEnu.z"] = quaternionEnu.z;

    data_logging_utils::DataLogger::log(mapLog);
    #endif

    odomInfo.nedVelocity = velocityNed;  
    odomInfo.orientation = quaternionEnu; // in this case we assume that orientation is knows, using other
                                          // sensor or odometry and only use position for robot position calculation

    odomInfo.timeTag = getCurrentTimeInSeconds();
    _logger->log(LogLevel::DEBUG, "time tag receive is: ", odomInfo.timeTag);
    _filter->prediction(odomInfo);
}

void SlamManager::featureDetectionCallback(const drone_msgs::msg::PointList features)
{
    _logger->log(LogLevel::INFO, "===== receive feature =====");

    Measurements meas;
    meas.reserve(features.points.size());

    //put it in an appropriate internal structure
    for (auto point : features.points)
    {
        // feature are in FLU (local ENU coordinate)
        meas.emplace_back(1, Position(point.x, point.y, point.z));
    }
    _association->onReceiveMeasurement(meas); // Comment to run without correction, purely odometry
}

void SlamManager::publishMap(const MapSummary& map)
{
    auto msg = drone_msgs::msg::MapSummary();

        // Fill RobotPose
        msg.robot.id = ROBOT_ID;
        msg.robot.pose.position.x = map.robot.pose.position.x;
        msg.robot.pose.position.y = map.robot.pose.position.y;
        msg.robot.pose.position.z = map.robot.pose.position.z;
        msg.robot.pose.orientation.x = map.robot.pose.quaternion.x;
        msg.robot.pose.orientation.y = map.robot.pose.quaternion.y;
        msg.robot.pose.orientation.z = map.robot.pose.quaternion.z;
        msg.robot.pose.orientation.w = map.robot.pose.quaternion.w;
        msg.robot.variance.xx = map.robot.variance.xx;
        msg.robot.variance.xy = map.robot.variance.xy;
        msg.robot.variance.yy = map.robot.variance.yy;

        // Fill Landmarks
        for (const auto& lm : map.landmarks) {
            drone_msgs::msg::Landmark landmark_msg;
            landmark_msg.id = lm.id;
            landmark_msg.observe_repeat = lm.observeRepeat;
            landmark_msg.position.x = lm.position.x;
            landmark_msg.position.y = lm.position.y;
            landmark_msg.position.z = lm.position.z;
            landmark_msg.variance.xx = lm.variance.xx;
            landmark_msg.variance.xy = lm.variance.xy;
            landmark_msg.variance.yy = lm.variance.yy;

            msg.landmarks.push_back(landmark_msg);
        }

        _mapPublisher->publish(msg);
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
      RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s", TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());
      return;
    }
  }
}

double SlamManager::getCurrentTimeInSeconds()
{
    // Get the current time_point
    auto now = std::chrono::system_clock::now();
    // Convert the time_point to a duration since epoch and then to seconds
    double seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
    
    return seconds;
}

/*
* temporary function to estimate the linear velocity, because cannot find an accurate enough velocity topic for odometry
*/
Eigen::Vector3f SlamManager::estimateLinearSpeed(const Eigen::Vector3f& newRobotPosition)
{
    Eigen::Vector3f  posDiff = newRobotPosition - _previousRobotPosition;
    double timeElapse = getCurrentTimeInSeconds() - _lastOdomTime;
    Eigen::Vector3f estimatedSpeed = posDiff / timeElapse;
    
    _previousRobotPosition = newRobotPosition;
    _lastOdomTime = getCurrentTimeInSeconds();

    return estimatedSpeed;
}