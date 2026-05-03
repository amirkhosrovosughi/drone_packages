#include "node/slam_manager.hpp"

#include <chrono>

#include "pipeline/pipeline_factory.hpp"
#include "observation/observation_builder.hpp"
#include "startup/slam_startup_contract.hpp"

#include "common_utilities/transform_util.hpp"
#include "common/ros_slam_logger.hpp"

static constexpr const char* FROM_FRAME = "base_link";
static constexpr const char* TO_FRAME   = "camera_frame";
static constexpr int ROBOT_ID = 1;
static constexpr const char* GPS_TOPIC = "/fmu/out/vehicle_gps_position";
static constexpr const char* ODOM_TOPIC = "/fmu/out/vehicle_odometry";
static constexpr const char* FEATURE_POINT_TOPIC = "/feature/coordinate/baseLink";
static constexpr const char* FEATURE_BBOX_TOPIC = "/feature/bbox/cameraFrame";
static constexpr const char* CAMERA_INFO_TOPIC = "/camera_info";
static constexpr const char* MAP_TOPIC = "/slam/map";
static constexpr std::chrono::milliseconds CAMERA_EXTRINSIC_TIMER_PERIOD_MS{100};
static constexpr std::chrono::milliseconds MAP_PUBLISH_TIMER_PERIOD_MS{500};
static constexpr std::chrono::milliseconds STARTUP_WATCHDOG_TIMER_PERIOD_MS{500};

SlamManager::SlamManager()
: Node("slam_manager")
{
  _logger = std::make_shared<RosSlamLogger>(this->get_logger());

  configureStartupContract();

  _measurementFactory = std::make_shared<MeasurementFactory>();
  _slam = slam::createPipeline(_logger, _measurementFactory);
  _slam->setLogger(_logger);
  _slam->initialize();

  createSubscribers();
  createPublishers();

  // TF
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tfListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

  _cameraExtrinsicTimer = this->create_wall_timer(
    CAMERA_EXTRINSIC_TIMER_PERIOD_MS,
    std::bind(&SlamManager::updateCameraExtrinsic, this));

  _mapPubTimer = this->create_wall_timer(
    MAP_PUBLISH_TIMER_PERIOD_MS,
    [this]()
    {
      auto snapshot = _slam->getMap();
      if (!snapshot.isValid())
      {
        return;
      }
      publishMap(snapshot);
    }
  );

  _startupWatchdogTimer = this->create_wall_timer(
    STARTUP_WATCHDOG_TIMER_PERIOD_MS,
    std::bind(&SlamManager::startupWatchdogCallback, this));
}

void SlamManager::configureStartupContract()
{
  SlamStartupContractConfig config =
    startup::loadContractFromNode(*this);

  _startupGate = std::make_unique<SlamStartupGate>();

  if (!config.warningMessage.empty())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "%s",
      config.warningMessage.c_str());
  }

  SlamStartupGate::logTransition(
    this->get_logger(),
    _startupGate->configure(std::move(config), this->now()));
}

void SlamManager::createSubscribers()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .best_effort()
               .transient_local();

  if (_startupGate && _startupGate->requiresGpsSubscription())
  {
    _gpsSub = this->create_subscription<px4_msgs::msg::SensorGps>(
      GPS_TOPIC,
      qos,
      std::bind(&SlamManager::gpsCallback, this, std::placeholders::_1));
  }

  _odomSub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    ODOM_TOPIC,
    qos,
    std::bind(&SlamManager::odometryCallback, this, std::placeholders::_1));

  _obs3dPointSub = this->create_subscription<drone_msgs::msg::PointList>(
    FEATURE_POINT_TOPIC,
    10,
    std::bind(&SlamManager::feature3dPointCallback, this, std::placeholders::_1));

  _obsBboxSub = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    FEATURE_BBOX_TOPIC,
    10,
    std::bind(&SlamManager::featureBboxCallback, this, std::placeholders::_1));

  _cameraIntrinsicSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    CAMERA_INFO_TOPIC,
    10,
    std::bind(&SlamManager::cameraIntrinsicCallback, this, std::placeholders::_1));
}

void SlamManager::createPublishers()
{
  _mapPub = this->create_publisher<drone_msgs::msg::MapSummary>(
    MAP_TOPIC,
    10);
}

void SlamManager::odometryCallback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  if (_startupGate && _startupGate->shouldDropSlamInput())
  {
    return;
  }

  MotionConstraint motion;

  // Convert NED → ENU
  Eigen::Vector3f posNed(
    msg->position[0],
    msg->position[1],
    msg->position[2]);

  Eigen::Vector3f posEnu = TransformUtil::nedToEnu(posNed);

  if (_lastPositionEnu == Eigen::Vector3f::Zero())
  {
    _lastPositionEnu = posEnu;
    return;
  }
  Eigen::Vector3f posDiff = posEnu - _lastPositionEnu;

  Quaternion qNed(
    msg->q[0],
    msg->q[1],
    msg->q[2],
    msg->q[3]);

  Eigen::Vector4d qEnuVec =
    TransformUtil::nedToEnuQuaternion(qNed.getVector());

  Quaternion qEnu(qEnuVec);

  motion.deltaPosition = Eigen::Vector3d(posDiff.x(), posDiff.y(), posDiff.z());
  motion.orientation = Eigen::Quaterniond(qEnu.w, qEnu.x, qEnu.y, qEnu.z);

  _slam->processMotion(motion);

  _lastPositionEnu = posEnu;
}

void SlamManager::gpsCallback(
  const px4_msgs::msg::SensorGps::SharedPtr msg)
{
  if (!_startupGate)
  {
    return;
  }

  const SlamStartupGate::GpsSampleResult result =
    _startupGate->onGpsSample(*msg, this->now());

  SlamStartupGate::logTransition(this->get_logger(), result.transition);

  if (result.status == SlamStartupGate::GpsSampleResult::Status::Rejected)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "GPS init sample rejected: %s",
      result.reason.c_str());
    return;
  }

  if (result.status == SlamStartupGate::GpsSampleResult::Status::Pending)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "GPS init pending: accepted samples=%zu",
      result.acceptedSampleCount);
    return;
  }

  if (result.status != SlamStartupGate::GpsSampleResult::Status::Ready ||
      !result.reference.has_value())
  {
    return;
  }

  const GpsReference& reference = *result.reference;

  RCLCPP_INFO(
    this->get_logger(),
    "GPS init complete with %zu samples: lat=%.10f lon=%.10f alt=%.3f",
    result.acceptedSampleCount,
    reference.latitudeDeg,
    reference.longitudeDeg,
    reference.altitudeM);
}

void SlamManager::feature3dPointCallback(
    const drone_msgs::msg::PointList::SharedPtr msg)
{
  if (_startupGate && _startupGate->shouldDropSlamInput())
    {
      return;
    }

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Dropping /feature/coordinate/baseLink message without header timestamp.");
      return;
    }
    const double timeTag = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;

    // Build Observations via builder
    slam::Observations obs = slam::ObservationBuilder::fromPointList(*msg, timeTag);
    _slam->processObservation(obs);
}

void SlamManager::featureBboxCallback(
    const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  if (_startupGate && _startupGate->shouldDropSlamInput())
  {
    return;
  }

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Dropping /feature/bbox/cameraFrame message without header timestamp.");
      return;
    }
    const double timeTag = msg->header.stamp.sec +
                           msg->header.stamp.nanosec * 1e-9;

  // Build Observations via builder
  slam::Observations obs = slam::ObservationBuilder::fromBboxArray(*msg, timeTag);
  if (!msg->detections.empty() && obs.empty())
  {
    RCLCPP_WARN(this->get_logger(),
                "Received %zu bbox detections but produced 0 SLAM observations. Camera info may not be initialized yet.",
                msg->detections.size());
  }
  _slam->processObservation(obs);
}

void SlamManager::publishMap(const MapSummary& map)
{
  drone_msgs::msg::MapSummary msg;

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

  for (const auto& lm : map.landmarks)
  {
    drone_msgs::msg::Landmark lm_msg;
    lm_msg.id = lm.id;
    lm_msg.observe_repeat = lm.observeRepeat;
    lm_msg.position.x = lm.position.x;
    lm_msg.position.y = lm.position.y;
    lm_msg.position.z = lm.position.z;
    lm_msg.variance.xx = lm.variance.xx;
    lm_msg.variance.xy = lm.variance.xy;
    lm_msg.variance.yy = lm.variance.yy;

    msg.landmarks.push_back(lm_msg);
  }

  _mapPub->publish(msg);
}

void SlamManager::updateCameraExtrinsic()
{
  if (_cameraExtrinsicLoaded)
    return;

  try
  {
    auto tf =
      _tfBuffer->lookupTransform(
        FROM_FRAME,
        TO_FRAME,
        tf2::TimePointZero);

    Eigen::Vector3d t(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);

    Eigen::Quaterniond q(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = t;

    // Convert camera body frame to optical frame (x right, y down, z forward)
    Eigen::Matrix4d body_to_optical_transform = Eigen::Matrix4d::Identity();
    body_to_optical_transform.block<3, 3>(0, 0) << 0, 0, 1,
                            -1, 0, 0,
                            0, -1, 0;
    T = T * body_to_optical_transform;

    _cameraInfo.extrinsics = CameraExtrinsics(T);
    _cameraExtrinsicLoaded = true;

    _cameraExtrinsicTimer->cancel();

    RCLCPP_INFO(this->get_logger(), "Camera extrinsic transform loaded");    
    initializeCameraInfo();
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(
      this->get_logger(),
      "TF unavailable: %s",
      ex.what());
  }
}

void SlamManager::cameraIntrinsicCallback(const sensor_msgs::msg::CameraInfo& cameraInfo)
{
  if (!_cameraIntrinsicLoaded)
  {
    if (cameraInfo.header.frame_id.find("IMX214") == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Skip Stereo camera frame info");
      return;
    }
    
    _cameraInfo.intrinsic = CameraIntrinsic(
      cameraInfo.width,
      cameraInfo.height,
      cameraInfo.k[0],
      cameraInfo.k[4],
      cameraInfo.k[2],
      cameraInfo.k[5]);

    _cameraIntrinsicLoaded = true;
    _cameraIntrinsicSubscriber.reset();

    RCLCPP_INFO(this->get_logger(), "Camera intrinsic parameters loaded");
    initializeCameraInfo();  
  }
}

void SlamManager::initializeCameraInfo()
{
  if (_cameraIntrinsicLoaded && _cameraExtrinsicLoaded)
  {
    _measurementFactory->setCameraInfo(_cameraInfo); // should I remove this??
    slam::ObservationBuilder::setCameraInfo(_cameraInfo);
  }
}

double SlamManager::getCurrentTimeInSeconds()
{
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

void SlamManager::startupWatchdogCallback()
{
  if (!_startupGate)
  {
    return;
  }

  const SlamStartupGate::WatchdogResult result =
    _startupGate->onWatchdogTick(this->now());

  SlamStartupGate::logTransition(this->get_logger(), result.transition);

  if (result.shouldWarnBlocked)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "GPS initialization still pending (%.1fs elapsed). SLAM inputs are blocked.",
      result.elapsedSec);
  }
}

