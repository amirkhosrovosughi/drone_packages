#include "node/slam_manager.hpp"

#include <chrono>

#include "backend/ekf_slam_backend.hpp"

#include "association/nearest_neighbor_association.hpp"
#include "filter/extended_kalman_filter.hpp"
#include "motion/position_only_motion_model.hpp"
#include "observation/observation_builder.hpp"

#include "common_utilities/transform_util.hpp"

static constexpr const char* FROM_FRAME = "base_link";
static constexpr const char* TO_FRAME   = "camera_frame";
static constexpr int ROBOT_ID = 1;

SlamManager::SlamManager()
: Node("slam_manager")
{
  _logger = std::make_shared<SlamLogger>(this->get_logger());


  // --------------------------------------------------------------------------
  // Backend construction (temporary: EKF hard-coded)
  // Later this becomes a factory
  // --------------------------------------------------------------------------
  // Create models and factory
  auto motionModel = std::make_shared<PositionOnlyMotionModel>();
  auto ekf = std::make_shared<ExtendedKalmanFilter>(motionModel);
  auto association = std::make_shared<NearestNeighborAssociation>();
  _measurementFactory = std::make_shared<MeasurementFactory>();

  association->setLogger(_logger);
  ekf->setLogger(_logger);

  // Construct backend with injected dependencies
  _backend = std::make_shared<slam::EkfSlamBackend>(ekf, association, _measurementFactory);
  _backend->setLogger(_logger);
  _backend->initialize();

  createSubscribers();
  createPublishers();

  // TF
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tfListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

  _cameraExtrinsicTimer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SlamManager::updateCameraExtrinsic, this));

  _mapPubTimer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    [this]()
    {
      auto snapshot = _backend->getMap();
      if (!snapshot.is_valid())
      {
        return;
      }
      publishMap(snapshot);
    }
  );
}

void SlamManager::createSubscribers()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .best_effort()
               .transient_local();

  _odomSub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry",
    qos,
    std::bind(&SlamManager::odometryCallback, this, std::placeholders::_1));

  _obsSub = this->create_subscription<drone_msgs::msg::PointList>(
    "/feature/coordinate/baseLink",
    10,
    std::bind(&SlamManager::featureDetectionCallback, this, std::placeholders::_1));

    _cameraIntrinsicSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info",
      10,
      std::bind(&SlamManager::cameraIntrinsicCallback, this, std::placeholders::_1));
}

void SlamManager::createPublishers()
{
  _mapPub = this->create_publisher<drone_msgs::msg::MapSummary>(
    "/slam/map",
    10);
}

void SlamManager::odometryCallback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  MotionConstraint motion;

  // Convert NED → ENU
  Eigen::Vector3f pos_ned(
    msg->position[0],
    msg->position[1],
    msg->position[2]);

  Eigen::Vector3f pos_enu = TransformUtil::nedToEnu(pos_ned);

  if (_last_position_enu == Eigen::Vector3f::Zero())
  {
    _last_position_enu = pos_enu;
    return;
  }
  Eigen::Vector3f pos_diff = pos_enu - _last_position_enu;

  Quaternion q_ned(
    msg->q[0],
    msg->q[1],
    msg->q[2],
    msg->q[3]);

  Eigen::Vector4d q_enu_vec =
    TransformUtil::nedToEnuQuaternion(q_ned.getVector());

  Quaternion q_enu(q_enu_vec);


  motion.delta_position = Eigen::Vector3d(pos_diff.x(), pos_diff.y(), pos_diff.z());
  motion.orientation = Eigen::Quaterniond(q_enu.w, q_enu.x, q_enu.y, q_enu.z);

  _backend->processMotion(motion);

  _last_position_enu = pos_enu;
}

void SlamManager::featureDetectionCallback(
    const drone_msgs::msg::PointList::SharedPtr msg)
{
    // Get timestamp (prefer message time if available)
    double timeTag;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
    {
      timeTag = msg->header.stamp.sec +
            msg->header.stamp.nanosec * 1e-9;
    }
    else
    {
      timeTag = getCurrentTimeInSeconds();
    }

    // Build Observations via builder
    slam::Observations obs = slam::ObservationBuilder::fromPointList(*msg, timeTag);
    _backend->processObservation(obs);
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

    // _backend->setSensorTransform(T); // TODO: this is wrong, it should go to ObservationBuilder
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

void SlamManager::cameraIntrinsicCallback(const sensor_msgs::msg::CameraInfo cameraInfo)
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
    _measurementFactory->setCameraInfo(_cameraInfo);
  }
}

double SlamManager::getCurrentTimeInSeconds()
{
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}
