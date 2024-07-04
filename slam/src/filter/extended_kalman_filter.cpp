#include "filter/extended_kalman_filter.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <chrono>

ExtendedKalmanFilter::ExtendedKalmanFilter()
{

#ifdef POSE_MOTION
    _motionModel = std::make_unique<PoseOdometryMotionModel>();
    _odometryType = OdometryType::PoseOdometry;
#elif POSITION_MOTION 
    _motionModel = std::make_unique<PositionOdometryMotionModel>();
    _odometryType = OdometryType::PositionOdometry;
#else
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Have not define compile tag for motion model");
#endif

#ifdef POSITION_MEASUREMENT
    _measurementModel = std::make_unique<PositionMeasurementModel>();
#elif TWO_DIMENSION_MEASUREMENT 
    _measurementModel = std::make_unique<TwoDiemnsionMeasurementModel>();
#else
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Have not define compile tag for measurement model");
#endif

    _slamMap = std::make_shared<SlamMap>(_motionModel->getDimension(), _measurementModel->getDimension());
}

void ExtendedKalmanFilter::prediction(const OdometryInfo& odom)
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processPrediction, this, odom); 
}

void ExtendedKalmanFilter::correction(const Measurements& meas)
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processCorrection, this, meas);
}

void ExtendedKalmanFilter::registerCallback(std::function<void(const MapSummary& map)> callback)
{
    _callback = callback;
}

void ExtendedKalmanFilter::processPrediction(const OdometryInfo& odom)
{
    std::lock_guard<std::mutex> lock(_mutex);

    //update mean
    double timeElapse = odom.timeTag - _lastUpdateTime;
    Eigen::VectorXd updatedRobotMean = _motionModel->stateUpdate(odom, _slamMap->getRobotMean(), timeElapse); // TODO: for time we have lock, so should check when we get the message
    _slamMap->setRobotMean(updatedRobotMean);

    Eigen::MatrixXd updatedRobotCorollation = _motionModel->corrolationUpdate(_slamMap->getRobotCorrelation());
    _slamMap->setRobotCorrelation(updatedRobotCorollation);

    for (int i = 0; i < _slamMap->landmarkCount ; i++)
    {
        Eigen::MatrixXd updatedRobotLandmarkCorollation = _motionModel->corrolationUpdate(_slamMap->getRobotlandmarkCorrelation(i));
        _slamMap->setRobotlandmarkCorrelation(updatedRobotLandmarkCorollation, i);
    }

    _robotQuaternion = odom.orientation;

    std::cout << "Extended Kalman Filter prediction step" << std::endl;
    _lastUpdateTime = getCurrentTimeInSeconds();
    MapSummary map = summerizeMap();
    if (_callback) _callback(map);
}

void ExtendedKalmanFilter::processCorrection(const Measurements& measurements)
{
    std::lock_guard<std::mutex> lock(_mutex);
    //
    for (Measurement meas : measurements)
    {
        if (meas.isNew)
        {
            addLandmark(meas);
        }
        else
        {
            updateLandmark(meas);
        }
    }

    std::cout << "Extended Kalman Filter correction step executed" << std::endl;
    // record the last update time
    _lastUpdateTime = getCurrentTimeInSeconds();
    MapSummary map = summerizeMap();
    if (_callback) _callback(map);
}

void ExtendedKalmanFilter::updateLandmark(const Measurement& measurement)
{
    int id = measurement.id;
    // kalman steps
    //===========================================================
    // 1) calculate measurement error
    Pose robotPose;
    if (_odometryType == OdometryType::PositionOdometry)
    {
        robotPose.position = Position(_slamMap->getRobotMean());
        robotPose.quaternion = _robotQuaternion;
    }
    else if (_odometryType == OdometryType::PoseOdometry)
    {
        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        robotPose.position = Position(robotState.head(3));
        robotPose.quaternion = Quaternion(robotState.tail(4));
    }
    else
    {
        std::cout << "Fail, Invalid motion model dimension" << std::endl;
        return;
    }

    Position landmarkPosition(_slamMap->getLandmarkMean(id));
    Measurement expectedMeasurement;
    if(!_measurementModel->directMeasurementModel(robotPose, landmarkPosition, expectedMeasurement))
    {
        std::cout << "Failed to update landmark, Cannot calculate the direct measurement model" << std::endl;
        return;
    }

    Eigen::Vector3d z = measurement.position.getPositionVector() - expectedMeasurement.position.getPositionVector(); // measurementError

    //===========================================================
    //2 Z -> calculate measurement error variance

    // add some check for compability of dimensions
    // construct P model
    Eigen::MatrixXd prr = _slamMap->getRobotCorrelation();
    Eigen::MatrixXd prl = _slamMap->getRobotlandmarkCorrelation(id);  
    Eigen::MatrixXd pll = _slamMap->getLandmarkSelfCorrelation(id);

    int r = prr.rows();                   // robot state lenght
    int l = pll.cols();                   // landmark state length
    int n = _slamMap->getLandmarkCount(); // number of all landmarks 

    Eigen::MatrixXd p(r + l, r + l);
    // Set the blocks of matrix p
    p.block(0, 0, r, r) = prr;                   // Top-left block
    p.block(0, r, r, l) = prl;                   // Top-right block
    p.block(r, 0, l, r) = prl.transpose();       // Bottom-left block
    p.block(r, r, l, l) = pll;                   // Bottom-right block

    //TODO
    // Eigen::MatrixXd measurementJacobian = _measurementModel->getJacobian(); // this part depens on both _motionModel and _measurementModel --> need to come with solution for that, for now, hard code the hr and hl
    
    Eigen::Quaterniond q(_robotQuaternion.w, _robotQuaternion.x, _robotQuaternion.y, _robotQuaternion.z);
    // Normalize the quaternion (optional if you know it's already normalized)
    q.normalize();
    // Convert the quaternion to a rotation matrix
    Eigen::Matrix3d hr = q.toRotationMatrix();
    Eigen::Matrix3d hl = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd h(l, r + l);
    h.block(0, 0, l, r) = hr;
    h.block(0, r, l, l) = hl; //-> this part is wrong

    Eigen::MatrixXd R = Eigen::Matrix3d::Identity()* 0.2;   // TODO: _measurementModel->getMeasurementNoise(); , do same for motion

    Eigen::MatrixXd Z = h * p * h.transpose() + R; // measurementErrorVariacne


    //===========================================================
    //3 Kalman gain, 
    Eigen::MatrixXd pBar(r + n * l, r + l);

    Eigen::MatrixXd pmr =  _slamMap->getRobotLandmarkFullCorrelations();   
    Eigen::MatrixXd pml =  _slamMap->getCrossLandmarkFullCorrelation(id); 


    p.block(0, 0, r, r) = prr;                   // Top-left block
    p.block(0, r, r, l) = prl;                   // Top-right block
    p.block(r, 0, n * l, r) = pmr;               // Top-left block
    p.block(r, r, n * l, l) = pml;               // Top-right block


    Eigen::MatrixXd K = pBar * h.transpose() * Z.inverse();
    //===========================================================
    //4 update means
    _slamMap->mapMean = _slamMap->mapMean - K * z;
    
    // 5 update variance
    _slamMap->mapCorrelation = _slamMap->mapCorrelation - K * Z * K.transpose();
}

void ExtendedKalmanFilter::addLandmark(const Measurement& meas)
{
    _slamMap->addLandmak(meas.position.getPositionVector());
}

void ExtendedKalmanFilter::setSensorInfo(const Eigen::Matrix4d& transform)
{
    _measurementModel->setSensorInfo(transform);
}

MapSummary ExtendedKalmanFilter::summerizeMap()
{
    MapSummary mapSummary;
    if (_odometryType == OdometryType::PositionOdometry)
    {
        mapSummary.robot.pose.position = Position(_slamMap->getRobotMean());
        mapSummary.robot.pose.quaternion = _robotQuaternion;
    }
    else if (_odometryType == OdometryType::PoseOdometry)
    {
        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        mapSummary.robot.pose.position = Position(robotState.head(3));
        mapSummary.robot.pose.quaternion = Quaternion(robotState.tail(4));
    }
    else
    {
        std::cout << "Fail, Invalid motion model dimension" << std::endl;
        throw std::invalid_argument( "received wrong odometry model" );
    }
    

    Eigen::MatrixXd prr = _slamMap->getRobotCorrelation();
    mapSummary.robot.variance = Variance2D(prr(0,0), prr(1,1), prr(0,1));

    for(int i = 0; i < _slamMap->getLandmarkCount(); i++)
    {
        Landmark landmark;
        landmark.id = i + 1; //(CAUTION) -> we go with assumption that index of landmark on Map is equal id, be caution if this assumption changes later
        landmark.position = Position(_slamMap->getLandmarkMean(i));

        Eigen::MatrixXd pll = _slamMap->getLandmarkSelfCorrelation(i);
        landmark.variance = Variance2D(pll(0,0), pll(1,1), pll(0,1));

        mapSummary.landmarks.push_back(landmark);
    }
    return mapSummary;
}

double ExtendedKalmanFilter::getCurrentTimeInSeconds()
{
    // Get the current time_point
    auto now = std::chrono::system_clock::now();
    // Convert the time_point to a duration since epoch and then to seconds
    double seconds = std::chrono::duration<double>(now.time_since_epoch()).count();
    
    return seconds;
}