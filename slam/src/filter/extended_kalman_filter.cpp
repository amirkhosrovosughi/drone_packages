#include "filter/extended_kalman_filter.hpp"
#include <iostream>
#include <thread>
#include <future>

ExtendedKalmanFilter::ExtendedKalmanFilter()
{

#ifdef POSE_MOTION
    _motionModel = std::make_unique<PoseOdometryMotionModel>();
#elif POSITION_MOTION 
    _motionModel = std::make_unique<PositionOdometryMotionModel>();
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

    // SlamMap slamMap(_motionModel->getDimension(),_measurementModel->getDimension());
    _slamMap = std::make_shared<SlamMap>(_motionModel->getDimension(), _measurementModel->getDimension());
}

void ExtendedKalmanFilter::prediction(const OdometryInfo& odom)
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processPrediction, this, odom); // , value  
}

void ExtendedKalmanFilter::correction(const Measurements& meas)
{
    std::future<void> result = std::async(std::launch::async, &ExtendedKalmanFilter::processCorrection, this, meas); // .means
}

void ExtendedKalmanFilter::registerCallback(std::function<void(const Map& map)> callback)
{
    _callback = callback;
}

void ExtendedKalmanFilter::processPrediction(const OdometryInfo& odom)
{
    std::lock_guard<std::mutex> lock(_mutex);

    //update mean
    Eigen::VectorXd updatedRobotMean = _motionModel->stateUpdate(odom, _slamMap->getRobotMean(), 0.05f); // for time we have lock, so should check when we get the message
    _slamMap->setRobotMean(updatedRobotMean);

    Eigen::MatrixXd updatedRobotCorollation = _motionModel->corrolationUpdate(_slamMap->getRobotCorrelation());
    _slamMap->setRobotCorrelation(updatedRobotCorollation);

    for (int i = 0; i < _slamMap->landmakrCount ; i++)
    {
        Eigen::MatrixXd updatedRobotLandmarkCorollation = _motionModel->corrolationUpdate(_slamMap->getRobotlandmarkCorrelation(i));
        _slamMap->setRobotlandmarkCorrelation(updatedRobotLandmarkCorollation, i);
    }

    std::cout << "Extended Kalman Filter prediction step" << std::endl;
    Map map; // -> update map based on _slamMap
    if (_callback) _callback(map);
}

void ExtendedKalmanFilter::processCorrection(const Measurements& measurements)
{
    std::lock_guard<std::mutex> lock(_mutex);
    //
    for (Measurement meas : measurements)
    {
        int id = meas.id;
        // do kalman steps
        // 1) z = yi - h(R, S, Li) --> need that function
        // for that need to now robot state _slamMap->getRobotMean()
        // need to know previous landmark position _slamMap->getLandmarkMean(id)
        // need transportation from robot body to sensor --> copy subscriber from 2dto_3d_transfer

        //2 Z -> observation error variance
        // need HR -> jacobian measurement to robot, I in our case
        // need HL ->  jacobian measurement to feature -> rotation matrix in our case

        //3 Kalman gain, 

        //4 _slamMap->setMean

        // 5 _slamMap->setVariance 
    }


    
    std::cout << "Extended Kalman Filter correction step" << std::endl;
    Map map;
    if (_callback) _callback(map);
}