#include "filter/extended_kalman_filter.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <chrono>

ExtendedKalmanFilter::ExtendedKalmanFilter()
{

#ifdef POSITION_MOTION_POSITION_MEASUREMENT
    _model = std::make_unique<PositionPositionMotionMeasurementModel>();
    _odometryType = OdometryType::PositionOdometry;
#elif POSE_MOTION_POSITION_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
    _odometryType = OdometryType::PoseOdometry;
#elif POSITION_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
    _odometryType = OdometryType::PositionOdometry;
#elif POSE_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
    _odometryType = OdometryType::PoseOdometry;
#else
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Have not define compile tag for motion model");
    throw std::runtime_error("motion measurement model is not specified");
#endif

    _lastUpdateTime = getCurrentTimeInSeconds();
    _slamMap = std::make_shared<SlamMap>(_model->getMotionDimension(), _model->getMeasurementDimension());
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
    {
        std::lock_guard<std::mutex> lock(_mutex);

        //update mean
        double timeElapse = odom.timeTag - _lastUpdateTime;
        std::cout << "_lastUpdateTime is: " << _lastUpdateTime << std::endl;
        std::cout << "timeElapse is: " << timeElapse << std::endl;

        // Eigen::VectorXd updatedRobotMean = _motionModel->stateUpdate(odom, _slamMap->getRobotMean(), timeElapse); // TODO: for time we have lock, so should check when we get the message
        // Velocity velocity = odom.NedVelocity;
        // Eigen::Vector3d linearVel{velocity.linear.x, velocity.linear.y, velocity.linear.z};
        // Eigen::Vector3d position{state[0], state[1], state[2]};
        // return position + linearVel * dt;

        // _model->getRobotToRobotJacobian(_slamMap->getRobotMean() + + linearVel * dt);

        if (_odometryType == OdometryType::PositionOdometry)
        {
            Velocity velocity = odom.NedVelocity; // change to EnuVelocity, use ned now for comparision and cross checking 
            Eigen::VectorXd linearVel(3);
            linearVel << velocity.linear.x, velocity.linear.y, velocity.linear.z;
            Eigen::VectorXd updatedRobotMean = _model->getRobotToRobotJacobian()*_slamMap->getRobotMean() + linearVel * timeElapse;
            std::cout << "updatedRobotMean is: " << "\n" << updatedRobotMean << std::endl;
            _slamMap->setRobotMean(updatedRobotMean);
        }
        else if (_odometryType == OdometryType::PoseOdometry)
        {
            throw std::runtime_error("have not implemented yet");
        }
        else
        {
            throw std::invalid_argument( "received wrong odometry dimension" );
        }
        
        Eigen::MatrixXd F = _model->getRobotToRobotJacobian();
        Eigen::MatrixXd Q = _model->getMotionNoise();
        Eigen::MatrixXd updatedRobotCorollation = F * _slamMap->getRobotCorrelation() * F.transpose() + Q;
        _slamMap->setRobotCorrelation(updatedRobotCorollation);

        Eigen::MatrixXd updatedRobotLandmarkFullCorrelationsHorizontal = F * _slamMap->getRobotLandmarkFullCorrelationsHorizontal();
        _slamMap->setRobotLandmarkFullCorrelationsHorizontal(updatedRobotLandmarkFullCorrelationsHorizontal);
        _slamMap->setRobotLandmarkFullCorrelationsVertical(updatedRobotLandmarkFullCorrelationsHorizontal.transpose());

        _robotQuaternion = odom.orientation;

        std::cout << "Extended Kalman Filter prediction step" << std::endl;
    }
    
    _lastUpdateTime = odom.timeTag;
    MapSummary map = summerizeMap();
    
    if (_callback)
    {
        std::async(std::launch::async, _callback, map);
    }
}

void ExtendedKalmanFilter::processCorrection(const Measurements& measurements)
{
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
    }
    // record the last update time
    _lastUpdateTime = getCurrentTimeInSeconds();
    MapSummary map = summerizeMap();

    if (_callback)
    {
        std::async(std::launch::async, _callback, map);
    }
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
    if(!_model->directMeasurementModel(robotPose, landmarkPosition, expectedMeasurement))
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

    Eigen::Matrix3d hr = _model->getMeasurementToRobotJacobian(robotPose);
    Eigen::Matrix3d hl = _model->getMeasurementToMeasurementJacobian(robotPose);

    Eigen::MatrixXd h(l, r + l);
    h.block(0, 0, l, r) = hr;
    h.block(0, r, l, l) = hl; //-> TODO: verifyt this part

    Eigen::MatrixXd R = _model->getMeasurementNoise();

    Eigen::MatrixXd Z = h * p * h.transpose() + R;


    //===========================================================
    //3 Kalman gain, 
    Eigen::MatrixXd pBar(r + n * l, r + l);

    Eigen::MatrixXd pmr =  _slamMap->getRobotLandmarkFullCorrelationsVertical();   
    Eigen::MatrixXd pml =  _slamMap->getCrossLandmarkFullCorrelation(id); 


    pBar.block(0, 0, r, r) = prr;                   // Top-left block
    pBar.block(0, r, r, l) = prl;                   // Top-right block
    pBar.block(r, 0, n * l, r) = pmr;               // Top-left block
    pBar.block(r, r, n * l, l) = pml;               // Top-right block


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
    _model->setSensorInfo(transform);
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