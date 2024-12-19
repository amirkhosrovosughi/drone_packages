#include "filter/extended_kalman_filter.hpp"
#include <iostream>
#include <thread>
#include <future>
#include <chrono>

static const LogLevel HIGH_LEVEL = LogLevel::INFO;
static const LogLevel LOW_LEVEL = LogLevel::DEBUG;
static const std::string LOG_SUBSECTION = "[filter] - ";

ExtendedKalmanFilter::ExtendedKalmanFilter()
{

#ifdef POSITION_MOTION_POSITION_MEASUREMENT
    _model = std::make_shared<PositionPositionMotionMeasurementModel>();
#elif POSE_MOTION_POSITION_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#elif POSITION_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#elif POSE_MOTION_2D_MEASUREMENT
    throw std::runtime_error("have not implemented yet");
#else
    RCLCPP_ERROR(rclcpp::get_logger("slam"), "Have not define compile tag for motion model");
    throw std::runtime_error("motion measurement model is not specified");
#endif

    // _model = model; ==> TODO , define model in slamManager and pass it by constructor/setter
    _odometryType = _model->getOdometryType();

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

void ExtendedKalmanFilter::setLogger(LoggerPtr logger)
{
    _logger = logger;
}

void ExtendedKalmanFilter::processPrediction(const OdometryInfo& odom)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);

        //update mean
        double timeElapse = odom.timeTag - _lastUpdateTime;
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "_lastUpdateTime is: ", _lastUpdateTime);
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "timeElapse is: ", timeElapse);

        if (_odometryType == MotionMeasurementModel::OdometryType::PositionOdometry)
        {
            Velocity velocity = odom.NedVelocity; // change to EnuVelocity, use ned now for comparision and cross checking 
            Eigen::VectorXd linearVel(3);
            linearVel << velocity.linear.x, velocity.linear.y, velocity.linear.z;
            Eigen::VectorXd updatedRobotMean = _model->getRobotToRobotJacobian()*_slamMap->getRobotMean() + linearVel * timeElapse;
            _logger->log(LOW_LEVEL, LOG_SUBSECTION, "updatedRobotMean is: ", "\n", updatedRobotMean);
            _slamMap->setRobotMean(updatedRobotMean);
        }
        else if (_odometryType == MotionMeasurementModel::OdometryType::PoseOdometry)
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

        _robotQuaternion = odom.orientation; // TODO: verify is for more complex case

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Extended Kalman Filter prediction step" );
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

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Extended Kalman Filter correction step executed.");
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
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Update existing landmark: ",  id );
    // kalman steps
    //===========================================================
    // 1) calculate measurement error
    Pose robotPose;
    if (_odometryType == MotionMeasurementModel::OdometryType::PositionOdometry)
    {
        robotPose.position = Position(_slamMap->getRobotMean());
        robotPose.quaternion = _robotQuaternion;
    }
    else if (_odometryType == MotionMeasurementModel::OdometryType::PoseOdometry)
    {
        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        robotPose.position = Position(robotState.head(3));
        robotPose.quaternion = Quaternion(robotState.tail(4));
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Fail, Invalid motion model dimension.");
        return;
    }

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Robot position is:\n", robotPose.position.getPositionVector());

    Position landmarkPosition(_slamMap->getLandmarkMean(id));
    _logger->log(LOW_LEVEL, LOG_SUBSECTION,  "expected landmarkPosition is:", landmarkPosition.getPositionVector());

    Measurement expectedMeasurement;
    if(!_model->directObservationModel(robotPose, landmarkPosition, expectedMeasurement))
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Failed to update landmark, Cannot calculate the direct measurement model.");
        return;
    }

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurement position is:", measurement.position.getPositionVector());
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "expected Measurement is:", expectedMeasurement.position.getPositionVector(), "\n.");

    Eigen::Vector3d z = measurement.position.getPositionVector() - expectedMeasurement.position.getPositionVector(); // measurementError

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION,  "measurementError z is:", z);

    //===========================================================
    //2 Z -> calculate measurement error variance

    // add some check for compability of dimensions
    // construct P model
    Eigen::MatrixXd prr = _slamMap->getRobotCorrelation();
    Eigen::MatrixXd prl = _slamMap->getRobotlandmarkCorrelation(id);  
    Eigen::MatrixXd pll = _slamMap->getLandmarkSelfCorrelation(id);


    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementError prr is:\n", prr );
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementError prl is:\n", prl);
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementError pll is:\n", pll);

    int r = prr.rows();                   // robot state lenght
    int l = pll.cols();                   // landmark state length
    int n = _slamMap->getLandmarkCount(); // number of all landmarks 

    Eigen::MatrixXd p(r + l, r + l);
    // Set the blocks of matrix p
    p.block(0, 0, r, r) = prr;                   // Top-left block
    p.block(0, r, r, l) = prl;                   // Top-right block
    p.block(r, 0, l, r) = prl.transpose();       // Bottom-left block
    p.block(r, r, l, l) = pll;                   // Bottom-right block

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "BIG Matrix p is:\n", p);

    Eigen::Matrix3d hr = _model->getMeasurementToRobotJacobian(robotPose);
    Eigen::Matrix3d hl = _model->getMeasurementToMeasurementJacobian(robotPose);

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "hr is:\n", hr);
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "hl is:\n", hl);

    Eigen::MatrixXd h(l, r + l);
    h.block(0, 0, l, r) = hr;
    h.block(0, r, l, l) = hl; //-> TODO: verifyt this part

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementError h is::\n", h);

    Eigen::MatrixXd R = _model->getMeasurementNoise();
     _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementNoise R is::\n", R);

    Eigen::MatrixXd Z = h * p * h.transpose() + R;
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "measurementError Z is::\n", Z);


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
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "- K * z is:\n", (- K * z));
    _slamMap->mapMean = _slamMap->mapMean - K * z;

    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "updated _slamMap->mapMean is:\n", _slamMap->mapMean);
    
    // 5 update variance
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "(- K * Z * K.transpose()) is:\n", (- K * Z * K.transpose()));
    _slamMap->mapCorrelation = _slamMap->mapCorrelation - K * Z * K.transpose();
}

void ExtendedKalmanFilter::addLandmark(const Measurement& meas)
{
    Pose robotPose;  //AMIR --> move this block to another method getRobotPose
    if (_odometryType == MotionMeasurementModel::OdometryType::PositionOdometry)
    {
        robotPose.position = Position(_slamMap->getRobotMean());
        robotPose.quaternion = _robotQuaternion;
    }
    else if (_odometryType == MotionMeasurementModel::OdometryType::PoseOdometry)
    {
        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        robotPose.position = Position(robotState.head(3));
        robotPose.quaternion = Quaternion(robotState.tail(4));
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Fail, Invalid motion model dimension.");
        return;
    }
    _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Adding landmark with measurement:\n", meas.position.getPositionVector());
    Position newLandmarkPosition = _model->inverseObservationModel(robotPose, meas);
    _slamMap->addLandmak(newLandmarkPosition.getPositionVector());
}

void ExtendedKalmanFilter::setSensorInfo(const Eigen::Matrix4d& transform)
{
    _model->setSensorInfo(transform);
}

MapSummary ExtendedKalmanFilter::summerizeMap()
{
    MapSummary mapSummary;
    if (_odometryType == MotionMeasurementModel::OdometryType::PositionOdometry)
    {
        mapSummary.robot.pose.position = Position(_slamMap->getRobotMean());
        mapSummary.robot.pose.quaternion = _robotQuaternion;
    }
    else if (_odometryType == MotionMeasurementModel::OdometryType::PoseOdometry)
    {
        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        mapSummary.robot.pose.position = Position(robotState.head(3));
        mapSummary.robot.pose.quaternion = Quaternion(robotState.tail(4));
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Fail, Invalid motion model dimension.");
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