#include "filter/extended_kalman_filter.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>
#include <future>
#include <chrono>

#ifdef STORE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif

static const LogLevel HIGH_LEVEL = LogLevel::INFO;
static const LogLevel LOW_LEVEL = LogLevel::DEBUG;
static const std::string LOG_SUBSECTION = "[filter] - ";
static const int LANDMARK_DIMENSION = 3;

namespace
{
constexpr int kAbsPosObservedDim = 3;

// Clamp bounds for absolute-position 1-sigma values (metres).
constexpr double kAbsPosSigmaXyMinM = 0.2;
constexpr double kAbsPosSigmaXyMaxM = 20.0;
constexpr double kAbsPosSigmaZMinM  = 0.3;
constexpr double kAbsPosSigmaZMaxM  = 30.0;

double sanitizeAndClampSigma(double sigmaM, double minSigmaM, double maxSigmaM)
{
    if (!std::isfinite(sigmaM))
    {
        return maxSigmaM;
    }
    return std::clamp(sigmaM, minSigmaM, maxSigmaM);
}
}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(
    std::shared_ptr<MotionModel> motionModel)
    : _motionModel(std::move(motionModel))
{
    _slamMap = std::make_shared<SlamMap>(_motionModel->getStateDimension(), LANDMARK_DIMENSION);
}

void ExtendedKalmanFilter::prediction(const PredictionInput& predictionInput)
{
    processPrediction(predictionInput);
}

void ExtendedKalmanFilter::correction(const AssignedMeasurements& meas)
{
    processCorrection(meas);
}

void ExtendedKalmanFilter::registerCallback(std::function<void(const MapSummary& map)> callback)
{
    _callback = callback;
}

void ExtendedKalmanFilter::setLogger(LoggerPtr logger)
{
    _logger = logger;
}

void ExtendedKalmanFilter::setSensorInfo(const Eigen::Matrix4d& transform)
{
    // TODO: Currently no-op. If sensor extrinsics are needed by measurement models,
    // the ObservationBuilder or MeasurementFactory should be updated to pass them.
    (void)transform;
}

void ExtendedKalmanFilter::processPrediction(const PredictionInput& predictionInput)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);

        #ifdef STORE_DEBUG_DATA
        std::map<std::string, double> mapLog;
        #endif

        Eigen::VectorXd robotState = _slamMap->getRobotMean();
        Eigen::VectorXd updatedRobotMean = _motionModel->propagate(robotState, predictionInput.deltaPosition);
        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "delta position is: ", "\n", predictionInput.deltaPosition);
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "updatedRobotMean is: ", "\n", updatedRobotMean);

        _slamMap->setRobotMean(updatedRobotMean);

        #ifdef STORE_DEBUG_DATA
        mapLog["updatedRobotMean[0]"] = updatedRobotMean[0];
        mapLog["updatedRobotMean[1]"] = updatedRobotMean[1];
        mapLog["updatedRobotMean[2]"] = updatedRobotMean[2];
        #endif

        Eigen::MatrixXd F = _motionModel->computeStateJacobian(robotState, predictionInput.deltaPosition);
        Eigen::MatrixXd Q = _motionModel->getProcessNoise();
        Eigen::MatrixXd updatedRobotCorrelation = F * _slamMap->getRobotCorrelation() * F.transpose() + Q;
        _slamMap->setRobotCorrelation(updatedRobotCorrelation);

        Eigen::MatrixXd updatedRobotLandmarkFullCorrelationsHorizontal = F * _slamMap->getRobotLandmarkFullCorrelationsHorizontal();
        _slamMap->setRobotLandmarkFullCorrelationsHorizontal(updatedRobotLandmarkFullCorrelationsHorizontal);
        _slamMap->setRobotLandmarkFullCorrelationsVertical(updatedRobotLandmarkFullCorrelationsHorizontal.transpose());

        // Convert Eigen quaternion to custom Quaternion type if needed
        _robotQuaternion = Quaternion(predictionInput.orientation.w(), predictionInput.orientation.x(), predictionInput.orientation.y(), predictionInput.orientation.z());

        #ifdef STORE_DEBUG_DATA
        mapLog["robotQuaternion.w"] = _robotQuaternion.w;
        mapLog["robotQuaternion.x"] = _robotQuaternion.x;
        mapLog["robotQuaternion.y"] = _robotQuaternion.y;
        mapLog["robotQuaternion.z"] = _robotQuaternion.z;

        data_logging_utils::DataLogger::log(mapLog);
        #endif

        _logger->log(LOW_LEVEL, LOG_SUBSECTION, "Extended Kalman Filter prediction step" );
    }

    MapSummary map = summarizeMap();
    if (_callback)
    {
        _callback(map);
    }
}

void ExtendedKalmanFilter::processCorrection(const AssignedMeasurements& assignedMeasurements)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (const AssignedMeasurement& meas : assignedMeasurements)
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
    MapSummary map = summarizeMap();
    if (_callback)
    {
        _callback(map);
    }
}

void ExtendedKalmanFilter::updateLandmark(const AssignedMeasurement& measurement)
{
    int id = measurement.id;

    if (_landmarkObservationCount.find(id) != _landmarkObservationCount.end())
    {
        _landmarkObservationCount[id]++;
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Update existing landmark: ",  id, ", Observed ", _landmarkObservationCount[id], " times");
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Record for landmark with id: ",  id , " does not exist");
    }
    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Update existing landmark: ",  id );
    // kalman steps
    //===========================================================
    // 1) calculate measurement error
    Pose robotPose;
    robotPose.position = Position(_slamMap->getRobotMean());
    robotPose.quaternion = _robotQuaternion;

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Robot position is:\n", robotPose.position.getPositionVector());

    Position landmarkPosition(_slamMap->getLandmarkMean(id));
    _logger->log(LOW_LEVEL, LOG_SUBSECTION,  "expected landmarkPosition is:", landmarkPosition.getPositionVector());

    // Use the measurement-specific model attached to the assigned measurement
    if (!measurement.measurement.model)
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Assigned measurement has no measurement model");
        return;
    }

    auto model = measurement.measurement.model;

    // Predicted measurement from current state
    Measurement expectedMeasurement = model->predict(robotPose, landmarkPosition);

    // Actual measurement vector
    Eigen::VectorXd z = measurement.measurement.payload;
    Eigen::VectorXd z_hat = expectedMeasurement.payload;

    // Innovation (residual)
    Eigen::VectorXd r = z - z_hat;

    // Measurement Jacobians
    Eigen::MatrixXd H_r = model->jacobianWrtRobot(robotPose, landmarkPosition); // (m x robotDim)
    Eigen::MatrixXd H_l = model->jacobianWrtLandmark(robotPose, landmarkPosition); // (m x landmarkDim)

    // Measurement noise
    Eigen::MatrixXd R = model->measurementNoise(); // (m x m)

    // State covariances
    Eigen::MatrixXd P_rr = _slamMap->getRobotCorrelation(); // (robotDim x robotDim)
    Eigen::MatrixXd P_rl_full = _slamMap->getRobotLandmarkFullCorrelationsHorizontal(); // (robotDim x landmarkDim*L)
    int robotDim = P_rr.rows();
    int landmarkDim = H_l.cols();
    int col_offset = id * landmarkDim;
    Eigen::MatrixXd P_rl = P_rl_full.block(0, col_offset, robotDim, landmarkDim);
    Eigen::MatrixXd P_lr = P_rl.transpose();
    Eigen::MatrixXd P_ll = _slamMap->getLandmarkSelfCorrelation(id);

    // Innovation covariance S = H_r P_rr H_r^T + H_r P_rl H_l^T + H_l P_lr H_r^T + H_l P_ll H_l^T + R
    Eigen::MatrixXd S = H_r * P_rr * H_r.transpose()
                        + H_r * P_rl * H_l.transpose()
                        + H_l * P_lr * H_r.transpose()
                        + H_l * P_ll * H_l.transpose()
                        + R;

    // Check S invertible
    Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
    if (!lu.isInvertible())
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Innovation covariance S is singular, skipping update for landmark ", id);
        return;
    }

    Eigen::MatrixXd S_inv = S.inverse();

    // Kalman gains for robot and landmark blocks
    Eigen::MatrixXd K_r = (P_rr * H_r.transpose() + P_rl * H_l.transpose()) * S_inv; // (robotDim x m)
    Eigen::MatrixXd K_l = (P_lr * H_r.transpose() + P_ll * H_l.transpose()) * S_inv; // (landmarkDim x m)

    // State updates
    Eigen::VectorXd delta_robot = K_r * r; // (robotDim)
    Eigen::VectorXd delta_landmark = K_l * r; // (landmarkDim)

    // Apply state mean updates
    Eigen::VectorXd robotMean = _slamMap->getRobotMean();
    robotMean += delta_robot;
    _slamMap->setRobotMean(robotMean);

    Eigen::VectorXd lmMean = _slamMap->getLandmarkMean(id);
    lmMean += delta_landmark;
    _slamMap->setLandmarkMean(lmMean, id);

    // Covariance updates (Joseph form simplified)
    Eigen::MatrixXd P_rr_new = P_rr - K_r * S * K_r.transpose();
    Eigen::MatrixXd P_ll_new = P_ll - K_l * S * K_l.transpose();
    Eigen::MatrixXd P_rl_new = P_rl - K_r * S * K_l.transpose();

    // Commit covariance updates
    _slamMap->setRobotCorrelation(P_rr_new);
    _slamMap->setLandmarkSelfCorrelation(P_ll_new, id);
    _slamMap->setRobotLandmarkCorrelation(P_rl_new, id);

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Completed EKF correction for landmark ", id);

}

void ExtendedKalmanFilter::addLandmark(const AssignedMeasurement& meas)
{
    int id = meas.id;
    if (_landmarkObservationCount.find(id) == _landmarkObservationCount.end())
    {
        _landmarkObservationCount[id] = 1;
    }
    else
    {
        _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Landmark id ",  id , " already exists");
    }

    Pose robotPose;
    robotPose.position = Position(_slamMap->getRobotMean());
    robotPose.quaternion = _robotQuaternion;

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Adding landmark with measurement payload. (payload not shown)");
    Position newLandmarkPosition;
    if (meas.hasInitializedPosition)
    {
        newLandmarkPosition = meas.position;
    }
    else
    {
        if (!meas.measurement.model) {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Assigned measurement has no model; cannot initialize landmark");
            return;
        }

        auto model = meas.measurement.model;
        auto optLandmarkPosition = model->inverse(robotPose, meas.measurement);
        if (!optLandmarkPosition) {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Failed to initialize landmark position (inverse failed)");
            return;
        }
        newLandmarkPosition = *optLandmarkPosition;
    }
    // TODO: // Landmark aging & pruning (for dynamic environments, spurious detections)

    _logger->log(HIGH_LEVEL, LOG_SUBSECTION, "Landmark added with coordinate:\n", newLandmarkPosition.getPositionVector());
    _slamMap->addLandmark(newLandmarkPosition.getPositionVector());
}

void ExtendedKalmanFilter::applyAbsolutePositionCorrection(const AbsolutePositionConstraint& constraint)
{
    // -----------------------------------------------------------------------
    // Full-joint EKF GPS position update
    //
    // State layout: mu = [x, y, z, ...(robot)..., l0_x, l0_y, l0_z, ...]
    // Measurement:  z  = enu_x, enu_y, enu_z  (3-vector)
    // Observation:  H  = [I_3 | 0_{3 x (totalDim - 3)}]  selects robot x,y,z
    // Noise:        R  = diag(sigmaXyM^2, sigmaXyM^2, sigmaZM^2)
    //
    // S = H P H^T + R
    // K = P H^T S^{-1}           (totalDim x 3)
    // mu_new = mu + K (z - H mu)
    // P_new  = (I - K H) P       (Joseph form not needed: R is diagonal, S
    //                             invertibility checked below)
    // -----------------------------------------------------------------------
    MapSummary map;
    {
        std::lock_guard<std::mutex> lock(_mutex);

        const int robotDim = _motionModel->getStateDimension();
        const Eigen::MatrixXd mu_full = _slamMap->getMapMean();
        const int totalDim = static_cast<int>(mu_full.rows());

        if (robotDim < kAbsPosObservedDim)
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION,
                "Absolute position correction skipped: robot state dimension < 3");
            return;
        }

        if (totalDim < robotDim)
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION,
                "Absolute position correction skipped: state not yet initialized");
            return;
        }

        // Observation matrix H (3 x totalDim): selects robot position block only.
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(kAbsPosObservedDim, totalDim);
        H.block<kAbsPosObservedDim, kAbsPosObservedDim>(0, 0) =
            Eigen::Matrix3d::Identity();

        // Measurement noise R
        const double sigmaXyM = sanitizeAndClampSigma(
            constraint.sigmaXyM,
            kAbsPosSigmaXyMinM,
            kAbsPosSigmaXyMaxM);
        const double sigmaZM = sanitizeAndClampSigma(
            constraint.sigmaZM,
            kAbsPosSigmaZMinM,
            kAbsPosSigmaZMaxM);

        const double sxy2 = sigmaXyM * sigmaXyM;
        const double sz2  = sigmaZM  * sigmaZM;
        Eigen::Matrix3d R = Eigen::Vector3d(sxy2, sxy2, sz2).asDiagonal();

        // Innovation covariance S
        Eigen::MatrixXd P_full = _slamMap->getMapCorrelation();
        Eigen::Matrix3d S = H * P_full * H.transpose() + R;

        // Guard against singular S
        Eigen::FullPivLU<Eigen::Matrix3d> lu(S);
        if (!lu.isInvertible())
        {
            _logger->log(HIGH_LEVEL, LOG_SUBSECTION,
                "Absolute position correction: innovation covariance S is singular, skipping");
            return;
        }

        const Eigen::Matrix3d S_inv = S.inverse();

        // Kalman gain K (totalDim x 3)
        const Eigen::MatrixXd K = P_full * H.transpose() * S_inv;

        // Innovation  (z - H mu)
        const Eigen::Vector3d z = constraint.enuPosition;
        const Eigen::Vector3d innovation = z - H * mu_full;

        // State update
        const Eigen::MatrixXd mu_new = mu_full + K * innovation;
        _slamMap->setMapMean(mu_new);

        // Covariance update  P = (I - K H) P
        const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(totalDim, totalDim) - K * H;
        Eigen::MatrixXd P_new = IKH * P_full;
        _slamMap->setMapCorrelation(P_new);

        _logger->log(HIGH_LEVEL, LOG_SUBSECTION,
            "Absolute position correction applied: innovation=[",
            innovation.x(), ", ", innovation.y(), ", ", innovation.z(),
            "], sigma_xy=", sigmaXyM,
            ", sigma_z=", sigmaZM);

        map = summarizeMap();
    }

    if (_callback)
    {
        _callback(map);
    }
}

MapSummary ExtendedKalmanFilter::summarizeMap()
{
    MapSummary mapSummary;
    mapSummary.robot.pose.position = Position(_slamMap->getRobotMean());
    mapSummary.robot.pose.quaternion = _robotQuaternion;

    Eigen::MatrixXd prr = _slamMap->getRobotCorrelation();
    mapSummary.robot.variance = Variance2D(prr(0,0), prr(1,1), prr(0,1));

    for(int i = 0; i < _slamMap->getLandmarkCount(); i++)
    {
        Landmark landmark;
        landmark.id = i; //(CAUTION) -> we go with assumption that index of landmark on Map is equal id, be caution if this assumption changes later
        landmark.observeRepeat = _landmarkObservationCount[landmark.id];
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

MapSummary ExtendedKalmanFilter::getMap()
{
    return summarizeMap();
}

void ExtendedKalmanFilter::reset()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_slamMap) {
        int robotDim = _motionModel->getStateDimension();
        int landmarkDim = 3; // or use previous value
        _slamMap = std::make_shared<SlamMap>(robotDim, landmarkDim);
    }
    _landmarkObservationCount.clear();
    // Optionally reset robot orientation, logger, etc.
}