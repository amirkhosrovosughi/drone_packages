#ifndef SLAM__MAP_
#define SLAM__MAP_

#include <vector>
#include <Eigen/Dense>
#include <iostream>

//(CAUTION) -> we go with assumption that index of landmark on Map is equal id, be caution if this assumption changes later

/**
 * @brief Represents the SLAM state (robot + landmarks) using mean vector and covariance matrix.
 *
 * Maintains robot pose mean and covariance, as well as all landmark means and correlations.
 * Provides methods for adding landmarks and updating/retrieving sub-blocks of the covariance matrix.
 *
 * @note Assumes landmark index in the map equals its ID.
 */
struct SlamMap
{
    int landmarkCount; ///< Current number of landmarks in the map
    int robotDimension; ///< State dimension of the robot
    int landmarkDimension; ///< State dimension of each landmark

    Eigen::VectorXd mapMean; ///< Full state mean (robot + landmarks)
    Eigen::MatrixXd mapCorrelation; ///< Full state covariance

    Eigen::MatrixXd initialLandmarkSelfCorrelation; ///< Initial covariance for a single landmark
    Eigen::MatrixXd initialLandmarkCrossCorrelation; ///< Initial cross-covariance between landmarks
    Eigen::MatrixXd initialRobotLandmarkCorrelation; ///< Initial cross-covariance between robot and landmark

    /**
     * @brief Construct a SlamMap with given robot and landmark state dimensions.
     */
    SlamMap(int _robotDimension, int _landmarkDimension);

    /**
     * @brief Add a new landmark into the map.
     * @param newLandmark State vector of the landmark
     * @throws std::invalid_argument if dimension mismatch
     */
    void addLandmark(const Eigen::VectorXd& newLandmark);


    int getLandmarkCount();

    Eigen::VectorXd getRobotMean();

    bool setRobotMean(const Eigen::VectorXd &updatedRobotMean);

    Eigen::MatrixXd getRobotCorrelation();

    bool setRobotCorrelation(const Eigen::MatrixXd &updatedRobotCorrelation);

    Eigen::VectorXd getLandmarkMean(const int indexLandmark);

    bool setLandmarkMean(const Eigen::VectorXd &updatedLandmarkMean, const int indexLandmark);

    Eigen::MatrixXd getLandmarkSelfCorrelation(const int indexLandmark);

    /**
     * @brief Set self-covariance of a landmark.
     * @param indexLandmark Landmark index
     * @param corr Self-covariance block
     */
    bool setLandmarkSelfCorrelation(const Eigen::MatrixXd &updatedLandmarkSelfCorrelation, const int indexLandmark);

    /**
     * @brief Get cross-covariance between two landmarks.
     * @param indexLandmark1 First landmark index
     * @param indexLandmark2 Second landmark index
     * @return Eigen::MatrixXd Cross-covariance block
     */
    Eigen::MatrixXd getLandmarkCrossCorrelation(const int IndexLandmark1, const int IndexLandmark2);

    /**
     * @brief Set cross-covariance between two landmarks.
     * @param indexLandmark1 First landmark index
     * @param indexLandmark2 Second landmark index
     * @param corr Cross-covariance block
     */
    bool setLandmarkCrossCorrelation(const Eigen::MatrixXd &updatedLandmarkCrossCorrelation, const int  IndexLandmark1, const int IndexLandmark2);
 
    Eigen::MatrixXd getMapMean();

    /**
     * @brief Get the full map covariance matrix.
     * @return Eigen::MatrixXd Map covariance
     */
    Eigen::MatrixXd getMapCorrelation();

    bool setMapMean(const Eigen::MatrixXd &inputVector);

     /**
     * @brief Set the full map covariance matrix.
     * @param corr New covariance matrix
     */
    bool setMapCorrelation(Eigen::MatrixXd &inputMatrix);

    /**
     * @brief Get covariance between robot and a landmark.
     * @param indexLandmark Landmark index
     * @return Eigen::MatrixXd Covariance block
     */
    Eigen::MatrixXd getRobotLandmarkCorrelation(int indexLandmark);

    /**
     * @brief Set covariance between robot and a landmark.
     * @param indexLandmark Landmark index
     * @param corr Covariance block
     */
    bool setRobotLandmarkCorrelation(const Eigen::MatrixXd &correlation, const int indexLandmark);

    /**
     * @brief Get self-covariance of a landmark.
     * @param indexLandmark Landmark index
     * @return Eigen::MatrixXd Self-covariance block
     */
    Eigen::MatrixXd getRobotLandmarkFullCorrelationsVertical();

    void setRobotLandmarkFullCorrelationsVertical(const Eigen::MatrixXd& inputMatrix);

    Eigen::MatrixXd getRobotLandmarkFullCorrelationsHorizontal();

    void setRobotLandmarkFullCorrelationsHorizontal(const Eigen::MatrixXd& inputMatrix);

    Eigen::MatrixXd getCrossLandmarkFullCorrelation(const int indexLandmark);

};

#endif  // SLAM__MAP_