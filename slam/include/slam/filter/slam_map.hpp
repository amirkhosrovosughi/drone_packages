#ifndef SLAM__MAP_
#define SLAM__MAP_

#include <vector>
#include <Eigen/Dense>
#include <iostream>

const static double robotInitialCorrelation = 0.1;
const static double landmarkInitialCorrelation = 2;

//(CAUTION) -> we go with assumption that index of landmark on Map is equal id, be caution if this assumption changes later
struct SlamMap
{
    int landmarkCount;
    int robotDimension;
    int landmarkDimension;

    Eigen::VectorXd mapMean;
    Eigen::MatrixXd mapCorrelation;

    Eigen::MatrixXd initialLandmarkSelfCorrelation;
    Eigen::MatrixXd initialLandmarkCrossCorrelation;
    Eigen::MatrixXd initialRobotLandmarkCorrelation;

    SlamMap(int _robotDimension, int _landmarkDimension):
    landmarkCount(0),
    robotDimension(_robotDimension),
    landmarkDimension(_landmarkDimension),
    initialLandmarkSelfCorrelation(_landmarkDimension, _landmarkDimension),
    initialLandmarkCrossCorrelation(_landmarkDimension, _landmarkDimension),
    initialRobotLandmarkCorrelation(_robotDimension, _landmarkDimension),
    mapMean(_robotDimension),
    mapCorrelation(_robotDimension, _robotDimension)
    {
        mapMean.setZero();
        mapCorrelation = robotInitialCorrelation * Eigen::MatrixXd::Identity(robotDimension, robotDimension);

        initialLandmarkSelfCorrelation = landmarkInitialCorrelation * Eigen::MatrixXd::Identity(_landmarkDimension, _landmarkDimension);
        initialLandmarkCrossCorrelation.setZero();
        initialRobotLandmarkCorrelation.setZero();
    }

    void addLandmak(const Eigen::VectorXd& newLandmark)
    {
        if (newLandmark.size() == landmarkDimension)
        {
            landmarkCount++;

            int oldDimension = mapMean.size();
            int newDimension = mapMean.size() + landmarkDimension;
            Eigen::VectorXd mapMeanTemp(newDimension);
            mapMeanTemp.segment(0, oldDimension) = mapMean;
            mapMeanTemp.segment(oldDimension, landmarkDimension) = newLandmark;
            mapMean = mapMeanTemp;

            Eigen::MatrixXd mapCorrelationTemp(newDimension, newDimension);
            mapCorrelationTemp.block(0, 0, oldDimension, oldDimension) = mapCorrelation;

            mapCorrelationTemp.block(0, oldDimension, robotDimension, landmarkDimension) = initialRobotLandmarkCorrelation;
            mapCorrelationTemp.block(oldDimension, 0, landmarkDimension, robotDimension) = initialRobotLandmarkCorrelation.transpose();

            mapCorrelationTemp.block(oldDimension, oldDimension, landmarkDimension, landmarkDimension) = initialLandmarkSelfCorrelation;
            
            for (int i = 0; i > landmarkCount -1; i++)
            {
                mapCorrelationTemp.block(robotDimension + i * landmarkDimension, oldDimension, landmarkDimension, landmarkDimension) = initialLandmarkCrossCorrelation;
                mapCorrelationTemp.block(oldDimension, robotDimension + i * landmarkDimension, landmarkDimension, landmarkDimension) = initialLandmarkCrossCorrelation.transpose();
            }
            mapCorrelation = mapCorrelationTemp;
            
        } else
        {
            std::cerr << "Error: Landmark size does not match specified dimensions (" << landmarkDimension << ")." << std::endl;
            throw std::invalid_argument( "received landmark with wrong dimension" );
        }
    }

    int getLandmarkCount()
    {
        return landmarkCount;
    }

    Eigen::VectorXd getRobotMean()
    {
        return mapMean.segment(0, robotDimension);
    }

    bool setRobotMean(const Eigen::VectorXd &updatedRobotMean)
    {
        if (updatedRobotMean.size() == robotDimension)
        {
            mapMean.segment(0, updatedRobotMean.size()) = updatedRobotMean;
        }
        else
        {
            std::cerr << "Error: Robot state size does not match specified dimensions (" << robotDimension << ")." << std::endl;
            throw std::invalid_argument( "received wrong diemnsion input" );
        }
        return true;
    }

    Eigen::MatrixXd getRobotCorrelation()
    {
        return mapCorrelation.block(0, 0, robotDimension, robotDimension);
    }

    bool setRobotCorrelation(const Eigen::MatrixXd &updatedRobotCorrelation)
    {
        mapCorrelation.block(0, 0, robotDimension, robotDimension) = updatedRobotCorrelation;
        return true;
    }

    Eigen::VectorXd getLandmarkMean(const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        return mapMean.segment(robotDimension + indexLandmark * landmarkDimension , landmarkDimension);
    }

    bool setLandmarkMean(const Eigen::VectorXd &updatedLandmarkMean, const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        mapMean.segment(robotDimension + indexLandmark * landmarkDimension , landmarkDimension) = updatedLandmarkMean;
        return true;
    }

    Eigen::MatrixXd getLandmarkSelfCorrelation(const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        int startIndex = robotDimension + indexLandmark * landmarkDimension;
        return mapCorrelation.block(startIndex , startIndex, landmarkDimension, landmarkDimension);
    }

    bool setLandmarkSelfCorrelation(const Eigen::MatrixXd &updatedLandmarkSelfCorrelation, const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        int startIndex = robotDimension + indexLandmark * landmarkDimension;
        mapCorrelation.block(startIndex, startIndex, landmarkDimension, landmarkDimension) = updatedLandmarkSelfCorrelation;
        return true;
    }

    Eigen::MatrixXd getLandmarkCrossCorrelation(const int IndexLandmark1, const int IndexLandmark2)
    {
        if (std::min(IndexLandmark1, IndexLandmark2) < 0 || std::max(IndexLandmark1, IndexLandmark2) > landmarkCount)
        {
            throw std::invalid_argument( "received wrong landmarks index" );
        }

        int row = robotDimension + IndexLandmark1 * landmarkDimension;
        int col = robotDimension + IndexLandmark1 * landmarkDimension;
        return mapCorrelation.block(row , col, landmarkDimension, landmarkDimension);
    }

    bool setLandmarkCrossCorrelation(const Eigen::MatrixXd &updatedLandmarkCrossCorrelation, const int  IndexLandmark1, const int IndexLandmark2)
    {
        if (std::min(IndexLandmark1, IndexLandmark2) < 0 || std::max(IndexLandmark1, IndexLandmark2) > landmarkCount)
        {
            throw std::invalid_argument( "received wrong landmarks index" );
        }

        int row = robotDimension + IndexLandmark1 * landmarkDimension;
        int col = robotDimension + IndexLandmark1 * landmarkDimension;
        mapCorrelation.block(row , col, landmarkDimension, landmarkDimension) = updatedLandmarkCrossCorrelation;
        return true;
    }
 
    Eigen::MatrixXd getMapMean()
    {
        return mapMean;
    }

    Eigen::MatrixXd getMapCorrelation()
    {
        return mapCorrelation;
    }

    bool setMapMean(const Eigen::MatrixXd &inputVector)
    {
        mapMean = inputVector;
        return true;
    }

    bool setmapCorrelation(Eigen::MatrixXd &inputMatrix)
    {
        mapCorrelation = inputMatrix;
        return true;
    }

    Eigen::MatrixXd getRobotlandmarkCorrelation(int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        int col = robotDimension + indexLandmark * landmarkDimension;
        return mapCorrelation.block(0 , col, robotDimension, landmarkDimension);
    }

    bool setRobotlandmarkCorrelation(const Eigen::MatrixXd &correlation, const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        int col = robotDimension + indexLandmark * landmarkDimension;
        mapCorrelation.block(0 , col, robotDimension, landmarkDimension) = correlation;
        return true;
    }

    Eigen::MatrixXd getRobotLandmarkFullCorrelations()
    {
        return mapCorrelation.block(robotDimension, 0, landmarkDimension * landmarkCount, robotDimension);
    }

    Eigen::MatrixXd getCrossLandmarkFullCorrelation(const int indexLandmark)
    {
        if (indexLandmark < 0 || indexLandmark > landmarkCount)
        {
            throw std::invalid_argument( "received wrong index landmark" );
        }

        int col = robotDimension + indexLandmark * landmarkDimension;
        return mapCorrelation.block(robotDimension, col, landmarkDimension * landmarkCount, landmarkDimension);
    }

};

#endif  // SLAM__MAP_