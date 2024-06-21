#ifndef SLAM__MAP_
#define SLAM__MAP_

#include <vector>
#include <Eigen/Dense>

const static double robotInitialCorrelation = 0.1;
const static double landmarkInitialCorrelation = 2;

struct SlamMap
{
    int landmakrCount;
    int robotDimension;
    int landmarkDimension;

    Eigen::VectorXd robotMean;
    std::vector<Eigen::MatrixXd> landmarkMean;

    Eigen::MatrixXd robotCorrelation;
    std::vector<Eigen::MatrixXd> landmarkSelfCorrelation;
    std::vector<Eigen::MatrixXd> landmarkCrossCorrelation;
    std::vector<Eigen::MatrixXd> robotlandmarkCorrelation;

    Eigen::MatrixXd initialLandmarkSelfCorrelation;
    Eigen::MatrixXd initialLandmarkCrossCorrelation;
    Eigen::MatrixXd initialRobotLandmarkCorrelation;

    SlamMap(int _robotDimension, int _landmarkDimension):
    landmakrCount(0),
    robotDimension(_robotDimension),
    landmarkDimension(_landmarkDimension),
    robotMean(_robotDimension),
    robotCorrelation(_robotDimension, _robotDimension),
    initialLandmarkSelfCorrelation(_landmarkDimension, _landmarkDimension),
    initialLandmarkCrossCorrelation(_landmarkDimension, _landmarkDimension),
    initialRobotLandmarkCorrelation(_robotDimension, _landmarkDimension)
    {
        robotMean.setZero();
        robotCorrelation = robotInitialCorrelation * Eigen::MatrixXd::Identity(_robotDimension, _robotDimension);

        initialLandmarkSelfCorrelation = landmarkInitialCorrelation * Eigen::MatrixXd::Identity(_landmarkDimension, _landmarkDimension);
        initialLandmarkCrossCorrelation.setZero();
        initialRobotLandmarkCorrelation.setZero();
    }

    void addLandmak(const Eigen::VectorXd& newLandmark)
    {
        if (newLandmark.size() == landmarkDimension)
        {
            landmakrCount++;
            landmarkMean.push_back(newLandmark);

            robotlandmarkCorrelation.push_back(initialRobotLandmarkCorrelation);
            landmarkSelfCorrelation.push_back(initialLandmarkSelfCorrelation);
            
            for (int i = 0; i > landmakrCount -1; i++)
            {
                landmarkCrossCorrelation.push_back(initialLandmarkCrossCorrelation);
            }
            
        } else
        {
            std::cerr << "Error: Landmark size does not match specified dimensions (" << landmarkDimension << ")." << std::endl;
        }
    }

    Eigen::VectorXd getRobotMean()
    {
        return robotMean;
    }

    bool setRobotMean(const Eigen::VectorXd &updatedRobotMean)
    {
        robotMean = updatedRobotMean;
        return true;
    }

    Eigen::MatrixXd getRobotCorrelation()
    {
        return robotCorrelation;
    }

    bool setRobotCorrelation(const Eigen::MatrixXd &updatedRobotCorrelation)
    {
        robotCorrelation = updatedRobotCorrelation;
        return true;
    }

    Eigen::VectorXd getLandmarkMean(const int index)
    {
        if (index < 0 || index > landmakrCount)
        {
            throw std::invalid_argument( "received wrong index" );
        }

        return landmarkMean[index];
    }

    bool setLandmarkMean(const Eigen::VectorXd &updatedLandmarkMean, const int index)
    {
        if (index < 0 || index > landmakrCount)
        {
            throw std::invalid_argument( "received wrong index" );
        }

        landmarkMean[index] = updatedLandmarkMean;
        return true;
    }


    Eigen::MatrixXd getLandmarkSelfCorrelation(const int index)
    {
        if (index < 0 || index > landmakrCount)
        {
            throw std::invalid_argument( "received wrong index" );
        }

        return landmarkSelfCorrelation[index];
    }

    bool setLandmarkSelfCorrelation(const Eigen::MatrixXd &updatedLandmarkSelfCorrelation, const int index)
    {
        if (index < 0 || index > landmakrCount)
        {
            throw std::invalid_argument( "received wrong index" );
        }

        landmarkSelfCorrelation[index] = updatedLandmarkSelfCorrelation;
        return true;
    }

    Eigen::MatrixXd getLandmarkCrossCorrelation(const int row, const int col)
    {
        if (std::min(row, col) < 0 || std::max(row, col) > landmakrCount)
        {
            throw std::invalid_argument( "received wrong row/col" );
        }

        return landmarkCrossCorrelation[findCrossLanddmarkCovBlockIndex(row, col)];
    }

    bool setLandmarkCrossCorrelation(const Eigen::MatrixXd &updatedLandmarkCrossCorrelation, const int  row, const int col)
    {
        if (std::min(row, col) < 0 || std::max(row, col) > landmakrCount)
        {
            throw std::invalid_argument( "received wrong row/col" );
        }

        landmarkCrossCorrelation[findCrossLanddmarkCovBlockIndex(row, col)] = updatedLandmarkCrossCorrelation;
        return true;
    }

    bool getMapMean(Eigen::MatrixXd &outputVector)
    {
        // construct big mean matrix
        return true;
    }

    bool getMapCorrelationMatrix(Eigen::MatrixXd &outputMatrix)
    {
        // Construct big correlation Matrix
        return true;
    }

    bool updateFromMapMean(const Eigen::MatrixXd &inputVector)
    {
        // updated states mean from big map mean
        return true;
    }

    bool updateFromMapCorrelationMatrix(Eigen::MatrixXd &inputMatrix)
    {
        // updated states from big map corollation
        return true;
    }

    int findCrossLanddmarkCovBlockIndex(const int row, const int col)
    {
        if (row >= col)
        {
            std::cerr << "column number ("<< col <<") should be bigger than row (" << row << ")." << std::endl;
            throw std::invalid_argument( "received wrong numbers" );
        }

        std::cout << "index of (row,col)= ("<< row << ","<< col <<") is: " << (col - 2)*(col - 2) / 2  + row << "\n";
        return (col - 2)*(col - 2) / 2  + row;
    }

};

#endif  // SLAM__MAP_