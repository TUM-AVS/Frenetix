#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

struct PoseWithCovariance
{
    PoseWithCovariance();
    PoseWithCovariance(
        const Eigen::Vector3d& position,
        const Eigen::Vector4d& orientation,
        const Eigen::Matrix<double,6,6>& covariance
    );

    Eigen::Vector3d position;
    Eigen::Vector4d orientation;

    Eigen::Matrix<double,6,6> covariance;

    friend std::ostream& operator<<(std::ostream& os, const PoseWithCovariance& pose);
};


struct PredictedObject
{
    PredictedObject(size_t length);
    PredictedObject(
        int object_id,
        std::vector<PoseWithCovariance> predictedPath
    );




    int object_id;
    std::vector<PoseWithCovariance> predictedPath;

    friend std::ostream& operator<<(std::ostream& os, const PredictedObject& obj);
};
