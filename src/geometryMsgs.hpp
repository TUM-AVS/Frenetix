#pragma once

#include <stddef.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

struct PoseWithCovariance
{
    PoseWithCovariance();
    PoseWithCovariance(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Matrix<double,6,6>& covariance
    );

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    Eigen::Matrix<double,6,6> covariance;

    friend std::ostream& operator<<(std::ostream& os, const PoseWithCovariance& pose);
};


struct PredictedObject
{
    PredictedObject(size_t length);
    PredictedObject(
        int object_id,
        std::vector<PoseWithCovariance> predictedPath,
        double length,
        double width
    );

    int object_id;
    double length;
    double width;
    std::vector<PoseWithCovariance> predictedPath;

    friend std::ostream& operator<<(std::ostream& os, const PredictedObject& obj);
};
