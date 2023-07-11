#pragma once

#include <array>

struct Position 
{
    double x, y, z;
};

struct Orientation 
{
    double x, y, z, w;
};

struct Pose 
{
    Position position;
    Orientation orientation;
};

struct PoseWithCovariance 
{
    Pose pose;
    std::array<double, 36> covariance;
};