#include "CoordinateSystemWrapper.hpp"

#include <stddef.h>
#include <cassert>
#include <cmath>
#include <iostream>

#include "geometry/curvilinear_coordinate_system.h"

CoordinateSystemWrapper::CoordinateSystemWrapper(Eigen::Ref<RowMatrixXd> refPath)
    : m_refPath (refPath)
    , m_refPolyline (util::matrixToVector2d(m_refPath))
    , m_system (std::make_shared<geometry::CurvilinearCoordinateSystem>(m_refPolyline))    
{
    m_refPolyLineFromCoordSys = m_system->referencePath();
    m_refPos = computePathlengthFromPolyline(m_refPolyLineFromCoordSys);
    m_refCurv = computeCurvatureFromPolyline(m_refPolyLineFromCoordSys);
    m_refTheta = computeOrientationFromPolyline(m_refPolyLineFromCoordSys);
    m_refCurvD = util::computeGradient(m_refCurv, m_refPos);
    m_refCurvDD = util::computeGradient(m_refCurvD, m_refPos);
    
    std::cerr << "CoordinateSystemWrapper initialized" << std::endl;

}

int CoordinateSystemWrapper::getS_idx(double s) const
{
    int s_idx=0;
    for(int k=0; k < m_refPos.size(); k++)
    {
        if(m_refPos[k]>s)
        {
            s_idx=k;
            break;
        }
    }
    return s_idx-1;
}

double CoordinateSystemWrapper::getSLambda(double s, int s_idx) const
{
    return (s-m_refPos[s_idx]) / (m_refPos[s_idx+1] - m_refPos[s_idx]);
}

Eigen::VectorXd CoordinateSystemWrapper::computePathlengthFromPolyline(geometry::EigenPolyline& polyline)
{
    // Check if polyline has more than 2 points
    assert(polyline.size() > 2 && "Polyline malformed for pathlength computation");

    // Initialize Eigen::VectorXd for distance directly
    Eigen::VectorXd distance(polyline.size());
    distance(0) = 0;
    
    for (size_t i = 1; i < polyline.size(); ++i) 
    {
        distance(i) = distance(i - 1) + (polyline[i] - polyline[i - 1]).norm();
    }

    return distance;
}

Eigen::VectorXd CoordinateSystemWrapper::computeCurvatureFromPolyline(geometry::EigenPolyline& polyline)
{
     // Check if polyline has more than 2 points
    assert(polyline.size() > 2 && "Polyline malformed for orientation computation");
    Eigen::VectorXd curvatures = m_system->computeCurvature(polyline);
    return curvatures;
}


Eigen::VectorXd CoordinateSystemWrapper::computeOrientationFromPolyline(geometry::EigenPolyline& polyline) 
{
     // Check if polyline has more than 2 points
    assert(polyline.size() > 2 && "Polyline malformed for orientation computation");

    // Initialize Eigen::VectorXd for orientation
    Eigen::VectorXd orientation(polyline.size());
    
    // Compute orientation for the first (n-1) points
    for (size_t i = 0; i < polyline.size() - 1; ++i) 
    {
        const Eigen::Vector2d& pt1 = polyline[i];
        const Eigen::Vector2d& pt2 = polyline[i + 1];
        Eigen::Vector2d tmp = pt2 - pt1;
        orientation(i) = std::atan2(tmp(1), tmp(0));
    }

    // Compute orientation for the last point
    size_t i = polyline.size() - 1;
    const Eigen::Vector2d& pt1 = polyline[i - 1];
    const Eigen::Vector2d& pt2 = polyline[i];
    Eigen::Vector2d tmp = pt2 - pt1;
    orientation(i) = std::atan2(tmp(1), tmp(0));

    double PI = std::atan(1.0) * 4;
    double threshold = PI;

    for (size_t i = 1; i < orientation.size(); ++i) {
        double delta = orientation[i] - orientation[i - 1];
        if (delta > threshold) {
            for (size_t j = i; j < orientation.size(); ++j) {
                orientation[j] -= 2 * PI;
            }
        } else if (delta < -threshold) {
            for (size_t j = i; j < orientation.size(); ++j) {
                orientation[j] += 2 * PI;
            }
        }
    }

    return orientation;
}





