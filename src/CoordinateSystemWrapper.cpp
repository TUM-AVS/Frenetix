#include "CoordinateSystemWrapper.hpp"

#include <stddef.h>
#include <cassert>
#include <cmath>
#include <iostream>

#include <stdexcept>

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

std::optional<int> CoordinateSystemWrapper::getS_idx(double s) const
{
    auto it = std::lower_bound(m_refPos.cbegin(), m_refPos.cend(), s);
    std::optional<int> new_idx = std::nullopt;

    if (it == m_refPos.cend()) {
        // In theory the following logic is correct, however we can't use the last segment since
        // we always need the following segment to be valid as well (in getSLambda etc.)
        #if 0
        if (m_refPos.size() >= 2 && s >= m_refPos[m_refPos.size() - 1]) {
            new_idx = m_refPos.size() - 1;
        }
        #endif
        new_idx = std::nullopt;
    } else if (std::distance(m_refPos.cbegin(), it) >= 1) {
        new_idx = std::distance(m_refPos.cbegin(), it) - 1;
    } else if (it == m_refPos.cbegin() && s >= *it) {
        new_idx = 0;
    } else {
        new_idx = std::nullopt;
    }

#ifndef NDEBUG
    int s_idx=0;
    for(int k=0; k < m_refPos.size(); k++)
    {
        if(m_refPos[k]>s)
        {
            s_idx=k;
            break;
        }
    }
    auto orig_idx = s_idx - 1;

    if (new_idx.value_or(-1) != orig_idx) {
        fprintf(stderr, "s=%lf old=%d new=%d i0=%lf sz=%zd\n", s, orig_idx, new_idx.value_or(-1), m_refPos[0], m_refPos.size());
        std::abort();
    }
#endif

    return new_idx;
}

double CoordinateSystemWrapper::getSLambda(double s, int s_idx) const
{
    if (s_idx + 1 >= m_refPos.size()) {
        throw std::invalid_argument { "out of range" };
    }

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





