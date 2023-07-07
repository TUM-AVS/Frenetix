#ifndef COORDINATESYSTEMWRAPPER_H
#define COORDINATESYSTEMWRAPPER_H

#include <cmath>
#include <algorithm>
#include <iterator>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cassert>
#include <memory>

#include "TrajectorySample.hpp"
#include "util.hpp"
#include "geometry/curvilinear_coordinate_system.h"
#include "geometry/util.h"


class CoordinateSystemWrapper
{
private:
    RowMatrixXd m_refPath;
    geometry::EigenPolyline m_refPolyline;

public:
    geometry::EigenPolyline m_refPolyLineFromCoordSys;
    Eigen::VectorXd m_refPos;
    Eigen::VectorXd m_refCurv;
    Eigen::VectorXd m_refTheta;
    Eigen::VectorXd m_refCurvD;
    Eigen::VectorXd m_refCurvDD;
private:

    std::shared_ptr<geometry::CurvilinearCoordinateSystem> m_system;

    Eigen::VectorXd computePathlengthFromPolyline(geometry::EigenPolyline& polyline); 
    Eigen::VectorXd computeCurvatureFromPolyline(geometry::EigenPolyline& polyline);
    Eigen::VectorXd computeOrientationFromPolyline(geometry::EigenPolyline& polyline); 
    
public:
    std::shared_ptr<geometry::CurvilinearCoordinateSystem> getSystem() const { return m_system; }
    void setSystem(std::shared_ptr<geometry::CurvilinearCoordinateSystem> system) { m_system = system; }
    
    CoordinateSystemWrapper(Eigen::Ref<RowMatrixXd> ref_path);

    int getS_idx(double s) const;
    double getSLambda(double s, int s_idx) const;

};



#endif //COORDINATESYSTEMWRAPPER_H