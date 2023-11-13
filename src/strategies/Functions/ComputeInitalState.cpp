#include "ComputeInitalState.hpp"

#include <assert.h>
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

#include "CartesianSample.hpp"
#include "CoordinateSystemWrapper.hpp"
#include "CurvilinearSample.hpp"
#include "TrajectorySample.hpp"
#include "geometry/curvilinear_coordinate_system.h"
#include "util.hpp"

ComputeInitialState::ComputeInitialState(std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
                                         double wheelBase,
                                         double steeringAngle,
                                         bool lowVelocityMode)
    : TrajectoryStrategy("Compute Initial State")
    , m_coordinateSystem(coordinateSystem)
    , m_wheelBase(wheelBase)
    , m_steeringAngle(steeringAngle)
    , m_lowVelocityMode(lowVelocityMode)
{

}

void ComputeInitialState::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd& refPos = m_coordinateSystem->m_refPos;
    const Eigen::VectorXd& refCurv = m_coordinateSystem->m_refCurv;
    const Eigen::VectorXd& refTheta = m_coordinateSystem->m_refTheta;
    const Eigen::VectorXd& refCurvD = m_coordinateSystem->m_refCurvD;

    double dp {0};
    double dpp {0};

    Eigen::Vector2d curviCords = m_coordinateSystem->getSystem()->convertToCurvilinearCoords(trajectory.m_cartesianSample.x[0],trajectory.m_cartesianSample.y[0]);

    trajectory.m_curvilinearSample.s[0] = curviCords[0];
    trajectory.m_curvilinearSample.d[0] = curviCords[1];    

    int s_idx = m_coordinateSystem->getS_idx(trajectory.m_curvilinearSample.s[0]);
    assert(s_idx != -1);
    double sLambda = m_coordinateSystem->getSLambda(trajectory.m_curvilinearSample.s[0], s_idx);

    trajectory.m_curvilinearSample.theta[0] = trajectory.m_cartesianSample.theta[0] - 
                                              util::interpolate_angle(trajectory.m_curvilinearSample.s[0],
                                              refPos[s_idx],
                                              refPos[s_idx + 1],
                                              refTheta[s_idx],
                                              refTheta[s_idx + 1]);

    trajectory.m_cartesianSample.kappa[0] = std::tan(m_steeringAngle) / m_wheelBase;

    // Interpolate curvature of reference path k_r at current position
    double k_r = (refCurv[s_idx+1] - refCurv[s_idx]) * sLambda + refCurv[s_idx];
    // Interpolate curvature rate of reference path k_r_d at current position
    double k_r_d = (refCurvD[s_idx+1] - refCurvD[s_idx]) * sLambda + refCurvD[s_idx];

    dp = (1-k_r*trajectory.m_curvilinearSample.d[0]) * std::tan(trajectory.m_curvilinearSample.theta[0]);

    dpp = -(k_r_d * trajectory.m_curvilinearSample.d[0] + k_r * dp) * std::tan(trajectory.m_curvilinearSample.theta[0]) 
          +((1 - k_r * trajectory.m_curvilinearSample.d[0]) / std::pow(std::cos(trajectory.m_curvilinearSample.theta[0]), 2)) 
          *(trajectory.m_cartesianSample.kappa[0] * (1 - k_r * trajectory.m_curvilinearSample.d[0]) / std::cos(trajectory.m_curvilinearSample.theta[0]) - k_r);


    trajectory.m_curvilinearSample.ss[0] = trajectory.m_cartesianSample.velocity[0] 
                                           * std::cos(trajectory.m_curvilinearSample.theta[0]) 
                                           / (1 - k_r * trajectory.m_curvilinearSample.d[0]);

    if(trajectory.m_curvilinearSample.ss[0] < 0)
    {
        throw std::runtime_error("Initial state or reference incorrect! Curvilinear velocity is negative which indicates"
                                 "that the ego vehicle is not driving in the same direction as specified by the reference");
    }

    trajectory.m_curvilinearSample.sss[0] = trajectory.m_cartesianSample.acceleration[0] 
                                            - std::pow(trajectory.m_curvilinearSample.ss[0], 2) / std::cos(trajectory.m_curvilinearSample.theta[0]) 
                                            * ((1 - k_r * trajectory.m_curvilinearSample.d[0]) * std::tan(trajectory.m_curvilinearSample.theta[0]) 
                                            * (trajectory.m_cartesianSample.kappa[0] * (1 - k_r * trajectory.m_curvilinearSample.d[0]) 
                                            / std::cos(trajectory.m_curvilinearSample.theta[0]) - k_r) - (k_r_d * trajectory.m_curvilinearSample.d[0] + k_r * dp)) 
                                            / ((1 - k_r * trajectory.m_curvilinearSample.d[0]) / std::cos(trajectory.m_curvilinearSample.theta[0]));

    if (m_lowVelocityMode)
    {
        trajectory.m_curvilinearSample.dd[0] = dp;
        trajectory.m_curvilinearSample.ddd[0] = dpp;
    }
    else
    {
        trajectory.m_curvilinearSample.dd[0] = trajectory.m_cartesianSample.velocity[0] * std::sin(trajectory.m_curvilinearSample.theta[0]);
        trajectory.m_curvilinearSample.ddd[0] = trajectory.m_curvilinearSample.sss[0] * dp + std::pow(trajectory.m_curvilinearSample.ss[0],2) * dpp;  
    }
}

