#include "FillCoordinates.hpp"

#include <stddef.h>
#include <Eigen/Core>
#include <cmath>
#include <exception>
#include <iostream>
#include <optional>

#include "CartesianSample.hpp"
#include "CoordinateSystemWrapper.hpp"
#include "CurvilinearSample.hpp"
#include "TrajectorySample.hpp"
#include "geometry/curvilinear_coordinate_system.h"
#include "polynomial.hpp"
#include "util.hpp"

FillCoordinates::FillCoordinates(bool lowVelocityMode, 
                                 double initialOrienation, 
                                 std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
                                 double horizon)
    : TrajectoryStrategy("Fill Coordinates")
    , m_lowVelocityMode(lowVelocityMode)
    , m_initialOrientation(initialOrienation)
    , m_coordinateSystem(coordinateSystem)
    , m_horizon(horizon)
{
}

void FillCoordinates::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd& refPos = m_coordinateSystem->m_refPos;
    const Eigen::VectorXd& refCurv = m_coordinateSystem->m_refCurv;
    const Eigen::VectorXd& refTheta = m_coordinateSystem->m_refTheta;
    const Eigen::VectorXd& refCurvD = m_coordinateSystem->m_refCurvD;

    double dp {0};
    double dpp {0};

    double currentHorizon = trajectory.m_samplingParameters[1] - trajectory.m_samplingParameters[0];
    size_t actualLength = static_cast<size_t>(1+(currentHorizon / trajectory.m_dT));
    size_t length {0};
    
    if(m_horizon > currentHorizon)
    {
        length = static_cast<size_t>(1+(m_horizon / trajectory.m_dT));
    }
    else
    {
        length = actualLength;        
    }

    trajectory.m_acutualSize = actualLength;
    trajectory.initArraysWithSize(length);
    trajectory.m_currentTimeStep = length;

    double t {0};
    
    for (int iii = 0; iii < length; ++iii) 
    {
        t = trajectory.m_samplingParameters[0] + trajectory.m_dT*iii;
        
        if(iii >= actualLength)
        {
            trajectory.m_curvilinearSample.s[iii] = trajectory.m_curvilinearSample.s[iii-1] + trajectory.m_dT*trajectory.m_curvilinearSample.ss[iii-1];
            trajectory.m_curvilinearSample.ss[iii] =  trajectory.m_curvilinearSample.ss[iii-1];
            trajectory.m_curvilinearSample.sss[iii] = 0;

            trajectory.m_curvilinearSample.d[iii] = trajectory.m_curvilinearSample.d[iii-1];
            trajectory.m_curvilinearSample.dd[iii] = 0;
            trajectory.m_curvilinearSample.ddd[iii] = 0;
        }
        else
        {
            const auto sv = trajectory.m_trajectoryLongitudinal.horner_eval(t);
            trajectory.m_curvilinearSample.s[iii] = sv.x;
            trajectory.m_curvilinearSample.ss[iii] = sv.xx; 
            trajectory.m_curvilinearSample.sss[iii] = sv.xxx;

            double ttt {t};

            if(m_lowVelocityMode)
            {
                ttt = trajectory.m_curvilinearSample.s[iii] -  trajectory.m_curvilinearSample.s[0];
            }

            const auto dv = trajectory.m_trajectoryLateral.horner_eval(ttt);
            trajectory.m_curvilinearSample.d[iii] = dv.x;
            trajectory.m_curvilinearSample.dd[iii] = dv.xx;
            trajectory.m_curvilinearSample.ddd[iii] = dv.xxx;
        }


        double _EPS = 1e-5;
        if(trajectory.m_curvilinearSample.ss[iii] < -_EPS)
        {
            trajectory.m_valid = false;
            trajectory.m_feasible = false;
            return;
        }
        else if(std::abs(trajectory.m_curvilinearSample.ss[iii]) < _EPS)
        {
            trajectory.m_curvilinearSample.ss[iii] = 0;
        }

        if(m_lowVelocityMode)
        {
            dp = trajectory.m_curvilinearSample.dd[iii];
            dpp = trajectory.m_curvilinearSample.ddd[iii];
        }
        else
        {
            if(trajectory.m_curvilinearSample.ss[iii] > 0.001)
            {
                dp = trajectory.m_curvilinearSample.dd[iii] / trajectory.m_curvilinearSample.ss[iii];
                double ddot = trajectory.m_curvilinearSample.ddd[iii] - dp * trajectory.m_curvilinearSample.sss[iii];
                dpp = ddot / (trajectory.m_curvilinearSample.ss[iii] * trajectory.m_curvilinearSample.ss[iii]);
            }
            else
            {
                dp = 0;
                dpp = 0;
            }
        }
        int s_idx = m_coordinateSystem->getS_idx(trajectory.m_curvilinearSample.s[iii]);
        double sLambda = m_coordinateSystem->getSLambda(trajectory.m_curvilinearSample.s[iii], s_idx);

        double interPolatedAngle = util::interpolate_angle(trajectory.m_curvilinearSample.s[iii],
                                                           refPos[s_idx],
                                                           refPos[s_idx + 1],
                                                           refTheta[s_idx],
                                                           refTheta[s_idx + 1]);


        if(trajectory.m_curvilinearSample.ss[iii] > 0.001)
        {
            trajectory.m_curvilinearSample.theta[iii] = std::atan2(dp, 1.0);
            trajectory.m_cartesianSample.theta[iii] = trajectory.m_curvilinearSample.theta[iii] + interPolatedAngle;
        }
        else
        {
            if(m_lowVelocityMode)
            {
                trajectory.m_curvilinearSample.theta[iii] = std::atan2(dp, 1.0);
                trajectory.m_cartesianSample.theta[iii] = trajectory.m_curvilinearSample.theta[iii] + interPolatedAngle;
            }
            else
            {
                trajectory.m_cartesianSample.theta[iii] = (iii == 0) ? m_initialOrientation : trajectory.m_cartesianSample.theta[iii-1];
                trajectory.m_curvilinearSample.theta[iii] = trajectory.m_cartesianSample.theta[iii] - interPolatedAngle;
            }
        }

        // Interpolate curvature of reference path k_r at current position
        double k_r = (refCurv[s_idx+1] - refCurv[s_idx]) * sLambda + refCurv[s_idx];
        // Interpolate curvature rate of reference path k_r_d at current position
        double k_r_d = (refCurvD[s_idx+1] - refCurvD[s_idx]) * sLambda + refCurvD[s_idx];

        double oneKrD = (1 - k_r*trajectory.m_curvilinearSample.d[iii]);
        double cosTheta = std::cos(trajectory.m_curvilinearSample.theta[iii]);
        double tanTheta = std::tan(trajectory.m_curvilinearSample.theta[iii]);

        trajectory.m_cartesianSample.kappa[iii] = (dpp + (k_r * dp + k_r_d * trajectory.m_curvilinearSample.d[iii]) * tanTheta)
                                                   * cosTheta * std::pow(cosTheta / oneKrD, 2)
                                                   + (cosTheta / oneKrD) * k_r;

        trajectory.m_cartesianSample.kappaDot[iii] = (iii == 0) ? 0 : (trajectory.m_cartesianSample.kappa[iii]- trajectory.m_cartesianSample.kappa[iii-1]);

        trajectory.m_cartesianSample.velocity[iii] = std::abs(trajectory.m_curvilinearSample.ss[iii] * (oneKrD / cosTheta));

        trajectory.m_cartesianSample.acceleration[iii] = trajectory.m_curvilinearSample.sss[iii] * oneKrD / cosTheta +
                                                         std::pow(trajectory.m_curvilinearSample.ss[iii], 2) / cosTheta
                                                         * (oneKrD * tanTheta * (trajectory.m_cartesianSample.kappa[iii] * oneKrD / cosTheta - k_r)
                                                         - (k_r_d * trajectory.m_curvilinearSample.d[iii] + k_r * dp));

        try
        {
            Eigen::Vector2d cartesianPoints = m_coordinateSystem->getSystem()->convertToCartesianCoords(trajectory.m_curvilinearSample.s[iii],trajectory.m_curvilinearSample.d[iii]);
            trajectory.m_cartesianSample.x[iii] = cartesianPoints[0];
            trajectory.m_cartesianSample.y[iii] = cartesianPoints[1];
        }
        catch(const std::exception& e)
        {
            trajectory.m_valid = false;
            trajectory.m_feasible = false;
            return;
        }
        catch(...)
        {
            std::cout << "Caught unknown exception" << std::endl;
        }
    }
}

