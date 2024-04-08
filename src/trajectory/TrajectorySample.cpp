#include "TrajectorySample.hpp"
#include "geometry/curvilinear_coordinate_system.h"

const TrajectorySample::LongitudinalTrajectory::OrderVectorX0 TrajectorySample::LongitudinalX0Order {0,1,2};
const TrajectorySample::LongitudinalTrajectory::OrderVectorXD TrajectorySample::LongitudinalXDOrder {1,2};

template class PolynomialTrajectory<4, 3, 2>;
template class PolynomialTrajectory<5, 3, 3>;

TrajectorySample::TrajectorySample(double dT,
                                   TrajectorySample::LongitudinalTrajectory trajectoryLongitudinal,
                                   TrajectorySample::LateralTrajectory trajectoryLateral,
                                   int uniqueId)
    : m_dT (dT)
    , m_cost (0)
    , m_harm_occ_module (0)
    , m_uniqueId (uniqueId)
    , m_feasible (true)
    , m_samplingParameters ()
    , m_trajectoryLongitudinal (trajectoryLongitudinal)
    , m_trajectoryLateral (trajectoryLateral)
    , m_cartesianSample ()
    , m_curvilinearSample ()
{

}

TrajectorySample::TrajectorySample(double dT,
                                   TrajectorySample::LongitudinalTrajectory trajectoryLongitudinal,
                                   TrajectorySample::LateralTrajectory trajectoryLateral,
                                   int uniqueId,
                                   Eigen::VectorXd samplingParameters)
    : m_dT (dT)
    , m_cost (0)
    , m_harm_occ_module (0)
    , m_uniqueId (uniqueId)
    , m_feasible (true)
    , m_samplingParameters (samplingParameters)
    , m_trajectoryLongitudinal (trajectoryLongitudinal)
    , m_trajectoryLateral (trajectoryLateral)
    , m_cartesianSample ()
    , m_curvilinearSample ()
{

}


TrajectorySample::TrajectorySample(double x_0,
                                   double y_0,
                                   double orientation_0,
                                   double acceleration_0,
                                   double velocity_0)
    : m_feasible (true)
{
    initArraysWithSize(1);
    m_cartesianSample.x[0] = x_0;
    m_cartesianSample.y[0] = y_0;
    m_cartesianSample.theta[0] = orientation_0;
    m_cartesianSample.acceleration[0] = acceleration_0;
    m_cartesianSample.velocity[0] = velocity_0;
}

void TrajectorySample::initArraysWithSize(size_t size)
{
    m_size = size;

    m_curvilinearSample.s.resize(size);
    m_curvilinearSample.ss.resize(size);
    m_curvilinearSample.sss.resize(size);
    m_curvilinearSample.d.resize(size);
    m_curvilinearSample.dd.resize(size);
    m_curvilinearSample.ddd.resize(size);
    m_curvilinearSample.theta.resize(size);

    m_cartesianSample.x.resize(size);
    m_cartesianSample.y.resize(size);
    m_cartesianSample.theta.resize(size);
    m_cartesianSample.kappa.resize(size);
    m_cartesianSample.kappaDot.resize(size);
    m_cartesianSample.acceleration.resize(size);
    m_cartesianSample.velocity.resize(size);
}


PlannerState::Curvilinear computeInitialState(
        std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
        PlannerState::Cartesian x_0,
        double wheelbase, 
        bool lowVelocityMode
) {
    const Eigen::VectorXd& refPos = coordinateSystem->m_refPos;
    const Eigen::VectorXd& refCurv = coordinateSystem->m_refCurv;
    const Eigen::VectorXd& refTheta = coordinateSystem->m_refTheta;
    const Eigen::VectorXd& refCurvD = coordinateSystem->m_refCurvD;

    PlannerState::Curvilinear x_cl;

    Eigen::Vector2d curviCords = coordinateSystem->getSystem()->convertToCurvilinearCoords(x_0.pos[0], x_0.pos[1]);

    x_cl.x0_lon[0] = curviCords[0];
    x_cl.x0_lat[0] = curviCords[1];    

    int s_idx = coordinateSystem->getS_idx(x_cl.x0_lon[0]);
    assert(s_idx != -1);
    double sLambda = coordinateSystem->getSLambda(x_cl.x0_lon[0], s_idx);

    double theta = x_0.orientation - 
        util::interpolate_angle(
            x_cl.x0_lon[0],
            refPos[s_idx],
            refPos[s_idx + 1],
            refTheta[s_idx],
            refTheta[s_idx + 1]
        );

    double kappa = std::tan(x_0.steering_angle) / wheelbase;

    // Interpolate curvature of reference path k_r at current position
    double k_r = (refCurv[s_idx+1] - refCurv[s_idx]) * sLambda + refCurv[s_idx];
    // Interpolate curvature rate of reference path k_r_d at current position
    double k_r_d = (refCurvD[s_idx+1] - refCurvD[s_idx]) * sLambda + refCurvD[s_idx];

    double dp = (1-k_r*x_cl.x0_lat[0]) * std::tan(theta);

    double dpp = -(k_r_d * x_cl.x0_lat[0] + k_r * dp) * std::tan(theta) 
          +((1 - k_r * x_cl.x0_lat[0]) / std::pow(std::cos(theta), 2)) 
          *(kappa * (1 - k_r * x_cl.x0_lat[0]) / std::cos(theta) - k_r);


    x_cl.x0_lon[1] = x_0.velocity * std::cos(theta) / (1 - k_r * x_cl.x0_lat[0]);

    if(x_cl.x0_lon[1] < 0)
    {
        throw std::runtime_error("Initial state or reference incorrect! Curvilinear velocity is negative which indicates"
                                 "that the ego vehicle is not driving in the same direction as specified by the reference");
    }

    x_cl.x0_lon[2] = x_0.acceleration
        - std::pow(x_cl.x0_lon[1], 2) / std::cos(theta) 
        * ((1 - k_r * x_cl.x0_lat[0]) * std::tan(theta) 
        * (kappa* (1 - k_r * x_cl.x0_lat[0]) 
        / std::cos(theta) - k_r) - (k_r_d * x_cl.x0_lat[0] + k_r * dp)) 
        / ((1 - k_r * x_cl.x0_lat[0]) / std::cos(theta));

    if (lowVelocityMode)
    {
        x_cl.x0_lat[1] = dp;
        x_cl.x0_lat[2] = dpp;
    }
    else
    {
        x_cl.x0_lat[1] = x_0.velocity * std::sin(theta);
        x_cl.x0_lat[2] = x_cl.x0_lon[2] * dp + std::pow(x_cl.x0_lon[1], 2) * dpp;  
    }

    return x_cl;
}

TrajectorySample TrajectorySample::standstillTrajectory(
        std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
        PlannerState state,
        double dt,
        double horizon
) {
    Eigen::Vector2d x1_lon {0.0, 0.0};

    TrajectorySample::LongitudinalTrajectory longitudinalTrajectory (
        0.0,
        horizon,
        state.x_cl.x0_lon,
        x1_lon,
        TrajectorySample::LongitudinalX0Order,
        TrajectorySample::LongitudinalXDOrder
        );

    Eigen::Vector3d x1_lat {state.x_cl.x0_lat[0], 0.0, 0.0};

    TrajectorySample::LateralTrajectory lateralTrajectory(
        0.0,
        horizon,
        state.x_cl.x0_lat,
        x1_lat
        );

    auto traj = TrajectorySample(
        dt,
        longitudinalTrajectory,
        lateralTrajectory,
        -1
    );

    size_t length = static_cast<size_t>(1+(horizon / dt));
    traj.m_acutualSize = length;
    traj.initArraysWithSize(length);

    double kappa_0 = tan(state.x_0.steering_angle) / state.wheelbase; 

    traj.m_cartesianSample.x.fill(state.x_0.pos[0]);
    traj.m_cartesianSample.y.fill(state.x_0.pos[1]);
    traj.m_cartesianSample.theta.fill(state.x_0.orientation);
    traj.m_cartesianSample.velocity.fill(0.0);
    traj.m_cartesianSample.acceleration.fill(0.0);
    traj.m_cartesianSample.acceleration[1] = - state.x_0.velocity / dt;
    
    traj.m_cartesianSample.kappa.fill(kappa_0);
    traj.m_cartesianSample.kappaDot.fill(0);

    const Eigen::VectorXd& refPos = coordinateSystem->m_refPos;
    const Eigen::VectorXd& refTheta = coordinateSystem->m_refTheta;

    int s_idx = coordinateSystem->getS_idx(state.x_cl.x0_lon[0]);

    double theta_cl = util::interpolate_angle(state.x_cl.x0_lon[0],
                                                       refPos[s_idx],
                                                       refPos[s_idx + 1],
                                                       refTheta[s_idx],
                                                       refTheta[s_idx + 1]);

    traj.m_curvilinearSample.s.fill(state.x_cl.x0_lon[0]);
    traj.m_curvilinearSample.ss.fill(state.x_cl.x0_lon[1]);
    traj.m_curvilinearSample.sss.fill(state.x_cl.x0_lon[2]);
    traj.m_curvilinearSample.d.fill(state.x_cl.x0_lat[0]);
    traj.m_curvilinearSample.dd.fill(state.x_cl.x0_lat[1]);
    traj.m_curvilinearSample.ddd.fill(state.x_cl.x0_lat[2]);
    traj.m_curvilinearSample.theta.fill(theta_cl);

    return traj;
}

void TrajectorySample::setCurrentTimeStep(int currentTimeStep)
{
    m_currentTimeStep = currentTimeStep;
}

void TrajectorySample::addCostValueToList(std::string costFunctionName, double cost, double costWeighted)
{
    m_cost += costWeighted;
    m_costMap[costFunctionName] = std::make_pair(cost, costWeighted);
}


void TrajectorySample::addFeasabilityValueToList(std::string feasabilityFunctionsName, double value)
{
    if(value) m_feasible = false;
    m_feasabilityMap[feasabilityFunctionsName] = value;
}

size_t TrajectorySample::size()
{
    return m_size;
}

