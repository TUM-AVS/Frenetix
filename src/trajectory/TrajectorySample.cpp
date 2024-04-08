#include "TrajectorySample.hpp"

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

    double kappa_0 = tan(state.steering_angle) / state.wheelbase; 

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

