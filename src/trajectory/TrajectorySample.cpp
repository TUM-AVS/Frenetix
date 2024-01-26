#include "TrajectorySample.hpp"

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

