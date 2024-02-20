#include "TrajectoryHandler.hpp"
#include "TrajectorySample.hpp"

#include <Eigen/Core>
#include <algorithm>

#include "CostStrategy.hpp"
#include "FeasabilityStrategy.hpp"
#include "TrajectorySample.hpp"
#include "TrajectoryStrategy.hpp"
#include "polynomial.hpp"

TrajectoryHandler::TrajectoryHandler(double dt)
    : m_dt(dt)
{

}

void TrajectoryHandler::setAllCostWeightsToZero()
{
    for(auto& [functionName, costStrategy] : m_costFunctions)
    {
        if(costStrategy)
        {
            costStrategy->updateCostWeight(0.0);
        }
    }
}

void TrajectoryHandler::clearCostFunctions()
{
    m_costFunctions.clear();
}

void TrajectoryHandler::addCostFunction(std::shared_ptr<CostStrategy> function)
{

    // check if there exits a cost weight for the given cost function
    m_costFunctions.insert_or_assign(function->getFunctionName(), function);
}

void TrajectoryHandler::addFeasabilityFunction(std::shared_ptr<FeasabilityStrategy> function)
{
    m_feasabilityFunctions.insert_or_assign(function->getFunctionName(),function);
}

void TrajectoryHandler::addFunction(std::shared_ptr<TrajectoryStrategy> function)
{
    m_otherFunctions.insert_or_assign(function->getFunctionName(),function);
}

void TrajectoryHandler::evaluateAllCurrentFunctions(bool calculateAllCosts)
{
    //Iterate over all trajectories
    for(auto& trajectory: m_trajectories)
    {
        //Iterate over all otherFunctions and evaluate it for the given trajectory
        for(auto& [funName, function] : m_otherFunctions)
        {
            function->evaluateTrajectory(trajectory);
        }

        if (!trajectory.m_valid) continue;

        //Iterate over all costFunctions and evaluate it for the given trajectory
        for(auto& [funName, function] : m_feasabilityFunctions)
        {
            function->evaluateTrajectory(trajectory);
        }

        if (trajectory.m_feasible || calculateAllCosts)
        {
            //All costFunctions
            for(auto& [funName, function] : m_costFunctions)
            {
                function->evaluateTrajectory(trajectory);
            }
        }
    }
}

void TrajectoryHandler::evaluateAllCurrentFunctionsConcurrent(bool calculateAllCosts)
{
    //Iterate over all trajectories
    for(auto& trajectory: m_trajectories)
    {
        tf::Task A = m_taskflow.emplace
        (
            [this, &trajectory]()
            {
                for(auto& [funName, function] : m_otherFunctions)
                {
                    function->evaluateTrajectory(trajectory);
                }
            }
        );

        tf::Task B = m_taskflow.emplace
        (
            [this, &trajectory]()
            {
                if (!trajectory.m_valid) return;

                for(auto& [funName, function] : m_feasabilityFunctions) 
                {
                    function->evaluateTrajectory(trajectory);
                }
            }
        );

        tf::Task C = m_taskflow.emplace
        (
            [this, &trajectory, calculateAllCosts]
            {
                if (!trajectory.m_valid) return;

                if (trajectory.m_feasible || calculateAllCosts) 
                {
                    for(auto& [funName, function] : m_costFunctions)
                    {
                        function->evaluateTrajectory(trajectory);
                    }
                }
            }
        );

        // Define the dependencies between the tasks
        A.precede(B);
        B.precede(C);
    }
    m_executor.run(m_taskflow).wait();

    m_taskflow.clear();

}

void TrajectoryHandler::sort()
{
    std::sort(
        m_trajectories.begin(),
        m_trajectories.end(),
        [](const TrajectorySample& a, const TrajectorySample& b) {
            // Feasible trajectories come first
            if (a.m_feasible != b.m_feasible)
                return a.m_feasible > b.m_feasible;
            // If feasibility is the same, sort by cost
            return a.m_cost < b.m_cost;
        }
    );
}

void TrajectoryHandler::generateTrajectories(const SamplingMatrixXd& samplingMatrix, bool lowVelocityMode)
{
    Eigen::Vector3d x0_lonOrder {0,1,2};
    Eigen::Vector2d x1_lonOrder {1,2};

    m_trajectories.reserve(samplingMatrix.rows());

    for(Eigen::Index iii = 0; iii < samplingMatrix.rows(); iii++)
    {
        Eigen::Vector3d x0_lon {samplingMatrix.row(iii)[2], samplingMatrix.row(iii)[3], samplingMatrix.row(iii)[4]};
        Eigen::Vector2d x1_lon {samplingMatrix.row(iii)[5], samplingMatrix.row(iii)[6]};

        TrajectorySample::LongitudinalTrajectory longitudinalTrajectory (
            samplingMatrix.row(iii)[0],
            samplingMatrix.row(iii)[1],
            x0_lon,
            x1_lon,
            x0_lonOrder,
            x1_lonOrder
            );

        double t1 = 0.0;
        if (lowVelocityMode) {
            t1 = longitudinalTrajectory(samplingMatrix.row(iii)[1]) - x0_lon[0];
            if (t1 <= 0.0) {
                t1 = samplingMatrix.row(iii)[1];
            }
        } else {
            t1 = samplingMatrix.row(iii)[1];
        }

        Eigen::Vector3d x0_lat {samplingMatrix.row(iii)[7], samplingMatrix.row(iii)[8], samplingMatrix.row(iii)[9]};
        Eigen::Vector3d x1_lat {samplingMatrix.row(iii)[10], samplingMatrix.row(iii)[11], samplingMatrix.row(iii)[12]};

        TrajectorySample::LateralTrajectory lateralTrajectory(
            samplingMatrix.row(iii)[0],
            t1,
            x0_lat,
            x1_lat
            );

        m_trajectories.emplace_back(
            m_dt,
            longitudinalTrajectory,
            lateralTrajectory,
            iii,
            samplingMatrix.row(iii)
            );
    }
}


void TrajectoryHandler::resetTrajectories()
{
    m_trajectories.clear();
}
