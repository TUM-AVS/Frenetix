#ifndef TRAJECTORYHANDLER_HPP
#define TRAJECTORYHANDLER_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <taskflow/taskflow.hpp>  // include the Taskflow library

#include "trajectory/TrajectorySample.hpp"
#include "util.hpp"

//Including the Strategy classes
#include "strategies/CostStrategy.hpp"
#include "strategies/FeasabilityStrategy.hpp"
#include "strategies/TrajectoryStrategy.hpp"


class TrajectoryHandler
{
public:
    TrajectoryHandler(double dt);


    /**
     * @brief Generates a trajectory for every row in samplingMatrix
     * 
     * @param samplingMatrix The Matrix needs to be of the form: [ t0
     *                                                           , t1
     *                                                           , s0
     *                                                           , ss0
     *                                                           , sss0
     *                                                           , ss1
     *                                                           , sss1
     *                                                           , d0
     *                                                           , dd0
     *                                                           , ddd0
     *                                                           , d1
     *                                                           , dd1
     *                                                           , ddd1]
     * 
     * @param lowVelocityMode If the vehicle is currently in the lowVelocityMode
     * 
     * t0: starting time
     * t1: end time
     * s0/s1:       start/end longitudinal 
     * ss0/ss1:     start/end longitudinal velocity
     * sss0/sss1:   start/end longitudinal acceleration
     * d0/d1:       start/end lateral
     * dd0/dd1:     start/end lateral velocity
     * ddd0/ddd1:   start/end lateral acceleration
     */
    void generateTrajectories(RowMatrixXd samplingMatrix, bool lowVelocityMode);
    void sort();
    void addFeasabilityFunction(std::shared_ptr<FeasabilityStrategy> function);
    void addFunction(std::shared_ptr<TrajectoryStrategy> function);
    void addCostFunction(std::shared_ptr<CostStrategy> function);

    /**
     * @brief Evaluates all current functions for each trajectory in the container.
     * 
     * This function iterates over all trajectories in the container, and for each trajectory,
     * evaluates the m_otherFunctions, m_feasabilityFunctions, and m_costFunctions. This function is 
     * used to update the trajectories' properties based on the given functions.
     * 
     * @note This function should be called after adding the need functions to the functions vectors.
     */
    void evaluateAllCurrentFunctions(bool calculateAllCosts);

    /**
     * @brief Evaluates all current functions for each trajectory in the container concurrently.
     * 
     */
    void evaluateAllCurrentFunctionsConcurrent(bool calculateAllCosts);

    /**
     * @brief Set the Cost Weights. The cost weights are used to scale the cost functions. 
     * The names of the cost functions are used as keys and can be extracted from the cost functions 
     * by calling the getName() function.
     * 
     * @param costWeightsMap 
     */
    void setCostWeights(std::map<std::string, double> costWeightsMap);

    /**
     * @brief Adds a cost weight for the given cost function name.
     * 
     * @param costFunctionName 
     * @param costWeight 
     */
    void addCostWeight(std::string costFunctionName, double costWeight);

    /**
     * @brief Resets the trajectories container.
     * 
     */
    void resetTrajectories();

    std::shared_ptr<std::map<std::string, double>> m_costWeightsMap;
    std::vector<TrajectorySample> m_trajectories;
    std::map<std::string, std::shared_ptr<CostStrategy>> m_costFunctions;
    std::map<std::string, std::shared_ptr<FeasabilityStrategy>> m_feasabilityFunctions;
    std::map<std::string, std::shared_ptr<TrajectoryStrategy>> m_otherFunctions;
private:
    double m_dt;
    tf::Taskflow m_taskflow;
    tf::Executor m_executor;
};

#endif