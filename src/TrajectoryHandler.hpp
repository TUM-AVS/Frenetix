#ifndef TRAJECTORYHANDLER_HPP
#define TRAJECTORYHANDLER_HPP

#include <taskflow/taskflow.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "trajectory/TrajectorySample.hpp"
#include "util.hpp"

class CostStrategy;
class FeasabilityStrategy;
class TrajectoryStrategy;


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
    void generateTrajectories(const SamplingMatrixXd& samplingMatrix, bool lowVelocityMode);
    void sort();
    void addFeasabilityFunction(std::shared_ptr<FeasabilityStrategy> function);
    void addFunction(std::shared_ptr<TrajectoryStrategy> function);
    void addCostFunction(std::shared_ptr<CostStrategy> function);
    void clearCostFunctions();
    void setAllCostWeightsToZero();

    /**
     * Get number of feasible trajectories.
     */
    size_t getFeasibleCount() const;

    /**
     * Get number of infeasible trajectories.
     */
    size_t getInfeasibleCount() const;

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
     * @brief Resets the trajectories container.
     * 
     */
    void resetTrajectories();

    std::vector<TrajectorySample> m_trajectories;
    std::map<std::string, std::shared_ptr<CostStrategy>> m_costFunctions;
    std::map<std::string, std::shared_ptr<FeasabilityStrategy>> m_feasabilityFunctions;
    std::map<std::string, std::shared_ptr<TrajectoryStrategy>> m_otherFunctions;
private:
    double m_dt;
    tf::Taskflow m_taskflow;
    tf::Executor m_executor;

    void removeInvalid();
};

#endif