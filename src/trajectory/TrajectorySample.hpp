#ifndef TRAJECTORYSAMPLE_HPP
#define TRAJECTORYSAMPLE_HPP

#include <map>
#include <string>
#include <Eigen/Dense>
#include <omp.h>
#include <memory>

#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "polynomial.hpp"


class TrajectorySample
{
    size_t m_size;
public:
    double m_horizon;
    double m_dT;
    double m_cost;
    bool m_collisionDetected;
    int m_actualTrajectoryLength;
    int m_uniqueId;
    double m_boundaryHarm;
    double m_egoRisk;
    double m_obstRisk;
    int m_currentTimeStep;
    bool m_feasible;


    std::map<std::string, std::pair<double,double>> m_costMap;
    std::map<std::string, double> m_feasabilityMap;
    std::shared_ptr<std::map<std::string, double>> m_costWeightMap;

    PolynomialTrajectory<4> m_trajectoryLongitudinal;
    PolynomialTrajectory<5> m_trajectoryLateral;
    CartesianSample m_cartesianSample;
    CurviLinearSample m_curvilinearSample;
    


    TrajectorySample(double horizon, 
                     double dt,
                     std::shared_ptr<std::map<std::string, double>> costWeightMap, 
                     PolynomialTrajectory<4>& trajectoryLongitudinal, 
                     PolynomialTrajectory<5>& trajectoryLateral, 
                     int uniqueId);

    TrajectorySample(double horizon, 
                    double dt,
                    PolynomialTrajectory<4>& trajectoryLongitudinal, 
                    PolynomialTrajectory<5>& trajectoryLateral, 
                    int uniqueId);

    /**
     * @brief Construct a new Trajectory Sample object just with the initial position.
     * 
     * @param x0 
     * @param y0 
     * @param orientation0
     */
    TrajectorySample(double x0,
                     double y0,
                     double orientation0,
                     double acceleration0,
                     double velocity0);

    /**
     * @brief Initialize arrays of the curvilinear and cartesian samples with the specified size.
     *
     * @param size The size to resize the arrays to.
     */
    void initArraysWithSize(int size);

    /**
     * @brief Set the current time step for the trajectory sample.
     *
     * @param currentTimeStep The current time step to set.
     */
    void setCurrentTimeStep(int currentTimeStep);

    /**
     * @brief Add a cost value to the list of cost values. The weight must therefore be speicified 
     * in the costWeightMap. Add a costWeight to the map if it is not specified yet by 
     * handler.addCostWeight(std::string functionName, double weight).
     * 
     * @param costFunctionName 
     * @param cost 
     * @param costWeighted
     */
    void addCostValueToList(std::string costFunctionName, double cost, double costWeighted);

    void addFeasabilityValueToList(std::string costFunctionName, double value);


    size_t size();
};

#endif //TRAJECTORYSAMPLE_HPP