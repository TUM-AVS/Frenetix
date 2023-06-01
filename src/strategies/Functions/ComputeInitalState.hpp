#ifndef COMPUTEINITIALSTATE_HPP
#define COMPUTEINITIALSTATE_HPP

#include "TrajectoryStrategy.hpp"
#include "TrajectorySample.hpp"
#include "CoordinateSystemWrapper.hpp"

#include <memory>
#include <Eigen/Dense>

class ComputeInitialState: public TrajectoryStrategy
{
public:
    ComputeInitialState(std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
                        double wheelBase,
                        double steeringAngle,
                        bool lowVelocityMode);

    void evaluateTrajectory(TrajectorySample& trajectory);

private:
    std::shared_ptr<CoordinateSystemWrapper> m_coordinateSystem;
    double m_wheelBase;
    double m_steeringAngle;
    bool m_lowVelocityMode;
};


#endif //COMPUTEINITIALSTATE_HPP