#ifndef COMPUTEINITIALSTATE_HPP
#define COMPUTEINITIALSTATE_HPP

#include <memory>

#include "TrajectoryStrategy.hpp"

class CoordinateSystemWrapper;
class TrajectorySample;

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