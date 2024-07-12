#ifndef FILLCOORDINATES_HPP
#define FILLCOORDINATES_HPP

#include <memory>

#include "TrajectoryStrategy.hpp"

class CoordinateSystemWrapper;
class TrajectorySample;

class FillCoordinates: public TrajectoryStrategy
{
public:
    FillCoordinates(bool lowVelocityMode, 
                    double initialOrienation, 
                    std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
                    double horizon);

    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;

private:
    bool m_lowVelocityMode;
    double m_initialOrientation;
    std::shared_ptr<CoordinateSystemWrapper> m_coordinateSystem;
    double m_horizon;
};


#endif //FILLCOORDINATES_HPP