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
                    double horizon,
                    bool allowNegativeVelocity);

    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;

private:
    bool m_lowVelocityMode;
    double m_initialOrientation;
    std::shared_ptr<CoordinateSystemWrapper> m_coordinateSystem;
    double m_horizon;
    bool m_allowNegativeVelocity;
};


#endif //FILLCOORDINATES_HPP