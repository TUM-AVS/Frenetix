#ifndef FILLCOORDINATES_HPP
#define FILLCOORDINATES_HPP

#include "TrajectoryStrategy.hpp"
#include "TrajectorySample.hpp"
#include "CoordinateSystemWrapper.hpp"

#include <memory>
#include <Eigen/Dense>

class FillCoordinates: public TrajectoryStrategy
{
public:
    FillCoordinates(Eigen::Ref<Eigen::VectorXd> times, 
                    bool lowVelocityMode, 
                    double initialOrienation, 
                    std::shared_ptr<CoordinateSystemWrapper> coordinateSystem);

    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;

private:
    Eigen::VectorXd m_times;
    bool m_lowVelocityMode;
    double m_initialOrientation;
    std::shared_ptr<CoordinateSystemWrapper> m_coordinateSystem;
};


#endif //FILLCOORDINATES_HPP