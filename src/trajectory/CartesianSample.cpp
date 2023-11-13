#include "CartesianSample.hpp"

#include <iomanip>



CartesianSample::CartesianSample() = default;

CartesianSample::CartesianSample(const Eigen::Ref<Eigen::VectorXd>& x_,
                                 const Eigen::Ref<Eigen::VectorXd>& y_,
                                 const Eigen::Ref<Eigen::VectorXd>& theta_,
                                 const Eigen::Ref<Eigen::VectorXd>& velocity_,
                                 const Eigen::Ref<Eigen::VectorXd>& acceleration_,
                                 const Eigen::Ref<Eigen::VectorXd>& kappa_,
                                 const Eigen::Ref<Eigen::VectorXd>& kappaDot_)
    : isInitialized(true)
    , x(x_)
    , y(y_)
    , theta (theta_)
    , velocity (velocity_)
    , acceleration (acceleration_)
    , kappa (kappa_)
    , kappaDot (kappaDot_) 
{

}

void CartesianSample::print(std::ostream& os) const
{
    int width = 15;
    os << std::fixed << std::setprecision(5);

    os << "CartesianSample:" << std::endl
       << std::setw(width) << "timeStep"
       << std::setw(width) << "x"
       << std::setw(width) << "y"
       << std::setw(width) << "theta"
       << std::setw(width) << "velocity"
       << std::setw(width) << "acceleration"
       << std::setw(width) << "kappa"
       << std::setw(width) << "kappaDot" << std::endl;

    int numRows = x.size();

    for (int i = 0; i < numRows; ++i)
    {
        os << std::setw(width) << i
           << std::setw(width) << x(i)
           << std::setw(width) << y(i)
           << std::setw(width) << theta(i)
           << std::setw(width) << velocity(i)
           << std::setw(width) << acceleration(i)
           << std::setw(width) << kappa(i)
           << std::setw(width) << kappaDot(i) << std::endl;
    }
}