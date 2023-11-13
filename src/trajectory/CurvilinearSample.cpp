#include "CurvilinearSample.hpp"

#include <iomanip>

CurviLinearSample::CurviLinearSample()
    : s ()
    , d ()
    , theta ()
    , dd ()
    , ddd ()
    , ss ()
    , sss () 
{

}

CurviLinearSample::CurviLinearSample(const Eigen::Ref<Eigen::VectorXd>& s_,
                                     const Eigen::Ref<Eigen::VectorXd>& d_,
                                     const Eigen::Ref<Eigen::VectorXd>& theta_,
                                     const Eigen::Ref<Eigen::VectorXd>& dd_,
                                     const Eigen::Ref<Eigen::VectorXd>& ddd_,
                                     const Eigen::Ref<Eigen::VectorXd>& ss_,
                                     const Eigen::Ref<Eigen::VectorXd>& sss_)
    : s(s_)
    , d(d_)
    , theta(theta_)
    , dd(dd_)
    , ddd(ddd_)
    , ss(ss_)
    , sss(sss_) 
{

}

void CurviLinearSample::print(std::ostream& os) const
{
    int width = 15;
    os << std::fixed << std::setprecision(5);

    os << "CurviLinearSample:" << std::endl
       << std::setw(width) << "timeStep"
       << std::setw(width) << "s"
       << std::setw(width) << "d"
       << std::setw(width) << "theta"
       << std::setw(width) << "dd"
       << std::setw(width) << "ddd"
       << std::setw(width) << "ss"
       << std::setw(width) << "sss" << std::endl;

    int numRows = s.size();

    for (int i = 0; i < numRows; ++i)
    {
        os << std::setw(width) << i
           << std::setw(width) << s(i)
           << std::setw(width) << d(i)
           << std::setw(width) << theta(i)
           << std::setw(width) << dd(i)
           << std::setw(width) << ddd(i)
           << std::setw(width) << ss(i)
           << std::setw(width) << sss(i) << std::endl;
    }
}