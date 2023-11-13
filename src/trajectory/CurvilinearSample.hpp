#ifndef CURVILINEARSAMPLE_HPP
#define CURVILINEARSAMPLE_HPP

#include <Eigen/Core>
#include <iostream>



class CurviLinearSample
{
public:

    /**
     * @brief Constructor for the CurvilinearSample class.
     * 
     * @param currentTimeStep Time step of Sample
     */
    CurviLinearSample();

    /**
     * @brief Constructor for the CurvilinearSample class.
     * 
     * @param s_ The longitudinal positions along the path.
     * @param d_ The lateral positions along the path.
     * @param theta_ The orientation of the sample.
     * @param currentTimeStep Time step of Sample
     * @param dd_ The lateral velocity of the sample.
     * @param ddd_ The lateral acceleration of the sample.
     * @param ss_ The longitudinal velocity of the sample.
     * @param sss_ The longitudinal acceleration of the sample.
     */
    CurviLinearSample(const Eigen::Ref<Eigen::VectorXd>& s_, 
                      const Eigen::Ref<Eigen::VectorXd>& d_,
                      const Eigen::Ref<Eigen::VectorXd>& theta_,
                      const Eigen::Ref<Eigen::VectorXd>& dd_,
                      const Eigen::Ref<Eigen::VectorXd>& ddd_, 
                      const Eigen::Ref<Eigen::VectorXd>& ss_,
                      const Eigen::Ref<Eigen::VectorXd>& sss_);


    void print(std::ostream& os) const;

    bool isInitialized = false;                  ///< If the curvilinear coordinates are filled
    Eigen::VectorXd s;
    Eigen::VectorXd d;
    Eigen::VectorXd theta;
    Eigen::VectorXd dd;
    Eigen::VectorXd ddd;
    Eigen::VectorXd ss;
    Eigen::VectorXd sss;
};

#endif //CURVILINEARSAMPLE_HPP