#ifndef CARTESIANSAMPLE_HPP
#define CARTESIANSAMPLE_HPP


#include <Eigen/Core>
#include <iostream>


class CartesianSample
{
public:
    /**
     * @brief Default constructor for the CartesianSample class.
     * 
     */
    CartesianSample();

    /**
     * @brief Constructor for the CartesianSample class.
     * 
     * @param position The Cartesian positions along the path.
     * @param theta The orientation of the sample.
     * @param velocity The velocity of the sample.
     * @param acceleration The acceleration of the sample.
     * @param kappa The curvature of the sample.
     * @param kappaDot The curvature rate of change of the sample.
     * @param currentTimeStep Time step of Sample
     */
    CartesianSample(const Eigen::Ref<Eigen::VectorXd>& x_,
                    const Eigen::Ref<Eigen::VectorXd>& y_,
                    const Eigen::Ref<Eigen::VectorXd>& theta_,
                    const Eigen::Ref<Eigen::VectorXd>& velocity_,
                    const Eigen::Ref<Eigen::VectorXd>& acceleration_,
                    const Eigen::Ref<Eigen::VectorXd>& kappa_,
                    const Eigen::Ref<Eigen::VectorXd>& kappaDot_);

    bool isInitialized = false;          ///< If the cartesisan coordinates are filled
    Eigen::VectorXd x;                   ///< The x positions in cartesian coordinates
    Eigen::VectorXd y;                   ///< The y positions in cartesian coordinates
    Eigen::VectorXd theta;               ///< The orientation of the sample.
    Eigen::VectorXd velocity;            ///< The velocity of the sample.
    Eigen::VectorXd acceleration;        ///< The acceleration of the sample.
    Eigen::VectorXd kappa;               ///< The curvature of the path.
    Eigen::VectorXd kappaDot;            ///< The curvature rate of change of the path.

    void print(std::ostream& os) const;
};


#endif //CARTESIANSAMPLE_HPP
