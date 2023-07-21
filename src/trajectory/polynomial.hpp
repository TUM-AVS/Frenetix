#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP


#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <cassert>

/**
 * @brief A class representing a polynomial trajectory of a given degree.
 *
 * The PolynomialTrajectory class is a template class, with the Degree template parameter
 * representing the degree of the polynomial trajectory. It is designed to create
 * a polynomial trajectory that satisfies a set of initial and final point conditions. 
 * 
 * The number of initial condions must euqal to the Degree + 1.
 *
 * The class solves a system of linear equations to determine the coefficients of the monom 
 * basis of the polynomial trajectory. The trajectory can be evaluated at a given time 
 * using the overloaded call operator with an optional derivative parameter.
 * 
 * @tparam Degree Degree of the polynomial trajectory.
 */
template <int Degree>
class PolynomialTrajectory 
{
public:
    /**
     * @brief Constructs a new polynomial trajectory.
     * 
     * @param t0 The first time.
     * @param t1 The second time.
     * @param x_0 The first point conditions.
     * @param x_d The second point conditions.
     */
    PolynomialTrajectory(double t0, double t1,
                         Eigen::VectorXd x_0,
                         Eigen::VectorXd x_d,
                         Eigen::VectorXd x_0_order = Eigen::VectorXd(),
                         Eigen::VectorXd x_d_order = Eigen::VectorXd());

    PolynomialTrajectory() = default;
    /**
     * @brief Calculates the polynomial coefficients.
     */
    void calc_coeffs(double& t0, double& t1);


    /**
     * @brief Gets the polynomial coefficients.
     *
     * @return The coefficients as an Eigen::Vector of size Degree + 1.
     */
    Eigen::Vector<double,Degree+1> getCoeffs();

    /**
     * @brief Evaluates the polynomial trajectory at a given time and derivative.
     *
     * @param t The time at which to evaluate the polynomial.
     * @param derivative The order of the derivative to evaluate, defaults to 0.
     * @return The value of the polynomial at the given time and derivative.
     */
    double operator()(double t, double derivative = 0) const;

    double get_t0() const;

    double get_t1() const;

    /**
    * @brief Computes the integral of the squared jerk over the trajectory for a given time \p t.
    *
    * For a polynomial trajectory, the jerk is the third derivative of the position.
    * This function calculates the squared jerk, integrated over time, up to the specified time \p t.
    * This is specifically implemented for polynomials of Degree 4 and 5.
    *
    * @tparam Degree The degree of the polynomial trajectory. Valid values are 4 and 5.
    * @param t The time up to which the squared jerk is integrated.
    * @return The value of the integral of the squared jerk from 0 to \p t.
    *
    * @throw std::invalid_argument If the Degree is not 4 or 5.
    */
    double squaredJerkIntegral(double t) const;

private:
    double m_t0, m_t1;
    Eigen::VectorXd x_0, x_d, x_0_order, x_d_order;
    Eigen::Vector<double, Degree+1> coeffs;
    void initialize_orders();
};

template<int Degree>
PolynomialTrajectory<Degree>::PolynomialTrajectory(double t0, double t1,
                                                   Eigen::VectorXd x_0,
                                                   Eigen::VectorXd x_d,
                                                   Eigen::VectorXd x_0_order,
                                                   Eigen::VectorXd x_d_order)
    : m_t0(t0)
    , m_t1(t1)
    , x_0(x_0)
    , x_d(x_d)
    , x_0_order(x_0_order)
    , x_d_order(x_d_order)
{
    assert(((x_0.size()+x_d.size()-1)==Degree) && "To many or to less equations for the choosen degree");

    initialize_orders();
    calc_coeffs(t0, t1);
}

template<int Degree>
void PolynomialTrajectory<Degree>::initialize_orders()
{
    if (x_0_order.size() == 0) 
    {
        x_0_order = Eigen::VectorXd::LinSpaced(x_0.size(), 0, x_0.size() - 1);
    }
    else
    {
        assert((x_0.size() == x_0_order.size()) && "Order size not correct");
    }

    if (x_d_order.size() == 0) 
    {
        x_d_order = Eigen::VectorXd::LinSpaced(x_d.size(), 0, x_d.size() - 1);
    }
    else
    {
        assert((x_d.size() == x_d_order.size()) && "Order size not correct");
    }
}

template<int Degree>
void PolynomialTrajectory<Degree>::calc_coeffs(double& t0, double& t1) 
{
    int n = Degree + 1;
    static Eigen::MatrixXd a(n, n);
    Eigen::VectorXd b(n);

    for(int iii = 0; iii < n; iii++)
    {
        bool isFirstPoint = iii < x_0.size();
        double t = isFirstPoint ? t0 : t1;
        int kkk = isFirstPoint ? x_0_order[iii] : x_d_order[iii - x_0.size()];


        for(int jjj = 0; jjj < n; jjj++)
        {
            if(jjj >= kkk)
            {
                double coeff {1};

                for(int uuu = kkk; uuu > 0; uuu--)
                {
                    coeff *= (jjj-uuu+1);
                }
                a(iii, jjj) = coeff * std::pow(t, jjj - kkk);
            }
            else
            {
                a(iii, jjj) = 0;
            }
        }
        b(iii) = isFirstPoint ? x_0[iii] : x_d[iii - x_0.size()];
    }
    coeffs = a.colPivHouseholderQr().solve(b);
}

template<int Degree>
Eigen::Vector<double,Degree+1> PolynomialTrajectory<Degree>::getCoeffs()
{
    return coeffs;
}

template<int Degree>
double PolynomialTrajectory<Degree>::operator()(double t, double derivative) const 
{
    double res {0};

    for (int iii = 0; iii < coeffs.size(); ++iii) 
    {
        if(iii >= derivative)
        {
            double coeff {1};
            
            for(int uuu = derivative; uuu > 0; uuu--)
            {
                coeff *= (iii-uuu+1);
            }
            res += coeffs[iii] * coeff *  std::pow(t, iii-derivative);
        }   
    }
    return res;
}

template<int Degree>
double PolynomialTrajectory<Degree>::get_t0() const
{
    return m_t0;
}

template<int Degree>
double PolynomialTrajectory<Degree>::get_t1() const
{
    return m_t1;
}

template<int Degree>
double PolynomialTrajectory<Degree>::squaredJerkIntegral(double t) const
{
    if constexpr(Degree == 5)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        double integral_squared_jerk = (36 * coeffs[3] * coeffs[3] * t +
                                        144 * coeffs[3] * coeffs[4] * t2 +
                                        240 * coeffs[3] * coeffs[5] * t3 +
                                        192 * coeffs[4] * coeffs[4] * t3 +
                                        720 * coeffs[4] * coeffs[5] * t4 +
                                        720 * coeffs[5] * coeffs[5] * t5);
        return integral_squared_jerk;
    }
    else if constexpr(Degree == 4)
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;

        double integral_squared_jerk = (36 * coeffs[3] * coeffs[3] * t +
                                        144 * coeffs[3] * coeffs[4] * t2 +
                                        192 * coeffs[4] * coeffs[4] * t3);
        return integral_squared_jerk;
    }
    else
    {
        throw std::invalid_argument("squared_jerk_integral() is only implemented for Degree 4 and 5");
    }
}


#endif //POLYNOMIAL_HPP