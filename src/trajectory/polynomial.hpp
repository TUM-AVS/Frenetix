#pragma once

#include <Eigen/Core>
#include <Eigen/QR>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include <iostream>

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
template <int Degree, int X0 = Eigen::Dynamic, int XD = Eigen::Dynamic>
class PolynomialTrajectory
{
    static_assert(X0 == Eigen::Dynamic || XD == Eigen::Dynamic || X0 + XD == Degree + 1, "dimensions error");

public:
    using VectorX0 = Eigen::Vector<double, X0>;
    using VectorXD = Eigen::Vector<double, XD>;

    explicit PolynomialTrajectory(const Eigen::Vector<double, Degree + 1>& coeffs) 
        : coeffs{coeffs} { }

    /**
     * @brief Constructs a new polynomial trajectory.
     *
     * @param t0 The first time.
     * @param t1 The second time.
     * @param x_0 The first point conditions.
     * @param x_d The second point conditions.
     */
    PolynomialTrajectory(double t0, double t1,
                         const VectorX0& x_0,
                         const VectorXD& x_d,
                         const VectorX0& x_0_order,
                         const VectorXD& x_d_order)
        : m_t0(t0)
        , m_t1(t1)
    {
        assert(((x_0.size()+x_d.size()-1)==Degree) && "To many or to less equations for the choosen degree");

        assert((x_0.size() == x_0_order.size()) && "Order size not correct");
        assert((x_d.size() == x_d_order.size()) && "Order size not correct");

        calc_coeffs(t0, t1, x_0, x_d, x_0_order, x_d_order);
    }

    /**
     * @brief Constructs a new polynomial trajectory.
     *
     * @param t0 The first time.
     * @param t1 The second time.
     * @param x_0 The first point conditions.
     * @param x_d The second point conditions.
     */
    PolynomialTrajectory(double t0, double t1,
                         const VectorX0& x_0,
                         const VectorXD& x_d)
        : m_t0(t0)
        , m_t1(t1)
    {
        VectorX0 x_0_order = VectorX0::LinSpaced(x_0.size(), 0, x_0.size() - 1);
        VectorXD x_d_order = VectorXD::LinSpaced(x_d.size(), 0, x_d.size() - 1);

        calc_coeffs(t0, t1, x_0, x_d, x_0_order, x_d_order);
    }

    PolynomialTrajectory() = default;

    /**
     * @brief Calculates the polynomial coefficients.
     */
    void calc_coeffs(double t0, double t1,
        VectorX0 x_0, VectorXD x_d,
        VectorX0 x_0_order, VectorXD x_d_order);


    /**
     * @brief Gets the polynomial coefficients.
     *
     * @return The coefficients as an Eigen::Vector of size Degree + 1.
     */
    Eigen::Vector<double,Degree+1> getCoeffs() const;

    /**
     * @brief Evaluates the polynomial trajectory at a given time and derivative.
     *
     * @param t The time at which to evaluate the polynomial.
     * @param derivative The order of the derivative to evaluate, defaults to 0.
     * @return The value of the polynomial at the given time and derivative.
     */
    double operator()(double t, double derivative = 0) const;

    template<int Order>
    constexpr double const_eval(double t) const;

    struct VVV {
        double x;
        double xx;
        double xxx;
    };

    constexpr VVV horner_eval(double t) const;

    double get_t0() const { return m_t0; }

    double get_t1() const { return m_t1; }

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
    constexpr double squaredJerkIntegral(double t) const;

private:
    Eigen::Vector<double, Degree+1> coeffs;
    double m_t0, m_t1;
};

template<int Degree, int X0, int XD>
void PolynomialTrajectory<Degree, X0, XD>::calc_coeffs(double t0, double t1,
    VectorX0 x_0, VectorXD x_d,
    VectorX0 x_0_order, VectorXD x_d_order)
{
    constexpr int n = Degree + 1;

    ////cache a if it stays the same
    //static double lastT0 = 0;
    //static double lastT1 = 0;

    Eigen::Matrix<double, n, n> a;
    Eigen::Vector<double, n> b;


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

template<int Degree, int X0, int XD>
Eigen::Vector<double,Degree+1> PolynomialTrajectory<Degree, X0, XD>::getCoeffs() const
{
    return coeffs;
}

template<int Degree, int X0, int XD>
inline constexpr
typename PolynomialTrajectory<Degree, X0, XD>::VVV
PolynomialTrajectory<Degree, X0, XD>::horner_eval(double t) const
{
    static_assert(Degree > 2, "invalid Degree: must greater than 2");

    double f = coeffs[Degree];
    double fp = f;
    double fpp = fp;

    for (int i = Degree - 1; i >= 2; i--) {
        f = std::fma(t, f, coeffs[i]);
        fp = std::fma(t, fp, f);
        fpp = std::fma(t, fpp, fp);
    }
    f = std::fma(t, f, coeffs[1]);
    fp = std::fma(t, fp, f);
    f = std::fma(t, f, coeffs[0]);

    return VVV {f, fp, fpp};
}

template<int Degree, int X0, int XD>
inline double PolynomialTrajectory<Degree, X0, XD>::operator()(double t, double derivative) const
{
    double res {0};

    if (std::trunc(derivative) != derivative) { std::abort(); }

    #pragma clang loop unroll(full)
    for (int iii = 0; iii < coeffs.size(); ++iii)
    {
        if(iii >= derivative)
        {
            double coeff {1};

            for(int uuu = derivative; uuu > 0; uuu--)
            {
                coeff *= (iii-uuu+1);
            }
            res += coeffs[iii] * coeff *  std::pow(t, iii-(int)derivative);
        }
    }
    return res;
}

template<int Degree, int X0, int XD>
constexpr double PolynomialTrajectory<Degree, X0, XD>::squaredJerkIntegral(double t) const
{
    static_assert(Degree == 5 || Degree == 4, "squared_jerk_integral() is only implemented for Degree 4 and 5");

    if (Degree == 5) {
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
    } else if constexpr(Degree == 4) {
        double t2 = t * t;
        double t3 = t2 * t;

        double integral_squared_jerk = (36 * coeffs[3] * coeffs[3] * t +
                                        144 * coeffs[3] * coeffs[4] * t2 +
                                        192 * coeffs[4] * coeffs[4] * t3);
        return integral_squared_jerk;
    }
}

