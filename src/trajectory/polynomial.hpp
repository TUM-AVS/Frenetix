#pragma once

#include <cmath>
#include <stdexcept>

#include <numeric>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU>

struct HornerEvalResult {
    double x;
    double xx;
    double xxx;
};

class LinearTrajectory {
public:
    virtual ~LinearTrajectory() = default;

    virtual HornerEvalResult horner_eval(double t) const = 0;
    virtual double squaredJerkIntegral(double t) const = 0;
};

/**
 * @brief A class representing a polynomial trajectory of a given degree.
 *
 * The PolynomialTrajectory class is a template class, with the Degree template parameter
 * representing the degree of the polynomial trajectory. It is designed to create
 * a polynomial trajectory that satisfies a set of initial and final point conditions.
 *
 * The number of initial condions must equal to the Degree + 1.
 *
 * The class solves a system of linear equations to determine the coefficients of the monom
 * basis of the polynomial trajectory. The trajectory can be evaluated at a given time
 * using the overloaded call operator with an optional derivative parameter.
 *
 * @tparam Degree Degree of the polynomial trajectory.
 * @tparam X0 Static dimension of first point conditions.
 * @tparam XD Static dimension of second point conditions.
 */
template <int Degree, int X0 = Eigen::Dynamic, int XD = Eigen::Dynamic>
class PolynomialTrajectory : public LinearTrajectory
{
    static_assert(X0 == Eigen::Dynamic || XD == Eigen::Dynamic || X0 + XD == Degree + 1, "dimensions error");
    static_assert((X0 >= 1 || X0 == Eigen::Dynamic) && (XD >= 1 || XD == Eigen::Dynamic), "dimensions need to be positive");
    static_assert(Degree >= 0, "degree needs to be positive");

    static constexpr bool DynamicOrder = X0 == Eigen::Dynamic || XD == Eigen::Dynamic;

    static constexpr int N = DynamicOrder ? Eigen::Dynamic : (Degree + 1);

public:
    using VectorX0 = Eigen::Vector<double, X0>;
    using VectorXD = Eigen::Vector<double, XD>;
    using OrderVectorX0 = Eigen::Vector<int, X0>;
    using OrderVectorXD = Eigen::Vector<int, XD>;
    using Coeffs = Eigen::Vector<double, N>;
    using TemporalMatrix = Eigen::Matrix<double, N, N>;

    // Both decompositions work - PartialPivLU is a little faster while providing comparable accuracy
    // using TemporalMatrixDecomp = Eigen::ColPivHouseholderQR<TemporalMatrix>;
    using TemporalMatrixDecomp = Eigen::PartialPivLU<TemporalMatrix>;

    explicit PolynomialTrajectory(const Coeffs& coeffs) 
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
                         const OrderVectorX0& x_0_order,
                         const OrderVectorXD& x_d_order)
        : m_t0(t0)
        , m_t1(t1)
    {
        if constexpr (X0 == Eigen::Dynamic || XD == Eigen::Dynamic) {
            if (x_0.size() + x_d.size() != Degree + 1) {
                throw std::invalid_argument { "Too many or too few equations for the chosen degree" };
            }

            if (x_0.size() != x_0_order.size() || x_d.size() != x_d_order.size()) {
                throw std::invalid_argument { "Order size not correct" };
            }
        }

        coeffs = calc_coeffs(t0, t1, x_0, x_d, x_0_order, x_d_order);
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
        : PolynomialTrajectory(t0, t1, x_0, x_d, defaultX0Order(), defaultXDOrder())
    {
    }

    /**
     * @brief Constructs a new polynomial trajectory.
     *
     * @param x_0 The first point conditions.
     * @param x_d The second point conditions.
     */
    PolynomialTrajectory(const TemporalMatrixDecomp& temp,
                         const VectorX0& x_0,
                         const VectorXD& x_d)
    {
        coeffs = calc_coeffs_cached(temp, x_0, x_d);

    }

    static constexpr OrderVectorX0 defaultX0Order()
    {
        OrderVectorX0 x_0_order;
        std::iota(x_0_order.begin(), x_0_order.end(), 0);
        return x_0_order;
    }

    static constexpr OrderVectorXD defaultXDOrder()
    {
        OrderVectorXD x_d_order;
        std::iota(x_d_order.begin(), x_d_order.end(), 0);
        return x_d_order;
    }

    PolynomialTrajectory() = default;

    /**
     * @brief Calculates the polynomial coefficients.
     */
    static Coeffs calc_coeffs(double t0, double t1,
        const VectorX0& x_0, const VectorXD& x_d,
        const OrderVectorX0& x_0_order, const OrderVectorXD& x_d_order);


    /**
     * @brief Gets the polynomial coefficients.
     *
     * @return The coefficients as an Eigen::Vector of size Degree + 1.
     */
    Coeffs getCoeffs() const;

    /**
     * @brief Evaluates the polynomial trajectory at a given time and derivative.
     *
     * @param t The time at which to evaluate the polynomial.
     * @param derivative The order of the derivative to evaluate, defaults to 0.
     * @return The value of the polynomial at the given time and derivative.
     */
    double operator()(double t, double derivative = 0) const;

    HornerEvalResult horner_eval(double t) const override;

    inline constexpr
    double horner_eval_single(double t) const
    {
        double f = coeffs[Degree];

        for (int i = Degree - 1; i >= 0; i--) {
            f = std::fma(t, f, coeffs[i]);
        }

        return f;
    }

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
    double squaredJerkIntegral(double t) const override;

    static inline typename Eigen::Vector<double, N> mkRow(int kkk, double t) {
        Eigen::Vector<double, N> x = Eigen::Vector<double, N>::Zero();

        double t_exp = 1.0;

        for (int colIdx = kkk; colIdx < N; colIdx++)
        {
            int coeff = 1;

            for(int uuu = colIdx + 1 - kkk; uuu <= colIdx; uuu++)
            {
                coeff *= uuu;
            }

            x(colIdx) = coeff * t_exp;
            t_exp *= t;
        }

        return x;
    }

    static TemporalMatrix temporalMatrix(double t0, double t1,
        const OrderVectorX0& x_0_order = defaultX0Order(), const OrderVectorXD& x_d_order = defaultXDOrder()) {
        static_assert(X0 != Eigen::Dynamic && XD != Eigen::Dynamic, "dynamic order unsupported");

        Eigen::Matrix<double, N, N> anew;

        for (int idx = 0; idx < X0; idx++)
        {
            anew.row(idx) = mkRow(x_0_order[idx], t0);
        }

        for (int idx = 0; idx < XD; idx++)
        {
            anew.row(X0 + idx) = mkRow(x_d_order[idx], t1);
        }

        return anew;
    }

    static inline TemporalMatrixDecomp temporalCoeffsDecomp(const TemporalMatrix& temp) {
        return TemporalMatrixDecomp(temp);
    }

    static TemporalMatrixDecomp temporalCoeffs(double t0, double t1,
        const OrderVectorX0& x_0_order = defaultX0Order(), const OrderVectorXD& x_d_order = defaultXDOrder()) {
        TemporalMatrix mat = temporalMatrix(t0, t1, x_0_order, x_d_order);
        return temporalCoeffsDecomp(mat);
    }

    /**
     * @brief Calculates the polynomial coefficients using a cached decomposition.
     */
    static Coeffs calc_coeffs_cached(const TemporalMatrixDecomp& temp,
        const VectorX0& x_0, const VectorXD& x_d) {
        static_assert(X0 != Eigen::Dynamic && XD != Eigen::Dynamic, "dynamic order unsupported");

        Eigen::Vector<double, N> b;

        b.template head<X0>() = x_0;
        b.template tail<XD>() = x_d;

        Coeffs solution = temp.solve(b);

        return solution;
    }

    /**
     * @brief Calculates the polynomial coefficients.
     */
    static Coeffs calc_coeffs_new(double t0, double t1,
        const OrderVectorX0& x_0_order, const OrderVectorXD& x_d_order,
        const VectorX0& x_0, const VectorXD& x_d) {
        const auto temp = temporalCoeffs(t0, t1, x_0_order, x_d_order);

        return calc_coeffs_cached(temp, x_0, x_d);
    }

private:
    Coeffs coeffs;
    double m_t0, m_t1;
};

template<int Degree, int X0, int XD>
typename PolynomialTrajectory<Degree, X0, XD>::Coeffs PolynomialTrajectory<Degree, X0, XD>::calc_coeffs(double t0, double t1,
    const VectorX0& x_0, const VectorXD& x_d,
    const OrderVectorX0& x_0_order, const OrderVectorXD& x_d_order)
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
    return a.colPivHouseholderQr().solve(b);
}

template<int Degree, int X0, int XD>
typename PolynomialTrajectory<Degree, X0, XD>::Coeffs PolynomialTrajectory<Degree, X0, XD>::getCoeffs() const
{
    return coeffs;
}

template<int Degree, int X0, int XD>
inline
HornerEvalResult
PolynomialTrajectory<Degree, X0, XD>::horner_eval(double t) const
{
    static_assert(Degree > 2, "invalid Degree: must greater than 2 for horner_eval");

    double f = coeffs[Degree];
    double fp = f;
    double fpp = fp;

    for (int i = Degree - 1; i >= 2; i--) {
        f = t * f + coeffs[i];
        fp = t * fp + f;
        fpp = t * fpp + fp;
    }
    f = t * f + coeffs[1];
    fp = t * fp + f;
    f = t * f + coeffs[0];

    return HornerEvalResult {f, fp, fpp};
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
double PolynomialTrajectory<Degree, X0, XD>::squaredJerkIntegral(double t) const
{
    static_assert(Degree == 5 || Degree == 4, "squared_jerk_integral() is only implemented for Degree 4 and 5");

    if constexpr (Degree == 5) {
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

extern template class PolynomialTrajectory<4, 3, 2>;
extern template class PolynomialTrajectory<5, 3, 3>;

