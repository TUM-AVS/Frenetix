#ifndef UTIL_HPP
#define UTIL_HPP

#include <math.h>
#include <Eigen/Core>
#include <array>

#include "geometry/util.h"

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using SamplingMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, 13, Eigen::RowMajor>;


namespace util
{
    constexpr double TWO_PI = 2 * M_PI;

    double absmin(Eigen::VectorXd x);

    double make_valid_orientation(double angle);

    double interpolate_angle(const double x,
                             const double x1, 
                             const double x2, 
                             const double y1, 
                             const double y2);

    /**
     * @brief Converts a 2-column RowMatrixXd to an EigenPolyline.
     * 
     * @param mat The input RowMatrixXd to be converted.
     * 
     * @return An EigenPolyline containing the converted Vector2d points.
     * 
     * This function takes an input matrix mat with two columns and converts it into an EigenPolyline
     * of Vector2d points. If the input matrix doesn't have 2 columns, a std::runtime_error is thrown.
     */
    geometry::EigenPolyline matrixToVector2d(const RowMatrixXd& mat);

    /**
     * @brief Computes the gradient of an array of values given the distances between them.
     *  
     * @param array A reference to a VectorXd array containing the values to compute the gradient for.
     * @param distance A reference to a VectorXd array containing the distances between the values in array.
     * 
     * @return A VectorXd containing the computed gradient values.
     * 
     * This function computes the gradient of the array by taking the difference between adjacent values
     * and dividing by the corresponding distances in distance. The result is stored in a new VectorXd gradient.
     * The function handles edge cases by using the first and last distances to compute the gradient at the edges.
     * Example usage:
     * Eigen::VectorXd arr(5);
     * Eigen::VectorXd dist(5);
     * arr << 1, 2, 4, 7, 11;
     * dist << 0, 1, 3, 6, 10;
     * Eigen::VectorXd grad = computeGradient(arr, dist);
     */
    Eigen::VectorXd computeGradient(Eigen::Ref<Eigen::VectorXd> array, Eigen::Ref<Eigen::VectorXd> distance);

    /**
     * @brief Numerically integrates the input array using Simpson's rule and Trapezoidal rule.
     *
     * This function uses Simpson's rule to compute the numerical integral of the input array over
     * a given interval. The interval is specified by the step size \c dT. Simpson's rule approximates
     * the definite integral of a function by a series of parabolic arcs. If the number of data points
     * is not even, it uses the Trapezoidal rule to compute the integral for the last interval.
     *
     * @param array An Eigen vector containing the data points to be integrated.
     * @param dT The step size, i.e., the distance between consecutive data points in the array.
     * @return The computed integral of the array over the specified interval.
     */
    double simpsonIntegration(Eigen::Ref<Eigen::VectorXd> array, double dT);




    struct Position 
    {
        double x, y, z;
    };

    struct Orientation 
    {
        double x, y, z, w;
    };

    struct Pose 
    {
        Position position;
        Orientation orientation;
    };

    struct PoseWithCovariance 
    {
        Pose pose;
        std::array<double, 36> covariance;
    };
}
#endif //UTIL_HPP