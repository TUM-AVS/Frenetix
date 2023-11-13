#include "util.hpp"

#include <stddef.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace util
{    
    double absmin(Eigen::VectorXd x)
    {
        auto abs_x = x.cwiseAbs().eval();
        Eigen::VectorXd::Index min_index;
        abs_x.col(0).minCoeff(&min_index);
        return x[min_index];
    }

    double make_valid_orientation(double angle)
    {
        while(angle > TWO_PI)
        {
            angle = angle - TWO_PI;
        }
        while(angle < -TWO_PI)
        {
            angle = angle + TWO_PI;
        }
            
        return angle;
    }

    double interpolate_angle(const double x,
                            const double x1, 
                            const double x2, 
                            const double y1, 
                            const double y2)
    {
        // Define an epsilon value for comparison
        const double epsilon = std::numeric_limits<double>::epsilon() * 100;

        // Check if x1 and x2 are almost equal
        if (std::abs(x1 - x2) <= epsilon) {
            return make_valid_orientation(y1);
        }

        auto delta = y2 - y1;
        auto delta_2pi_minus = delta - TWO_PI;
        auto delta_2pi_plus = delta + TWO_PI;
        Eigen::VectorXd vec(3);
        vec << delta, delta_2pi_minus, delta_2pi_plus;
        delta = absmin(vec);
        return make_valid_orientation(delta * (x - x1) / (x2 - x1) + y1);
    }

    geometry::EigenPolyline matrixToVector2d(const RowMatrixXd& mat)
    {
        if (mat.cols() != 2)
        {
            throw std::runtime_error("Input matrix must have 2 columns.");
        }

        geometry::EigenPolyline vec;

        for (int i = 0; i < mat.rows(); ++i)
        {
            Eigen::Vector2d v(mat(i, 0), mat(i, 1));
            vec.push_back(v);
        }

        return vec;
    }

    Eigen::VectorXd computeGradient(const Eigen::Ref<Eigen::VectorXd> array, const Eigen::Ref<Eigen::VectorXd> distance)
    {
        size_t n = array.size();

        Eigen::VectorXd gradient (n);

        for (size_t i = 1; i < n - 1; ++i) 
        {
            gradient(i) = (array(i + 1) - array(i - 1)) / (distance(i + 1) - distance(i - 1));
        }

        // Handle edge cases
        gradient(0) = (array(1) - array(0)) / (distance(1) - distance(0));
        gradient(n - 1) = (array(n - 1) - array(n - 2)) / (distance(n - 1) - distance(n - 2));

        return gradient;
    }

    double simpsonIntegration(Eigen::Ref<Eigen::VectorXd> array, double dT)
    {
        double value {0};
        int size = array.size();

        for (int iii = 0; iii < size - 2; iii += 2) 
        {
            value += array(iii) + 4*array(iii+1) + array(iii+2);
        }

        // If the number of data points is odd, use the trapezoidal rule for the last interval.
        if (size % 2 == 0)
        {
            value += (array(size - 1) + array(size - 2)) / 2.0;
        }

        return value * dT / 3;
    }
}