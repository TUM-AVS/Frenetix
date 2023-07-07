#ifndef POLYNOMIALTRAJECTORYBINDING_HPP
#define POLYNOMIALTRAJECTORYBINDING_HPP

#pragma clang diagnostic ignored "-Wdeprecated-anon-enum-enum-conversion"

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "polynomial.hpp"

//#pragma clang diagnostic push
//#pragma clang diagnostic pop


namespace nb = nanobind;

namespace plannerCPP
{

    /**
     * @brief Function to create polynomial Trajectories of different degrees and giving it a suffix
     *
     * @tparam Degree
     * @param m
     * @param pre Pre Suffix for the class
     */
    template <int Degree>
    void bindPolynomialtrajectory(nb::module_ &m, const std::string &pre)
    {
        nb::class_<PolynomialTrajectory<Degree>>(m, (pre + "Trajectory").c_str())
        /*
            .def(nb::init([](double t0,
                            double t1,
                            Eigen::VectorXd x_0,
                            Eigen::VectorXd x_d,
                            Eigen::VectorXd x_0_order,
                            Eigen::VectorXd x_d_order)
                            {return PolynomialTrajectory<Degree>(t0, t1, x_0, x_d, x_0_order, x_d_order);}),
                            nb::arg("tau_0"),
                            nb::arg("delta_tau"),
                            nb::arg("x_0"),
                            nb::arg("x_d"),
                            nb::arg("x_0_order") = Eigen::VectorXd(),
                            nb::arg("x_d_order") = Eigen::VectorXd()) */
            .def("coeffs", &PolynomialTrajectory<Degree>::getCoeffs)
            .def("__call__", &PolynomialTrajectory<Degree>::operator())

            //.def("__call__", nb::vectorize(&PolynomialTrajectory<Degree>::operator()))
            .def("squared_jerk_integral", &PolynomialTrajectory<Degree>::squaredJerkIntegral)
            .def_prop_rw("delta_tau",
                        &PolynomialTrajectory<Degree>::get_t1,
                        nullptr);
    }

    void initBindPolynomialTrajectory(nb::module_ &m)
    {
        // Bind the PolynomialTrajectory class
        bindPolynomialtrajectory<4>(m, "Quartic");
        bindPolynomialtrajectory<5>(m, "Quintic");
    }

} //plannerCPP

#endif //POLYNOMIALTRAJECTORYBINDING_HPP
