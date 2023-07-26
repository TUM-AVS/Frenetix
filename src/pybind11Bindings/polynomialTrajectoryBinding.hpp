#ifndef POLYNOMIALTRAJECTORYBINDING_HPP
#define POLYNOMIALTRAJECTORYBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "polynomial.hpp"

namespace py = pybind11;

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
    void bindPolynomialtrajectory(py::module &m, const std::string &pre) 
    {
        py::class_<PolynomialTrajectory<Degree>>(m, (pre + "Trajectory").c_str())
            .def(py::init([](double t0, 
                            double t1, 
                            Eigen::VectorXd x_0, 
                            Eigen::VectorXd x_d, 
                            Eigen::VectorXd x_0_order, 
                            Eigen::VectorXd x_d_order) 
                            {return new PolynomialTrajectory<Degree>(t0, t1, x_0, x_d, x_0_order, x_d_order);}), 
                            py::arg("tau_0"), 
                            py::arg("delta_tau"), 
                            py::arg("x_0"), 
                            py::arg("x_d"),
                            py::arg("x_0_order") = Eigen::VectorXd(), 
                            py::arg("x_d_order") = Eigen::VectorXd())
            .def_property_readonly("coeffs", &PolynomialTrajectory<Degree>::getCoeffs)
            .def("__call__", py::vectorize(&PolynomialTrajectory<Degree>::operator()))
            .def("squared_jerk_integral", &PolynomialTrajectory<Degree>::squaredJerkIntegral)
            .def_property_readonly("delta_tau",
                        &PolynomialTrajectory<Degree>::get_t1);
    }

    void initBindPolynomialTrajectory(pybind11::module &m) 
    {
        // Bind the PolynomialTrajectory class
        bindPolynomialtrajectory<4>(m, "Quartic");
        bindPolynomialtrajectory<5>(m, "Quintic");
    }

} //plannerCPP

#endif //POLYNOMIALTRAJECTORYBINDING_HPP