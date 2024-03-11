//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <string>
#include <type_traits>

#include "polynomial.hpp"
#include "TrajectorySample.hpp"

#include "polynomialTrajectoryBinding.hpp"

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
    template <int Degree, int X0, int XD, typename RefType>
    void bindPolynomialtrajectory(py::module &m, const std::string &pre) 
    {
        static_assert(std::is_same<RefType, PolynomialTrajectory<Degree, X0, XD>>::value, "RefType mismatch");

        py::class_<PolynomialTrajectory<Degree, X0, XD>>(m, (pre + "Trajectory").c_str())
            .def(py::init([](double t0, 
                            double t1, 
                            Eigen::VectorXd x_0, 
                            Eigen::VectorXd x_d, 
                            Eigen::VectorXd x_0_order, 
                            Eigen::VectorXd x_d_order) 
                            {return new PolynomialTrajectory<Degree, X0, XD>(t0, t1, x_0, x_d, x_0_order, x_d_order);}), 
                            py::arg("tau_0"), 
                            py::arg("delta_tau"), 
                            py::arg("x_0"), 
                            py::arg("x_d"),
                            py::arg("x_0_order") = Eigen::VectorXd(), 
                            py::arg("x_d_order") = Eigen::VectorXd())
            .def_property_readonly("coeffs", &PolynomialTrajectory<Degree, X0, XD>::getCoeffs)
            .def("__call__", py::vectorize(&PolynomialTrajectory<Degree, X0, XD>::operator()))
            .def("squared_jerk_integral", &PolynomialTrajectory<Degree, X0, XD>::squaredJerkIntegral)
            .def_property_readonly("delta_tau", &PolynomialTrajectory<Degree, X0, XD>::get_t1)
            .def(py::pickle(
                [](const PolynomialTrajectory<Degree, X0, XD> &traj) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal
                    const auto coeffs = traj.getCoeffs();

                    py::dict d(
                        "coeffs"_a=coeffs
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    py::object obj = d["coeffs"];
                    typename Eigen::Vector<double, Degree + 1> coeffs = obj.cast<Eigen::Vector<double, Degree + 1>>();
                    PolynomialTrajectory<Degree, X0, XD> traj { coeffs };

                    return traj;
                }
            ));
    }

    void initBindPolynomialTrajectory(pybind11::module &m) 
    {
        // Bind the PolynomialTrajectory class
        bindPolynomialtrajectory<4, 3, 2, TrajectorySample::LongitudinalTrajectory>(m, "Quartic");
        bindPolynomialtrajectory<5, 3, 3, TrajectorySample::LateralTrajectory>(m, "Quintic");
    }

} //plannerCPP

