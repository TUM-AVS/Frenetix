//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/stl/optional.h>

#include <Eigen/Core>

#include <string>
#include <type_traits>

#include "polynomial.hpp"
#include "TrajectorySample.hpp"

#include "polynomialTrajectoryBinding.hpp"

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
    template <int Degree, int X0, int XD, typename RefType>
    void bindPolynomialtrajectory(nb::module_ &m, const std::string &pre) 
    {
        using Traj = PolynomialTrajectory<Degree, X0, XD>;

        static_assert(std::is_same<RefType, Traj>::value, "RefType mismatch");

        nb::class_<Traj, LinearTrajectory>(m, (pre + "Trajectory").c_str())
            .def("__init__", [](Traj *traj,
                             double t0, 
                            double t1,
                            typename Traj::VectorX0 x_0, 
                            typename Traj::VectorXD x_d, 
                            std::optional<typename Traj::OrderVectorX0> x_0_order, 
                            std::optional<typename Traj::OrderVectorXD> x_d_order) 
                            {
                    new (traj) Traj(t0, t1, x_0, x_d, x_0_order.value_or(Traj::defaultX0Order()), x_d_order.value_or(Traj::defaultXDOrder()));
                 }, 
                            nb::arg("tau_0"), 
                            nb::arg("delta_tau"), 
                            nb::arg("x_0"), 
                            nb::arg("x_d"),
                            nb::arg("x_0_order"), 
                            nb::arg("x_d_order"))
            .def_prop_ro("coeffs", &Traj::getCoeffs)
            .def("__call__", [] (const Traj& traj, double t, double derivative) { return traj(t, derivative); }, nb::arg("t"), nb::arg("derivative") = 0.0)
            .def("squared_jerk_integral", &Traj::squaredJerkIntegral)
            .def_prop_ro("delta_tau", &Traj::get_t1)
            .def("__getstate__",
                [](const Traj &traj) {
                    const auto coeffs = traj.getCoeffs();

                    nb::dict d;
                    d["coeffs"] = coeffs;

                    return d;
                })
            .def("__setstate__",
                [](PolynomialTrajectory<Degree> &traj, nb::dict d) {
                    nb::object obj = d["coeffs"];
                    auto coeffs = nb::cast<typename Traj::Coeffs>(obj);
                    new (&traj) Traj { coeffs };
                }
            )
    ;
    }

    void initBindPolynomialTrajectory(nb::module_ &m) 
    {
        nb::class_<LinearTrajectory>(m, "LinearTrajectory")
            .def("__call__", [] (const LinearTrajectory& traj, double t) { return traj.horner_eval(t).x; })
            .def("squared_jerk_integral", &LinearTrajectory::squaredJerkIntegral);

        // Bind the PolynomialTrajectory class
        bindPolynomialtrajectory<4, 3, 2, TrajectorySample::LongitudinalTrajectory>(m, "Quartic");
        bindPolynomialtrajectory<5, 3, 3, TrajectorySample::LateralTrajectory>(m, "Quintic");
    }

} //plannerCPP

