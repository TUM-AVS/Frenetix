#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/QR>

#include "TrajectoryHandler.hpp"

#include "CostStrategy.hpp"
#include "FeasabilityStrategy.hpp"
#include "TrajectorySample.hpp"
#include "TrajectoryStrategy.hpp"
#include "polynomial.hpp"

#include <taskflow/algorithm/transform.hpp>
#include <taskflow/algorithm/for_each.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>


TrajectoryHandler::TrajectoryHandler(double dt)
    : m_dt(dt)
{

}

void TrajectoryHandler::setAllCostWeightsToZero()
{
    for(auto& [functionName, costStrategy] : m_costFunctions)
    {
        if(costStrategy)
        {
            costStrategy->updateCostWeight(0.0);
        }
    }
}

void TrajectoryHandler::clearCostFunctions()
{
    m_costFunctions.clear();
}

void TrajectoryHandler::addCostFunction(std::shared_ptr<CostStrategy> function)
{

    // check if there exits a cost weight for the given cost function
    m_costFunctions.insert_or_assign(function->getFunctionName(), function);
}

void TrajectoryHandler::addFeasabilityFunction(std::shared_ptr<FeasabilityStrategy> function)
{
    m_feasabilityFunctions.insert_or_assign(function->getFunctionName(),function);
}

void TrajectoryHandler::addFunction(std::shared_ptr<TrajectoryStrategy> function)
{
    m_otherFunctions.insert_or_assign(function->getFunctionName(),function);
}

void TrajectoryHandler::evaluateAllCurrentFunctions(bool calculateAllCosts)
{
    //Iterate over all trajectories
    for(auto& trajectory: m_trajectories)
    {
        evaluateTrajectory(trajectory, calculateAllCosts);    
    }

    removeInvalid();
}

void TrajectoryHandler::evaluateTrajectory(TrajectorySample& trajectory, bool calculateAllCosts)
{
    for(auto& [funName, function] : m_otherFunctions)
    {
        function->evaluateTrajectory(trajectory);
    }

    if (!trajectory.m_valid) return;

    for(auto& [funName, function] : m_feasabilityFunctions) 
    {
        function->evaluateTrajectory(trajectory);
    }

    if (!trajectory.m_valid) return;

    if (trajectory.m_feasible || calculateAllCosts) 
    {
        for(auto& [funName, function] : m_costFunctions)
        {
            function->evaluateTrajectory(trajectory);
        }
    }
}

void TrajectoryHandler::evaluateAllCurrentFunctionsConcurrent(bool calculateAllCosts)
{
    //Iterate over all trajectories
    tf::Task t_eval = m_taskflow.for_each(m_trajectories.begin(), m_trajectories.end(), [this, calculateAllCosts] (auto& trajectory) {
        evaluateTrajectory(trajectory, calculateAllCosts);
    });

    m_executor.run(m_taskflow).wait();

    m_taskflow.clear();

    removeInvalid();
}

void TrajectoryHandler::removeInvalid() {
    auto new_end = std::remove_if(
        m_trajectories.begin(),
        m_trajectories.end(),
        [](const TrajectorySample& traj) {
            return !traj.m_valid;
        }
    );
    m_trajectories.erase(new_end, m_trajectories.end());
}

size_t TrajectoryHandler::getFeasibleCount() const {
    return std::count_if(
        m_trajectories.cbegin(),
        m_trajectories.cend(),
        [](const TrajectorySample& traj) {
            return traj.m_valid && traj.m_feasible;
        }
    );
}

size_t TrajectoryHandler::getInfeasibleCount() const {
    return std::count_if(
        m_trajectories.cbegin(),
        m_trajectories.cend(),
        [](const TrajectorySample& traj) {
            return traj.m_valid && !traj.m_feasible;
        }
    );
}

void TrajectoryHandler::sort()
{
    std::sort(
        m_trajectories.begin(),
        m_trajectories.end(),
        [](const TrajectorySample& a, const TrajectorySample& b) {
            // Feasible trajectories come first
            if (a.m_feasible != b.m_feasible)
                return a.m_feasible > b.m_feasible;
            // If feasibility is the same, sort by cost
            return a.m_cost < b.m_cost;
        }
    );
}

void TrajectoryHandler::generateTrajectories(const SamplingMatrixXd& samplingMatrix, bool lowVelocityMode)
{
    m_trajectories.clear();
    m_trajectories.reserve(samplingMatrix.rows());

    for(Eigen::Index iii = 0; iii < samplingMatrix.rows(); iii++)
    {
        Eigen::Vector3d x0_lon {samplingMatrix.row(iii)[2], samplingMatrix.row(iii)[3], samplingMatrix.row(iii)[4]};
        Eigen::Vector2d x1_lon {samplingMatrix.row(iii)[5], samplingMatrix.row(iii)[6]};

        TrajectorySample::LongitudinalTrajectory longitudinalTrajectory (
            samplingMatrix.row(iii)[0],
            samplingMatrix.row(iii)[1],
            x0_lon,
            x1_lon,
            TrajectorySample::LongitudinalX0Order,
            TrajectorySample::LongitudinalXDOrder
        );

        double t1 = 0.0;
        if (lowVelocityMode) {
            t1 = longitudinalTrajectory(samplingMatrix.row(iii)[1]) - x0_lon[0];
            if (t1 <= 0.0) {
                t1 = samplingMatrix.row(iii)[1];
            }
        } else {
            t1 = samplingMatrix.row(iii)[1];
        }

        Eigen::Vector3d x0_lat {samplingMatrix.row(iii)[7], samplingMatrix.row(iii)[8], samplingMatrix.row(iii)[9]};
        Eigen::Vector3d x1_lat {samplingMatrix.row(iii)[10], samplingMatrix.row(iii)[11], samplingMatrix.row(iii)[12]};

        TrajectorySample::LateralTrajectory lateralTrajectory(
            samplingMatrix.row(iii)[0],
            t1,
            x0_lat,
            x1_lat
            );

        m_trajectories.emplace_back(
            m_dt,
            longitudinalTrajectory,
            lateralTrajectory,
            iii,
            samplingMatrix.row(iii)
            );
    }
}

#if __cpp_lib_interpolate >= 201902L
#define lerp std::lerp
#else
static inline double lerp(double a, double b, double t) {
    return a + t * (b - a);
}
#endif

template <> struct fmt::formatter<Eigen::ArrayXd> : fmt::ostream_formatter {};
template <> struct fmt::formatter<Eigen::Transpose<Eigen::ArrayXd>> : fmt::ostream_formatter {};
template <> struct fmt::formatter<Eigen::WithFormat<Eigen::Transpose<Eigen::ArrayXd>>> : fmt::ostream_formatter {};

void TrajectoryHandler::generateStoppingTrajectories(const PlannerState& state, SamplingConfiguration samplingConfig, double stop_point_s, double stop_point_v, bool lowVelocityMode) {
    if (state.x_cl.x0_lon(1) < 0.0) {
        throw std::invalid_argument { "invalid initial state: negative initial speed" };
    }
    if (stop_point_v < 0.0) {
        throw std::invalid_argument { "invalid negative desired speed" };
    }
    if (state.x_cl.x0_lon(1) < stop_point_s) {
        throw std::invalid_argument { "invalid stop point: behind current longitudinal position" };
    }

    auto linear_sample = [&] (auto min, auto max) -> Eigen::ArrayXd {
        return Eigen::ArrayXd::LinSpaced(samplingConfig.samplingLevel, min, max);
    };
    auto linear_sample_odd = [&] (auto min, auto max) -> Eigen::ArrayXd {
        auto level = samplingConfig.samplingLevel;
        if (level % 2 == 0) {
            level++;
        }
        return Eigen::ArrayXd::LinSpaced(level, min, max);
    };

    Eigen::ArrayXd svals = linear_sample(
        lerp(state.x_cl.x0_lon(0), stop_point_s, 0.7),
        stop_point_s
    );

    double base_distance_to_stop_point = stop_point_s - state.x_cl.x0_lon(0);
    // double base_nominal_time = base_distance_to_stop_point / state.x_cl.x0_lon(1);
    double base_nominal_time_final = base_distance_to_stop_point / stop_point_v;
    double base_delta_v = stop_point_v - state.x_cl.x0_lon(1);
    double desired_step_acc = base_delta_v / base_nominal_time_final;
    double a_max = 10.0;
    if (stop_point_v > state.x_cl.x0_lon(1)) {
        // Acceleration
        if (desired_step_acc > a_max) {
            stop_point_v = lerp(state.x_cl.x0_lon(1), stop_point_v, a_max / desired_step_acc);
            SPDLOG_WARN("desired_step_acc is {:.2}, reducing stop_point_v to {}", desired_step_acc, stop_point_v);
        }
    } else {
        // Deceleration
    }

    Eigen::ArrayXd sdvals;
    if (samplingConfig.strictVelocitySampling) {
        if (stop_point_v <= 1e-2) {
            sdvals = Eigen::Array<double, 1, 1>::Zero();
        } else {
            if (stop_point_v > state.x_cl.x0_lon(1)) {
                sdvals = linear_sample(
                    lerp(state.x_cl.x0_lon(1), stop_point_v, 0.7),
                    stop_point_v
                );
            } else {
                sdvals = linear_sample(
                    std::max(0.0, lerp(state.x_cl.x0_lon(1), stop_point_v, 1.3)),
                    stop_point_v
                );
            }
        }
    } else {
        sdvals = linear_sample(
            lerp(state.x_cl.x0_lon(1), stop_point_v, 0.7),
            std::max(0.0, lerp(state.x_cl.x0_lon(1), stop_point_v, 1.3))
        );
    }

    Eigen::ArrayXd dvals = linear_sample( 
        state.x_cl.x0_lat(0) - samplingConfig.d_delta,
        state.x_cl.x0_lat(0) + samplingConfig.d_delta
    );

    int iii = 0;

    Eigen::IOFormat clean(2, 0, ", ", "\n", "[", "]");
    SPDLOG_INFO(" s={}", svals.transpose().format(clean));
    SPDLOG_INFO("ss={}", sdvals.transpose().format(clean));

    for (auto s: svals) {
        for (auto sd: sdvals) {
            double distance_to_stop_point = s - state.x_cl.x0_lon(0);

            double nominal_time = distance_to_stop_point / state.x_cl.x0_lon(1);
            double nominal_time_final = distance_to_stop_point / sd;
            if (sd <= 1e-3) {
                nominal_time_final = distance_to_stop_point / (0.5 * state.x_cl.x0_lon(1));
            }

            double t_min = samplingConfig.t_min;
            double t_max = samplingConfig.t_max;

            if (sd > state.x_cl.x0_lon(1)) {
                // Acceleration
                // Final speed greater than initial speed -> should not take more than current time at current velocity
                if (samplingConfig.enforceTimeBounds) {
                    t_max = nominal_time;
                    t_min = nominal_time_final;
                } else {
                    t_max = std::min(t_max, nominal_time);
                    t_min = std::max(t_min, nominal_time_final);
                }
                if (t_min > t_max) {
                    t_min = 0.5 * t_max;
                }
            } else {
                // Deceleration
                if (samplingConfig.enforceTimeBounds) {
                    t_min = nominal_time;
                    t_max = nominal_time_final;
                } else {
                    t_min = std::max(t_min, nominal_time);
                    t_max = std::min(t_max, nominal_time_final);
                }
                if (t_min > t_max) {
                    t_max = 2.0 * t_min;
                }
            }

            auto max_samples = static_cast<decltype(samplingConfig.samplingLevel)>(std::ceil((t_max - t_min) / samplingConfig.dt));
            auto samples = std::min(max_samples, samplingConfig.samplingLevel);
            Eigen::ArrayXd ts = Eigen::ArrayXd::LinSpaced(samples, t_min, t_max);

            SPDLOG_INFO("sampling {:3.1} -> {:4.1} t={}", t_min, t_max, ts.transpose().format(clean));
            SPDLOG_DEBUG("nominal={:4.1} nominal_final={:4.1} t={}", nominal_time, nominal_time_final);

            for (auto t: ts) {
                if (!std::isfinite(t) || t <= 0.0) {
                    throw std::runtime_error { "internal error: sampled invalid time" };
                }

                auto scaled_d_delta = (t / t_max) * samplingConfig.d_delta;
                auto delta = samplingConfig.timeBasedLateralDeltaScaling ? scaled_d_delta : samplingConfig.d_delta;
                Eigen::ArrayXd dvals = linear_sample_odd(
                    state.x_cl.x0_lat(0) - delta,
                    state.x_cl.x0_lat(0) + delta
                );
                SPDLOG_DEBUG("d={}", dvals.transpose().format(clean));

                Eigen::Vector3d x1_lon {s, sd, 0.0};

                Eigen::Vector<double, 13> samplingParameters;
                samplingParameters(0) = 0.0;
                samplingParameters(1) = t;
                samplingParameters(2) = state.x_cl.x0_lon(0);
                samplingParameters(3) = state.x_cl.x0_lon(1);
                samplingParameters(4) = state.x_cl.x0_lon(2);
                samplingParameters(5) = x1_lon(0);
                samplingParameters(6) = x1_lon(1);
                samplingParameters(7) = state.x_cl.x0_lat(0);
                samplingParameters(8) = state.x_cl.x0_lat(1);
                samplingParameters(9) = state.x_cl.x0_lat(2);

                TrajectorySample::FixedLongitudinalTrajectory longitudinalTrajectory (
                    0.0,
                    t,
                    state.x_cl.x0_lon,
                    x1_lon
                    );

                double t1 = 0.0;
                if (lowVelocityMode) {
                    t1 = longitudinalTrajectory(t) - state.x_cl.x0_lon(0);
                    if (t1 <= 0.0) {
                        t1 = t;
                    }
                } else {
                    t1 = t;
                }

                for (auto d: dvals) {
                    Eigen::Vector3d x1_lat {d, 0.0, 0.0};

                    samplingParameters(10) = x1_lat(0);
                    samplingParameters(11) = x1_lat(1);
                    samplingParameters(12) = x1_lat(2);

                    TrajectorySample::LateralTrajectory lateralTrajectory(
                        0.0,
                        t1,
                        state.x_cl.x0_lat,
                        x1_lat
                    );

                    m_trajectories.emplace_back(
                        samplingConfig.dt,
                        longitudinalTrajectory,
                        lateralTrajectory,
                        iii++,
                        samplingParameters
                    );
                }
            }
        }
    }
}

void TrajectoryHandler::resetTrajectories()
{
    m_trajectories.clear();
}
