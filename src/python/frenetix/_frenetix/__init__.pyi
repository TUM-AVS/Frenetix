from __future__ import annotations
import numpy
import typing
from . import trajectory_functions
__all__ = ['CartesianSample', 'CoordinateSystemWrapper', 'CurviLinearSample', 'InvalidCovarianceMatrixError', 'PoseWithCovariance', 'PredictedObject', 'QuarticTrajectory', 'QuinticTrajectory', 'TrajectoryHandler', 'TrajectorySample', 'trajectory_functions']
class CartesianSample:
    a: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    is_initialized: bool
    kappa: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    kappa_dot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    theta: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    def __getstate__(self) -> dict:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, x: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], y: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], theta_gl: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], v: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], a: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], kappa_gl: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], kappa_dot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
    def __str__(self) -> str:
        ...
class CoordinateSystemWrapper:
    ref_curv: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    ref_curv_d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    ref_curv_dd: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    ref_line: list[numpy.ndarray[tuple[typing.Literal[2], typing.Literal[1]], numpy.dtype[numpy.float64]]]
    ref_pos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    ref_theta: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    system: ...
    def __getstate__(self) -> dict:
        ...
    def __init__(self, ref_path: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
class CurviLinearSample:
    d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    d_ddot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    d_dot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    is_initialized: bool
    s: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    s_ddot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    s_dot: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    theta: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    def __getstate__(self) -> dict:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, s: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], theta_gl: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], dd: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], ddd: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], ss: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], sss: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
    def __str__(self) -> str:
        ...
class InvalidCovarianceMatrixError(ValueError):
    pass
class PoseWithCovariance:
    def __init__(self, arg0: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], arg1: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]], arg2: numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def covariance(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[6]], numpy.dtype[numpy.float64]]:
        ...
    @property
    def orientation(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @property
    def position(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
class PredictedObject:
    def __init__(self, arg0: int, arg1: list[PoseWithCovariance], arg2: float, arg3: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def length(self) -> float:
        ...
    @property
    def object_id(self) -> int:
        ...
    @property
    def predictedPath(self) -> list[PoseWithCovariance]:
        ...
    @property
    def width(self) -> float:
        ...
class QuarticTrajectory:
    def __call__(self, arg0: numpy.ndarray[typing.Any, numpy.dtype[numpy.float64]], arg1: numpy.ndarray[typing.Any, numpy.dtype[numpy.float64]]) -> typing.Any:
        ...
    def __getstate__(self) -> dict:
        ...
    def __init__(self, tau_0: float, delta_tau: float, x_0: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], x_d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], x_0_order: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]] = ..., x_d_order: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]] = ...) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
    def squared_jerk_integral(self, arg0: float) -> float:
        ...
    @property
    def coeffs(self) -> numpy.ndarray[tuple[typing.Literal[5], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @property
    def delta_tau(self) -> float:
        ...
class QuinticTrajectory:
    def __call__(self, arg0: numpy.ndarray[typing.Any, numpy.dtype[numpy.float64]], arg1: numpy.ndarray[typing.Any, numpy.dtype[numpy.float64]]) -> typing.Any:
        ...
    def __getstate__(self) -> dict:
        ...
    def __init__(self, tau_0: float, delta_tau: float, x_0: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], x_d: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]], x_0_order: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]] = ..., x_d_order: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]] = ...) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
    def squared_jerk_integral(self, arg0: float) -> float:
        ...
    @property
    def coeffs(self) -> numpy.ndarray[tuple[typing.Literal[6], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    @property
    def delta_tau(self) -> float:
        ...
class TrajectoryHandler:
    def __init__(self, dt: float) -> None:
        ...
    def add_cost_function(self, arg0: trajectory_functions.CostStrategy) -> None:
        ...
    def add_feasability_function(self, arg0: trajectory_functions.FeasabilityStrategy) -> None:
        ...
    def add_function(self, arg0: trajectory_functions.TrajectoryStrategy) -> None:
        ...
    def clear_cost_functions(self) -> None:
        """
        Clears all cost functions.
        """
    def evaluate_all_current_functions(self, calculateAllCosts: bool = False) -> None:
        ...
    def evaluate_all_current_functions_concurrent(self, calculateAllCosts: bool = False) -> None:
        ...
    def generate_trajectories(self, samplingMatrix: numpy.ndarray[tuple[M, N], numpy.dtype[numpy.float64]], lowVelocityMode: bool) -> None:
        ...
    def get_cost_functions(self) -> typing.Iterator:
        ...
    def get_feasability_functions(self) -> typing.Iterator:
        ...
    def get_other_functions(self) -> typing.Iterator:
        ...
    def get_sorted_trajectories(self) -> typing.Iterator:
        ...
    def reset_Trajectories(self) -> None:
        """
        Resets the trajectories container.
        """
    def set_all_cost_weights_to_zero(self) -> None:
        """
        Sets all cost function weights to zero.
        """
    def sort(self) -> None:
        ...
class TrajectorySample:
    _coll_detected: bool | None
    _cost: float
    _ego_risk: float | None
    _harm_occ_module: float | None
    _obst_risk: float | None
    boundary_harm: float | None
    cartesian: CartesianSample
    cost: float
    costMap: dict[str, tuple[float, float]]
    curvilinear: CurviLinearSample
    dt: float
    feasabilityMap: dict[str, float]
    feasible: bool
    harm_occ_module: float | None
    sampling_parameters: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]
    trajectory_lat: QuinticTrajectory
    trajectory_long: QuarticTrajectory
    uniqueId: int | None
    valid: bool
    def __getstate__(self) -> dict:
        ...
    @typing.overload
    def __init__(self, dt: float, trajectoryLongitudinal: QuarticTrajectory, trajectoryLateral: QuinticTrajectory, uniqueId: int) -> None:
        ...
    @typing.overload
    def __init__(self, x0: float, y0: float, orientation0: float, acceleration0: float, velocity0: float) -> None:
        ...
    def __setstate__(self, arg0: dict) -> None:
        ...
    def add_cost_value_to_list(self, cost_function_name: str, cost: float, weighted_costs: float) -> None:
        """
        Add a cost value to the list of cost values. This includes the weighted and unweighted cost.
        """
