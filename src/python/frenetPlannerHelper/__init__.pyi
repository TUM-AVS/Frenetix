from __future__ import annotations
import frenetPlannerHelper
import typing
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "CartesianSample",
    "CoordinateSystemWrapper",
    "CurviLinearSample",
    "PoseWithCovariance",
    "PredictedObject",
    "QuarticTrajectory",
    "QuinticTrajectory",
    "TrajectoryHandler",
    "TrajectorySample",
    "trajectory_functions"
]


class CartesianSample():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, x: numpy.ndarray[numpy.float64, _Shape[m, 1]], y: numpy.ndarray[numpy.float64, _Shape[m, 1]], theta_gl: numpy.ndarray[numpy.float64, _Shape[m, 1]], v: numpy.ndarray[numpy.float64, _Shape[m, 1]], a: numpy.ndarray[numpy.float64, _Shape[m, 1]], kappa_gl: numpy.ndarray[numpy.float64, _Shape[m, 1]], kappa_dot: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: ...
    def __str__(self) -> str: ...
    @property
    def a(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @a.setter
    def a(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def is_initialized(self) -> bool:
        """
        :type: bool
        """
    @is_initialized.setter
    def is_initialized(self, arg0: bool) -> None:
        pass
    @property
    def kappa(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @kappa.setter
    def kappa(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def kappa_dot(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @kappa_dot.setter
    def kappa_dot(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def theta(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @theta.setter
    def theta(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def v(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @v.setter
    def v(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def x(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @x.setter
    def x(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def y(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @y.setter
    def y(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    pass
class CoordinateSystemWrapper():
    def __init__(self, ref_path: numpy.ndarray[numpy.float64, _Shape[m, n]]) -> None: ...
    @property
    def ref_curv(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @ref_curv.setter
    def ref_curv(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def ref_curv_d(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @ref_curv_d.setter
    def ref_curv_d(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def ref_curv_dd(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @ref_curv_dd.setter
    def ref_curv_dd(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def ref_line(self) -> typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]:
        """
        :type: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]
        """
    @ref_line.setter
    def ref_line(self, arg0: typing.List[numpy.ndarray[numpy.float64, _Shape[2, 1]]]) -> None:
        pass
    @property
    def ref_pos(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @ref_pos.setter
    def ref_pos(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def ref_theta(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @ref_theta.setter
    def ref_theta(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def system(self) -> geometry::CurvilinearCoordinateSystem:
        """
        :type: geometry::CurvilinearCoordinateSystem
        """
    @system.setter
    def system(self, arg1: geometry::CurvilinearCoordinateSystem) -> None:
        pass
    pass
class CurviLinearSample():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, s: numpy.ndarray[numpy.float64, _Shape[m, 1]], d: numpy.ndarray[numpy.float64, _Shape[m, 1]], theta_gl: numpy.ndarray[numpy.float64, _Shape[m, 1]], dd: numpy.ndarray[numpy.float64, _Shape[m, 1]], ddd: numpy.ndarray[numpy.float64, _Shape[m, 1]], ss: numpy.ndarray[numpy.float64, _Shape[m, 1]], sss: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: ...
    def __str__(self) -> str: ...
    @property
    def d(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @d.setter
    def d(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def d_ddot(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @d_ddot.setter
    def d_ddot(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def d_dot(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @d_dot.setter
    def d_dot(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def is_initialized(self) -> bool:
        """
        :type: bool
        """
    @is_initialized.setter
    def is_initialized(self, arg0: bool) -> None:
        pass
    @property
    def s(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @s.setter
    def s(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def s_ddot(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @s_ddot.setter
    def s_ddot(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def s_dot(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @s_dot.setter
    def s_dot(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def theta(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @theta.setter
    def theta(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    pass
class PoseWithCovariance():
    def __init__(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg1: numpy.ndarray[numpy.float64, _Shape[4, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[6, 6]]) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def covariance(self) -> numpy.ndarray[numpy.float64, _Shape[6, 6]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[6, 6]]
        """
    @property
    def orientation(self) -> numpy.ndarray[numpy.float64, _Shape[4, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[4, 1]]
        """
    @property
    def position(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    pass
class PredictedObject():
    def __init__(self, arg0: int, arg1: typing.List[PoseWithCovariance]) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def object_id(self) -> int:
        """
        :type: int
        """
    @property
    def predictedPath(self) -> typing.List[PoseWithCovariance]:
        """
        :type: typing.List[PoseWithCovariance]
        """
    pass
class QuarticTrajectory():
    def __call__(self, arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.float64]) -> object: ...
    def __init__(self, tau_0: float, delta_tau: float, x_0: numpy.ndarray[numpy.float64, _Shape[m, 1]], x_d: numpy.ndarray[numpy.float64, _Shape[m, 1]], x_0_order: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64), x_d_order: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64)) -> None: ...
    def squared_jerk_integral(self, arg0: float) -> float: ...
    @property
    def coeffs(self) -> numpy.ndarray[numpy.float64, _Shape[5, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[5, 1]]
        """
    @property
    def delta_tau(self) -> float:
        """
        :type: float
        """
    pass
class QuinticTrajectory():
    def __call__(self, arg0: numpy.ndarray[numpy.float64], arg1: numpy.ndarray[numpy.float64]) -> object: ...
    def __init__(self, tau_0: float, delta_tau: float, x_0: numpy.ndarray[numpy.float64, _Shape[m, 1]], x_d: numpy.ndarray[numpy.float64, _Shape[m, 1]], x_0_order: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64), x_d_order: numpy.ndarray[numpy.float64, _Shape[m, 1]] = array([], dtype=float64)) -> None: ...
    def squared_jerk_integral(self, arg0: float) -> float: ...
    @property
    def coeffs(self) -> numpy.ndarray[numpy.float64, _Shape[6, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[6, 1]]
        """
    @property
    def delta_tau(self) -> float:
        """
        :type: float
        """
    pass
class TrajectoryHandler():
    def __init__(self, dt: float) -> None: ...
    def add_cost_function(self, arg0: CostStrategy) -> None: ...
    def add_feasability_function(self, arg0: FeasabilityStrategy) -> None: ...
    def add_function(self, arg0: TrajectoryStrategy) -> None: ...
    def evaluate_all_current_functions(self, calculateAllCosts: bool = False) -> None: ...
    def evaluate_all_current_functions_concurrent(self, calculateAllCosts: bool = False) -> None: ...
    def generate_trajectories(self, samplingMatrix: numpy.ndarray[numpy.float64, _Shape[m, n]], lowVelocityMode: bool) -> None: ...
    def get_cost_functions(self) -> typing.Iterator: ...
    def get_feasability_functions(self) -> typing.Iterator: ...
    def get_other_functions(self) -> typing.Iterator: ...
    def get_sorted_trajectories(self) -> typing.Iterator: ...
    def reset_Trajectories(self) -> None: 
        """
        Resets the trajectories container.
        """
    def sort(self) -> None: ...
    pass
class TrajectorySample():
    @typing.overload
    def __init__(self, dt: float, trajectoryLongitudinal: QuarticTrajectory, trajectoryLateral: QuinticTrajectory, uniqueId: int) -> None: ...
    @typing.overload
    def __init__(self, x0: float, y0: float, orientation0: float, acceleration0: float, velocity0: float) -> None: ...
    def add_cost_value_to_list(self, cost_function_name: str, cost: float, weighted_costs: float) -> None: 
        """
        Add a cost value to the list of cost values. This includes the weighted and unweighted cost.
        """
    @property
    def _coll_detected(self) -> typing.Optional[bool]:
        """
        :type: typing.Optional[bool]
        """
    @_coll_detected.setter
    def _coll_detected(self, arg0: typing.Optional[bool]) -> None:
        pass
    @property
    def _cost(self) -> float:
        """
        :type: float
        """
    @_cost.setter
    def _cost(self, arg0: float) -> None:
        pass
    @property
    def _ego_risk(self) -> typing.Optional[float]:
        """
        :type: typing.Optional[float]
        """
    @_ego_risk.setter
    def _ego_risk(self, arg0: typing.Optional[float]) -> None:
        pass
    @property
    def _obst_risk(self) -> typing.Optional[float]:
        """
        :type: typing.Optional[float]
        """
    @_obst_risk.setter
    def _obst_risk(self, arg0: typing.Optional[float]) -> None:
        pass
    @property
    def boundary_harm(self) -> typing.Optional[float]:
        """
        :type: typing.Optional[float]
        """
    @boundary_harm.setter
    def boundary_harm(self, arg0: typing.Optional[float]) -> None:
        pass
    @property
    def cartesian(self) -> CartesianSample:
        """
        :type: CartesianSample
        """
    @cartesian.setter
    def cartesian(self, arg0: CartesianSample) -> None:
        pass
    @property
    def cost(self) -> float:
        """
        :type: float
        """
    @cost.setter
    def cost(self, arg0: float) -> None:
        pass
    @property
    def costMap(self) -> typing.Dict[str, typing.Tuple[float, float]]:
        """
        :type: typing.Dict[str, typing.Tuple[float, float]]
        """
    @costMap.setter
    def costMap(self, arg0: typing.Dict[str, typing.Tuple[float, float]]) -> None:
        pass
    @property
    def curvilinear(self) -> CurviLinearSample:
        """
        :type: CurviLinearSample
        """
    @curvilinear.setter
    def curvilinear(self, arg0: CurviLinearSample) -> None:
        pass
    @property
    def dt(self) -> float:
        """
        :type: float
        """
    @dt.setter
    def dt(self, arg0: float) -> None:
        pass
    @property
    def feasabilityMap(self) -> typing.Dict[str, float]:
        """
        :type: typing.Dict[str, float]
        """
    @feasabilityMap.setter
    def feasabilityMap(self, arg0: typing.Dict[str, float]) -> None:
        pass
    @property
    def feasible(self) -> bool:
        """
        :type: bool
        """
    @feasible.setter
    def feasible(self, arg0: bool) -> None:
        pass
    @property
    def sampling_parameters(self) -> numpy.ndarray[numpy.float64, _Shape[m, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[m, 1]]
        """
    @sampling_parameters.setter
    def sampling_parameters(self, arg1: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None:
        pass
    @property
    def trajectory_lat(self) -> QuinticTrajectory:
        """
        :type: QuinticTrajectory
        """
    @trajectory_lat.setter
    def trajectory_lat(self, arg0: QuinticTrajectory) -> None:
        pass
    @property
    def trajectory_long(self) -> QuarticTrajectory:
        """
        :type: QuarticTrajectory
        """
    @trajectory_long.setter
    def trajectory_long(self, arg0: QuarticTrajectory) -> None:
        pass
    @property
    def uniqueId(self) -> typing.Optional[int]:
        """
        :type: typing.Optional[int]
        """
    @uniqueId.setter
    def uniqueId(self, arg0: typing.Optional[int]) -> None:
        pass
    pass
