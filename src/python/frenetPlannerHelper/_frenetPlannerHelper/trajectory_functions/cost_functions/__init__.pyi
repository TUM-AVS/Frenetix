from __future__ import annotations
import frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.cost_functions
import typing
import frenetPlannerHelper._frenetPlannerHelper.trajectory_functions
import frenetPlannerHelper._frenetPlannerHelper
import numpy

__all__ = [
    "CalculateAccelerationCost",
    "CalculateCollisionProbabilityFast",
    "CalculateCollisionProbabilityMahalanobis",
    "CalculateDistanceToObstacleCost",
    "CalculateDistanceToReferencePathCost",
    "CalculateJerkCost",
    "CalculateLaneCenterOffsetCost",
    "CalculateLateralJerkCost",
    "CalculateLongitudinalJerkCost",
    "CalculateLongitudinalVelocityCost",
    "CalculateOrientationOffsetCost",
    "CalculateSteeringAngleCost",
    "CalculateSteeringRateCost",
    "CalculateVelocityOffsetCost",
    "CalculateYawCost"
]


class CalculateAccelerationCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateCollisionProbabilityFast(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, predictions: typing.Dict[int, frenetPlannerHelper._frenetPlannerHelper.PredictedObject], vehicleLength: float, vehicleWidth: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    def printPredictions(self) -> None: ...
    pass
class CalculateCollisionProbabilityMahalanobis(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, predictions: typing.Dict[int, frenetPlannerHelper._frenetPlannerHelper.PredictedObject]) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateDistanceToObstacleCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, obstacles: numpy.ndarray) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateDistanceToReferencePathCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateJerkCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateLaneCenterOffsetCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateLateralJerkCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateLongitudinalJerkCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateLongitudinalVelocityCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateOrientationOffsetCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateSteeringAngleCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateSteeringRateCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateVelocityOffsetCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, desiredSpeed: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CalculateYawCost(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.CostStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass