from __future__ import annotations
import frenetix._frenetix.trajectory_functions.cost_functions
import typing
import frenetix._frenetix.trajectory_functions
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


class CalculateAccelerationCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateCollisionProbabilityFast(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, predictions: typing.Dict[int, PredictedObject], vehicleLength: float, vehicleWidth: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    def printPredictions(self) -> None: ...
    pass
class CalculateCollisionProbabilityMahalanobis(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, predictions: typing.Dict[int, PredictedObject]) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateDistanceToObstacleCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, obstacles: numpy.ndarray) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateDistanceToReferencePathCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateJerkCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateLaneCenterOffsetCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateLateralJerkCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateLongitudinalJerkCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateLongitudinalVelocityCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateOrientationOffsetCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateSteeringAngleCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateSteeringRateCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateVelocityOffsetCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float, desiredSpeed: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CalculateYawCost(frenetix._frenetix.trajectory_functions.CostStrategy, frenetix._frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, function_name: str, cost_weight: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
