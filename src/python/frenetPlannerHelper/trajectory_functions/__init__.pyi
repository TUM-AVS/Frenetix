from __future__ import annotations
import frenetPlannerHelper.trajectory_functions
import typing
import frenetPlannerHelper

__all__ = [
    "ComputeInitialState",
    "CostStrategy",
    "FeasabilityStrategy",
    "FillCoordinates",
    "TrajectoryStrategy",
    "cost_functions",
    "feasability_functions"
]


class TrajectoryStrategy():
    pass
class CostStrategy(TrajectoryStrategy):
    pass
class FeasabilityStrategy(TrajectoryStrategy):
    pass
class FillCoordinates(TrajectoryStrategy):
    def __init__(self, lowVelocityMode: bool, initialOrientation: float, coordinateSystem: frenetPlannerHelper.CoordinateSystemWrapper) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class ComputeInitialState(TrajectoryStrategy):
    def __init__(self, coordinateSystem: frenetPlannerHelper.CoordinateSystemWrapper, wheelBase: float, steeringAngle: float, lowVelocityMode: bool) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
