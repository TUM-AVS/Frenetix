from __future__ import annotations
import frenetix._frenetix.trajectory_functions
import typing
import frenetix._frenetix

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
    def __init__(self, lowVelocityMode: bool, initialOrientation: float, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetix._frenetix.TrajectorySample) -> None: ...
    pass
class ComputeInitialState(TrajectoryStrategy):
    def __init__(self, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper, wheelBase: float, steeringAngle: float, lowVelocityMode: bool) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetix._frenetix.TrajectorySample) -> None: ...
    pass
