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
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    pass
class CostStrategy(TrajectoryStrategy):
    pass
class FeasabilityStrategy(TrajectoryStrategy):
    pass
class FillCoordinates(TrajectoryStrategy):
    def __init__(self, lowVelocityMode: bool, initialOrientation: float, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper, horizon: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class ComputeInitialState(TrajectoryStrategy):
    def __init__(self, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper, wheelBase: float, steeringAngle: float, lowVelocityMode: bool) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
