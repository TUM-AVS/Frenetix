from __future__ import annotations
import frenetix._frenetix
from . import cost_functions
from . import feasability_functions
__all__ = ['ComputeInitialState', 'CostStrategy', 'FeasabilityStrategy', 'FillCoordinates', 'TrajectoryStrategy', 'cost_functions', 'feasability_functions']
class ComputeInitialState(TrajectoryStrategy):
    def __init__(self, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper, wheelBase: float, steeringAngle: float, lowVelocityMode: bool) -> None:
        ...
    def evaluate_trajectory(self, trajectory: frenetix._frenetix.TrajectorySample) -> None:
        ...
class CostStrategy(TrajectoryStrategy):
    pass
class FeasabilityStrategy(TrajectoryStrategy):
    pass
class FillCoordinates(TrajectoryStrategy):
    def __init__(self, lowVelocityMode: bool, initialOrientation: float, coordinateSystem: frenetix._frenetix.CoordinateSystemWrapper, horizon: float) -> None:
        ...
    def evaluate_trajectory(self, trajectory: frenetix._frenetix.TrajectorySample) -> None:
        ...
class TrajectoryStrategy:
    def evaluate_trajectory(self, trajectory: frenetix._frenetix.TrajectorySample) -> None:
        ...
    @property
    def name(self) -> str:
        ...
