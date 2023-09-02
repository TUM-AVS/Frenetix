from __future__ import annotations
import frenetix.trajectory_functions.feasability_functions
import typing
import frenetix.trajectory_functions
from frenetix import TrajectorySample

__all__ = [
    "CheckAccelerationConstraint",
    "CheckCurvatureConstraint",
    "CheckCurvatureRateConstraint",
    "CheckVelocityConstraint",
    "CheckYawRateConstraint"
]


class CheckAccelerationConstraint(frenetix.trajectory_functions.FeasabilityStrategy, frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, switchingVelocity: float, maxAcceleration: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckCurvatureConstraint(frenetix.trajectory_functions.FeasabilityStrategy, frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckCurvatureRateConstraint(frenetix.trajectory_functions.FeasabilityStrategy, frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, wheelbase: float, velocityDeltaMax: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckVelocityConstraint(frenetix.trajectory_functions.FeasabilityStrategy, frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckYawRateConstraint(frenetix.trajectory_functions.FeasabilityStrategy, frenetix.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
