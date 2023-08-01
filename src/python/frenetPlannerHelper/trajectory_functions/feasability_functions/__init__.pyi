from __future__ import annotations
import frenetPlannerHelper.trajectory_functions.feasability_functions
import typing
import frenetPlannerHelper.trajectory_functions
from frenetPlannerHelper import TrajectorySample

__all__ = [
    "CheckAccelerationConstraint",
    "CheckCurvatureConstraint",
    "CheckCurvatureRateConstraint",
    "CheckVelocityConstraint",
    "CheckYawRateConstraint"
]


class CheckAccelerationConstraint(frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, switchingVelocity: float, maxAcceleration: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckCurvatureConstraint(frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckCurvatureRateConstraint(frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, wheelbase: float, velocityDeltaMax: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckVelocityConstraint(frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
class CheckYawRateConstraint(frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: TrajectorySample) -> None: ...
    pass
