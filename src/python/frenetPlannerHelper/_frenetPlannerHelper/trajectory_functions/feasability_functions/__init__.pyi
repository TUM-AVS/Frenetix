from __future__ import annotations
import frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.feasability_functions
import typing
import frenetPlannerHelper._frenetPlannerHelper.trajectory_functions
import frenetPlannerHelper._frenetPlannerHelper

__all__ = [
    "CheckAccelerationConstraint",
    "CheckCurvatureConstraint",
    "CheckCurvatureRateConstraint",
    "CheckVelocityConstraint",
    "CheckYawRateConstraint"
]


class CheckAccelerationConstraint(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, switchingVelocity: float, maxAcceleration: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CheckCurvatureConstraint(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CheckCurvatureRateConstraint(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, wheelbase: float, velocityDeltaMax: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CheckVelocityConstraint(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
class CheckYawRateConstraint(frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.FeasabilityStrategy, frenetPlannerHelper._frenetPlannerHelper.trajectory_functions.TrajectoryStrategy):
    def __init__(self, deltaMax: float, wheelbase: float) -> None: ...
    def evaluate_trajectory(self, trajectory: frenetPlannerHelper._frenetPlannerHelper.TrajectorySample) -> None: ...
    pass
