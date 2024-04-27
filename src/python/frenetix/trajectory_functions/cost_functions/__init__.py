from ... import _frenetix
from ..._frenetix import TrajectorySample, PredictedObject

__all__ = ["TrajectorySample", "PredictedObject"]

for function_name in dir(_frenetix.trajectory_functions.cost_functions):
    globals().update({function_name: getattr(_frenetix.trajectory_functions.cost_functions, function_name)})
    __all__.append(function_name)

