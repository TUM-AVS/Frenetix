from ... import _frenetix
#import trajectory_functions

__all__ = []

for function_name in dir(_frenetix.trajectory_functions.cost_functions):
    globals().update({function_name: getattr(_frenetix.trajectory_functions.cost_functions, function_name)})
    __all__.append(function_name)

