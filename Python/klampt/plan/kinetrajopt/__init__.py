try:
    import cvxpy
    from .utils import CostInterface, ConstrInterface
    from .kinetrajopt import TrajOptSettings, KineTrajOpt
except ImportError:
    print('kinetrajopt is not supported because cvxpy cannot be imported. Try "pip install cvxpy"')
    raise