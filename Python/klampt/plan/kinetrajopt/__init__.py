try:
    import cvxpy
    import autograd
    from .utils import CostInterface, ConstrInterface
    from .kinetrajopt import TrajOptConfig, KineTrajOpt
except:
    print("Cannot import cvxpy and autograd, kinedynopt is not imported")