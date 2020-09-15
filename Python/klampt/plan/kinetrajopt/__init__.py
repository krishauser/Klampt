try:
    import cvxpy
    try:
        import autograd
        from .utils import CostInterface, ConstrInterface
        from .kinetrajopt import TrajOptConfig, KineTrajOpt
    except ImportError:
        print('kinetrajopt is not supported because autograd cannot be imported. Try "pip install autograd"')
        raise
except ImportError:
    print('kinetrajopt is not supported because cvxpy cannot be imported. Try "pip install cvxpy"')
    raise