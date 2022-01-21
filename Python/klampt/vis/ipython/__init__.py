"""Defines a Jupyter Notebook interface to Klampt."""

from ._version import version_info, __version__

try:
    from klampt_jupyter import KlamptWidget,EditPoint,EditTransform,EditConfig,Playback
except (ImportError,ModuleNotFoundError):
    print("Import klampt_jupyter failed. Falling back to old widgets")
    print('(Try "pip install klampt_jupyter".)')
    from .widgets import *

