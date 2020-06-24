__all__ = ['cspace','robotcspace','robotplanning', 'kinetrajopt']
from . import motionplanning
from . import cspace
from . import robotcspace
from . import robotplanning
from . import kinetrajopt
import atexit
atexit.register(motionplanning.destroy)
