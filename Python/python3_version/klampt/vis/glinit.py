import sys
global _PyQtAvailable,_PyQt5Available,_PyQt4Available,_GLUTAvailable,_GLBackend
_PyQtAvailable = False
_PyQt5Available = False
_PyQt4Available = False
_GLUTAvailable = False
_GLBackend = None

if not _PyQtAvailable and not _GLUTAvailable:
    try:
        from PyQt5.QtCore import *
        from PyQt5.QtGui import *
        from PyQt5.QtWidgets import *
        from .qtbackend import QtBackend
        _PyQtAvailable = True
        _PyQt5Available = True
        _GLBackend = QtBackend()
        print("***  klampt.vis: using Qt5 as the visualization backend  ***")
    except ImportError:
        try:
            from PyQt4.QtCore import *
            from PyQt4.QtGui import *
            from .qtbackend import QtBackend
            _PyQtAvailable = True
            _PyQt4Available = True
            _GLBackend = QtBackend()
            print("***  klampt.vis: using Qt4 as the visualization backend  ***")
        except ImportError:
          if sys.version_info[0] < 3:
              print('PyQt4/PyQt5 are not available... try running "pip install PyQt5"')
          else:
              print('PyQt4/PyQt5 are not available... try running "pip3 install PyQt5"')
          try:
              from OpenGL.GLUT import *
              from .glutbackend import GLUTBackend
              _GLUTAvailable = True
              _GLBackend = GLUTBackend()
              print("*** klampt.vis: using GLUT as the visualization backend ***")
              print("***      Some functionality may not be available!       ***")
          except ImportError:
              print("Neither QT nor GLUT are available... visualization disabled")
