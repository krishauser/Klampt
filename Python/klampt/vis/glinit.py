global _PyQtAvailable,_GLUTAvailable,_GLBackend
_PyQtAvailable = False
_GLUTAvailable = False
_GLBackend = None

if not _PyQtAvailable and not _GLUTAvailable:
    try:
        from PyQt5.QtCore import *
        from PyQt5.QtGui import *
        from PyQt5.QtWidgets import *
        from qtbackend import QtBackend
        _PyQtAvailable = True
        _GLBackend = QtBackend()
        print "***  klampt.vis: using Qt as the visualization backend  ***"
    except ImportError:
        print "QT is not available... try sudo apt-get install python-qt4 python-qt4-gl"
        try:
            from OpenGL.GLUT import *
            from glutbackend import GLUTBackend
            _GLUTAvailable = True
            _GLBackend = GLUTBackend()
            print "*** klampt.vis: using GLUT as the visualization backend ***"
            print "***      Some functionality may not be available!       ***"
        except ImportError:
            print "Neither QT nor GLUT are available... visualization disabled"
