import sys
global _GLAvailable,_GLModule,_BackendStatus,_GLBackend
_GLAvailable = None
_GLModule = None
_BackendStatus = dict()
_GLBackend = None
_BackendString = None
_PIP_String = 'pip' if sys.version_info[0] < 3 else 'pip3'

#compatibility with MacOS X 11.x
try:
    import OpenGL.GL   # this fails in <=2020 versions of Python on OS X 11.x
    _GLAvailable = True
    _GLModule = OpenGL.GL
except ImportError:
    import sys
    if sys.platform == 'darwin':
        print("Couldn't import OpenGL, trying patch for Big Sur")
        from ctypes import util
        orig_util_find_library = util.find_library
        def new_util_find_library( name ):
            res = orig_util_find_library( name )
            if res: return res
            return '/System/Library/Frameworks/'+name+'.framework/'+name
        util.find_library = new_util_find_library
        try:
            import OpenGL.GL
            _GLAvailable = True
            _GLModule = OpenGL.GL
        except ImportError:
            print("Couldn't import OpenGL, try running pip install PyOpenGL")
            _GLAvailable = False
    else:
        print("Couldn't import OpenGL, try running pip install PyOpenGL")
        _GLAvailable = False

def init(backends=['PyQt','GLUT']):
    """Initializes the OpenGL system with one of the backends provided in the
    `backends` list.  Each backend is tried in order.

    If this has already been called, then the previously initialized backend is
    returned.

    Returns:
        QtBackend, GLUTBackend, or None
    """
    global _GLAvailable,_BackendStatus,_BackendString,_GLBackend
    if _GLAvailable == False:
        return None
    if _GLBackend is not None:
        return _GLBackend
    #print("glinit TRYING BACKENDS",backends)
    for backend in backends:
        if tried(backend): continue
        if backend == 'PyQt':
            try:
                from PyQt5 import QtCore
                from PyQt5 import QtGui
                from PyQt5 import QtWidgets
                from .backends.qtbackend import QtBackend
                _BackendStatus[backend] = 'available'
                _BackendStatus['PyQt5'] = 'available'
                _BackendString = 'PyQt5'
                _GLBackend = QtBackend()
                print("***  klampt.vis: using Qt5 as the visualization backend  ***")
                return _GLBackend
            except ImportError:
                try:
                    from PyQt4 import QtCore
                    from PyQt4 import QtGui
                    from .backends.qtbackend import QtBackend
                    _BackendStatus[backend] = 'available'
                    _BackendStatus['PyQt4'] = 'available'
                    _BackendString = 'PyQt4'
                    _GLBackend = QtBackend()
                    print("***  klampt.vis: using Qt4 as the visualization backend  ***")
                    return _GLBackend
                except ImportError:
                    print('PyQt4/PyQt5 are not available... try running "%s install PyQt5"'%(_PIP_String,))
                    _BackendStatus[backend] = 'unavailable'
                    _BackendStatus['PyQt4'] = 'unavailable'
                    _BackendStatus['PyQt5'] = 'unavailable'
        elif backend == 'PyQt5':
            try:
                from PyQt5 import QtCore
                from PyQt5 import QtGui
                from PyQt5 import QtWidgets
                from .backends.qtbackend import QtBackend
                _BackendStatus[backend] = 'available'
                _BackendStatus['PyQt'] = 'available'
                _BackendString = 'PyQt5'
                _GLBackend = QtBackend()
                print("***  klampt.vis: using Qt5 as the visualization backend  ***")
                return _GLBackend
            except ImportError:
                print('PyQt5 is not available... try running "%s install PyQt5"'%(_PIP_String,))
                _BackendStatus[backend] = 'unavailable'
        elif backend == 'PyQt4':
            try:
                from PyQt4 import QtCore
                from PyQt4 import QtGui
                from .backends.qtbackend import QtBackend
                _BackendStatus[backend] = 'available'
                _BackendStatus['PyQt4'] = 'available'
                _BackendString = 'PyQt4'
                _GLBackend = QtBackend()
                print("***  klampt.vis: using Qt4 as the visualization backend  ***")
                return _GLBackend
            except ImportError:
                print('PyQt4 is not available... try running "%s install PyQt4"'%(_PIP_String,))
                _BackendStatus[backend] = 'unavailable'
        elif backend == 'GLUT':
            try:
                from OpenGL import GLUT
                from .backends.glutbackend import GLUTBackend
                _BackendString = 'GLUT'
                _BackendStatus['GLUT'] = 'available'
                _GLBackend = GLUTBackend()
                print("*** klampt.vis: using GLUT as the visualization backend ***")
                print("***      Some functionality may not be available!       ***")
                return _GLBackend
            except ImportError as e:
                print(e)
                import traceback
                traceback.print_exc()
                _BackendStatus['GLUT'] = 'unavailable'

    print("Neither QT nor GLUT are available... visualization disabled")
    print(_BackendStatus)
    return None

def GL():
    global _GLModule
    return _GLModule

def active():
    global _BackendString
    return _BackendString

def available(backend):
    global _BackendStatus
    return _BackendStatus.get(backend,False) == 'available'

def tried(backend):
    global _BackendStatus
    return backend in _BackendStatus
