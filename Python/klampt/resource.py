"""Use the get(), set(), and edit() functions to retrieve / store / edit
resources dynamically.

Use getDirectory() and setDirectory() to set the directory under which
resources are stored.  Alternatively, the directory=[DIRNAME] keyword
argument can be provided to get, set, and edit.

Example usage can be seen in demos/resourcetest.py.
"""

import loader
import trajectory
import multipath
import robotsim
import vectorops,se3,so3
import os
import time
from robotsim import WidgetSet,RobotPoser,ObjectPoser,TransformPoser,PointPoser,WorldModel,RobotModelLink,RigidObjectModel,IKObjective
from contact import ContactPoint
from hold import Hold
import visualization
import glcommon
#from OpenGL.GL import *
#import gldraw

#_PyQtAvailable = False
#try:
#    from PyQt4.QtCore import *
#    from PyQt4.QtGui import *
#    import qtprogram
#    _PyQtAvailable = True
#except ImportError:
#    print "QT is not available... try sudo apt-get install python-qt4 python-qt4-gl"
#    pass


_directory = 'resources'

def getDirectory():
    """Returns the current resource directory."""
    return _directory

def setDirectory(value):
    """Sets the current resource directory."""
    global _directory
    _directory = value

def _ensure_dir(f):
    d = os.path.dirname(f)
    if len(d)==0: return
    if not os.path.exists(d):
        os.makedirs(d)

extensionToType = {'.config':'Config',
                   '.configs':'Configs',
                   '.tri':'Geometry3D',
                   '.off':'Geometry3D',
                   '.stl':'Geometry3D',
                   '.poly':'Geometry3D',
                   '.geom':'GeometricPrimitive',
                   '.pcd':'Geometry3D',
                   '.vector3':'Vector3',
                   '.ikgoal':'IKGoal',
                   '.xform':'RigidTransform',
                   '.hold':'Hold',
                   '.stance':'Stance',
                   '.grasp':'Grasp'}

typeToExtension = dict((v,k) for (k,v) in extensionToType.items())


def filenameToType(name):
    fileName, fileExtension = os.path.splitext(name)
    if fileExtension in extensionToType:
        return extensionToType[fileExtension]
    elif fileExtension == '.xml':
        return 'xml'  #dynamic loading
    elif fileExtension == '.json':
        return 'json'  #dynamic loading
    else:
        raise RuntimeError("Cannot determine type of resource from name "+name)

def objectToTypes(object,world=None):
    """Returns all possible types that could be associated with the given
    Python Klamp't object."""
    if hasattr(object,'type'):
        return object.type
    if isinstance(object,ContactPoint):
        return 'ContactPoint'
    elif isinstance(object,Hold):
        return 'Hold'
    elif isinstance(object,IKObjective):
        return 'IKGoal'
    elif hasattr(object,'__iter__'):
        if hasattr(object[0],'__iter__'):
            #list of lists or tuples
            if len(object)==2:
                if len(object[0])==9 and len(object[1])==3:
                    #se3
                    return 'RigidTransform'
            return 'Configs'
        else:
            if len(object)==2:
                #2d point
                return ['Vector2','Config']
            elif len(object)==3:
                #3d point
                return ['Vector3','Config']
            elif len(object)==9:
                #so3 or 9d point?
                return ['Matrix3','Config']
            else:
                return 'Config'
    else:
        raise ValueError("Unknown object passed to objectToTypes")

def get(name,type='auto',directory=None,default=None,doedit='auto',description=None,editor='visual',world=None,frame=None):
    """Retrieve a resource of the given name from the current resources
    directory.  Resources may be of type Config, Configs, IKGoal, Hold,
    Stance, MultiPath, LinearPath, etc. (see Klampt/Modeling/Resources.h for
    a complete list; not all are supported in the Python API).  They can
    also be edited using RobotPose.

    If the resource does not already exist, an edit prompt will be
    shown, and the result from the prompt will be saved to disk under the
    given name.

    Arguments:
        - name: the resource name.  If type='auto', this is assumed to have
          a suffix of a file of the desired type.  The name can also be
          nested of the form 'group/subgroup/name.type'.
        - type: the resource type, or 'auto' to determine it automatically.
        - directory: the search directory.  If None, uses the current
          resource directory.
        - default: the default value if the resource does not exist on disk.
          If None, some arbitrary default value will be inferred.
        - doedit: if 'auto', if the resource does not exist on disk, an
          edit prompt will be displayed.  If False, an RuntimeError will be
          raised if the resource does not exist on disk.  If True, the
          user will be given an edit prompt to optionally edit the resource
          before this call returns.
        - description: an optional text description of the resource, for use
          in the edit prompt.
        - editor: either 'visual' or 'console', determining whether to use the
          visual OpenGL or console editor.
        - world: for a visual editor, this world will be shown along with
          the item to edit.  If this is a string it points to a file
          that will be loaded for the world (e.g., a world XML file, or a
          robot file).
        - frame: for rigid transforms / rotations / points, the reference
          frame for the visual editor.  This is an element of se3, or an
          ObjectModel, or a RobotModelLink, or a string indicating a named
          rigid element of the world.
          """
    if name==None:
        if doedit==False:
            raise RuntimeError("Can't get() an anonymous resource without launching editor")
        success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,frame=frame)
        if not success:
            print "Cancel pressed, returning None"
            return None
        else:
            print "Ok pressed, returning anonymous resource"
            return newvalue
    if directory==None:
        directory = getDirectory()
    if type == 'auto':
        type = filenameToType(name)
    value = None
    try:
        fn = os.path.join(directory,name)
        if type == 'xml':
            raise NotImplementedError("TODO: load xml files from Python API")
        elif type == 'json':
            f = open(fn,'r')
            text = ''.join(f.readlines())
            f.close()
            value = loader.fromJson(text,type=type)
        else:
            value = loader.load(type,fn)
        if value==None:
            raise IOError
        if doedit==True:
            success,newvalue = edit(name,value=value,type=type,description=description,editor=editor,world=world,frame=frame)
            if success:
                print "Ok pressed, saving resource to",name
                value = newvalue
                set(name,value,type=type,directory=directory)
            else:
                print "Cancel pressed, not saving resource to disk"
            return value
        return value
    except IOError:
        if doedit!=False:
            print "Resource",fn,"does not exist, launching editor..."
            success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,frame=frame)
            if success:
                print "Ok pressed, saving resource to",name
                value = newvalue
                set(name,value=value,type=type,directory=directory)
            else:
                print "Cancel pressed, not saving resource to disk"
            return value
        elif default is not None:
            return default
        else:
            raise RuntimeError("Resource "+name+" does not exist")
    return

def set(name,value,type='auto',directory=None):
    """Saves a resource to disk under the given name."""
    if type == 'auto':
        type = filenameToType(name)
    if directory==None:
        directory = getDirectory()    
    fn = os.path.join(directory,name)
    _ensure_dir(fn)
    if type == 'xml':
        raise NotImplementedError("TODO: save xml files from Python API")
    elif type == 'json':
        f = open(fn,'w')
        f.write(loader.toJson(value,type=type))
        f.write('\n')
        f.close()
        return True
    else:
        return loader.save(value,type,fn)

class _VisualEditorBase(glcommon.GLWidgetPlugin):
    #A base class for editing resources.
    
    def __init__(self,name,value,description,world):
        glcommon.GLWidgetPlugin.__init__(self)
        self.name = name
        self.value = value
        self.description = description
        self.world = world
        #can create a Qt object to assist in editing
    def instructions(self):
        return None
    def display(self):
        if self.world: self.world.drawGL()
        self.klamptwidgetmaster.drawGL(self.viewport())
        return True
    def addDialogItems(self,parent,ui='qt'):
        return
    def display_screen(self):
        glDisable(GL_LIGHTING)
        glColor3f(0,0,0)
        h = 30
        """
        if self.instructions() != None:
            glRasterPos(20,h)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,"Instructions: "+self.instructions())
            h += 20
        if self.description != None:
            glRasterPos(20,h)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,"Description: "+self.description)
            h += 20
        glRasterPos(20,h)
        gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,"Press 'x' to exit without saving, 'q' to save+exit")
        """
        return True


class _ConfigVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world):
        _VisualEditorBase.__init__(self,name,value,description,world)
        world.robot(0).setConfig(value)
        self.clicked = None
        self.hovered = None
        self.robotposer = RobotPoser(world.robot(0))
        self.addWidget(self.robotposer)
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot'

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = self.robotposer.get()
            return True
        return False

    def display(self):
        #Override display handler since the widget draws the robot
        #the next few lines draw everything but the robot
        for i in xrange(self.world.numTerrains()):
            self.world.terrain(i).drawGL()
        for i in xrange(self.world.numRigidObjects()):
            self.world.rigidObject(i).drawGL()
        for i in xrange(1,self.world.numRobots()):
            self.world.robot(i).drawGL()
        #this line will draw the robot
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False

class _ConfigsVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world):
        _VisualEditorBase.__init__(self,name,value,description,world)
        if len(value) > 0:
            world.robot(0).setConfig(value[0])
        self.editingIndex = len(value)-1
        self.clicked = None
        self.hovered = None
        self.robotposer = RobotPoser(world.robot(0))
        self.addWidget(self.robotposer)
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot.\nKeyboard i: insert, d: delete, < to select previous, > to select next'

    def addDialogItems(self,parent,ui='qt'):
        self.indexSpinBox = QSpinBox()
        self.indexSpinBox.setRange(0,len(self.value)-1)
        layout = QHBoxLayout(parent)
        label = QLabel("Index")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.indexSpinBox)
        self.insertButton = QPushButton("Insert")
        self.deleteButton = QPushButton("Delete")
        layout.addWidget(self.insertButton)
        layout.addWidget(self.deleteButton)
        self.insertButton.clicked.connect(self.insert)
        self.deleteButton.clicked.connect(self.delete)
        self.indexSpinBox.valueChanged.connect(self.indexChanged)

    def insert(self):
        if self.editingIndex < 0:
            self.value.append(self.robotposer.get())
            self.editingIndex = len(self.value)-1
        else:
            self.value.insert(self.editingIndex+1,self.robotposer.get())
            self.editingIndex += 1
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.value)-1)
            self.indexSpinBox.setValue(self.editingIndex)
        self.refresh()

    def delete(self):
        if self.editingIndex >= 0:
            del self.value[self.editingIndex]
            if self.editingIndex >= len(self.value):
                self.editingIndex = len(self.value)-1
            if self.editingIndex >= 0:
                self.robotposer.set(self.value[self.editingIndex])
            print "Now has",len(self.value),"configs, editing index",self.editingIndex
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.value)-1)
            self.indexSpinBox.setValue(self.editingIndex)
        self.refresh()

    def indexChanged(self,index):
        self.editingIndex = index
        if index >= 0 and index < len(self.value):
            self.robotposer.set(self.value[self.editingIndex]) 
        self.refresh()

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            if self.editingIndex >= 0:
                self.value[self.editingIndex] = self.robotposer.get()
            return True
        return False
    
    def keyboardfunc(self,c,x,y):
        if c=='i':
            self.insert()
            return True
        elif c=='d':
            self.delete()
            return True
        elif c==',' or c=='<':
            self.editingIndex -= 1
            if self.editingIndex < 0:
                self.editingIndex = min(len(self.value)-1,0)
            if self.editingIndex >= 0:
                self.robotposer.set(self.value[self.editingIndex]) 
                self.refresh()
            return True
        elif c=='.' or c=='>':
            self.editingIndex += 1
            self.editingIndex = min(len(self.value)-1,self.editingIndex)
            if self.editingIndex >= 0:
                self.robotposer.set(self.value[self.editingIndex]) 
                self.refresh()
            return True

    def display(self):
        #Override display handler since the widget draws the robot
        #the next few lines draw everything but the robot
        for i in xrange(self.world.numTerrains()):
            self.world.terrain(i).drawGL()
        for i in xrange(self.world.numRigidObjects()):
            self.world.rigidObject(i).drawGL()
        for i in xrange(1,self.world.numRobots()):
            self.world.robot(i).drawGL()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        #draw most opaque first
        order = []
        if self.editingIndex < 0:
            order = range(len(self.value))
        else:
            order = [self.editingIndex]
            n = max(self.editingIndex,len(self.value)-self.editingIndex)
            for i in range(1,n+1):
                if self.editingIndex + i < len(self.value): order.append(self.editingIndex +i)
                if self.editingIndex - i >= 0: order.append(self.editingIndex -i)
        for i in order:
            #draw transparent
            opacity = pow(0.5,abs(i-self.editingIndex))
            for j in xrange(self.world.robot(0).numLinks()):
                self.world.robot(0).link(j).appearance().setColor(0.5,0.5,0.5,opacity)
            if i == self.editingIndex:
                #this line will draw the robot at the current editing config
                self.klamptwidgetmaster.drawGL(self.viewport())
            else:
                self.world.robot(0).setConfig(self.value[i])
                self.world.robot(0).drawGL()
        for j in xrange(self.world.robot(0).numLinks()):
            self.world.robot(0).link(j).appearance().setColor(0.5,0.5,0.5,1)
        glDisable(GL_BLEND)

class _PointVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        _VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.pointposer = PointPoser()
        self.pointposer.set(se3.apply(self.frame,value))
        self.pointposer.setAxes(self.frame[0])
        self.addWidget(self.pointposer)
   
    def instructions(self):
        return 'Right-click and drag on the widget to pose the point'

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.apply(se3.inv(self.frame),self.pointposer.get())
            return True
        return False

class _RotationVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        _VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,(value,[0,0,0])))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(False)
        self.addWidget(self.xformposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the rotation'

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())[0]
            return True
        return False

class _RigidTransformVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        _VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,value))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(True)
        self.addWidget(self.xformposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the transform'

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())
            return True
        return False


class _ObjectTransformVisualEditor(_VisualEditorBase):
    def __init__(self,name,value,description,world,object):
        _VisualEditorBase.__init__(self,name,value,description,world)
        self.object = object
        self.objposer = ObjectPoser(object)
        self.addWidget(self.objposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the object'

    def mousefunc(self,button,state,x,y):
        if _VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = self.objposer.get()
            return True
        return False

#Qt stuff
if glcommon._PyQtAvailable:
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
    from OpenGL.GL import *
    _dialog = None

    class _MyDialog(QDialog):
        def __init__(self,glwidget):
            QDialog.__init__(self)
            glwidget.setMinimumSize(glwidget.width,glwidget.height)
            glwidget.setMaximumSize(4000,4000)
            glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
            self.instructions = QLabel()
            self.description = QLabel()
            self.description2 = QLabel("Press OK to save, Cancel to continue without saving")
            self.topBox = QFrame()
            self.topBoxLayout = QVBoxLayout(self.topBox)
            self.topBoxLayout.addWidget(self.description)
            self.topBoxLayout.addWidget(self.instructions)
            self.extraDialog = QFrame()
            self.extraDialog.setSizePolicy(QSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum))
            self.topBoxLayout.addWidget(self.extraDialog)
            self.layout = QVBoxLayout(self)
            self.layout.addWidget(self.topBox)
            self.layout.addWidget(glwidget)
            self.layout.addWidget(self.description2)
            self.layout.setStretchFactor(glwidget,10)
            self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel,Qt.Horizontal, self)
            self.buttons.accepted.connect(self.accept)
            self.buttons.rejected.connect(self.reject)
            self.layout.addWidget(self.buttons)
            self.doexit = False
        def setEditor(self,editorObject):
            self.editorObject = editorObject
            self.setWindowTitle("Editing "+editorObject.name)
            visualization.setPlugin(editorObject)
            if editorObject.description==None:
                self.description.setText("")
            else:
                self.description.setText(editorObject.description)
            self.instructions.setText(editorObject.instructions())
            editorObject.addDialogItems(self.extraDialog,ui='qt')

        def closeEvent(self,event):
            reply = QMessageBox.question(self, 'Message',
                 "Are you sure to quit the program?", QMessageBox.Yes | 
                 QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                self.doexit = True
                event.accept()
            else:
                event.ignore()     
            
        def finish(self):
            visualization.setPlugin(None)
            res = self.editorObject.value
            self.topBoxLayout.removeWidget(self.extraDialog)
            self.extraDialog.setParent(None)
            self.extraDialog = QFrame()
            self.extraDialog.setSizePolicy(QSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum))
            self.topBoxLayout.addWidget(self.extraDialog)
            self.editorObject = None
            return res

    def _makeDialog(editorObject):
        global _dialog
        if _dialog == None:
            _dialog=_MyDialog(visualization._widget)
        _dialog.setEditor(editorObject)
        res = _dialog.exec_()
        if _dialog.doexit:
            return None,None
        retVal = _dialog.finish()
        return res,retVal

    def _launch(editorObject):
        global _dialog
        olditems = visualization._vis.items.copy()
        visualization._vis.items = {}

        oldtitle = visualization.getWindowTitle()
        visualization.setWindowTitle("Resource Editor")
        res,retVal = visualization.customRun(_makeDialog,args=(editorObject,))
        visualization.setWindowTitle(oldtitle)

        if _dialog.doexit:
            visualization.kill()
            print "Exiting program."
            exit(0)


        visualization._vis.items = olditems
        return res,retVal
else:
    def _launch(editorObject):
        raise ValueError("Unable to perform visual editing without PyQt")

def console_edit(name,value,type,description=None,world=None,frame=None):
    print "*********************************************************"
    print 
    print "Editing resource",name,"of type",type
    print
    if description!=None:
        print description
        print
    if frame!=None:
        if isinstance(frame,(RigidObjectModel,RobotModelLink)):
            print "Reference frame:",frame.getName()
        else:
            print "Reference frame:",frame
        print
    print "*********************************************************"
    print "Current value:",value
    print "Do you wish to change it? (y/n/q) >",
    choice = ''
    while choice not in ['y','n','q']:
        choice = raw_input()[0].lower()
        if choice not in ['y','n','q']:
            print "Please enter y/n/q indicating yes/no/quit."
            print ">",
    if choice=='y':
        print "Enter the new desired value below.  You may use native text,"
        print "JSON strings, or file(fn) to indicate a file name."
        print "New value >",
        import json
        data = raw_input()
        if data.startswith('{') or data.startswith('['):
            jsonobj = json.loads(data)
            try:
                obj = loader.fromJson(jsonobj,type)
                return True,obj
            except Exception:
                print "Error loading from JSON, press enter to continue..."
                raw_input()
                return False,value
        elif data.startswith('file('):
            try:
                obj = get(data[5:-1],type,doedit=False)
                if obj==None:
                    return False,value
                return True,obj
            except Exception:
                print "Error loading from file, press enter to continue..."
                raw_input()
                return False,value
        else:
            try:
                obj = loader.read(type,data)
                return True,obj
            except Exception:
                print "Error loading from text, press enter to continue..."
                raw_input()
                return False,value
    elif choice=='n':
        print "Using current value."
        print "*********************************************************"
        return False,value
    elif choice=='q':
        return False,None

_editTemporaryWorlds = {}

def edit(name,value,type='auto',description=None,editor='visual',world=None,frame=None):
    """Launches an editor for the given value.  Returns a pair (save,result)
    where save indicates what the user wanted to do with the edited value
    and result is the edited value."""
    if name == None and type=='auto':
        raise RuntimeError("Cannot do an anonymous edit without the 'type' argument specified")
    if name == None:
        name = 'Anonymous'
    if type == 'auto':
        type = objectToTypes(value)
        if type is None:
            raise RuntimeError("Could not autodetect type of object "+name)
        if isinstance(type,(list,tuple)):
            type = type[0]
    if not glcommon._PyQtAvailable and editor=='visual':
        print "PyQt is not available, defaulting to console editor"
        editor = 'console'
            
    if isinstance(world,str):
        #a single argument, e.g., a robot file
        global _editTemporaryWorlds
        if world not in _editTemporaryWorlds:
            _editTemporaryWorlds[world] = WorldModel()
            if not _editTemporaryWorlds[world].readFile(world):
                raise RuntimeError("Error loading world file "+world)
        world = _editTemporaryWorlds[world]    
    if isinstance(frame,str):
        try:
            oframe = world.rigidObject(frame)
            frame = oframe
        except RuntimeError:
            try:
                oframe = world.robot(0).link(frame)
                frame = oframe
            except RuntimeError:
                try:
                    oframe = world.terrain(frame)
                    frame = oframe
                except RuntimeError:
                    raise RuntimeError('Named frame "'+frame+'" is not a valid frame')
    if value==None:
        if type == 'Config':
            if world==None:
                raise RuntimeError("Cannot visually edit a Config resource without a world")
            value = world.robot(0).getConfig()
        elif type == 'Configs':
            if world==None:
                raise RuntimeError("Cannot visually edit a Configs resource without a world")
            value = [world.robot(0).getConfig()]
        elif type == 'IKGoal':
            value = IKObjective()
        elif type == 'Vector3' or type == 'Point':
            value = [0,0,0]
        elif type == 'Rotation':
            value = so3.identity()
        elif type == 'RigidTransform':
            value = se3.identity()
        else:
            raise RuntimeError("Don't know how to edit objects of type "+type)

    if editor == 'console':
        return console_edit(name,value,type,description,world,frame)
    elif editor == 'visual':
        if type == 'Config':
            return _launch(_ConfigVisualEditor(name,value,description,world))
        elif type == 'Configs':
            return _launch(_ConfigsVisualEditor(name,value,description,world))
        elif type == 'Vector3' or type == 'Point':
            if isinstance(frame,(RigidObjectModel,RobotModelLink)):
                frame = frame.getTransform()
            return _launch(_PointVisualEditor(name,value,description,world,frame))
        elif type == 'Rotation':
            if isinstance(frame,(RigidObjectModel,RobotModelLink)):
                frame = frame.getTransform()
            return _launch(_RotationVisualEditor(name,value,description,world,frame))
        elif type == 'RigidTransform':
            if isinstance(frame,RigidObjectModel):
                return _launch(_ObjectTransformVisualEditor(name,value,description,world,frame))
            if isinstance(frame,RobotModelLink):
                frame = frame.getTransform()
            return _launch(_RigidTransformVisualEditor(name,value,description,world,frame))
        else:
            raise RuntimeError("Visual editing of objects of type "+type+" not supported yet")
    else:
        raise ValueError("Invalid value for argument 'editor', must be either 'visual' or 'console'")

