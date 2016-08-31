import glcommon
import glinit
import visualization
from ..math import vectorops,so3,se3
from ..robotsim import WidgetSet,RobotPoser,ObjectPoser,TransformPoser,PointPoser,WorldModel,RobotModelLink,RigidObjectModel,IKObjective
from ..model.subrobot import SubRobotModel
from ..model import collide
from OpenGL.GL import *

class VisualEditorBase(glcommon.GLWidgetPlugin):
    """A base class for editing resources."""
    
    def __init__(self,name,value,description,world):
        glcommon.GLWidgetPlugin.__init__(self)
        self.name = name
        self.value = value
        self.description = description
        self.world = world
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


class ConfigEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        if robot is None:
            robot = world.robot(0)
        robot.setConfig(value)
        self.robot = robot
        self.clicked = None
        self.hovered = None
        if isinstance(robot,SubRobotModel):
            self.robotposer = RobotPoser(robot._robot)
            self.robotposer.setActiveDofs(robot._links)
        else:
            self.robotposer = RobotPoser(robot)
        self.addWidget(self.robotposer)
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = self.robotposer.get()
            return True
        return False

    def display(self):
        #Override display handler since the widget draws the robot
        #the next few lines draw everything but the robot
        if self.world:
            for i in xrange(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in xrange(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in xrange(self.world.numRobots()):
                if i != self.robot.index:
                    self.world.robot(i).drawGL()
        #this line will draw the robot
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False

class ConfigsEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        if robot is None:
            robot = world.robot(0)
        if len(value) > 0:
            robot.setConfig(value[0])
        self.robot = robot
        self.editingIndex = len(value)-1
        self.clicked = None
        self.hovered = None
        self.robotposer = RobotPoser(robot)
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
        if VisualEditorBase.mousefunc(self,button,state,x,y):
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
        if self.world != None:
            for i in xrange(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in xrange(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in xrange(self.world.numRobots()):
                if i != self.robot.index:
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
            for j in xrange(self.robot.numLinks()):
                self.robot.link(j).appearance().setColor(0.5,0.5,0.5,opacity)
            if i == self.editingIndex:
                #this line will draw the robot at the current editing config
                self.klamptwidgetmaster.drawGL(self.viewport())
            else:
                self.robot.setConfig(self.value[i])
                self.robot.drawGL()
        for j in xrange(self.robot.numLinks()):
            self.robot.link(j).appearance().setColor(0.5,0.5,0.5,1)
        glDisable(GL_BLEND)

class SelectionEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.robot = robot
        self.lastClicked = -1
        self.clicked = None
        self.hovered = None
        self.oldAppearances = {}
        self.newAppearances = {}

    def instructions(self):
        return 'Right-click to toggle selection of robot links / objects in the world.\nKeyboard: < to deselect previous, > to select next'

    def addDialogItems(self,parent,ui='qt'):
        layout = QHBoxLayout(parent)
        self.clearButton = QPushButton("Clear")
        self.selectAllButton = QPushButton("Select all")
        self.selectionList = QListWidget()
        self.selectionList.setSelectionMode(QAbstractItemView.MultiSelection)
        if self.robot != None:
            for i in xrange(self.robot.numLinks()):
                self.selectionList.addItem(self.robot.link(i).getName())
        elif self.world != None:
            for i in xrange(self.world.numIDs()):
                self.selectionList.addItem(self.world.getName(i))
        for i in self.value:
            self.selectionList.setCurrentItem(self.selectionList.item(i),QItemSelectionModel.Select)
        layout.addWidget(self.clearButton)
        layout.addWidget(self.selectAllButton)
        layout.addWidget(self.selectionList)
        self.clearButton.clicked.connect(self.clear)
        self.selectAllButton.clicked.connect(self.selectAll)
        self.selectionList.itemSelectionChanged.connect(self.selectionListChanged)
        self.selectionListChangeFlag = False

    def clear(self):
        self.value = []
        self.selectionList.clearSelection()
        self.refresh()

    def selectAll(self):
        if self.robot == None:
            #select all ids in the world
            pass
        else:
            self.value = [l for l in range(self.robot.numLinks())]
        self.selectionListChangeFlag = True
        for i in self.value:
            self.selectionList.setCurrentItem(self.selectionList.item(i),QItemSelectionModel.Select)
        self.selectionListChangeFlag = False
        self.refresh()        

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        geoms = [self.world.geometry(i) for i in range(self.world.numIDs())]
        res = collide.ray_cast(geoms,s,d)
        if not res:
            return
        id,geom = res
        self.toggle_selection(id)
        self.lastClicked = id
        self.refresh()

    def click_robot(self,x,y):
        """Helper: returns a list of robot objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        geoms = [self.robot.link(i).geometry() for i in range(self.robot.numLinks())]
        self.robot.setConfig(self.robot.getConfig())
        print [g.getCurrentTransform() for g in geoms]
        res = collide.ray_cast(geoms,s,d)
        if not res:
            return
        id,geom = res
        self.toggle_selection(id)
        self.lastClicked = id
        self.refresh()

    def selectionListChanged(self):
        #if the GUI has changed the selection then don't update the selection list
        if self.selectionListChangeFlag: return
        self.value = []
        for item in self.selectionList.selectedItems():
            row = self.selectionList.row(item)
            self.value.append(row)
        self.refresh()

    def add_selection(self,id):
        self.selectionListChangeFlag = True
        if id not in self.value:
            self.selectionList.setCurrentItem(self.selectionList.item(id),QItemSelectionModel.Select)
            self.value.append(id)
        self.selectionListChangeFlag = False

    def remove_selection(self,id):
        self.selectionListChangeFlag = False
        if id in self.value:
            self.value.remove(id)
            self.selectionList.setCurrentItem(self.selectionList.item(id),QItemSelectionModel.Deselect)
        self.selectionListChangeFlag = True

    def toggle_selection(self,id):
        self.selectionListChangeFlag = True
        if id in self.value:
            self.value.remove(id)
            self.selectionList.setCurrentItem(self.selectionList.item(id),QItemSelectionModel.Deselect)
        else:
            self.selectionList.setCurrentItem(self.selectionList.item(id),QItemSelectionModel.Select)
            self.value.append(id)
        self.selectionListChangeFlag = False

    def mousefunc(self,button,state,x,y):
        if button==2 and state==0:
            if self.robot == None:
                self.click_world(x,y)
            else:
                self.click_robot(x,y)
            return True
        return VisualEditorBase.mousefunc(self,button,state,x,y)
    
    def keyboardfunc(self,c,x,y):
        if c==',' or c=='<':
            if self.lastClicked >= 0:
                self.remove_selection(self.lastClicked)
            if self.lastClicked >= 0:
                self.lastClicked -= 1
            self.refresh()
            return True
        elif c=='.' or c=='>':
            Nmax = (self.robot.numLinks() if self.robot else self.world.numIDs())
            if self.lastClicked < Nmax:
                self.lastClicked += 1
            if self.lastClicked < Nmax:
                self.add_selection(self.lastClicked)
            self.refresh()
            return True

    def display(self):
        #Override display handler to highlight selected links

        #save old appearance and set new appearance
        apps = {}
        if self.robot != None:
            for i in xrange(self.robot.numLinks()):
                apps[i] = self.robot.link(i).appearance()
        elif self.world != None:
            for i in xrange(self.world.numIDs()):
                apps[i] = self.world.appearance(i)
        changed = self.value[:]
        if self.lastClicked >= 0: changed.append(self.lastClicked)
        for i in changed:
            if i not in self.oldAppearances:
                self.oldAppearances[i] = apps[i].clone()
                self.newAppearances[i] = apps[i].clone()
            if i == self.lastClicked:
                if i in self.value:
                    self.newAppearances[i].setColor(1,0.5,0)
                else:
                    self.newAppearances[i].setColor(1,0,0)
            else:
                self.newAppearances[i].setColor(1,1,0)
            apps[i].set(self.newAppearances[i])
        #draw
        self.world.drawGL()
        #restore old appearance
        for i in changed:
            apps[i].set(self.oldAppearances[i])
        glDisable(GL_BLEND)


class PointEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.pointposer = PointPoser()
        self.pointposer.set(se3.apply(self.frame,value))
        self.pointposer.setAxes(self.frame[0])
        self.addWidget(self.pointposer)
   
    def instructions(self):
        return 'Right-click and drag on the widget to pose the point'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.apply(se3.inv(self.frame),self.pointposer.get())
            return True
        return False

class RotationEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,(value,[0,0,0])))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(False)
        self.addWidget(self.xformposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the rotation'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())[0]
            return True
        return False

class RigidTransformEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,value))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(True)
        self.addWidget(self.xformposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the transform'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())
            return True
        return False


class ObjectTransformEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,object):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.object = object
        self.objposer = ObjectPoser(object)
        self.addWidget(self.objposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the object'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = self.objposer.get()
            return True
        return False



#Qt stuff
if glinit._PyQtAvailable:
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *
    _vis_id = None
    _my_dialog_res = None
    _my_dialog_retval = None
    _doexit = False

    class _EditDialog(QDialog):
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

        def setEditor(self,editorObject):
            self.editorObject = editorObject
            self.setWindowTitle("Editing "+editorObject.name)
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
                _doexit = True
                event.accept()
            else:
                event.ignore() 
            
        def accept(self):
            global _my_dialog_res
            _my_dialog_res = True
            QDialog.accept(self)
        def reject(self):
            global _my_dialog_res
            _my_dialog_res = False
            QDialog.reject(self)


    def run(editorObject):
        """Returns a pair (res,value) where res is True / False if OK / Cancel was pressed, respectively, 
        and value is the return value of the editor object
        """
        assert isinstance(editorObject,VisualEditorBase),"Must provide a VisualEditorBase instance to run()"
        global _vis_id, _my_dialog_res, _my_dialog_retval

        old_vis_window = visualization.getWindow()
        #if _vis_id == None:
        #    _vis_id = visualization.createWindow("Resource Editor")
        #else:
        #    visualization.setWindow(_vis_id)
        visualization.setPlugin(editorObject)
        def makefunc(gl_backend):
            res = _EditDialog(gl_backend)
            res.setEditor(editorObject)
            return res
        visualization.customUI(makefunc)
        visualization.dialog()
        res,retVal = _my_dialog_res,editorObject.value

        if _doexit:
            visualization.kill()
            print "Exiting program."
            exit(0)

        #visualization.setWindow(old_vis_window)
        visualization.setPlugin(None)
        visualization.customUI(None)

        print "Result",res,"return value",retVal
        return res,retVal
else:
    def run(editorObject):
        raise ValueError("Unable to perform visual editing without PyQt")
