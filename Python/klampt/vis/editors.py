import glcommon
import glinit
import visualization
import time
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
        if isinstance(robot,SubRobotModel):
            self.robotposer = RobotPoser(robot._robot)
            self.robotposer.setActiveDofs(robot._links)
        else:
            self.robotposer = RobotPoser(robot)
        self.addWidget(self.robotposer)
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot'

    def mousefunc(self,button,state,x,y):
        if self.robotposer.hasFocus():
            self.value = self.robotposer.get()
        return VisualEditorBase.mousefunc(self,button,state,x,y)

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
        if self.editingIndex >= 0 and self.robotposer.hasFocus():
            #mouse release
            self.value[self.editingIndex] = self.robotposer.get()
        return VisualEditorBase.mousefunc(self,button,state,x,y)
    
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
                self.editingIndex = min(len(self.durations)-1,0)
            self.indexEditBox.setValue(self.editingIndex)
            self.indexChanged(self.editingIndex)
            return True
        elif c=='.' or c=='>':
            self.editingIndex += 1
            self.editingIndex = min(len(self.durations)-1,self.editingIndex)
            self.indexEditBox.setValue(self.editingIndex)
            self.indexChanged(self.editingIndex)
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

class TrajectoryEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        if robot is None:
            robot = world.robot(0)
        if len(value.milestones) > 0:
            robot.setConfig(value.milestones[0])
        self.robot = robot
        self.editingIndex = len(value.milestones)-1
        self.durations = []
        if len(value.times) > 0:
            self.durations.append(value.times[0])
            for i in xrange(len(value.times)-1):
                self.durations.append(value.times[i+1]-value.times[i])
        self.animTrajectory = None
        self.animTrajectoryTime = 0.0
        self.animating = False
        self.animSelectorValue = 0
        self.lastAnimTrajectoryTime = None
        self.robotposer = RobotPoser(robot)
        self.addWidget(self.robotposer)
        self.updateAnimTrajectory()
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot.\nKeyboard i: insert, d: delete, < to select previous, > to select next'

    def addDialogItems(self,parent,ui='qt'):
        vlayout = QVBoxLayout(parent)
        #adding and editing keyframes
        self.indexSpinBox = QSpinBox()
        self.indexSpinBox.setRange(0,len(self.durations)-1)
        self.durationSpinBox = QDoubleSpinBox()
        self.durationSpinBox.setRange(0,10.0)
        self.durationSpinBox.setSingleStep(0.01)
        self.durationSpinBox.setDecimals(4)
        self.insertButton = QPushButton("Insert")
        self.deleteButton = QPushButton("Delete")
        self.indexSpinBox.valueChanged.connect(self.indexChanged)
        self.durationSpinBox.valueChanged.connect(self.durationChanged)
        self.insertButton.clicked.connect(self.insert)
        self.deleteButton.clicked.connect(self.delete)

        layout = QHBoxLayout()
        vlayout.addLayout(layout)
        label = QLabel("Index")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.indexSpinBox)
        label = QLabel("Duration")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.durationSpinBox)
        layout.addWidget(self.insertButton)
        layout.addWidget(self.deleteButton)        

        #playback
        self.timeDriver = QSlider()
        self.timeDriver.setOrientation(Qt.Horizontal)
        self.timeDriver.setRange(0,1000)
        self.timeDriver.valueChanged.connect(self.timeDriverChanged)
        self.playButton = QPushButton("Play")
        self.playButton.setCheckable(True)
        self.playButton.toggled.connect(self.togglePlay)

        layout = QHBoxLayout()
        vlayout.addLayout(layout)
        self.animSelector = QComboBox()
        self.animSelector.addItem("Linear")
        self.animSelector.addItem("Spline")
        #self.animSelector.addItem("Retimed")
        #self.animSelector.addItem("Retimed-spline")
        self.animSelector.currentIndexChanged.connect(self.animSelectorChanged)

        label = QLabel("Time")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.timeDriver)
        layout.addWidget(self.playButton)
        
        label = QLabel("Interp.")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.animSelector)

    def insert(self):
        if self.editingIndex < 0:
            self.value.times.append(0.0)
            self.value.milestones.append(self.robotposer.get())
            self.editingIndex = len(self.durations)-1
        else:
            newdur = 1.0
            if self.editingIndex+1 == len(self.durations):
                #extrapolate from previous
                if self.editingIndex > 0:
                    newdur = self.value.times[self.editingIndex] - self.value.times[self.editingIndex-1]
            elif self.editingIndex == 0:
                #shift everything else
                if len(self.durations) > 1:
                    newdur = self.value.times[1]-self.value.times[0]
            else:
                #subdivide time between milesones
                newdur = self.value.times[self.editingIndex]-self.value.times[self.editingIndex-1]
            self.durations.insert(self.editingIndex+1,newdur)
            self.value.milestones.insert(self.editingIndex+1,self.robotposer.get())
            self.onDurationsChanged()
            self.editingIndex += 1
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.durations)-1)
            self.indexSpinBox.setValue(self.editingIndex)
        self.refresh()

    def delete(self):
        if self.editingIndex >= 0:
            del self.durations[self.editingIndex]
            del self.value.milestones[self.editingIndex]
            if self.editingIndex >= len(self.durations):
                self.editingIndex = len(self.durations)-1
            if self.editingIndex >= 0:
                self.robotposer.set(self.value.milestones[self.editingIndex])
            self.onDurationsChanged()
            print "Now has",len(self.durations),"configs, editing index",self.editingIndex
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.durations)-1)
            self.indexSpinBox.setValue(self.editingIndex)
            if self.editingIndex >= 0:
                self.durationSpinBox.setValue(self.durations[self.editingIndex])
        self.refresh()

    def indexChanged(self,index):
        self.editingIndex = index
        if index >= 0 and index < len(self.durations):
            self.durationSpinBox.setValue(self.durations[self.editingIndex])
            self.robotposer.set(self.value.milestones[self.editingIndex]) 
            if not self.animating:
                self.animTrajectoryTime = self.value.times[index]
                self.timeDriver.setValue(int(1000*(self.animTrajectoryTime - self.value.times[0])/self.value.duration()))
        self.refresh()

    def durationChanged(self,value):
        if self.editingIndex >= 0 and self.editingIndex < len(self.durations):
            self.durations[self.editingIndex] = max(value,0.0)
            self.onDurationsChanged()
        self.refresh()

    def timeDriverChanged(self,value):
        u = value * 0.001
        self.animTrajectoryTime = self.animTrajectory.times[0] + u*self.animTrajectory.duration()
        self.refresh()

    def animSelectorChanged(self,value):
        self.animSelectorValue = value
        self.updateAnimTrajectory()
        self.refresh()

    def togglePlay(self,value):
        self.animating = value
        self.refresh()
        if value:
            self.idlesleep(0)
        else:
            self.idlesleep(float('inf'))

    def onDurationsChanged(self):
        """Update the trajectory times"""
        if len(self.durations)==0:
            self.value.times = []
        else:
            self.value.times = [self.durations[0]]
            for i in range(1,len(self.durations)):
                self.value.times.append(self.value.times[-1] + self.durations[i])
        self.updateAnimTrajectory()
        if not self.animating:
            if hasattr(self,'timeDriver'):
                self.timeDriver.setValue(int(1000*(self.animTrajectoryTime - self.value.times[0])/self.value.duration()))

    def updateAnimTrajectory(self):
        from ..model import trajectory
        if self.animSelectorValue == 1:
            traj = trajectory.HermiteTrajectory()
            traj.makeSpline(self.value)
            self.animTrajectory = traj.configTrajectory()
        else:
            #TODO: other selections
            self.animTrajectory = self.value

    def mousefunc(self,button,state,x,y):
        if self.editingIndex >= 0 and self.robotposer.hasFocus():
            self.value.milestones[self.editingIndex] = self.robotposer.get()
        return VisualEditorBase.mousefunc(self,button,state,x,y)
    
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
                self.editingIndex = min(len(self.durations)-1,0)
            if hasattr(self,'indexEditBox'):
                self.indexEditBox.setValue(self.editingIndex)
                self.indexChanged(self.editingIndex)
            return True
        elif c=='.' or c=='>':
            self.editingIndex += 1
            self.editingIndex = min(len(self.durations)-1,self.editingIndex)
            if hasattr(self,'indexEditBox'):
                self.indexEditBox.setValue(self.editingIndex)
                self.indexChanged(self.editingIndex)
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

        #draw animation, if available
        if self.animTrajectoryTime is not None:
            for j in xrange(self.robot.numLinks()):
                self.robot.link(j).appearance().setColor(1.0,1.0,0,0.5)
            q = self.animTrajectory.eval(self.animTrajectoryTime,'loop')
            self.robot.setConfig(q)
            self.robot.drawGL()
            for j in xrange(self.robot.numLinks()):
                self.robot.link(j).appearance().setColor(0.5,0.5,0.5,1)
        
        #draw most opaque first
        order = []
        if self.editingIndex < 0:
            order = range(len(self.durations))
        else:
            order = [self.editingIndex]
            n = max(self.editingIndex,len(self.durations)-self.editingIndex)
            for i in range(1,n+1):
                if self.editingIndex + i < len(self.durations): order.append(self.editingIndex +i)
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
                self.robot.setConfig(self.value.milestones[i])
                self.robot.drawGL()
        for j in xrange(self.robot.numLinks()):
            self.robot.link(j).appearance().setColor(0.5,0.5,0.5,1)
        glDisable(GL_BLEND)

    def idle(self):
        import time
        t = time.time()
        if self.animating:
            self.animTrajectoryTime += t - self.lastAnimTrajectoryTime
            if self.animTrajectoryTime > self.value.times[-1]:
                self.animTrajectoryTime -= self.value.duration()
            self.timeDriver.setValue(int(1000*(self.animTrajectoryTime - self.value.times[0])/self.value.duration()))
            self.refresh()
        self.lastAnimTrajectoryTime = t
        return False



class SelectionEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.robot = robot
        self.lastClicked = -1
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
            self.value = range(self.world.numIDs())
        else:
            self.value = range(self.robot.numLinks())
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
        if self.pointposer.hasFocus():
            self.value = se3.apply(se3.inv(self.frame),self.pointposer.get())
        return VisualEditorBase.mousefunc(self,button,state,x,y)

class RigidTransformEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,frame=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame==None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,value))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(True)
        self.addWidget(self.xformposer)
        self.attachedObjects = []
        self.attachedRelativePoses = []
        self.rotationEnabled = True
        self.translationEnabled = True

    def disableTranslation(self):
        self.translationEnabled = False
        self.xformposer.enableTranslation(False)

    def disableRotation(self):
        self.rotationEnabled = False
        self.xformposer.enableRotation(False)

    def attach(self,object):
        assert hasattr(object,'setTransform'),"Can only attach objects with setTransform and getTransform methods"
        assert hasattr(object,'getTransform'),"Can only attach objects with setTransform and getTransform methods"
        self.attachedObjects.append(object)
    
    def instructions(self):
        if self.rotationEnabled and self.translationEnabled:
            return 'Right-click and drag on the widget to pose the transform'
        elif self.rotationEnabled:
            return 'Right-click and drag on the widget to pose the rotation'
        else:
            return 'Right-click and drag on the widget to change the translation'

    def mousefunc(self,button,state,x,y):
        if VisualEditorBase.mousefunc(self,button,state,x,y):
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())
            if len(self.attachedRelativePoses) < len(self.attachedObjects):
                for o in self.attachedObjects[len(self.attachedRelativePoses):]:
                    self.attachedRelativePoses.append(se3.mul(se3.inv(self.xformposer.get()),o.getTransform()))
            return True
        return False

    def motionfunc(self,x,y,dx,dy):
        if self.xformposer.hasFocus():
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())
            for o,p in zip(self.attachedObjects,self.attachedRelativePoses):
                o.setTransform(*se3.mul(self.xformposer.get(),p))
        return VisualEditorBase.motionfunc(self,x,y,dx,dy)

    def display(self):
        VisualEditorBase.display(self)
        return True


class ObjectTransformEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,object):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.object = object
        self.objposer = ObjectPoser(object)
        self.addWidget(self.objposer)
    
    def instructions(self):
        return 'Right-click and drag on the widget to pose the object'

    def mousefunc(self,button,state,x,y):
        if self.objposer.hasFocus():
            self.value = self.objposer.get()
        return VisualEditorBase.mousefunc(self,button,state,x,y)


class WorldEditor(VisualEditorBase):
    def __init__(self,name,value,description):
        VisualEditorBase.__init__(self,name,value,description,value)
        world = value
        self.world = value
        self.robotPosers = [RobotPoser(world.robot(i)) for i in range(world.numRobots())]
        self.objectPosers = [ObjectPoser(world.rigidObject(i)) for i in range(world.numRigidObjects())]
        self.terrainPosers = [TransformPoser() for i in range(world.numTerrains())]
        self.terrainGeometryCenters = []
        for i in range(world.numTerrains()):
            bmin,bmax=world.terrain(i).geometry().getBB()
            gc = vectorops.interpolate(bmin,bmax,0.5)
            T0 = world.terrain(i).geometry().getCurrentTransform()
            self.terrainGeometryCenters.append(se3.apply(se3.inv(T0),gc))
            #Tw.R = Tg.R
            #Tw.t = gc = Tg.R*gcloc + Tg.t
            self.terrainPosers[i].set(T0[0],gc)
        for r in self.robotPosers:
            self.addWidget(r)
        for r in self.objectPosers:
            self.addWidget(r)
        for r in self.terrainPosers:
            self.addWidget(r)

    def display(self):
        #Override display handler since the widget draws the rigid objects and transforms
        for i in xrange(self.world.numTerrains()):
            #self.world.terrain(i).geometry().getCurrentTransform()
            self.world.terrain(i).appearance().drawWorldGL(self.world.terrain(i).geometry())
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False

    def finalize(self):
        """Applies the transforms to all the terrain geometries."""
        for i in xrange(self.world.numTerrains()):
            T0 = self.world.terrain(i).geometry().getCurrentTransform()
            self.world.terrain(i).geometry().setCurrentTransform(*se3.identity())
            self.world.terrain(i).geometry().transform(*T0)
    
    def instructions(self):
        return 'Right-click and drag on the widgets to pose the world objects'

    def motionfunc(self,button,state,x,y):
        for i,r in enumerate(self.terrainPosers):
            if r.hasFocus():
                Tw = r.get()
                gcloc = self.terrainGeometryCenters[i]
                #Tw.t = Tw.R*gc + Tg.t
                self.world.terrain(i).geometry().setCurrentTransform(Tw[0],vectorops.sub(Tw[1],so3.apply(Tw[0],gcloc)))
        return VisualEditorBase.motionfunc(self,button,state,x,y)


#Qt stuff
if glinit._PyQtAvailable:
    if glinit._PyQt5Available:
        from PyQt5.QtCore import *
        from PyQt5.QtGui import *
        from PyQt5.QtWidgets import *
    else:
        from PyQt4.QtCore import *
        from PyQt4.QtGui import *
    global _vis_id,_my_dialog_res,_doexit
    _vis_id = None
    _my_dialog_res = None
    _doexit = False

    class _EditDialog(QDialog):
        def __init__(self,glwidget):
            QDialog.__init__(self)
            self.glwidget = glwidget
            #glwidget.setMinimumSize(glwidget.width,glwidget.height)
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
            self.layout = QVBoxLayout()
            #self.layout.addWidget(self.topBox)
            self.layout.addWidget(glwidget)
            self.layout.addWidget(self.description2)
            self.layout.setStretchFactor(glwidget,10)
            #self.layout.setStretchFactor(self.topBox,0)
            self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel,Qt.Horizontal, self)
            self.buttons.accepted.connect(self.accept)
            self.buttons.rejected.connect(self.reject)
            self.layout.addWidget(self.buttons)
            self.splitter = QSplitter(Qt.Vertical)
            top = QWidget(self)
            bottom = QWidget(self)
            top.setLayout(self.topBoxLayout)
            bottom.setLayout(self.layout)
            self.splitter.addWidget(top)
            self.splitter.addWidget(bottom)
            hbox = QHBoxLayout(self)
            hbox.addWidget(self.splitter)
            self.splitter.setSizes([self.topBoxLayout.sizeHint().height(),self.layout.sizeHint().height()])


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
            print "#########################################"
            print "klampt.vis: EditDialog accept"
            print "#########################################"
            return QDialog.accept(self)
        def reject(self):
            global _my_dialog_res
            _my_dialog_res = False
            print "#########################################"
            print "klampt.vis: EditDialog reject"
            print "#########################################"
            return QDialog.reject(self)


    def run(editorObject):
        """Returns a pair (res,value) where res is True / False if OK / Cancel was pressed, respectively, 
        and value is the return value of the editor object
        """
        assert isinstance(editorObject,VisualEditorBase),"Must provide a VisualEditorBase instance to vis.editors.run()"
        global _vis_id, _my_dialog_res

        old_vis_window = visualization.getWindow()
        if _vis_id == None:
            _vis_id = visualization.createWindow("Resource Editor")
        else:
            visualization.setWindow(_vis_id)
        visualization.setPlugin(editorObject)
        def makefunc(gl_backend):
            assert gl_backend is not None
            res = _EditDialog(gl_backend)
            res.setEditor(editorObject)
            visualization._checkWindowCurrent(editorObject.world)
            return res
        visualization.customUI(makefunc)
        visualization.dialog()
        res,retVal = _my_dialog_res,editorObject.value
        assert res is not None,"vis.editors.run(): There may be something wrong with the vis module not catching the customUI, or terminating from a prior dialog?"

        if _doexit:
            visualization.kill()
            print "Exiting program."
            exit(0)

        visualization.setPlugin(None)
        visualization.customUI(None)
        visualization.setWindow(old_vis_window)

        print "vis.editors.run(): Result",res,"return value",retVal
        return res,retVal
else:
    def run(editorObject):
        raise ValueError("Unable to perform visual editing without PyQt")
