"""Functions for visual editing.  Used by the klampt.io.resource module
in ``resource.get(...)`` and ``resource.edit(...)``.

A couple editors, SelectionEditor and WorldEditor, cannot be launched from
the ``resource`` module.  To use these, call::

    from klampt.vis import editors
    ed = editors.SelectionEditor("Some links",[],"Select the links that you want to modify",world))
    indices = editors.run(ed)

"""

from . import glcommon
from . import glinit
from . import gldraw
from . import visualization
import time
from ..math import vectorops,so3,se3
from ..robotsim import WidgetSet,RobotPoser,ObjectPoser,TransformPoser,PointPoser,AABBPoser,BoxPoser,SpherePoser,WorldModel,RobotModel,RobotModelLink,RigidObjectModel,TerrainModel,IKObjective,Appearance,Geometry3D
from ..model.subrobot import SubRobotModel
from ..model import trajectory
from ..model import collide
import warnings
from ..model.typing import Config, Vector, Vector3, RigidTransform
from typing import Optional, Sequence, List, Tuple, Any
from OpenGL.GL import *

class VisualEditorBase(glcommon.GLWidgetPlugin):
    """A base class for editing resources.  Used in conjunction with
    :func:`run`.
    """
    
    def __init__(self, name : str, value, description : str, world : WorldModel):
        glcommon.GLWidgetPlugin.__init__(self)
        self.name = name
        self.value = value
        self.description = description
        self.world = world
    def instructions(self) -> Optional[str]:
        return None
    def loadable(self) -> bool:
        """Whether Load... should be shown"""
        return True
    def savable(self) -> bool:
        """Whether Save... should be shown"""
        return True
    def display(self) -> bool:
        if self.world: self.world.drawGL()
        self.klamptwidgetmaster.drawGL(self.viewport())
        return True
    def update_gui_from_value(self):
        """Called when the value is externally changed (from Load...)"""
        return
    def update_value_from_gui(self):
        """Called when the value is requested (from Save... and OK)"""
        return
    def add_dialog_items(self,parent,ui='qt'):
        return
    def display_screen(self):
        pass
        """
        #for GLUT?
        glDisable(GL_LIGHTING)
        glColor3f(0,0,0)
        h = 30
        if self.instructions() is not None:
            self.window.draw_text((20,h),"Instructions: "+self.instructions())
            h += 20
        if self.description is not None:
            self.window.draw_text((20,h),"Description: "+self.description)
            h += 20
        glRasterPos(20,h)
        self.window.draw_text((20,h),"Press 'x' to exit without saving, 'q' to save+exit")
        """
        return True


class ConfigEditor(VisualEditorBase):
    """Edits a Config object.

    Either a world or a :class:`~klampt.RobotModel` or
    :class:`~klampt.SubRobotModel` must be provided.
    """
    def __init__(self, name : str, value : Config, description : str, world : WorldModel, robot : RobotModel=None):
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
            for i in range(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in range(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in range(self.world.numRobots()):
                if i != self.robot.index:
                    self.world.robot(i).drawGL()
        #this line will draw the robot
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False

    def update_gui_from_value(self):
        self.robotposer.set(self.value)
        self.refresh()


class ConfigsEditor(VisualEditorBase):
    """Edits a Configs object. The GUI lets the user select which index
    to edit, and then can click and drag on the robot widget to pose the
    selected configuration.

    Either a world or a :class:`~klampt.RobotModel` or
    :class:`~klampt.SubRobotModel` must be provided.
    """
    def __init__(self, name : str, value : List[Config], description : str, world : WorldModel, robot : RobotModel=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        if robot is None:
            robot = world.robot(0)
        if len(value) > 0:
            robot.setConfig(value[0])
        self.robot = robot
        self.editingIndex = len(value)-1
        if isinstance(robot,SubRobotModel):
            self.robotposer = RobotPoser(robot._robot)
            self.robotposer.setActiveDofs(robot._links)
        else:
            self.robotposer = RobotPoser(robot)
        self.addWidget(self.robotposer)
    
    def instructions(self):
        return 'Right-click and drag on the robot links to pose the robot.\nKeyboard i: insert, d: delete, < to select previous, > to select next'

    def add_dialog_items(self,parent,ui='qt'):
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
        self.indexSpinBox.valueChanged.connect(self.index_changed)

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
                if isinstance(self.robot,SubRobotModel):
                    self.robot.setConfig(self.value[self.editingIndex])
                    self.robotposer.set(self.robot._robot.getConfig())
                else:
                    self.robotposer.set(self.value[self.editingIndex])
            #print("Now has",len(self.value),"configs, editing index",self.editingIndex)
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.value)-1)
            self.indexSpinBox.setValue(self.editingIndex)
        self.refresh()

    def index_changed(self,index):
        self.editingIndex = index
        if index >= 0 and index < len(self.value):
            if isinstance(self.robot,SubRobotModel):
                self.robot.setConfig(self.value[self.editingIndex])
                self.robotposer.set(self.robot._robot.getConfig())
            else:
                self.robotposer.set(self.value[self.editingIndex]) 
        self.refresh()

    def mousefunc(self,button,state,x,y):
        if self.editingIndex >= 0 and self.robotposer.hasFocus():
            #mouse release
            if isinstance(self.robot,SubRobotModel):
                self.robot._robot.setConfig(self.robotposer.get())
                self.value[self.editingIndex] = self.robot.getConfig()
            else:
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
            self.indexSpinBox.setValue(self.editingIndex)
            self.index_changed(self.editingIndex)
            return True
        elif c=='.' or c=='>':
            self.editingIndex += 1
            self.editingIndex = min(len(self.durations)-1,self.editingIndex)
            self.indexSpinBox.setValue(self.editingIndex)
            self.index_changed(self.editingIndex)
            return True

    def display(self):
        #Override display handler since the widget draws the robot
        #the next few lines draw everything but the robot
        if self.world != None:
            for i in range(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in range(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in range(self.world.numRobots()):
                if i != self.robot.index:
                    self.world.robot(i).drawGL()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        #draw most opaque first
        order = []
        if self.editingIndex < 0:
            order = list(range(len(self.value)))
        else:
            order = [self.editingIndex]
            n = max(self.editingIndex,len(self.value)-self.editingIndex)
            for i in range(1,n+1):
                if self.editingIndex + i < len(self.value): order.append(self.editingIndex +i)
                if self.editingIndex - i >= 0: order.append(self.editingIndex -i)
        oldAppearances = [self.robot.link(j).appearance().clone() for j in range(self.robot.numLinks())]
        for i in order:
            #draw transparent
            opacity = pow(0.5,abs(i-self.editingIndex))
            for j in range(self.robot.numLinks()):
                self.robot.link(j).appearance().setColor(0.5,0.5,0.5,opacity)
                if opacity == 1:
                    self.robot.link(j).appearance().setSilhouette(0.004)
                else:
                    self.robot.link(j).appearance().setSilhouette(0)
            if i == self.editingIndex:
                #this line will draw the robot at the current editing config
                self.klamptwidgetmaster.drawGL(self.viewport())
            else:
                self.robot.setConfig(self.value[i])
                self.robot.drawGL()
        for j in range(self.robot.numLinks()):
            self.robot.link(j).appearance().set(oldAppearances[j])
        glDisable(GL_BLEND)

    def update_gui_from_value(self):
        if self.editingIndex >= len(self.value):
            self.editingIndex = len(self.value)-1
        self.indexSpinBox.setValue(self.editingIndex)
        self.index_changed(self.editingIndex)


class TrajectoryEditor(VisualEditorBase):
    """Edits a :class:`Trajectory`, :class:`RobotTrajectory`,
    or :class:`SE3Trajectory` object. 

    If a Trajectory is given, then it must either be attached to a robot or
    a 1D, 2D, or 3D trajectory.
    """
    def __init__(self, name : str, value : trajectory.Trajectory, description : str, world : WorldModel, robot : RobotModel=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        if robot is None:
            if isinstance(value,trajectory.RobotTrajectory):
                robot = value.robot
            elif isinstance(value,trajectory.SE3Trajectory):
                pass
            elif world.numRobots() > 0 and len(value.milestones) > 0 and len(value.milestones[0]) == world.robot(0).numLinks():
                robot = world.robot(0)
        self.robot = robot
        self.attachedObjects = []
        self.attachedRelativePoses = []
        self.attachedAppearances = []
        self.editingIndex = len(value.milestones)-1
        self.durations = []
        if len(value.times) > 0:
            self.durations.append(value.times[0])
            for i in range(len(value.times)-1):
                self.durations.append(value.times[i+1]-value.times[i])
        self.animTrajectory = None
        self.animTrajectoryTime = 0.0
        self.animating = False
        self.animSelectorValue = 0
        self.lastAnimTrajectoryTime = None
        if robot is not None:
            if len(value.milestones) > 0:
                qold = robot.getConfig()
                robot.setConfig(value.milestones[self.editingIndex])
            if isinstance(robot,SubRobotModel):
                self.milestoneposer = RobotPoser(robot._robot)
                self.milestoneposer.setActiveDofs(robot._links)
            else:
                self.milestoneposer = RobotPoser(robot)
            if len(value.milestones) > 0:
                robot.setConfig(qold)
        elif isinstance(value,trajectory.SE3Trajectory):
            self.milestoneposer = TransformPoser()
            if len(value.milestones) > 0:
                self.milestone_to_poser(value.milestones[self.editingIndex])
        elif len(value.milestones) > 0 and len(value.milestones[0])<=3:
            self.ndims = len(value.milestones[0])
            self.milestoneposer = PointPoser()
            if self.ndims == 2:
                self.milestoneposer.enableAxes(True,True,False)
            elif self.ndims == 1:
                self.milestoneposer.enableAxes(True,False,False)
            else:
                assert len(value.milestones[0])>0,"Need to have dimension >= 1"
            self.milestone_to_poser(value.milestones[self.editingIndex])
        else:
            raise NotImplementedError("Can't edit trajectories except for robot trajectories, R^2, R^3, or SE(3) trajectories yet")
        self.addWidget(self.milestoneposer)
        self.update_anim_trajectory()
    
    def attach(self,object,relativePose=None):
        """For an SE3 trajectory, shows the given object relative to the edited transform trajectory"""
        assert isinstance(object,(Geometry3D,RigidObjectModel,RobotModelLink,TerrainModel))
        assert self.robot is None,"Can't attach items to an editor for a RobotTrajectory"
        self.attachedObjects.append(object)
        if relativePose is None:
            self.attachedRelativePoses.append(se3.identity())
        else:
            self.attachedRelativePoses.append(relativePose)
        if hasattr(object,'appearance'):
            self.attachedAppearances.append(object.appearance())
        else:
            self.attachedAppearances.append(Appearance())
        
    def milestone_to_poser(self,m):
        if self.robot is not None:
            if isinstance(self.robot,SubRobotModel):
                self.robot.setConfig(m)
                self.milestoneposer.set(self.robot._robot.getConfig())
            else:
                self.milestoneposer.set(m)
        elif isinstance(self.value,trajectory.SE3Trajectory):
            self.milestoneposer.set(*self.value.to_se3(m))
        else:
            assert len(m) == self.ndims
            self.milestoneposer.set(m + [0]*(3-self.ndims))

    def poser_to_milestone(self):
        q = self.milestoneposer.get()
        if self.robot is not None:
            if isinstance(self.robot,SubRobotModel):
                self.robot._robot.setConfig(q)
                return self.robot.getConfig()
            else:
                return q
        elif isinstance(self.value,trajectory.SE3Trajectory):
            return q[0] + q[1]
        else:
            return q[:self.ndims]

    def instructions(self):
        if self.robot is not None:
            return 'Right-click and drag on the robot links to pose keyframes.\nKeyboard i: insert, d: delete, < to select previous, > to select next'
        else:
            return 'Right-click and drag on the poser to set keyframes.\nKeyboard i: insert, d: delete, < to select previous, > to select next'

    def add_dialog_items(self,parent,ui='qt'):
        vlayout = QVBoxLayout(parent)
        #adding and editing keyframes
        self.indexSpinBox = QSpinBox()
        self.indexSpinBox.setRange(0,len(self.durations)-1)
        self.indexSpinBox.setValue(self.editingIndex)
        self.durationSpinBox = QDoubleSpinBox()
        self.durationSpinBox.setRange(0,10.0)
        self.durationSpinBox.setSingleStep(0.01)
        self.durationSpinBox.setDecimals(4)
        self.insertButton = QPushButton("Insert")
        self.deleteButton = QPushButton("Delete")
        self.indexSpinBox.valueChanged.connect(self.index_changed)
        self.durationSpinBox.valueChanged.connect(self.duration_changed)
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
        self.timeDriver.valueChanged.connect(self.time_driver_changed)
        self.playButton = QPushButton("Play")
        self.playButton.setCheckable(True)
        self.playButton.toggled.connect(self.toggle_play)

        layout = QHBoxLayout()
        vlayout.addLayout(layout)
        self.animSelector = QComboBox()
        self.animSelector.addItem("Linear")
        self.animSelector.addItem("Spline")
        if self.robot is not None and not isinstance(self.value,trajectory.RobotTrajectory):
            self.animSelector.addItem("Linear (RobotTrajectory)")
        #self.animSelector.addItem("Retimed")
        #self.animSelector.addItem("Retimed-spline")
        self.animSelector.currentIndexChanged.connect(self.anim_selector_changed)

        label = QLabel("Time")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.timeDriver)
        layout.addWidget(self.playButton)
        
        label = QLabel("Interp.")
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(label)
        layout.addWidget(self.animSelector)
        self.index_changed(self.editingIndex)

    def insert(self):
        if self.editingIndex < 0:
            if len(self.value.times) == 0:
                self.value.times.append(0.0)
                self.durations.append(0.0)
                self.value.milestones.append(self.poser_to_milestone())
            self.value.times.append(self.value.times[-1]+self.durationSpinBox.value())
            self.value.milestones.append(self.poser_to_milestone())
            self.durations.append(self.durationSpinBox.value())
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
            self.value.milestones.insert(self.editingIndex+1,self.poser_to_milestone())
            self.on_durations_changed()
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
                self.milestone_to_poser(self.value.milestones[self.editingIndex])
            self.on_durations_changed()
            #print("Now has",len(self.durations),"configs, editing index",self.editingIndex)
        if hasattr(self,'indexSpinBox'):
            self.indexSpinBox.setRange(0,len(self.durations)-1)
            self.indexSpinBox.setValue(self.editingIndex)
            if self.editingIndex >= 0:
                self.durationSpinBox.setValue(self.durations[self.editingIndex])
        self.refresh()

    def index_changed(self,index):
        self.editingIndex = index
        if index >= 0 and index < len(self.durations):
            self.durationSpinBox.setValue(self.durations[self.editingIndex])
            self.milestone_to_poser(self.value.milestones[self.editingIndex]) 
            if not self.animating:
                self.animTrajectoryTime = self.value.times[index]
                if self.value.duration()==0:
                    self.timeDriver.setValue(0)
                else:
                    self.timeDriver.setValue(int(1000*(self.animTrajectoryTime - self.value.times[0])/self.value.duration()))
        self.refresh()

    def duration_changed(self,value):
        if self.editingIndex >= 0 and self.editingIndex < len(self.durations):
            self.durations[self.editingIndex] = max(value,0.0)
            self.on_durations_changed()
        self.refresh()

    def time_driver_changed(self,value):
        u = value * 0.001
        self.animTrajectoryTime = self.animTrajectory.times[0] + u*self.animTrajectory.duration()
        self.refresh()

    def anim_selector_changed(self,value):
        self.animSelectorValue = value
        self.update_anim_trajectory()
        self.refresh()

    def toggle_play(self,value):
        self.animating = value
        self.refresh()
        if value:
            self.idlesleep(0)
        else:
            #self.idlesleep(float('inf'))
            self.idlesleep(0.1)

    def on_durations_changed(self):
        """Update the trajectory times"""
        if len(self.durations)==0:
            self.value.times = []
        else:
            self.value.times = [self.durations[0]]
            for i in range(1,len(self.durations)):
                self.value.times.append(self.value.times[-1] + self.durations[i])
        self.update_anim_trajectory()
        if not self.animating:
            if hasattr(self,'timeDriver'):
                self.timeDriver.setValue(int(1000*(self.animTrajectoryTime - self.value.times[0])/self.value.duration()))

    def update_anim_trajectory(self):
        from ..model import trajectory
        if self.animSelectorValue == 1:
            if isinstance(self.value,trajectory.SE3Trajectory):
                traj = trajectory.SE3HermiteTrajectory()
            else:
                traj = trajectory.HermiteTrajectory()
            traj.makeSpline(self.value)
            self.animTrajectory = traj
        elif self.animSelectorValue == 2:
            self.animTrajectory = trajectory.RobotTrajectory(self.robot,self.value.times,self.value.milestones)
        else:
            #TODO: other selections
            self.animTrajectory = self.value

    def mousefunc(self,button,state,x,y):
        if self.editingIndex >= 0 and self.milestoneposer.hasFocus():
            self.value.milestones[self.editingIndex] = self.poser_to_milestone()
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
            if hasattr(self,'indexSpinBox'):
                self.indexSpinBox.setValue(self.editingIndex)
                self.index_changed(self.editingIndex)
            return True
        elif c=='.' or c=='>':
            self.editingIndex += 1
            self.editingIndex = min(len(self.durations)-1,self.editingIndex)
            if hasattr(self,'indexSpinBox'):
                self.indexSpinBox.setValue(self.editingIndex)
                self.index_changed(self.editingIndex)
            return True

    def display(self):
        #Override display handler since the widget draws the robot
        #the next few lines draw everything but the robot
        if self.world is not None:
            for i in range(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in range(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in range(self.world.numRobots()):
                if self.robot is None or i != self.robot.index:
                    self.world.robot(i).drawGL()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        if self.robot is None:
            glDisable(GL_LIGHTING)

        if self.robot is not None:
            oldAppearances = [self.robot.link(j).appearance().clone() for j in range(self.robot.numLinks())]

        #draw animation, if available
        if self.animTrajectoryTime is not None and self.animTrajectory.times:
            if self.robot is not None:
                for j in range(self.robot.numLinks()):
                    self.robot.link(j).appearance().setColor(1.0,1.0,0,0.5)
                q = self.animTrajectory.eval(self.animTrajectoryTime,'loop')
                self.robot.setConfig(q)
                self.robot.drawGL()
                for j in range(self.robot.numLinks()):
                    self.robot.link(j).appearance().set(oldAppearances[j])
            elif isinstance(self.value,trajectory.SE3Trajectory):
                T = self.animTrajectory.eval(self.animTrajectoryTime,'loop')
                glEnable(GL_LIGHTING)
                for (o,a,Trel) in zip(self.attachedObjects,self.attachedAppearances,self.attachedRelativePoses):
                    if hasattr(o,'setCurrentTransform'):
                        o.setCurrentTransform(*se3.mul(T,Trel))
                        a.drawWorldGL(o)
                    else:
                        o.setTransform(*se3.mul(T,Trel))
                        a.drawWorldGL(o.geometry())
                length = 0.1
                width = 0.01
                fancy = True
                gldraw.xform_widget(T,length,width,fancy)
                glDisable(GL_LIGHTING)
            else:
                x = self.animTrajectory.eval(self.animTrajectoryTime,'loop')
                pt = x + [0]*(3-self.ndims)
                if len(self.attachedObjects) > 0:
                    glEnable(GL_LIGHTING)
                    for (o,a,Trel) in zip(self.attachedObjects,self.attachedAppearances,self.attachedRelativePoses):
                        if hasattr(o,'setCurrentTransform'):
                            o.setCurrentTransform(Trel[0],vectorops.add(pt,Trel[1]))
                            a.drawWorldGL(o)
                        else:
                            o.setTransform(Trel[0],vectorops.add(pt,Trel[1]))
                            a.drawWorldGL(o.geometry())
                    glDisable(GL_LIGHTING)
                glPointSize(7.0)
                glColor3f(1,0.5,0)
                gldraw.point(pt)
        
        #draw most opaque first
        order = []
        if self.editingIndex < 0:
            order = list(range(len(self.durations)))
        else:
            order = [self.editingIndex]
            n = max(self.editingIndex,len(self.durations)-self.editingIndex)
            for i in range(1,n+1):
                if self.editingIndex + i < len(self.durations): order.append(self.editingIndex +i)
                if self.editingIndex - i >= 0: order.append(self.editingIndex -i)

        for i in order:
            #draw transparent
            opacity = pow(0.5,abs(i-self.editingIndex))
            if opacity < 5e-3:
                continue
            if self.robot is not None:
                for j in range(self.robot.numLinks()):
                    self.robot.link(j).appearance().setColor(0.5,0.5,0.5,opacity)
                    if opacity == 1:
                        self.robot.link(j).appearance().setSilhouette(0.004)
                    else:
                        self.robot.link(j).appearance().setSilhouette(0)
            if i == self.editingIndex:
                #this line will draw the robot at the current editing config
                self.klamptwidgetmaster.drawGL(self.viewport())
                if self.robot is None:
                    if isinstance(self.value,trajectory.SE3Trajectory):
                        T = self.value.to_se3(self.value.milestones[i])
                        for (o,a,Trel) in zip(self.attachedObjects,self.attachedAppearances,self.attachedRelativePoses):
                            if hasattr(o,'setCurrentTransform'):
                                o.setCurrentTransform(*se3.mul(T,Trel))
                                a.drawWorldGL(o)
                            else:
                                o.setTransform(*se3.mul(T,Trel))
                                if self.world is None or self.world.index != o.world:
                                    a.drawWorldGL(o.geometry())
                    else:
                        glEnable(GL_LIGHTING)
                        for (o,a,Trel) in zip(self.attachedObjects,self.attachedAppearances,self.attachedRelativePoses):
                            if hasattr(o,'setCurrentTransform'):
                                o.setCurrentTransform(Trel[0],vectorops.add(pt,Trel[1]))
                                a.drawWorldGL(o)
                            else:
                                o.setTransform(Trel[0],vectorops.add(pt,Trel[1]))
                                if self.world is None or self.world.index != o.world:
                                    a.drawWorldGL(o.geometry())
                    glDisable(GL_LIGHTING)
            else:
                if self.robot is not None:
                    self.robot.setConfig(self.value.milestones[i])
                    self.robot.drawGL()
                elif isinstance(self.value,trajectory.SE3Trajectory):
                    T = self.value.to_se3(self.value.milestones[i])
                    length = 0.1*opacity
                    width = 0.01*opacity
                    gldraw.xform_widget(T,length,width,False)
                else:
                    pt = self.value.milestones[i] + [0]*(3-self.ndims)
                    glPointSize(5.0)
                    glColor4f(1,0.5,0,opacity)
                    gldraw.point(pt)

        if self.robot is not None:
            for j in range(self.robot.numLinks()):
                self.robot.link(j).appearance().set(oldAppearances[j])
        elif len(self.value.milestones) > 1:
            glColor3f(1,1,0)
            glBegin(GL_LINE_STRIP)
            for i in range(0,len(self.value.milestones)):
                a = self.value.milestones[i]
                if isinstance(self.value,trajectory.SE3Trajectory):
                    glVertex3fv(a[9:12])
                else:
                    glVertex3fv(a + [0]*(3-self.ndims))
            glEnd()

        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)

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

    def update_gui_from_value(self):
        if self.editingIndex >= len(self.value.times):
            self.editingIndex = len(self.value.times)-1
        self.durations = []
        if len(self.value.times) > 0:
            self.durations.append(self.value.times[0])
            for i in range(len(self.value.times)-1):
                self.durations.append(self.value.times[i+1]-self.value.times[i])
        self.indexSpinBox.setValue(self.editingIndex)
        self.index_changed(self.editingIndex)
        self.on_durations_changed()


class SelectionEditor(VisualEditorBase):
    """Edits a list of indices selecting some links of a robot.
    """
    def __init__(self, name : str, value : List[int], description : str, world : WorldModel, robot : RobotModel=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.robot = robot
        self.lastClicked = -1
        self.oldAppearances = {}
        self.newAppearances = {}

    def instructions(self):
        return 'Right-click to toggle selection of robot links / objects in the world.\nKeyboard: < to deselect previous, > to select next'

    def add_dialog_items(self,parent,ui='qt'):
        layout = QHBoxLayout(parent)
        self.clearButton = QPushButton("Clear")
        self.selectAllButton = QPushButton("Select all")
        self.selectionList = QListWidget()
        self.selectionList.setSelectionMode(QAbstractItemView.MultiSelection)
        if self.robot != None:
            for i in range(self.robot.numLinks()):
                self.selectionList.addItem(self.robot.link(i).getName())
        elif self.world != None:
            for i in range(self.world.numIDs()):
                self.selectionList.addItem(self.world.getName(i))
        self.update_gui_from_value()
        layout.addWidget(self.clearButton)
        layout.addWidget(self.selectAllButton)
        layout.addWidget(self.selectionList)
        self.clearButton.clicked.connect(self.clear)
        self.selectAllButton.clicked.connect(self.select_all)
        self.selectionList.itemSelectionChanged.connect(self.selection_list_changed)
        self.selectionListChangeFlag = False

    def clear(self):
        self.value = []
        self.selectionList.clearSelection()
        self.refresh()

    def select_all(self):
        if self.robot is None:
            #select all ids in the world
            self.value = list(range(self.world.numIDs()))
        else:
            self.value = list(range(self.robot.numLinks()))
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

    def selection_list_changed(self):
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
            if self.robot is None:
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
            for i in range(self.robot.numLinks()):
                apps[i] = self.robot.link(i).appearance()
        elif self.world != None:
            for i in range(self.world.numIDs()):
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

    def update_gui_from_value(self):
        self.selectionList.clearSelection()
        for i in self.value:
            self.selectionList.setCurrentItem(self.selectionList.item(i),QItemSelectionModel.Select)


class PointEditor(VisualEditorBase):
    """Edits a 3-D point.

    If ``frame`` is given, then it is a :mod:`klampt.math.se3` element, and the
    input and output are measured with respect to that frame.  However, the
    editing is done in world coordinates.
    """
    def __init__(self, name : str, value : Vector3, description : str, world : WorldModel, frame : RigidTransform=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame is None else frame
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

    def update_gui_from_value(self):
        self.pointposer.set(se3.apply(self.frame,self.value))
        self.refresh()


class RigidTransformEditor(VisualEditorBase):
    """Edits a 3-D transform.

    If ``frame`` is given, then it is :mod:`klampt.math.se3` element, and the
    input and output are measured with respect to that frame.  However, the
    editing is done in world coordinates.

    Visualization objects can be attached to the edited transform using
    the :meth:`attach` method.
    """
    def __init__(self, name : str, value : RigidTransform, description : str, world : WorldModel, frame : RigidTransform=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame is None else frame
        self.xformposer = TransformPoser()
        self.xformposer.set(*se3.mul(self.frame,value))
        self.xformposer.enableRotation(True)
        self.xformposer.enableTranslation(True)
        self.addWidget(self.xformposer)
        self.attachedObjects = []
        self.attachedRelativePoses = []
        self.attachedAppearances = []
        self.rotationEnabled = True
        self.translationEnabled = True

    def disableTranslation(self):
        warnings.warn("disableTranslation will be deprecated in favor of disable_translation in a future version of Klampt",DeprecationWarning)
        self.disable_translation()

    def disableRotation(self):
        warnings.warn("disableRotation will be deprecated in favor of disable_rotation in a future version of Klampt",DeprecationWarning)
        self.disable_rotation()

    def disable_translation(self):
        """Turns off editing of translation."""
        self.translationEnabled = False
        self.xformposer.enableTranslation(False)

    def disable_rotation(self):
        """Turns off editing of rotation."""
        self.rotationEnabled = False
        self.xformposer.enableRotation(False)

    def attach(self,object,relativePose=None):
        """Attaches an object to visually move along with the edited
        transform.

        Args:
            object (RigidObjectModel or Geometry3D): the object to move
            relativePose (se3 element, optional): if given, the relative pose
                of the object w.r.t. the transform
        """
        assert hasattr(object,'setTransform') or hasattr(object,'setCurrentTransform'),"Can only attach objects with setTransform and getTransform methods"
        assert hasattr(object,'getTransform') or hasattr(object,'getCurrentTransform'),"Can only attach objects with setTransform and getTransform methods"
        self.attachedObjects.append(object)
        self.attachedRelativePoses.append(se3.identity() if relativePose is None else relativePose)
        if self.world is not None:
            #check to see if the object is drawn in the world
            if hasattr(object,'world') and object.world == self.world.index:
                self.attachedAppearances.append(None)
                return
        if hasattr(object,'appearance'):
            self.attachedAppearances.append(object.appearance())
        else:
            self.attachedAppearances.append(Appearance())
    
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
            return True
        return False

    def motionfunc(self,x,y,dx,dy):
        if self.xformposer.hasFocus():
            self.value = se3.mul(se3.inv(self.frame),self.xformposer.get())
            for o,p in zip(self.attachedObjects,self.attachedRelativePoses):
                if hasattr(o,'setCurrentTransform'):  #Geometry3D
                    o.setCurrentTransform(*se3.mul(self.xformposer.get(),p))
                else:
                    o.setTransform(*se3.mul(self.xformposer.get(),p))
        return VisualEditorBase.motionfunc(self,x,y,dx,dy)

    def display(self):
        VisualEditorBase.display(self)
        for (o,a,Trel) in zip(self.attachedObjects,self.attachedAppearances,self.attachedRelativePoses):
            if a is None: continue
            if hasattr(o,'geometry'):
                a.drawWorldGL(o.geometry())
            else:
                a.drawWorldGL(o)
        return True

    def update_gui_from_value(self):
        self.xformposer.set(*se3.mul(self.frame,self.value))
        self.refresh()


class AABBEditor(VisualEditorBase):
    """Edits a 3D axis-aligned bounding box ``(bmin,bmax)``.

    If ``frame`` is given, then it is :mod:`klampt.math.se3` element, and the
    input and output are measured with respect to that frame.  However, the
    editing is done in world coordinates.
    """
    def __init__(self, name : str, value : Tuple[Vector3,Vector3], description : str, world, frame : RigidTransform=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.aabbposer = AABBPoser()
        self.aabbposer.set(value[0],value[1])
        if frame is not None:
            self.aabbposer.setFrame(frame[0],frame[1])
        self.addWidget(self.aabbposer)
   
    def instructions(self):
        return 'Right-click and drag on the widget to set the box'

    def mousefunc(self,button,state,x,y):
        if self.aabbposer.hasFocus():
            self.value = self.aabbposer.get()
        return VisualEditorBase.mousefunc(self,button,state,x,y)

    def update_gui_from_value(self):
        self.aabbposer.set(self.value[0],self.value[1])
        self.refresh()


class SphereEditor(VisualEditorBase):
    """Edits a sphere represented by a tuple ``(center,radius)``.

    If ``frame`` is given, then it is :mod:`klampt.math.se3` element, and the
    input and output are measured with respect to that frame.  However, the
    editing is done in world coordinates.
    """
    def __init__(self, name : str, value : Tuple[Vector3,float], description : str, world : WorldModel, frame : RigidTransform=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame is None else frame
        self.sphereposer = SpherePoser()
        self.sphereposer.set(se3.apply(frame,value[0])+[value[1]])
        self.addWidget(self.sphereposer)
   
    def instructions(self):
        return 'Right-click and drag on the widget to set the sphere'

    def mousefunc(self,button,state,x,y):
        if self.sphereposer.hasFocus():
            cr = self.sphereposer.get()
            self.value = (se3.apply(se3.inv(self.frame),cr[:3]),cr[3])
        return VisualEditorBase.mousefunc(self,button,state,x,y)

    def update_gui_from_value(self):
        self.sphereposer.set(se3.apply(self.frame,self.value[0])+[self.value[1]])
        self.refresh()


class GeometricPrimitiveEditor(VisualEditorBase):
    """Edits a :class:`GeometricPrimitive`. 

    Can only handle Point, Sphere, AABB, and Box types for now.

    If ``frame`` is given, then it is :mod:`klampt.math.se3` element, and the
    input and output are measured with respect to that frame.  However, the
    editing is done in world coordinates.
    """
    def __init__(self,name,value,description,world,frame=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.frame = se3.identity() if frame is None else frame
        if value.type == 'Point':
            self.poser = PointPoser()
        elif value.type == 'Sphere':
            self.poser = SpherePoser()
        elif value.type == 'AABB':
            self.poser = AABBPoser()
        elif value.type == 'Box':
            self.poser = BoxPoser()
        else:
            raise NotImplementedError("Can't edit GeometricPrimitive of type "+value.type+" yet")
        self.update_gui_from_value()
        self.addWidget(self.poser)

    def instructions(self):
        return 'Right-click and drag on the widget to edit the primitive'

    def mousefunc(self,button,state,x,y):
        if self.poser.hasFocus():
            if self.value.type == 'Point':
                p = self.poser.get()
                self.value.setPoint(se3.apply(se3.inv(self.frame),p))
            elif self.value.type == 'Sphere':
                cr = self.poser.get()
                self.value.setSphere(se3.apply(se3.inv(self.frame),cr[:3]),cr[3])
            elif self.value.type == 'AABB':
                bmin,bmax = self.poser.get()
                self.value.setAABB(bmin,bmax)
            elif self.value.type == 'Box':
                Tw = self.poser.getTransform()
                Rl,tl = se3.mul(se3.inv(self.frame),Tw)
                dims = self.poser.getDims()
                self.value.setBox(tl,Rl,dims)
            else:
                raise NotImplementedError("Can't edit GeometricPrimitive of type "+self.value.type+" yet")
        return VisualEditorBase.mousefunc(self,button,state,x,y)

    def update_gui_from_value(self):
        if self.value.type == 'Point':
            c = self.value.properties[0:3]
            self.poser.set(se3.apply(self.frame,c))
            self.poser.setAxes(self.frame[0])
        elif self.value.type == 'Sphere':
            c = self.value.properties[0:3]
            r = self.value.properties[3]
            self.poser.set(se3.apply(self.frame,c) + [r])
        elif self.value.type == 'AABB':
            bmin = self.value.properties[0:3]
            bmax = self.value.properties[3:6]
            self.poser.set(bmin,bmax)
            self.poser.setFrame(self.frame[0],self.frame[1])
        elif self.value.type == 'Box':
            t = self.value.properties[0:3]
            R = self.value.properties[3:12]
            dims = self.value.properties[12:15]
            Rw,tw = se3.mul(self.frame,(R,t))
            self.poser.set(Rw,tw,dims)
        else:
            raise NotImplementedError("Can't edit GeometricPrimitive of type "+self.value.type+" yet")
        self.refresh()


class ObjectTransformEditor(VisualEditorBase):
    """Edits rigid object transforms. 

    ``value`` is the :mod:`klampt.math.se3` transform.

    ``object`` is the :class:~klampt.RigidObjectModel`.

    """
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

    def update_gui_from_value(self):
        self.objposer.set(*self.value)
        self.refresh()


class WorldEditor(VisualEditorBase):
    """Edits poses of robots, rigid objects, and terrains in a world.

    .. note::
        Edits to robot and rigid object poses are done immediately, but
        since terrains do not have poses, their actual geometry needs to
        be updated.  To apply the edited poses to the terrain geometries, 
        call :meth:`update_value_from_gui` after the editor has been run.

    """
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
            self.add_widget(r)
        for r in self.objectPosers:
            self.add_widget(r)
        for r in self.terrainPosers:
            self.add_widget(r)

    def loadable(self):
        return False

    def savable(self):
        return True

    def display(self):
        #Override display handler since the widget draws the rigid objects and transforms
        for i in range(self.world.numTerrains()):
            #self.world.terrain(i).geometry().getCurrentTransform()
            self.world.terrain(i).appearance().drawWorldGL(self.world.terrain(i).geometry())
        self.klamptwidgetmaster.drawGL(self.viewport())
        return False

    def update_value_from_gui(self):
        """Applies the transforms to all the terrain geometries."""
        for i in range(self.world.numTerrains()):
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


class SensorEditor(RigidTransformEditor):
    """Edits a SimRobotSensor.

    Assumes that the sensor is attached to the first robot in the world, if
    ``world`` is provided.
    """
    def __init__(self,name,value,description,world):
        from klampt import SimRobotSensor
        from ..model import sensing
        assert isinstance(value,SimRobotSensor),"Need to provide SensorEditor with a SimRobotSensor"
        T = sensing.get_sensor_xform(value)
        link_frame = None
        if world is not None:
            robot = world.robot(0)
            link = int(value.getSetting('link'))
            link_frame = robot.link(link).getTransform()

        RigidTransformEditor.__init__(self,name,T,description,world,frame=link_frame)
        self.value = value
        self.measurements = []
        
    def initialize(self):
        res = RigidTransformEditor.initialize(self)
        self.refresh_visualization()
        return res

    def loadable(self):
        return False
    def savable(self):
        return False
    
    def add_dialog_items(self,parent,ui='qt'):
        layout = QHBoxLayout(parent)
        self.selectionList = QListWidget()
        settings = self.value.settings()
        self.settingsOrder = sorted([k for k in settings if k not in ['Tsensor','rate']])
        for k in self.settingsOrder:
            self.selectionList.addItem(k)
        self.selectionList.currentRowChanged.connect(self.on_setting_selected)
        self.editBox = QLineEdit("value")
        self.editBox.editingFinished.connect(self.on_setting_edited)
        self.on_setting_selected()
        layout.addWidget(self.selectionList)
        layout.addWidget(self.editBox)
        
    def refresh_visualization(self):
        self.value.kinematicSimulate(0.01)
        self.measurements = self.value.getMeasurements()
        self.refresh()
    
    def display(self):
        from ..model import sensing
        sensor = self.value
        self.value = sensing.get_sensor_xform(sensor)
        RigidTransformEditor.display(self)
        self.value = sensor

        self.value.drawGL(self.measurements)

    def on_setting_selected(self):
        if self.selectionList.currentRow() < 0:
            self.editBox.setText('')
            return
        item = self.settingsOrder[self.selectionList.currentRow()]
        value = self.value.getSetting(item)
        self.editBox.setText(value)

    def on_setting_edited(self):
        if self.selectionList.currentRow() < 0:
            return
        value = str(self.editBox.text())
        item = self.settingsOrder[self.selectionList.currentRow()]
        try:
            self.value.setSetting(item,value)
            self.refresh_visualization()
        except Exception as e:
            QMessageBox.warning(self,"Invalid sensor setting","Sensor does not accept setting: "+str(e))
            self.editBox.setText(self.value.getSetting(item))

    def instructions(self):
        return 'Right-click and drag on the widget to pose the sensor'

    def mousefunc(self,button,state,x,y):
        from ..model import sensing
        sensor = self.value
        self.value = sensing.get_sensor_xform(sensor)
        res = RigidTransformEditor.mousefunc(self,button,state,x,y)
        if res:
            sensing.set_sensor_xform(sensor,self.value)
            self.value = sensor
            self.refresh_visualization()
            return True
        self.value = sensor
        return False

    def motionfunc(self,x,y,dx,dy):
        from ..model import sensing
        sensor = self.value
        self.value = sensing.get_sensor_xform(sensor)
        res = RigidTransformEditor.motionfunc(self,x,y,dx,dy)
        sensing.set_sensor_xform(sensor,self.value)
        self.value = sensor
        return res

    def update_value_from_gui(self):
        sensor = self.value
        from ..model import sensing
        self.value = sensing.get_sensor_xform(sensor)
        RigidTransformEditor.update_value_from_gui(self)
        sensing.set_sensor_xform(sensor,self.value)
        self.value = sensor

    def update_gui_from_value(self):
        sensor = self.value
        from ..model import sensing
        self.value = sensing.get_sensor_xform(sensor)
        RigidTransformEditor.update_gui_from_value(self)
        self.value = sensor


#Qt stuff
global _has_qt
_has_qt = False
try:
    from PyQt5.QtCore import *
    from PyQt5.QtGui import *
    from PyQt5.QtWidgets import *
    _has_qt = True
except ImportError:
    try:
        from PyQt4.QtCore import *
        from PyQt4.QtGui import *
        _has_qt = True
    except ImportError:
        _has_qt = False

if _has_qt:
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

            self.loadButton = QPushButton("Load...")
            self.saveButton = QPushButton("Save...")
            self.loadsave = QFrame()
            self.loadButton.clicked.connect(self.load)
            self.saveButton.clicked.connect(self.save)

            self.extraDialog = QFrame()
            self.extraDialog.setSizePolicy(QSizePolicy(QSizePolicy.Minimum,QSizePolicy.Minimum))
            self.topBoxLayout.addWidget(self.extraDialog)
            self.layout = QVBoxLayout()
            self.layout.addWidget(self.topBox)
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
            if editorObject.description is None:
                self.description.setText("")
            else:
                self.description.setText(editorObject.description)
            self.instructions.setText(editorObject.instructions())

            loadsavelayout = QHBoxLayout(self.loadsave)
            if editorObject.loadable():
                loadsavelayout.addWidget(self.loadButton)
            if editorObject.savable():
                loadsavelayout.addWidget(self.saveButton)
            if editorObject.loadable() or editorObject.savable():
                self.topBoxLayout.addWidget(self.loadsave)

            editorObject.add_dialog_items(self.extraDialog,ui='qt')

        def closeEvent(self,event):
            global _doexit
            reply = QMessageBox.question(self, 'Message',
                 "Are you sure you wish to quit the program?", QMessageBox.Yes | 
                 QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                _doexit = True
                event.accept()
            else:
                event.ignore() 
            
        def accept(self):
            global _my_dialog_res
            _my_dialog_res = True
            print("#########################################")
            print("klampt.vis: EditDialog accept")
            print("#########################################")
            self.editorObject.update_value_from_gui()
            return QDialog.accept(self)

        def reject(self):
            global _my_dialog_res
            _my_dialog_res = False
            print("#########################################")
            print("klampt.vis: EditDialog reject")
            print("#########################################")
            return QDialog.reject(self)

        def load(self):
            from ..io import resource
            from ..model import types
            type = types.object_to_type(self.editorObject.value)
            res = resource.load(type,'.')
            if res is not None:
                #TODO: check compatibility with world
                self.editorObject.value = res[1]
                self.editorObject.update_gui_from_value()

        def save(self):
            from ..io import resource
            self.editorObject.update_value_from_gui()
            resource.save(self.editorObject.value,'auto','.')
            


    def run(editorObject : VisualEditorBase) -> Tuple[bool,Any]:
        """
        Launches a visual editor.

        Args:
            editorObject (VisualEditorBase): some subclass of VisualEditorBase

        Returns:
            tuple: A pair (res,value) containing: 

                * res (bool):True / False if OK / Cancel was pressed, respectively, 
                * value: the return value of the editor object

        """
        assert isinstance(editorObject,VisualEditorBase),"Must provide a VisualEditorBase instance to vis.editors.run()"
        global _vis_id, _my_dialog_res

        old_vis_window = visualization.getWindow()
        if _vis_id is None:
            _vis_id = visualization.createWindow("Resource Editor")
        else:
            visualization.setWindow(_vis_id)
        visualization.setPlugin(editorObject)
        def makefunc(gl_backend):
            assert gl_backend is not None
            res = _EditDialog(gl_backend)
            res.setEditor(editorObject)
            return res
        visualization.customUI(makefunc)
        visualization.dialog()
        res,retVal = _my_dialog_res,editorObject.value
        
        if _doexit:
            visualization.kill()
            print("Exiting program.")
            exit(0)

        assert res is not None,"vis.editors.run(): There may be something wrong with the vis module not catching the customUI, or terminating from a prior dialog?"

        visualization.setPlugin(None)
        visualization.customUI(None)
        if old_vis_window is not None:
            print("Returning from editor to old window",old_vis_window)
            visualization.setWindow(old_vis_window)
        else:
            print("Returning from editor to window 0")
            visualization.setWindow(0)

        print("vis.editors.run(): Result",res,"return value",retVal)
        return res,retVal
else:
    def run(editorObject : VisualEditorBase) -> Tuple[bool,Any]:
        """
        Args:
            editorObject (VisualEditorBase): some subclass of VisualEditorBase

        Returns:
            res,value (bool, value pair): 

                * res is True / False if OK / Cancel was pressed, respectively, 
                * value is the return value of the editor object
        """
        raise ValueError("Unable to perform visual editing without PyQt")
