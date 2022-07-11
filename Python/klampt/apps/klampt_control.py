from klampt import *
from klampt import WidgetSet,RobotPoser,PointPoser,TransformPoser
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.robotinfo import RobotInfo,_resolve_file
from klampt.io import loader
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.vis import gldraw
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.networkrobotinterface import XMLRPCRobotInterfaceClient,XMLRPCRobotInterfaceServer
from klampt.model import trajectory, ik, types
import math
import time
import sys
import os
import weakref
import pkg_resources

from klampt.control.robotinterfaceutils import StepContext,klamptCartesianPosition

vis.init("PyQt5")

from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import uic
from OpenGL import GL


class ControllerStepContext:
    def __init__(self,gui):
        if not isinstance(gui,ControllerGUI):
            raise ValueError("need to call this with a GUI")
        self.gui = gui
    def __enter__(self):
        if not self.gui.advancing:
            return self
        try:
            self.gui.controller.beginStep()  
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.gui.addException("beginStep",e)
            self.gui.advancing = False
        return self
    def __exit__(self,type,value,tb):
        if not self.gui.advancing:
            return
        try:
            self.gui.controller.endStep()  
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.gui.addException("endStep",e)
            self.gui.advancing = False


class ControllerGLPlugin(GLWidgetPlugin):
    def __init__(self, world : WorldModel, controller : RobotInterfaceBase, gui : 'ControllerGUI'):
        GLWidgetPlugin.__init__(self)

        self.world = world
        self.controller = controller
        self.gui = gui
        #read commanded configuration
        self.controller.beginStep()
        try:
            q = self.controller.commandedPosition()
        except Exception as e:
            try:
                q = self.controller.sensedPosition()
            except Exception:
                q = None
        self.controller.endStep()
        if q is not None and all(v is not None for v in q):
            qklampt = self.controller.configToKlampt(q)
            world.robot(0).setConfig(qklampt)
        self.robotPoser = RobotPoser(world.robot(0))
        self.toolCoordinatesPoser = PointPoser()
        self.toolCoordinatesPoser.set([0,0,0])
        self.cartesianGoalPoser = TransformPoser()
        self.cartesianGoalPoser.set(*se3.identity())
        self.addWidget(self.robotPoser)
        self.addWidget(self.toolCoordinatesPoser)
        self.addWidget(self.cartesianGoalPoser)
        self.klamptwidgetmaster.enable(self.toolCoordinatesPoser,False)
        self.klamptwidgetmaster.enable(self.cartesianGoalPoser,False)
        self.cartesianControlEnabled = False
        self.toolCoordinates = None
        self.endEffectorLink = None

        robot = world.robot(0)
        self.qcmd = None
        self.qsns = None
        self.Tcmd = None
        self.Tsns = None
        try:
            self.dt = 1.0/controller.controlRate()
        except NotImplementedError:
            self.dt = 0.1
        self.tNextIdle = 0
        self.showPreset = False
        self.preset = None
        
    def enableCartesianWidget(self,enabled,link=None):
        self.cartesianControlEnabled = enabled
        self.endEffectorLink = link
        self.klamptwidgetmaster.enable(self.cartesianGoalPoser,enabled)
        self.klamptwidgetmaster.enable(self.toolCoordinatesPoser,False)
        if not enabled:
            self.Tcmd,self.Tsns = None,None
    
    def setToolCoordinates(self,tool):
        self.toolCoordinates = tool
        self.toolCoordinatesPoser.set(tool)  #TODO: convert to end effector link frame
    
    def setCartesianPoses(self,Tsns,Tcmd, updatePoser=True):
        self.Tcmd = Tcmd
        self.Tsns = Tsns
        if updatePoser:
            if Tcmd is not None:
                self.cartesianGoalPoser.set(*Tcmd)
            else:
                self.cartesianGoalPoser.set(*Tsns)

    def setTargetConfig(self,qtgt):
        """Sets the target configuration and updates the Cartesian pose."""
        if qtgt is None:
            qtgt = self.robotPoser.get()
        else:
            self.robotPoser.set(qtgt)
        if self.cartesianControlEnabled:
            if self.toolCoordinates is None:
                print("Warning: cartesianControlEnabled=True but toolCoordinates=None?")
                return
            robot = self.world.robot(0)
            robot.setConfig(qtgt)
            ee = robot.link(self.endEffectorLink)
            t = ee.getWorldPosition(self.toolCoordinates)
            R = ee.getTransform()[0] 
            self.cartesianGoalPoser.set(R,t)
    
    def setTargetPose(self,Ttgt, q0 = None, activeLinks = None):
        """Sets the target pose and update the IK.
        
        If Ttgt = None, gets the target from the goal poser.
        """
        if not self.cartesianControlEnabled: return
        if Ttgt is None:
            Ttgt = self.cartesianGoalPoser.get()
        else:
            self.cartesianGoalPoser.set(*Ttgt)
        if q0 is None:
            q0 = self.robotPoser.get()
        tool_coordinates = self.toolCoordinates
        link_origin_transform = vectorops.sub(Ttgt[1],so3.apply(Ttgt[0],tool_coordinates))

        robot = self.world.robot(0)
        obj = ik.objective(robot.link(activeLinks[-1]),R=Ttgt[0],t=link_origin_transform)
        solver = ik.solver(obj)
        if activeLinks is not None:
            solver.setActiveDofs(activeLinks)
        robot.setConfig(q0)
        res = solver.solve()
        if res:
            self.robotPoser.set(robot.getConfig())

    def display(self):
        robot = self.world.robot(0)
        #draw robot poser in transparent yellow
        oldcolors = [robot.link(i).appearance().getColor() for i in range(robot.numLinks())]
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(1,1,0,0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(*oldcolors[i])

        #this line will draw the world
        if self.qsns is not None:
            robot.setConfig(self.qsns)
        self.world.drawGL()

        #draw commanded configuration in transparent green
        if self.qcmd is not None:
            for i in range(robot.numLinks()):
                robot.link(i).appearance().setColor(0,1,0,0.5)
        
            robot.setConfig(self.qcmd)
            robot.drawGL()
            #restore colors
            for i in range(robot.numLinks()):
                robot.link(i).appearance().setColor(*oldcolors[i])

        gldraw.xform_widget(se3.identity(),0.2,0.01, fancy=True)

        #draw transform frames
        if self.Tsns is not None:
            len = 0.3
            GL.glPushMatrix()
            cols = list(zip(*se3.homogeneous(self.Tsns)))
            flattened = sum([list(v) for v in cols],[])
            GL.glMultMatrixf(flattened)
            GL.glDisable(GL.GL_LIGHTING)
            GL.glBegin(GL.GL_LINES)
            GL.glColor3f(1,0.5,0)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(len,0,0)
            GL.glColor3f(0.5,1,0)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(0,len,0)
            GL.glColor3f(0.5,0.5,1)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(0,0,len)
            GL.glEnd()
            GL.glPopMatrix()
            GL.glEnable(GL.GL_LIGHTING)
        
        if self.Tcmd is not None:
            len = 0.3
            GL.glPushMatrix()
            cols = list(zip(*se3.homogeneous(self.Tcmd)))
            flattened = sum([list(v) for v in cols],[])
            GL.glMultMatrixf(flattened)
            GL.glDisable(GL.GL_LIGHTING)
            GL.glBegin(GL.GL_LINES)
            GL.glColor3f(1,0,0.5)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(len,0,0)
            GL.glColor3f(0,1,0.5)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(0,len,0)
            GL.glColor3f(0,0,1)
            GL.glVertex3f(0,0,0)
            GL.glVertex3f(0,0,len)
            GL.glEnd()
            GL.glPopMatrix()
            GL.glEnable(GL.GL_LIGHTING)
        
        if self.showPreset:
            if isinstance(self.preset,tuple):
                GL.glColor3f(1,0,1,0.5)
                gldraw.xform_widget(self.preset,0.1,0.01,lighting=False)
            elif isinstance(self.preset,list):
                GL.glEnable(GL.GL_LIGHTING)
                robot.setConfig(self.preset)
                for i in range(robot.numLinks()):
                    robot.link(i).appearance().setColor(1,0,1,0.5)
                robot.drawGL()
                #restore colors
                for i in range(robot.numLinks()):
                    robot.link(i).appearance().setColor(*oldcolors[i])
            elif isinstance(self.preset,trajectory.RobotTrajectory):
                #draw line strip
                GL.glLineWidth(4.0)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_BLEND)
                GL.glColor4f(1,0,1,0.5)
                GL.glBegin(GL.GL_LINE_STRIP)
                ee = robot.link(robot.numLinks()-1)
                for m in self.preset.milestones:
                    robot.setConfig(m)
                    eepos = ee.getWorldPosition(self.toolCoordinates if self.toolCoordinates is not None else (0,0,0))
                    GL.glVertex3fv(eepos)
                GL.glEnd()
            elif isinstance(self.preset,trajectory.SE3Trajectory):
                #draw line strip
                GL.glLineWidth(4.0)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_BLEND)
                GL.glColor4f(1,0,1,0.5)
                GL.glBegin(GL.GL_LINE_STRIP)
                for m in self.preset.milestones:
                    R = m[:9]
                    t = m[9:12]
                    GL.glVertex3fv(t)
                GL.glEnd()
            elif isinstance(self.preset,trajectory.Trajectory):
                #draw line strip
                GL.glLineWidth(4.0)
                GL.glDisable(GL.GL_LIGHTING)
                GL.glEnable(GL.GL_BLEND)
                GL.glColor4f(1,0,1,0.5)
                GL.glBegin(GL.GL_LINE_STRIP)
                for m in self.preset.milestones:
                    GL.glVertex3fv(m[:3])
                GL.glEnd()
        

    def idle(self):
        self.gui.onIdle()

        t = time.time()
        if t < self.tNextIdle:
            self.idlesleep(self.tNextIdle-t)
        if t > self.tNextIdle + self.dt:
            self.tNextIdle = t  #overrun, immediate refresh
            self.idlesleep(0)
        else:
            self.tNextIdle += self.dt
        return GLWidgetPlugin.idle(self)


class ControllerGUI(QtWidgets.QMainWindow):
    def __init__(self, glwidget : ControllerGLPlugin, robotinfo:RobotInfo, controller:RobotInterfaceBase, plugin, parent=None):
        QtWidgets.QMainWindow.__init__(self,parent)
        self.robotinfo = robotinfo           # type: RobotInfo
        self.controller = controller         # type: RobotInterfaceBase
        self.activeController = controller   # type: RobotInterfaceBase
        self.advancing = True 
        self.activePart = None
        self.glwidget = glwidget 
        self.plugin = plugin                 # type: ControllerGLPlugin
        self.qsns = None   #controller config
        self.qcmd = None   #controller config
        self.robot = controller.klamptModel()
        self.configPresets = dict()
        self.trajectoryPresets = dict()
        self.cartesianPosePresets = dict()
        self.cartesianTrajectoryPresets = dict()
        assert self.robot is not None,"klamptModel() method must be implemented for klampt_control to work"
        self.idleCount = 0
        # Splitter to show 2 views in same widget
        self.splitter = QtWidgets.QSplitter()
        self.panel = QtWidgets.QWidget()
        ui_filename = pkg_resources.resource_filename('klampt','data/klampt_control.ui')
        uic.loadUi(ui_filename, self.panel)
        self.glwidget.setFixedSize(QtWidgets.QWIDGETSIZE_MAX,QtWidgets.QWIDGETSIZE_MAX)
        self.glwidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Expanding))
        self.glwidget.adjustSize()
        self.glwidget.refresh()
        self.splitter.setHandleWidth(7)
        self.setCentralWidget(self.splitter)
        
        self.suppressGuiEvents = False
        self.panel.centralTabWidget.currentChanged.connect(self.onTabChange)
        self.panel.controllerLabel.setText("Controller: {}".format(controller))
        try:
            rate = controller.controlRate()
            self.rateStr = '{} Hz'.format(rate)
        except NotImplementedError:
            self.rateStr = 'Unknown (controlRate() method not implemented)'
        self.panel.rateLabel.setText("Rate: {}".format(self.rateStr))
    
        self.controllerParts = None
        self.partList = self.panel.partList
        try:
            parts = controller.parts()
            self.controllerParts = []
            self.partList.addItem("Whole robot")
            self.controllerParts.append(None)
            for k in parts:
                if k is not None:
                    self.controllerParts.append(k)
                    self.partList.addItem(str(k))
            self.partList.setToolTip("Results from parts() method")
        except NotImplementedError:
            self.partList.setEnabled(False)
            self.partList.setToolTip("parts() method not implemented")
        self.partList.currentIndexChanged.connect(self.onPartChange)
        
        #find map from parts to RobotInfo's end effectors
        self.partsToEEs = dict()
        self.unmatchedEEs = set(eename for eename in self.robotinfo.endEffectors)
        for part in self.controllerParts:
            eematches = []
            partIndices = self.controller.indices(part)
            for eename,ee in self.robotinfo.endEffectors.items():
                activeDrivers = self.robotinfo.toDriverIndices(ee.activeLinks)
                if partIndices == activeDrivers:
                    if part is None:
                        print("Controller is a match for end effector",eename)
                    else:
                        print("Controller for part",part,"is a match for end effector",eename)
                    eematches.append(eename)
                    self.unmatchedEEs.remove(eename)
            self.partsToEEs[part] = eematches

        #top region
        self.partList.currentIndexChanged.connect(self.onPartChange)
        self.panel.resetButton.clicked.connect(self.onReset)
        self.panel.estopButton.clicked.connect(self.onEstop)
        self.panel.softStopButton.clicked.connect(self.onSoftStop)
        
        #joint control panel
        self.panel.configClipboardButton.clicked.connect(self.onClipConfig)
        self.jointScrollAreaLayout = self.panel.jointScrollAreaLayout
        self.jointScrollAreaLayout.setAlignment(QtCore.Qt.AlignTop)
        self.jointForms = []
        self.panel.moveToPositionButton.clicked.connect(self.onMoveToPosition)
        self.panel.setPositionButton.clicked.connect(self.onSetPosition)
        self.panel.setVelocityButton.clicked.connect(self.onSetVelocity)
        
        #cartesian panel
        self.panel.moveToCartesianPositionButton.clicked.connect(self.onMoveToCartesianPosition)
        self.panel.setCartesianPositionButton.clicked.connect(self.onSetCartesianPosition)
        self.panel.setCartesianVelocityButton.clicked.connect(self.onSetCartesianVelocity)
        self.panel.endEffectorList.currentIndexChanged.connect(self.onEndEffectorChanged)
        self.panel.toolXSpinBox.valueChanged.connect(self.onToolCoordinatesChanged)
        self.panel.toolYSpinBox.valueChanged.connect(self.onToolCoordinatesChanged)
        self.panel.toolZSpinBox.valueChanged.connect(self.onToolCoordinatesChanged)

        #logging panel

        #motion planning panel

        #presets panel
        self.panel.loadPresetsDirButton.clicked.connect(self.onLoadPresetsDir)
        self.panel.loadPresetButton.clicked.connect(self.onLoadPreset)
        if self.robotinfo.resourceDir is not None:
            resourceDir = _resolve_file(self.robotinfo.resourceDir,self.robotinfo.filePaths)
            for item in os.listdir(resourceDir):
                self.loadPreset(os.path.join(resourceDir,item))
            self.refreshPresetLists()
        self.panel.configurationList.itemSelectionChanged.connect(lambda : self.onPresetSelected(self.panel.configurationList))
        self.panel.trajectoryList.itemSelectionChanged.connect(lambda : self.onPresetSelected(self.panel.trajectoryList))
        self.panel.cartesianPoseList.itemSelectionChanged.connect(lambda : self.onPresetSelected(self.panel.cartesianPoseList))
        self.panel.cartesianTrajectoryList.itemSelectionChanged.connect(lambda : self.onPresetSelected(self.panel.cartesianTrajectoryList))
        self.panel.sendPresetToPoser.clicked.connect(self.onSendPresetToPoser)
        self.panel.moveToPreset.clicked.connect(self.onMoveToPreset)
        self.panel.executePreset.clicked.connect(self.onExecutePreset)
        
        self.errorText = []
        self.panel.errorTextarea.setPlainText("")
        
        self.splitter.addWidget(self.panel)
        self.splitter.addWidget(self.glwidget)
        self.splitter.setSizes([100,640])

        self.updateActiveController(None,controller)
        for ee in self.unmatchedEEs:
            self.addError("End effector {} in RobotInfo does not match a part".format(ee))
    
    def onTabChange(self,index):
        if index == 3:
            #presets
            self.plugin.showPreset = True
        else:
            self.plugin.showPreset = False
    
    def updateActiveController(self, activeName:str, active:RobotInterfaceBase):
        self.activePart = activeName
        self.activeController = active
        self.updateStatus()
        nj = active.numJoints()
        #clear scroll area
        while True:
            w = self.jointScrollAreaLayout.takeAt(0)
            if not w:
                break
        indices = self.controller.indices(self.activePart)
        robotModel = self.controller.klamptModel()
        qmin,qmax = [],[]
        for i in range(robotModel.numDrivers()):
            a,b = robotModel.driver(i).getLimits()
            if robotModel.getJointType(robotModel.driver(i).getAffectedLink()) == 'spin':
                a = 0
                b = math.pi*2
            qmin.append(a)
            qmax.append(b)
        if len(indices) != nj:
            self.addError("Part's numJoints() returns {} joints, while controller's indices({}) returns {}".format(nj,self.activePart,len(indices)))
            return
        for form in self.jointForms:
            form.setParent(None)
            del form
        self.jointForms = []
        for i in range(nj):
            j = indices[i]
            jointWidget = QtWidgets.QWidget()
            ui_filename = pkg_resources.resource_filename('klampt','data/joint_edit.ui')
            uic.loadUi(ui_filename, jointWidget)
            if math.isinf(qmin[j]):
                self.addError("Infinite joint limit on joint {}".format(self.controller.jointName(j)))
                if math.isinf(qmax[j]):
                    qmin[j] = -1
                    qmax[j] = 1
                else:
                    qmin[j] = qmax[j]-1
            elif math.isinf(qmax[j]):
                self.addError("Infinite joint limit on joint {}".format(self.controller.jointName(j)))
                qmax[j] = qmin[j] + 1
            if qmax[j] == qmin[j]:
                print("Warning, joint {} has equal minimum and maximum {}, must be frozen?".format(self.controller.jointName(j),qmin[j]))
                input()
            jointWidget.sensedSpinBox.setMinimum(qmin[j])
            jointWidget.sensedSpinBox.setMaximum(qmax[j])
            jointWidget.commandSpinBox.setMinimum(qmin[j])
            jointWidget.commandSpinBox.setMaximum(qmax[j])
            jointWidget.targetSpinBox.setMinimum(qmin[j])
            jointWidget.targetSpinBox.setMaximum(qmax[j])
            jointWidget.commandSlider.valueChanged.connect(lambda value,idx=i:self.onCommandChange(idx,qmin[idx] + value/1024.0*(qmax[idx]-qmin[idx])))
            jointWidget.commandSpinBox.valueChanged.connect(lambda value,idx=i:self.onCommandChange(idx,value))
            jointWidget.targetSlider.valueChanged.connect(lambda value,idx=i:self.onTargetChange(idx,qmin[idx] + value/1024.0*(qmax[idx]-qmin[idx])))
            jointWidget.targetSpinBox.valueChanged.connect(lambda value,idx=i:self.onTargetChange(idx,value))
            self.jointForms.append(jointWidget)
            try:
                name = active.jointName(i)
                jointWidget.nameLabel.setText(str(name))
            except NotImplementedError:
                jointWidget.nameLabel.setText('jointName({}) not implemented'.format(i))
            self.jointScrollAreaLayout.addWidget(jointWidget)
        
        eematches = self.partsToEEs[activeName]
        self.panel.endEffectorList.clear()
        for eename in eematches:
            self.panel.endEffectorList.addItem(eename)    

        if len(eematches) == 0:
            self.panel.endEffectorList.setEnabled(False)
            self.selectedEndEffector = None
            self.panel.toolXSpinBox.setValue(0)
            self.panel.toolYSpinBox.setValue(0)
            self.panel.toolZSpinBox.setValue(0)
        else:
            self.panel.endEffectorList.setEnabled(True)
            self.selectedEndEffector = eematches[0]
            self.onEndEffectorChanged(0)
        
        #setup cartesian UI items
        cartesianEnabled = False
        cartesianLink = None
        self.advancing = True
        try:
            with ControllerStepContext(self):
                tool = active.getToolCoordinates()
            cartesianEnabled = True
            cartesianLink = self.robot.driver(self.controller.indices(self.activePart)[-1]).getAffectedLink()
        except NotImplementedError as e:
            #may need to force tool coordinates on the item
            if self.selectedEndEffector is not None:
                try:
                    tool = self.setEndEffectorToolCoordinates(self.selectedEndEffector)
                    print("setToolCoordinates worked... waiting to see if we get a cartesian position")
                    self.activeController.beginStep()
                    self.activeController.endStep()
                    print(self.controller._settings.to_json())
                    time.sleep(0.01)
                    cartesianEnabled = True
                except NotImplementedError:
                    self.addError("Unable to set tool coordinates for end effector {}, part {}".format(eename,self.activePart))
                except Exception as e:
                    self.addException("setToolCoordinates",e)
            else:
                #disable cartesian control
                print("getToolCoordinates is not implemented by {}, disabling cartesian control".format(str(active)))
                cartesianEnabled = False
        except Exception as e:
            self.addException("getToolCoordinates",e)
        if cartesianEnabled:
            print("Trying to get one of sensedCartesianPosition or commandedCartesianPosition")
            with ControllerStepContext(self):
                try:
                    Tsns = active.sensedCartesianPosition()
                    try:
                        Tcmd = active.commandedCartesianPosition()
                    except NotImplementedError:
                        Tcmd = None
                    except Exception as e:
                        self.addException("commandedCartesianPosition",e)
                    self.plugin.setToolCoordinates(tool)
                    self.plugin.setCartesianPoses(Tsns,Tcmd)
                except ValueError as e:
                    import traceback
                    traceback.print_exc()
                    self.addException("sensedCartesianPosition",e)
                    cartesianEnabled = False
                except Exception as e:
                    import traceback
                    traceback.print_exc()
                    self.addException("sensedCartesianPosition",e)
                    cartesianEnabled = False
        
        self.panel.endEffectorList.setEnabled(cartesianEnabled)
        self.panel.moveToCartesianPositionButton.setEnabled(cartesianEnabled)
        self.panel.setCartesianPositionButton.setEnabled(cartesianEnabled)
        self.panel.setCartesianVelocityButton.setEnabled(cartesianEnabled)
        self.panel.toolXSpinBox.setEnabled(cartesianEnabled)
        self.panel.toolYSpinBox.setEnabled(cartesianEnabled)
        self.panel.toolZSpinBox.setEnabled(cartesianEnabled)
        self.panel.visualEditToolButton.setEnabled(cartesianEnabled)
        partname = 'whole robot' if self.activePart is None else 'part '+str(self.activePart)
        if cartesianEnabled:
            self.panel.cartesianInfoText.setText("Cartesian control is enabled for {}.".format(partname))
        else:
            self.panel.cartesianInfoText.setText("Cartesian control is not enabled for {}.\nSelect a different part or reconfigure the controller.".format(partname))
        self.plugin.enableCartesianWidget(cartesianEnabled,cartesianLink)
    
    def onIdle(self):
        with ControllerStepContext(self):
            if not self.advancing:
                return
        
            try:
                clock = self.controller.clock()
                vis.add("clock","%.3f"%clock,position=(10,10))
            except NotImplementedError:
                vis.add("clock","clock() method not implemented",position=(10,10))
                self.addNotImplementedError('clock')
            except Exception as e:
                self.addException('clock',e)

            try:
                status = self.activeController.status()    
                self.panel.statusLabel.setText('Status: '+status)
            except NotImplementedError:
                self.panel.statusLabel.setText('status() method not implemented by {}'.format(self.activeController))
            except Exception as e:
                self.addException("status",e)

            try:
                qsns = self.controller.sensedPosition()
            except NotImplementedError:
                qsns = None
                self.addNotImplementedError('sensedPosition')
            try:
                qcmd = self.controller.commandedPosition()
            except NotImplementedError:
                qcmd = None
                self.addNotImplementedError('commandedPosition')
            self.qsns = qsns
            self.qcmd = qcmd
            self.plugin.qsns = self.controller.configToKlampt(qsns) if qsns is not None else None
            self.plugin.qcmd = self.controller.configToKlampt(qcmd) if qcmd is not None else None

            if self.plugin.cartesianControlEnabled:  #update cartesian poses
                try:
                    Tsns = self.activeController.sensedCartesianPosition()
                    try:
                        Tcmd = self.activeController.commandedCartesianPosition()
                    except NotImplementedError:
                        Tcmd = None
                    except Exception as e:
                        self.addException("commandedCartesianPosition",e)
                    self.plugin.setCartesianPoses(Tsns,Tcmd,False)
                except NotImplementedError:
                    cartesianEnabled = False
                except ValueError:
                    cartesianEnabled = False
                except Exception as e:
                    self.addException("sensedCartesianPosition",e)
                if self.plugin.robotPoser.hasFocus():
                    #change poser
                    self.plugin.setTargetConfig(None)
                    #q = self.plugin.robotPoser.get()
                    # T = None
                    # qcontroller = self.controller.configFromKlampt(q)
                    # activeIndices = self.controller.indices(self.activePart)
                    
                    # tool_coordinates = self.activeController.getToolCoordinates()
                    # T = klamptCartesianPosition(self.robot,qcontroller,activeIndices,tool_coordinates,'world')
                    # self.plugin.cartesianGoalPoser.set(*T)
                if self.plugin.cartesianGoalPoser.hasFocus():
                    #solve IK problem to update the robot poser
                    q0 = self.controller.configToKlampt(self.controller.commandedPosition())
                    activeIndices = self.controller.indices(self.activePart)
                    activeLinks = sum([self.robot.driver(i).getAffectedLinks() for i in activeIndices],[])
                    self.plugin.setTargetPose(None,q0,activeLinks)
                    # Tgoal = self.plugin.cartesianGoalPoser.get()
                    # q0 = self.controller.configToKlampt(self.controller.commandedPosition())
                    # activeIndices = self.controller.indices(self.activePart)
                    # activeLinks = sum([self.robot.driver(i).getAffectedLinks() for i in activeIndices],[])
                    # tool_coordinates = self.activeController.getToolCoordinates()
                    # link_origin_transform = vectorops.sub(Tgoal[1],so3.apply(Tgoal[0],tool_coordinates))

                    # obj = ik.objective(self.robot.link(activeLinks[-1]),R=Tgoal[0],t=link_origin_transform)
                    # solver = ik.solver(obj)
                    # solver.setActiveDofs(activeLinks)
                    # self.robot.setConfig(q0)
                    # res = solver.solve()
                    # if res:
                    #     self.plugin.robotPoser.set(self.robot.getConfig())
                    pass
        
        self.idleCount += 1
        if self.idleCount % 10 == 1:  #TODO: sync this with the control rate to get reasonable visual updates?
            self.suppressGuiEvents = True
            indices = self.controller.indices(self.activePart)
            if len(indices) != len(self.jointForms):
                self.addError("Invalid result from indices({}): {} should be {}".format(self.activePart,len(indices),len(self.jointForms)))
                self.suppressGuiEvents = False
                return
            if self.qsns is not None:
                for i,j in enumerate(indices):
                    vmin = self.jointForms[i].sensedSpinBox.minimum()
                    vmax = self.jointForms[i].sensedSpinBox.maximum()
                    if vmin == vmax: 
                        print("Frozen joint?",i)
                        continue
                    self.jointForms[i].sensedSlider.setValue(int(1024*(self.qsns[i]-vmin)/(vmax-vmin)))
                    self.jointForms[i].sensedSpinBox.setValue(self.qsns[i])
            if self.qcmd is not None:
                for i,j in enumerate(indices):
                    vmin = self.jointForms[i].sensedSpinBox.minimum()
                    vmax = self.jointForms[i].sensedSpinBox.maximum()
                    if vmin == vmax: 
                        print("Frozen joint?",i)
                        continue
                    self.jointForms[i].commandSlider.setValue(int(1024*(self.qcmd[i]-vmin)/(vmax-vmin)))
                    self.jointForms[i].commandSpinBox.setValue(self.qcmd[i])
            qtgt_rob = self.plugin.robotPoser.get()
            qtgt = self.controller.configFromKlampt(qtgt_rob)
            for i,j in enumerate(indices):
                vmin = self.jointForms[i].sensedSpinBox.minimum()
                vmax = self.jointForms[i].sensedSpinBox.maximum()
                if vmin == vmax: 
                    print("Frozen joint?",i)
                    continue
                self.jointForms[i].targetSlider.setValue(int(1024*(qtgt[j]-vmin)/(vmax-vmin)))
                self.jointForms[i].targetSpinBox.setValue(qtgt[i])
            self.suppressGuiEvents = False

    def onPartChange(self,index):
        part = self.controllerParts[index]
        try:
            partController = self.controller.partInterface(part)
        except NotImplementedError:
            self.addNotImplementedError("partInterface")
            return
        except Exception as e:
            self.addException("partInterface",e)
            return
        self.updateActiveController(part,partController)
    
    def onReset(self):
        with ControllerStepContext(self):
            self.activeController.reset()
    
    def onEstop(self):
        with ControllerStepContext(self):
            self.activeController.estop()
    
    def onSoftStop(self):
        with ControllerStepContext(self):
            self.activeController.softStop()

    def onTargetChange(self,index,value):
        if self.suppressGuiEvents: return
        indices = self.controller.indices(self.activePart)
        j = indices[index]
        link = self.robot.driver(j).getAffectedLink()
        qtgt = self.plugin.robotPoser.get()
        self.robot.setConfig(qtgt)
        self.robot.driver(j).setValue(value)
        qtgt = self.robot.getConfig()
        self.plugin.setTargetConfig(qtgt)
    
    def onCommandChange(self,index,value):
        if self.suppressGuiEvents: return
        if self.qcmd is None: return
        if self.jointForms[index].lockButton.isChecked(): return
        if not self.advancing: return
        indices = self.controller.indices(self.activePart)
        j = indices[index]
        target = [v for v in self.qcmd]
        target[j] = value
        with ControllerStepContext(self):
            if self.advancing:
                try:
                    self.controller.setPosition(target)
                except NotImplementedError:
                    self.addNotImplementedError('setPosition')
                except Exception as e:
                    self.addException('setPosition',e)
        #update destination too
        qtgt = self.controller.configToKlampt(target)
        self.plugin.setTargetConfig(qtgt)

    def onMoveToPosition(self):
        if not self.advancing: return
        qtgt = self.plugin.robotPoser.get()
        qtgt = self.controller.configFromKlampt(qtgt)
        if self.activeController is not self.controller:
            qtgt_active = [qtgt[i] for i in self.controller.indices(self.activePart)]
        else:
            qtgt_active = qtgt
        speed = self.panel.speedSpinBox.value()
        with ControllerStepContext(self):
            if self.advancing:
                try:
                    self.activeController.moveToPosition(qtgt_active,speed)
                except NotImplementedError:
                    self.addNotImplementedError('moveToPosition')
                except Exception as e:
                    self.addException('moveToPosition',e)

    def onSetPosition(self):
        """Set Position called in joint poser. Either linearly interpolate or
        call setPosition, depending on whether Interpolate is checked."""
        if not self.advancing: return
        qtgt = self.plugin.robotPoser.get()
        qtgt = self.controller.configFromKlampt(qtgt)
        if self.activeController is not self.controller:
            qtgt_active = [qtgt[i] for i in self.controller.indices(self.activePart)]
        else:
            qtgt_active = qtgt
        speed = self.panel.speedSpinBox.value()
        with ControllerStepContext(self):
            if self.advancing:
                if self.panel.jointInterpolateCheck.isChecked():
                    if speed > 0:
                        times = [1.0/speed]
                        milestones = [qtgt_active]
                        try:
                            self.activeController.setPiecewiseLinear(times,milestones)
                        except NotImplementedError:
                            self.addNotImplementedError('setPiecewiseLinear')
                        except Exception as e:
                            self.addException('setPiecewiseLinear',e)
                    else:
                        print("Need speed > 0 to interpolate")
                else:
                    try:
                        self.activeController.setPosition(qtgt_active)
                    except NotImplementedError:
                        self.addNotImplementedError('setPosition')
                    except Exception as e:
                        self.addException('setPosition',e)
            

    def onSetVelocity(self):
        """Set Velocity called in joint poser. """
        if not self.advancing: return
        if not self.panel.jointInterpolateCheck.isChecked():
            print("Need Interpolate to be checked")
            return
        qtgt = self.plugin.robotPoser.get()
        qtgt = self.controller.configFromKlampt(qtgt)
        if self.activeController is not self.controller:
            qtgt_active = [qtgt[i] for i in self.controller.indices(self.activePart)]
        else:
            qtgt_active = qtgt
        speed = self.panel.speedSpinBox.value()
        if not (speed > 0):
            print("Need speed > 0 to set velocity")
            return
        
        with ControllerStepContext(self):
            qcur_active = self.activeController.commandedPosition()
            vtgt = vectorops.div(vectorops.sub(qtgt_active,qcur_active),speed)
            try:
                self.activeController.setVelocity(vtgt,1.0)
            except NotImplementedError:
                self.addNotImplementedError('setVelocity')
            except Exception as e:
                self.addException('setVelocity',e)

    def onMoveToCartesianPosition(self):
        if not self.advancing: return
        Ttgt = self.plugin.cartesianGoalPoser.get()
        speed = self.panel.cartesianSpeedSpinBox.value()
        if self.panel.cartesianInterpolateCheck.isChecked():
            func = 'moveToCartesianPositionLinear'
        else:
            func = 'moveToCartesianPosition'
        with ControllerStepContext(self):
            try:
                getattr(self.activeController,func)(Ttgt,speed)
            except NotImplementedError:
                self.addNotImplementedError(func)
            except Exception as e:
                self.addException(func,e)

    def onSetCartesianPosition(self):
        if not self.advancing: return
        Ttgt = self.plugin.cartesianGoalPoser.get()
        speed = self.panel.cartesianSpeedSpinBox.value()
        if not self.panel.cartesianInterpolateCheck.isChecked():
            func = 'setCartesianPosition'
            args = (Ttgt,)
        else:
            func = 'moveToCartesianPosition'
            args = (Ttgt,speed)
        with ControllerStepContext(self):
            try:
                getattr(self.activeController,func)(*args)
            except NotImplementedError:
                self.addNotImplementedError(func)
            except Exception as e:
                self.addException(func,e)

    def onSetCartesianVelocity(self):
        if not self.advancing: return
        if not self.panel.cartesianInterpolateCheck.isChecked():
            print("Need Interpolate to be checked")
            return
        Ttgt = self.plugin.cartesianGoalPoser.get()
        speed = self.panel.cartesianSpeedSpinBox.value()
        if not (speed > 0):
            print("Need speed > 0 to set velocity")
            return
        
        with ControllerStepContext(self):
            Tcur = self.activeController.commandedCartesianPosition()
            wvtgt = vectorops.div(se3.error(Ttgt,Tcur),speed)
            try:
                self.activeController.setCartesianVelocity((wvtgt[:3],wvtgt[3:]),1.0)
            except NotImplementedError:
                self.addNotImplementedError('setVelocity')
            except Exception as e:
                self.addException('setVelocity',e)

    def onClipConfig(self):
        source = self.panel.configSourceList.currentIndex()
        if source == 0: 
            q = self.plugin.qsns
        elif source == 1:
            q = self.plugin.qcmd
        elif source == 2:
            q = self.plugin.robotPoser.get()
        elif source == 3:
            q = self.qsns
        elif source == 4:
            q = self.qcmd
        elif source == 5:
            q = self.controller.configFromKlampt(self.plugin.robotPoser.get())
        else:
            assert False,"This code should not be reached"
        if q is None:
            self.addError("Configuration source is not provided by controller")
            return
        type = self.panel.configFormatList.currentIndex()
        if type == 0:  #python list
            text = str(q)
        elif type == 1:   #.config
            text = loader.write(q,'Config')
        elif type == 2:   #raw
            text = ' '.join(str(v) for v in q)
        else:
            assert False,"This code should not be reached"
        print("Text copied:",text)
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.clear()
        clipboard.setText(text,QtGui.QClipboard.Clipboard)
        if clipboard.supportsSelection():
            clipboard.setText(text,QtGui.QClipboard.Selection)
    
    def setEndEffectorToolCoordinates(self,eename):
        ee = self.robotinfo.endEffectors[eename]
        last_link = self.robot.driver(self.activeController.indices()[-1]).getAffectedLink()
        obj = ee.ikObjective   # type: IKObjective
        if obj is None:
            local = [0,0,0]
        else:
            local,world = obj.getPosition()
        if ee.link != last_link:  #need to transform to last actuated link
            local = self.robot.link(last_link).getLocalPosition(self.robot.link(ee.link).getWorldPosition(local))
        if self.advancing:
            with ControllerStepContext(self):
                self.activeController.setToolCoordinates(local)
            #update target to match
            time.sleep(0.01)
            with ControllerStepContext(self):
                Tcmd = self.activeController.commandedCartesianPosition()
                self.plugin.cartesianGoalPoser.set(*Tcmd)
        self.plugin.setToolCoordinates(local)
        return local

    def onEndEffectorChanged(self,index):
        if self.suppressGuiEvents: return
        self.suppressGuiEvents = True
        ees = self.partsToEEs[self.activePart]
        if index < 0 or len(ees)==0: 
            self.panel.toolXSpinBox.setValue(0)
            self.panel.toolYSpinBox.setValue(0)
            self.panel.toolZSpinBox.setValue(0)
            return
        assert index >= 0 and index < len(ees)
        eename = ees[index]
        try:
            tool = self.setEndEffectorToolCoordinates(eename)
            self.panel.toolXSpinBox.setValue(tool[0])
            self.panel.toolYSpinBox.setValue(tool[1])
            self.panel.toolZSpinBox.setValue(tool[2])
        except NotImplementedError as e:
            import traceback
            traceback.print_exc()
            print(e)
            self.addError("Unable to set tool coordinates for end effector {}, part {}".format(eename,self.activePart))
        except Exception as e:
            self.addException("setToolCoordinates",e)
        self.suppressGuiEvents = False
    
    def onToolCoordinatesChanged(self,value):
        if self.suppressGuiEvents: return
        x = self.panel.toolXSpinBox.value()
        y = self.panel.toolYSpinBox.value()
        z = self.panel.toolZSpinBox.value()
        local = x,y,z
        self.panel.endEffectorList.setCurrentIndex(-1)
        if self.advancing:
            with ControllerStepContext(self):
                self.activeController.setToolCoordinates(local)
            time.sleep(0.01)
            with ControllerStepContext(self):
                Tcmd = self.activeController.commandedCartesianPosition()
                self.plugin.cartesianGoalPoser.set(*Tcmd)
        self.plugin.setToolCoordinates(local)
    
    def onLoadPresetsDir(self):
        rootdir = self.robotinfo.resourceDir if self.robotinfo.resourceDir is not None else ''
        file = str(QtWidgets.QFileDialog.getExistingDirectory(self, "Select Directory",directory=rootdir))
        if not file: return
        for item in os.listdir(file):
            self.loadPreset(os.path.join(file,item))
        self.refreshPresetLists()
    
    def onLoadPreset(self):
        rootdir = self.robotinfo.resourceDir if self.robotinfo.resourceDir is not None else ''
        strlist = QtWidgets.QFileDialog.getOpenFileNames(self, "Select One or more files", directory=rootdir, filter="Configs (*.config);;Points (*.vector3);;Rigid transforms (*.xform);;Trajectories (*.path, *.traj);;JSON files (*.json)")
        if len(strlist) == 0: return
        if len(strlist) == 2 and str(strlist[0]).startswith("['"):  #Qt quirk -- string as list needs to be parsed?
            items = str(strlist[0]).split("'")
            strlist = []
            for i in items:
                if i in ['[',']',', ']:
                    continue
                strlist.append(i)
        for i in range(len(strlist)):
            item = str(strlist[i])
            self.loadPreset(item)
        self.refreshPresetLists()
    
    def refreshPresetLists(self):
        self.panel.configurationList.clear()
        self.panel.trajectoryList.clear()
        self.panel.cartesianPoseList.clear()
        self.panel.cartesianTrajectoryList.clear()
        for k in sorted(self.configPresets.keys()):
            self.panel.configurationList.addItem(k)
        for k in sorted(self.cartesianPosePresets.keys()):
            self.panel.cartesianPoseList.addItem(k)
        for k in sorted(self.trajectoryPresets.keys()):
            self.panel.trajectoryList.addItem(k)
        for k in sorted(self.cartesianTrajectoryPresets.keys()):
            self.panel.cartesianTrajectoryList.addItem(k)

    def loadPreset(self,item):
        itemfile = os.path.split(item)[1]
        itemname = os.path.splitext(itemfile)[0]
        try:
            res = loader.load('auto',item)
        except Exception as e:
            print(e)
            print("{}: couldn't read as a Klampt type, skipping".format(itemfile))
            return False
        type = types.object_to_type(res,['Config','RigidTransform','Trajectory'])
        if type is None:
            print("{}: couldn't read as supported preset type, skipping".format(itemfile))
            return False
        if type == 'Config':
            if len(res) not in [self.robot.numLinks(),self.robot.numDrivers(),3]:
                print("{}: vector does not match the robot's # of links, # of drivers, and is not a point".format(itemfile))
                return False
            if len(res) == self.robot.numLinks():
                self.configPresets[itemname] = res
            elif len(res) == self.robot.numDrivers():
                self.configPresets[itemname] = self.robot.configFromDrivers(res)
            else:
                self.cartesianPosePresets[itemname] = res
            print("{}: recognized as Config".format(itemfile))
        elif type == 'RigidTransform':
            print("{}: recognized as RigidTransform".format(itemfile))
            self.cartesianPosePresets[itemname] = res
        else:
            #trajectory
            if len(res.milestones) == 0:
                print("{}: empty trajectory, skipping".format(itemfile))
                return False
            n = len(res.milestones[0])
            if n not in [self.robot.numLinks(),self.robot.numLinks()*2,self.robot.numDrivers(),12,3]:
                print("{}: trajectory doesn't match the robot's # of links, # of drivers, and is not Cartesian".format(itemfile))
                return False
            if n == self.robot.numLinks():
                print("{}: recognized as Trajectory".format(itemfile))
                if n in [12,3]:
                    print("Warning {} is ambiguous, could be cartesian trajectory".format(itemfile))
                self.trajectoryPresets[itemname] = trajectory.RobotTrajectory(self.robot,res.times,res.milestones)
            elif n == self.robot.numLinks()*2:
                print("{}: recognized as HermiteTrajectory")
                if n in [12,3]:
                    print("Warning {} is ambiguous, could be cartesian trajectory".format(itemfile))
                self.trajectoryPresets[itemname] = trajectory.HermiteTrajectory(res.times,res.milestones)
            elif n == self.robot.numDrivers():
                print("{}: recognized as Trajectory")
                if n in [12,3]:
                    print("Warning {} is ambiguous, could be cartesian trajectory".format(itemfile))
                for i,m in enumerate(res.milestones):
                    res.milestones[i] = self.controller.configToKlampt(m)
                self.trajectoryPresets[itemname] = trajectory.RobotTrajectory(self.robot,res.times,res.milestones)
            elif n == 12:
                print("{}: recognized as SE3Trajectory")
                self.cartesianTrajectoryPresets[itemname] = trajectory.SE3Trajectory(res.times,res.milestones)
            else:
                print("{}: recognized as Vector3 Trajectory")
                self.cartesianTrajectoryPresets[itemname] = res
        return True
    
    def onPresetSelected(self,widget):
        if self.suppressGuiEvents: return
        self.suppressGuiEvents = True
        deselected = 0
        for widget2 in [self.panel.configurationList,self.panel.trajectoryList,self.panel.cartesianPoseList,self.panel.cartesianTrajectoryList]:
            if widget != widget2:
                deselected += 1
                widget2.setCurrentItem(None)
                widget2.setCurrentRow(-1)
        val = self.getPreset()
        if val is not None:
            self.plugin.preset = val
        self.suppressGuiEvents = False
    
    def getPreset(self,start=False):
        if self.panel.configurationList.currentItem() is not None:
            return self.configPresets[self.panel.configurationList.currentItem().text()]
        elif self.panel.cartesianPoseList.currentItem() is not None:
            tgt = self.cartesianPosePresets[self.panel.cartesianPoseList.currentItem().text()]
            if len(tgt) == 3:
                T = self.plugin.cartesianGoalPoser.get()
                tgt = (T[0],tgt)
            return tgt
        elif self.panel.trajectoryList.currentItem() is not None:
            traj = self.trajectoryPresets[self.panel.trajectoryList.currentItem().text()]
            if start:
                return traj.eval(traj.startTime())
            else:
                return traj
        elif self.panel.cartesianTrajectoryList.currentItem() is not None:
            traj = self.cartesianTrajectoryPresets[self.panel.cartesianTrajectoryList.currentItem().text()]
            if start:
                tgt = traj.eval(traj.startTime())
                if len(tgt) == 3:
                    T = self.plugin.cartesianGoalPoser.get()
                    tgt = (T[0],tgt)
                return tgt
            else:
                return traj
        return None
    
    def onSendPresetToPoser(self):
        val = self.getPreset(True)
        if val is None: return
        if isinstance(val,tuple):
            self.plugin.setTargetPose(val)
        else:
            self.plugin.setTargetConfig(val)
    
    def onMoveToPreset(self):
        val = self.getPreset(True)
        if val is None: return
        if isinstance(val,tuple):
            self.plugin.setTargetPose(val)
            self.onMoveToCartesianPosition()
        else:
            self.plugin.setTargetConfig(val)
            self.onMoveToPosition()
    
    def onExecutePreset(self):
        val = self.getPreset()
        if isinstance(val,tuple):
            self.plugin.setTargetPose(val)
            self.onMoveToCartesianPosition()
        elif isinstance(val,list):
            self.plugin.setTargetConfig(val)
            self.onMoveToPosition()
        elif isinstance(val,trajectory.RobotTrajectory):
            start = self.getPreset(True)
            self.plugin.setTargetConfig(start)
            self.onMoveToPosition()
            speed = self.panel.speedSpinBox.value()
            smoothing = [None,'pause','spline'][self.panel.smoothingComboBox.currentIndex()]
            with ControllerStepContext(self):
                trajectory.execute_trajectory(val,self.controller,speed=speed,smoothing=smoothing)
        elif isinstance(val,trajectory.HermiteTrajectory):
            start = self.getPreset(True)
            self.plugin.setTargetConfig(start)
            self.onMoveToPosition()
            speed = self.panel.speedSpinBox.value()
            smoothing = [None,'pause','spline'][self.panel.smoothingComboBox.currentIndex()]
            with ControllerStepContext(self):
                trajectory.execute_trajectory(val,self.controller,speed=speed,smoothing=smoothing)
        else:
            print("TODO: execute cartesian trajectory")
            pass

    def updateStatus(self):
        if not self.advancing: return
        with ControllerStepContext(self):           
            try:
                status = self.activeController.status()    
                self.panel.statusLabel.setText('Status: '+status)
            except NotImplementedError:
                self.panel.statusLabel.setText('status() method not implemented by {}'.format(self.activeController))
            except Exception as e:
                self.addException("status",e)

    def addError(self,text):
        print("addError",text)
        if len(self.errorText) > 0 and text == self.errorText[-1]: 
            return
        self.errorText.append(text)
        self.panel.errorTextarea.setPlainText('\n'.join(self.errorText))

    def addNotImplementedError(self,method):
        for text in self.errorText:
            if text.startswith(method+'()'):
                return
        self.errorText.append('{}() method not implemented by {}'.format(method,self.activeController))
        self.panel.errorTextarea.setPlainText('\n'.join(self.errorText))

    def addException(self,method,e):
        self.addError('Exception on {}: {}'.format(method,e))
        

def main():
    print("klampt_control: a GUI to control a Klamp't Robot Interface Layer (RIL) robot")
    print()

    import textwrap
    USAGE = textwrap.dedent("""
        USAGE: klampt_control [OPTS] FILES
        
        Where FILES is one or more strings referencing:
        1. A Python file (/path/to/file.py)
        2. A Python module (package.module)
        3. A RobotInfo JSON module (info_file.json)
        4. A robot file, world file, or objects for the visualization/planning
        
        In case 1 and 2, the Python file or module must contain a function
        make(robotModel,robotInfo) or make(robotModel) returning a
        RobotInterfaceBase instance.
        
        In cases 1 and 2, a robot file must be provided on the command line.
        
        OPTS can be any of
        --server IP: launch the controller as a standalone XML-RPC server
            (recommend using "0.0.0.0:7881")
        --client IP: use an XML-RPC client as the controller.  Try
            "http://localhost:7881".
        --sim: simulate the robot and use a standard controller.  Equivalent to
            including klampt.control.simrobotinterface in FILES
        """)

    if len(sys.argv) < 2:
        print(USAGE)
        exit(0)
    info = RobotInfo('untitled')
    world = WorldModel()
    mode = 'normal'
    ip = None
    idx = 1
    while idx < len(sys.argv):
        fn = sys.argv[idx]
        idx += 1
        if fn.startswith('-'):
            if fn == '--server':
                mode = 'server'
                ip = sys.argv[idx]
                idx += 1
            elif fn == '--client':
                mode = 'client'
                ip = sys.argv[idx]
                idx += 1
            elif fn == '--sim':
                info.controllerFile = 'klampt.control.simrobotinterface'
            else:
                print("Invalid command-line option")
                print()
                print(USAGE)
                exit(1)
            continue
        basename,ext = os.path.splitext(fn)
        if ext.lower() in ['.rob','.urdf','.xml','.off','.stl','.dae','.ply','.obj','.env']:
            world.loadFile(fn)
            if ext.lower() in ['.rob','.urdf']:
                info.modelFile = os.path.abspath(fn)
            elif ext.lower() == '.xml':
                #TODO: parse XML for robot file?
                pass
        elif ext in ['.py','.pyc']:
            info.controllerFile = fn
        elif ext == '.json':
            info = RobotInfo.load(fn)
            if info.controllerFile is None:
                print("RobotInfo file",fn,"doesn't specify controllerFile")
        else: #assume module
            info.controllerFile = fn
    
    if world.numRobots()==0:
        if info.modelFile is not None:
            print("Loading",info.modelFile,"from paths",info.filePaths)
            model = info.klamptModel()
            world.add(info.name,model)

    if world.numRobots() != 0:
        if info.name =='untitled':
            info.name = world.robot(0).getName()
        info.robotModel = world.robot(0)
        RobotInfo.register(info)

    if mode == 'normal':
        controller = info.controller()
    elif mode == 'client':
        controller = XMLRPCRobotInterfaceClient(ip)
    elif mode == 'server':
        if ':' in ip:
            addr,port = ip.split(':')
        else:
            addr=ip
            port = 7881
        controller = info.controller()
        server = XMLRPCRobotInterfaceServer(controller,addr,int(port))
        print("Beginning Robot Interface Layer server for controller",controller)
        print("Press Ctrl+C to exit...")
        server.serve()
        exit(0)

    res = controller.initialize()
    if not res:
        print("Error starting up controller")
        exit(1)

    import atexit
    def close_controller():
        controller.close()
    atexit.register(close_controller)
    
    if controller.properties.get('klamptModelFile',None):
        if world.numRobots()==0:
            world.loadFile(controller.properties['klamptModelFile'])
            print("Loaded robot file",controller.properties['klamptModelFile'])
        elif controller.properties['klamptModelFile'] != info.modelFile:
            print("Controller is based on robot file",controller.properties['klamptModelFile'],"while info is based on",info.modelFile)
    else:
        print("Controller doesn't have klamptModelFile property...")
    if world.numRobots()==0:
        print("No robot models loaded, can't run the visualization")
        exit(1)

    g_gui = None
    g_plugin = ControllerGLPlugin(world,controller,None)
    def makefunc(gl_backend):
        global g_gui
        gui = ControllerGUI(gl_backend,info,controller,g_plugin)
        g_plugin.gui = weakref.proxy(gui)
        g_gui = gui
        dw = QtWidgets.QDesktopWidget()
        x=dw.width()*0.8
        y=dw.height()*0.8
        gui.resize(x,y)
        return gui
    vis.customUI(makefunc)
    vis.pushPlugin(g_plugin)
    vis.nativeWindow().dt = g_plugin.dt
    vis.setWindowTitle("klampt_control {}".format(info.name))
    vis.run()
    del g_gui
    
    controller.close()
    atexit.unregister(close_controller)
    return



if __name__ == "__main__":
    main()
