from klampt import *
from klampt import WidgetSet,RobotPoser,PointPoser,TransformPoser
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model import robotinfo
from klampt.model.robotinfo import RobotInfo
from klampt.io import loader
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.networkrobotinterface import RobotInterfaceClient,RobotInterfaceServer
from klampt.model import trajectory
import math
import time
import sys
import os
import weakref
import pkg_resources

vis.init("PyQt5")

from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import uic
from OpenGL import GL



class ControllerGLPlugin(GLWidgetPlugin):
    def __init__(self,world,controller,gui):
        GLWidgetPlugin.__init__(self)

        self.world = world
        self.controller = controller
        self.gui = gui
        #read commanded configuration
        q = self.controller.configToKlampt(self.controller.commandedPosition())
        world.robot(0).setConfig(q)
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
        self.robotPoser.set(qtgt)
        if self.cartesianControlEnabled:
            robot = self.world.robot(0)
            robot.setConfig(qtgt)
            ee = robot.link(self.endEffectorLink)
            t = ee.getWorldPosition(self.toolCoordinates)
            R = ee.getTransform()[0] 
            self.cartesianGoalPoser.set(R,t)            

    def display(self):
        robot = self.world.robot(0)
        #draw robot poser in transparent yellow
        oldcolors = [robot.link(i).appearance().getColor() for i in range(robot.numLinks())]
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(1,1,0,0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(*oldcolors[i])
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

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
    def __init__(self,glwidget,robotinfo,controller:RobotInterfaceBase,plugin,parent=None):
        QtWidgets.QMainWindow.__init__(self,parent)
        self.robotinfo = robotinfo           # type: RobotInfo
        self.controller = controller         # type: RobotInterfaceBase
        self.activeController = controller   # type: RobotInterfaceBase
        self.activePart = None
        self.glwidget = glwidget 
        self.plugin = plugin                 # type: ControllerGLPlugin
        self.qsns = None   #controller config
        self.qcmd = None   #controller config
        self.robot = controller.klamptModel()
        assert self.robot is not None,"klamptModel() method must be implemented for klampt_control to work"
        self.idleCount = 0
        # Splitter to show 2 views in same widget easily.
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

        self.partList.currentIndexChanged.connect(self.onPartChange)
        self.panel.resetButton.clicked.connect(self.onReset)
        self.panel.estopButton.clicked.connect(self.onEstop)
        self.panel.softStopButton.clicked.connect(self.onSoftStop)
        
        self.panel.configClipboardButton.clicked.connect(self.onClipConfig)
        self.jointScrollAreaLayout = self.panel.jointScrollAreaLayout
        self.jointScrollAreaLayout.setAlignment(QtCore.Qt.AlignTop)
        self.jointForms = []
        self.suppressGuiEvents = False
        
        self.panel.moveToPositionButton.clicked.connect(self.onMoveToPosition)
        self.panel.setPositionButton.clicked.connect(self.onSetPosition)
        self.panel.setVelocityButton.clicked.connect(self.onSetVelocity)
        self.panel.moveToCartesianPositionButton.clicked.connect(self.onMoveToCartesianPosition)
        self.panel.setCartesianPositionButton.clicked.connect(self.onSetCartesianPosition)
        self.panel.setCartesianVelocityButton.clicked.connect(self.onSetCartesianVelocity)
    
        self.panel.endEffectorList.currentIndexChanged.connect(self.onEndEffectorChanged)
        
        self.errorText = []
        self.errorTextarea = self.panel.errorTextarea
        
        self.splitter.addWidget(self.panel)
        self.splitter.addWidget(self.glwidget)
        self.splitter.setSizes([100,640])

        self.updateActiveController(None,controller)
        for ee in self.unmatchedEEs:
            self.addError("End effector {} in RobotInfo does not match a part".format(ee))
    
    def updateActiveController(self,activeName:str,active:RobotInterfaceBase):
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
        qmin_rob,qmax_rob = robotModel.getJointLimits()
        for i in range(robotModel.numLinks()):
            if robotModel.getJointType(i)=='spin':
                qmin_rob[i] = 0
                qmax_rob[i] = math.pi*2
        qmin = self.controller.configFromKlampt(qmin_rob)
        qmax = self.controller.configFromKlampt(qmax_rob)
        if len(indices) != nj:
            self.addError("Part's numJoints() returns {} joints, while controller's indices({}) returns {}".format(nj,self.activePart,len(indices)))
            return
        for form in self.jointForms:
            form.setParent(None)
            del form
        self.jointForms = []
        for i in range(nj):
            jointWidget = QtWidgets.QWidget()
            ui_filename = pkg_resources.resource_filename('klampt','data/joint_edit.ui')
            uic.loadUi(ui_filename, jointWidget)
            if math.isinf(qmin_rob[i]):
                self.addError("Infinite joint limit on joint {}".format(self.controller.jointName(indices[i])))
                qmin_rob[i] = -1
                if math.isinf(qmax_rob[i]):
                    qmax_rob[i] = 1
            elif math.isinf(qmax_rob[i]):
                self.addError("Infinite joint limit on joint {}".format(self.controller.jointName(indices[i])))
                qmax_rob[i] = 1
            jointWidget.sensedSpinBox.setMinimum(qmin[indices[i]])
            jointWidget.sensedSpinBox.setMaximum(qmax[indices[i]])
            jointWidget.commandSpinBox.setMinimum(qmin[indices[i]])
            jointWidget.commandSpinBox.setMaximum(qmax[indices[i]])
            jointWidget.targetSpinBox.setMinimum(qmin[indices[i]])
            jointWidget.targetSpinBox.setMaximum(qmax[indices[i]])
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
            self.selectedEndEffector = None
            self.panel.toolXSpinBox.setValue(0)
            self.panel.toolYSpinBox.setValue(0)
            self.panel.toolZSpinBox.setValue(0)
        else:
            self.selectedEndEffector = eematches[0]
            self.onEndEffectorChanged(0)
        
        cartesianEnabled = False
        cartesianLink = None
        self.controller.beginStep()            
        try:
            tool = active.getToolCoordinates()
            cartesianEnabled = True
            cartesianLink = self.robot.driver(self.controller.indices(self.activePart)[-1]).getAffectedLink()
        except NotImplementedError as e:
            #disable cartesian control
            cartesianEnabled = False
        except Exception as e:
            self.addException("getToolCoordinates",e)
        if cartesianEnabled:
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
            except ValueError:
                cartesianEnabled = False
            except Exception as e:
                self.addException("sensedCartesianPosition",e)
        self.controller.endStep()  
        self.panel.moveToCartesianPositionButton.setEnabled(cartesianEnabled)
        self.panel.setCartesianPositionButton.setEnabled(cartesianEnabled)
        self.panel.setCartesianVelocityButton.setEnabled(cartesianEnabled)
        self.plugin.enableCartesianWidget(cartesianEnabled,cartesianLink)
    
    def onIdle(self):
        self.controller.beginStep()
        try:
            clock = self.controller.clock()
            vis.add("clock","%.3f"%clock,position=(10,10))
        except NotImplementedError:
            vis.add("clock","clock() method not implemented",position=(10,10))
            self.addNotImplementedError('clock')
        except Exception as e:
            self.addException('clock',e)
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

        self.controller.endStep()

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
            except ValueError:
                cartesianEnabled = False
            except Exception as e:
                self.addException("sensedCartesianPosition",e)
            if self.plugin.robotPoser.hasFocus():
                #change poser
                T = None
                q = self.plugin.robotPoser.get()
                qcontroller = self.controller.configFromKlampt(q)
                try:
                    T = self.activeController.cartesianPosition(qcontroller)
                except NotImplementedError:
                    from klampt.control import robotinterfaceutils
                    T = robotinterfaceutils.klamptCartesianPosition(self.robot,qcontroller,self.controller.indices(self.activePart),self.activeController.getToolCoordinates())
                    self.addNotImplementedError("cartesianPosition")
                except Exception as e:
                    self.addException("cartesianPosition",e)
                if T is not None:
                    self.plugin.cartesianGoalPoser.set(*T)
            if self.plugin.cartesianGoalPoser.hasFocus():
                #TODO: solve IK problem to update the robot poser
                pass
            
        self.idleCount += 1
        if self.idleCount % 10 == 1:  #TODO: sync this with the control rate to get reasonable visual updates?
            self.suppressGuiEvents = True
            indices = self.controller.indices(self.activePart)
            qmin_rob,qmax_rob = self.robot.getJointLimits()
            for i in range(self.robot.numLinks()):
                if self.robot.getJointType(i)=='spin':
                    qmin_rob[i] = 0
                    qmax_rob[i] = math.pi*2
            qmin = self.controller.configFromKlampt(qmin_rob)
            qmax = self.controller.configFromKlampt(qmax_rob)
            if len(indices) != len(self.jointForms):
                self.addError("Invalid result from indices({}): {} should be {}".format(self.activePart,len(indices),len(self.jointForms)))
                self.suppressGuiEvents = False
                return
            if self.qsns is not None:
                for i,j in enumerate(indices):
                    self.jointForms[i].sensedSlider.setValue(int(1024*(self.qsns[i]-qmin[j])/(qmax[j]-qmin[j])))
                    self.jointForms[i].sensedSpinBox.setValue(self.qsns[i])
            if self.qcmd is not None:
                for i,j in enumerate(indices):
                    self.jointForms[i].commandSlider.setValue(int(1024*(self.qcmd[i]-qmin[j])/(qmax[j]-qmin[j])))
                    self.jointForms[i].commandSpinBox.setValue(self.qcmd[i])
            qtgt_rob = self.plugin.robotPoser.get()
            qtgt = self.controller.configFromKlampt(qtgt_rob)
            for i,j in enumerate(indices):
                self.jointForms[i].targetSlider.setValue(int(1024*(qtgt[j]-qmin[j])/(qmax[j]-qmin[j])))
                self.jointForms[i].targetSpinBox.setValue(qtgt[j])
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
        self.activeController.reset()
    
    def onEstop(self):
        self.activeController.estop()
    
    def onSoftStop(self):
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
        indices = self.controller.indices(self.activePart)
        j = indices[index]
        target = [v for v in self.qcmd]
        target[j] = value
        self.controller.beginStep()
        try:
            self.controller.setPosition(target)
        except NotImplementedError:
            self.addNotImplementedError('setPosition')
        except Exception as e:
            self.addException('setPosition',e)
        self.controller.endStep()
        #update destination too
        qtgt = self.controller.configToKlampt(target)
        self.plugin.setTargetConfig(qtgt)

    def onMoveToPosition(self):
        qtgt = self.plugin.robotPoser.get()
        qtgt = self.controller.configFromKlampt(qtgt)
        if self.activeController is not self.controller:
            qtgt_active = [qtgt[i] for i in self.controller.indices(self.activePart)]
        else:
            qtgt_active = qtgt
        speed = self.panel.speedSpinBox.value()
        self.controller.beginStep()
        try:
            self.activeController.moveToPosition(qtgt_active,speed)
        except NotImplementedError:
            self.addNotImplementedError('moveToPosition')
        except Exception as e:
            self.addException('moveToPosition',e)
        self.controller.endStep()

    def onSetPosition(self):
        pass

    def onSetVelocity(self):
        pass

    def onClipConfig(self):
        source = self.panel.configSourceList.currentIndex()
        if source == 0: 
            q = self.plugin.qsns
        elif source == 1:
            q = self.plugin.qcmd
        elif source == 2:
            q = self.plugin.robotPoser.get()
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
        
    def onEndEffectorChanged(self,index):
        ees = self.partsToEEs[self.activePart]
        if index < 0 or len(ees)==0: 
            self.panel.toolXSpinBox.setValue(0)
            self.panel.toolYSpinBox.setValue(0)
            self.panel.toolZSpinBox.setValue(0)
            return
        assert index >= 0 and index < len(ees)
        eename = ees[index]
        obj = self.robotinfo.endEffectors[eename].ikObjective   # type: IKObjective
        if obj is None:
            self.activeController.setToolCoordinates([0,0,0])
            self.panel.toolXSpinBox.setValue(0)
            self.panel.toolYSpinBox.setValue(0)
            self.panel.toolZSpinBox.setValue(0)
            return
        local,world = obj.getPosition()
        try:
            self.activeController.setToolCoordinates(local)
        except NotImplementedError:
            self.addError("Unable to set tool coordinates for end effector {}, part {}".format(eename,self.activePart))
        except Exception as e:
            self.addException("setToolCoordinates",e)
        self.panel.toolXSpinBox.setValue(local[0])
        self.panel.toolYSpinBox.setValue(local[1])
        self.panel.toolZSpinBox.setValue(local[2])

    def updateStatus(self):
        self.controller.beginStep()
        try:
            status = self.activeController.status()    
            self.panel.statusLabel.setText('Status: '+status)
        except NotImplementedError:
            self.panel.statusLabel.setText('status() method not implemented by {}'.format(self.activeController))
        except Exception as e:
            self.addException("status",e)
        self.controller.endStep()

    def addError(self,text):
        print("addError",text)
        if len(self.errorText) > 0 and text == self.errorText[-1]: 
            return
        self.errorText.append(text)
        self.errorTextarea.setPlainText('\n'.join(self.errorText))

    def addNotImplementedError(self,method):
        for text in self.errorText:
            if text.startswith(method+'()'):
                return
        self.errorText.append('{}() method not implemented by {}'.format(method,self.activeController))
        self.errorTextarea.setPlainText('\n'.join(self.errorText))

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
        make(robotModel) returning a RobotInterfaceBase instance.
        
        In cases 2 and 3, a robot file must be provided.
        
        OPTS can be any of
        --server IP: launch the controller as a standalone server (recommend
            using "localhost:7881")
        --client IP: use a client as the controller
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
            info = robotinfo.load(fn)
            if info.controllerFile is None:
                print("RobotInfo file",fn,"doesn't specify controllerFile")
        else: #assume module
            info.controllerFile = fn
    if world.numRobots()==0:
        if info.modelFile is not None:
            print("Loading",info.modelFile,"from paths",info.filePaths)
            model = info.klamptModel()
            world.add(info.name,model)
        else:
            print("No robot models loaded, can't run the visualization")
            exit(1)

    if info.name =='untitled':
        info.name = world.robot(0).getName()
    info.robotModel = world.robot(0)
    robotinfo.register(info)

    if mode == 'normal':
        controller = info.controller()
    elif mode == 'client':
        if ':' in ip:
            addr,port = ip.split(':')
        else:
            addr=ip
            port = 7881
        controller = RobotInterfaceClient(addr,int(port))
    elif mode == 'server':
        if ':' in ip:
            addr,port = ip.split(':')
        else:
            addr=ip
            port = 7881
        controller = info.controller()
        server = RobotInterfaceServer(controller,addr,int(port))
        print("Beginning Robot Interface Layer server for controller",controller)
        print("Press Ctrl+C to exit...")
        server.serve()
        exit(0)

    res = controller.initialize()
    if not res:
        print("Error starting up controller")
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
        gui.setFixedSize(x,y)
        return gui
    vis.customUI(makefunc)
    vis.pushPlugin(g_plugin)
    vis.nativeWindow().dt = g_plugin.dt
    vis.setWindowTitle("klampt_control {}".format(info.name))
    vis.run()
    del g_gui
    
    #close the controller
    controller.close()
    return



if __name__ == "__main__":
    main()
