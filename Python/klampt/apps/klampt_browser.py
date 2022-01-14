from klampt import *
from klampt.io import loader,resource
from klampt.math import se3
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.model.multipath import MultiPath
from klampt.model import types
from klampt import vis
from klampt.vis.glcommon import GLMultiViewportProgram
vis.init("PyQt5")
from klampt.vis.backends.vis_gl import GLVisualizationPlugin
from klampt.vis.backends.qtbackend import QtGLWindow
import sys,os,time
from PyQt5 import QtGui
from PyQt5 import QtCore
from PyQt5 import QtWidgets

world_item_extensions = set(['.obj','.rob','.urdf','.env'])
robot_override_types = ['Config','Configs']
animation_types = ['Trajectory','LinearPath','MultiPath']
create_types = resource.visual_edit_types()[:-1]

def save(obj,fn):
    if hasattr(obj,'saveFile'):
        return obj.saveFile(fn)
    if hasattr(obj,'save'):
        return obj.save(fn)
    type = loader.filename_to_type(fn)
    return loader.save(obj,type,fn)

MAX_VIS_ITEMS = 1000
MAX_VIS_CACHE = 10

def copyCamera(cam,camDest):
    camDest.rot = cam.rot[:]
    camDest.tgt = cam.tgt[:]
    camDest.dist = cam.dist


class MyMultiViewportProgram(GLMultiViewportProgram):
    def __init__(self):
        GLMultiViewportProgram.__init__(self)
        self.animating = False
        self.animationTime = 0
        self.animationDuration = 0
        self.animationStartTime = 0
        self.items = dict()
    def startAnim(self):
        self.animating = True
        self.animationStartTime = time.time()
        #self.idlesleep(0)
    def stopAnim(self):
        self.animating = False
        #self.idlesleep(float('inf'))
    def setAnimTime(self,t):
        self.stopAnim()
        self.animationTime = t
        self._updateTime(t)
    def _updateTime(self,t):
        #print "_updateTime",t
        def clearStartTime(v):
            v.animationStartTime = 0
            for n,subapp in v.subAppearances.items():
                clearStartTime(subapp)
        for (k,item) in self.items.items():
            item.plugin.animationTime(self.animationTime)
            for (k,v) in item.plugin.items.items():
                #do animation updates
                clearStartTime(v)
                v.updateAnimation(t)
        self.refresh()
    def idlefunc(self):
        if not self.animating:
            GLMultiViewportProgram.idlefunc(self)
            return
        t = time.time()
        self.animationTime = t - self.animationStartTime
        if self.animationDuration == 0:
            self.animationTime = 0
        else:
            self.animationTime = self.animationTime % self.animationDuration
            self._updateTime(self.animationTime)

class ResourceItem:
    def __init__(self,obj):
        self.obj = obj
        self.plugin = None
        self.program = None
        self.animationBuddy = None

class ResourceBrowser(QtWidgets.QMainWindow):
    def __init__(self,glwindow=None,parent=None):
        QtWidgets.QMainWindow.__init__(self,parent)
         # Splitter to show 2 views in same widget easily.
        self.splitter = QtWidgets.QSplitter()
        # The model.
        self.model = QtWidgets.QFileSystemModel()
        # You can setRootPath to any path.
        self.model.setRootPath(QtCore.QDir.rootPath())
        # Add filters
        filters = []
        print("ALLOWABLE FILE EXTENSIONS")
        for k,v in loader.EXTENSION_TO_TYPES.items():
            filters.append("*"+k)
            print(" ",k)
        filters.append("*.xml")
        filters.append("*.json")
        filters.append("*.txt")
        filters.append("*.obj")
        filters.append("*.rob")
        filters.append("*.urdf")
        filters.append("*.env")
        self.model.setNameFilters(filters)

        # Create the view in the splitter.
        self.view = QtWidgets.QTreeView()
        # Set the model of the view.
        self.view.setModel(self.model)
        #nicer size for columns
        self.view.header().resizeSection(0, 200)
        self.view.header().resizeSection(1, 75)
        self.view.header().resizeSection(2, 75)
        self.view.header().resizeSection(3, 150)
        # Set the root index of the view as the user's home directory.
        #self.view.setRootIndex(self.model.index(QtCore.QDir.homePath()))
        self.view.setRootIndex(self.model.index(os.getcwd()))
        self.view.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)

        self.world = WorldModel()
        self.tempWorld = WorldModel()
        self.active = dict()
        self.emptyVisPlugin = GLVisualizationPlugin()
        self.emptyVisPlugin.add("world",self.world)
        self.emptyVisProgram = None
        self.selected = set()
        self.visCache = []
        self.modified = set()
        
        self.left = QtWidgets.QFrame()
        self.right = QtWidgets.QFrame()
        self.leftLayout = QtWidgets.QVBoxLayout()
        self.left.setLayout(self.leftLayout)

        self.upButton = QtWidgets.QPushButton("Up")
        self.leftLayout.addWidget(self.upButton)
        self.leftLayout.addWidget(self.view)
        #visualization configuration
        vbuttonLayout = QtWidgets.QHBoxLayout()
        self.autoFitCameraButton = QtWidgets.QCheckBox("Auto-fit cameras")
        self.lockCameraCheck = QtWidgets.QCheckBox("Lock cameras")
        #self.overlayCheck = QtWidgets.QCheckBox("Overlay")
        self.maxGridItemsLabel = QtWidgets.QLabel("Grid width")
        self.maxGridItems = QtWidgets.QSpinBox()
        self.maxGridItems.setRange(3,15)
        self.autoFitCameraButton.setToolTip("If checked, the camera is automatically fit the the items in the scene")
        self.lockCameraCheck.setToolTip("If checked, all cameras are navigated simultaneously")
        #self.overlayCheck.setTooltip("If checked, all items are drawn on top of one another")
        self.maxGridItems.setToolTip("Max # height/width in the visualization pane")
        vbuttonLayout.addWidget(self.autoFitCameraButton)
        vbuttonLayout.addWidget(self.lockCameraCheck)
        #vbuttonLayout.addWidget(self.overlayCheck)
        vbuttonLayout.addWidget(self.maxGridItemsLabel)
        vbuttonLayout.addWidget(self.maxGridItems)
        self.leftLayout.addLayout(vbuttonLayout)

        #playback
        self.timeDriver = QtWidgets.QSlider()
        self.timeDriver.setOrientation(QtCore.Qt.Horizontal)
        self.timeDriver.setRange(0,1000)
        #self.timeDriver.setSizeHint()
        self.playButton = QtWidgets.QPushButton("Play")
        self.playButton.setCheckable(True)
        self.stopButton = QtWidgets.QPushButton("Stop")
        self.playButton.setToolTip("Starts/pauses playing any selected animations")
        self.stopButton.setToolTip("Stops playing any selected animations")
        label = QtWidgets.QLabel("Time")
        label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        vbuttonLayout = QtWidgets.QHBoxLayout()
        vbuttonLayout.addWidget(label)
        vbuttonLayout.addWidget(self.timeDriver)
        vbuttonLayout.addWidget(self.playButton)
        vbuttonLayout.addWidget(self.stopButton)
        self.leftLayout.addLayout(vbuttonLayout)

        #editing
        vbuttonLayout = QtWidgets.QHBoxLayout()
        self.editButton = QtWidgets.QPushButton("Edit item")
        self.saveButton = QtWidgets.QPushButton("Save item")
        self.editButton.setToolTip("Pops up a dialog to edit the selected item, if available")
        self.saveButton.setToolTip("Saves changes to edited items")
        self.saveButton.setEnabled(False)
        self.createComboBox = QtWidgets.QComboBox()
        self.createComboBox.addItem("Create new item...")
        for n in create_types:
            self.createComboBox.addItem(n)
        vbuttonLayout.addWidget(self.editButton)
        vbuttonLayout.addWidget(self.saveButton)
        vbuttonLayout.addWidget(self.createComboBox)
        self.leftLayout.addLayout(vbuttonLayout)

        #world configuration
        vbuttonLayout = QtWidgets.QHBoxLayout()
        self.addButton = QtWidgets.QPushButton("Add to world")
        self.clearButton = QtWidgets.QPushButton("Clear world")
        self.addButton.setToolTip("Adds the selected item(s) to the reference world")
        self.clearButton.setToolTip("Clears the reference world")
        vbuttonLayout.addWidget(self.addButton)
        vbuttonLayout.addWidget(self.clearButton)
        self.leftLayout.addLayout(vbuttonLayout)
        self.splitter.addWidget(self.left)
        self.splitter.addWidget(self.right)
        self.splitter.setHandleWidth(7)
        self.setCentralWidget(self.splitter)

        self.rightLayout = QtWidgets.QVBoxLayout()
        self.right.setLayout(self.rightLayout)
        if glwindow is None:
            self.glwidget = QtGLWindow("viewport")
        else:
            self.glwidget = glwindow
        self.rightLayout.addWidget(self.glwidget)
        self.glviewportManager = MyMultiViewportProgram()
        self.glwidget.setProgram(self.glviewportManager)
        self.glwidget.setParent(self.splitter)
        self.glviewportManager.sizePolicy = 'squeeze'
        self.glviewportManager.addView(self.emptyVisPlugin)
        self.glviewportManager.items = self.active
        self.emptyVisProgram = self.glviewportManager.views[-1]
        self.glwidget.setFixedSize(QtWidgets.QWIDGETSIZE_MAX,QtWidgets.QWIDGETSIZE_MAX)
        self.glwidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Expanding))
        self.glwidget.adjustSize()
        self.glwidget.refresh()

        self.upButton.clicked.connect(self.onUpClicked)
        self.view.selectionModel().selectionChanged.connect(self.selection_changed) 
        self.view.doubleClicked.connect(self.onViewDoubleClick)
        self.autoFitCameraButton.clicked.connect(self.onAutoFitCamera)
        self.maxGridItems.valueChanged.connect(self.maxGridItemsChanged)
        self.lockCameraCheck.toggled.connect(self.onLockCamerasToggled)
        self.timeDriver.valueChanged.connect(self.timeDriverChanged)
        self.playButton.toggled.connect(self.togglePlay)
        self.stopButton.clicked.connect(self.stopPlay)
        self.editButton.clicked.connect(self.onEditClicked)
        self.saveButton.clicked.connect(self.onSaveClicked)
        self.createComboBox.currentIndexChanged.connect(self.onCreateIndexChanged)
        self.addButton.clicked.connect(self.onAddClicked)
        self.clearButton.clicked.connect(self.onClearClicked)

    def closeEvent(self,event):
        if len(self.modified) > 0:
            reply = QtWidgets.QMessageBox.question(self, "Unsaved changes", "Would you like to save changes to " + ', '.join(self.modified)+ "?",
                                    QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No);
            if reply == QtWidgets.QMessageBox.Yes:
                self.onSaveClicked()
        vis.show(False)

    def onViewDoubleClick(self):
        indices = self.view.selectedIndexes()
        if len(indices) == 0: return
        item = indices[0]
        name = str(self.model.filePath(item))
        oldselected = self.selected
        self.selected = set([name])
        self.onAddClicked()
        self.selected = oldselected

    def onUpClicked(self):
        currentRoot = self.view.rootIndex()
        self.view.setRootIndex(currentRoot.parent())

    def selection_changed(self,newSelection,deselected):
        #print "klampt_browser: Selection changed!"
        for i in newSelection.indexes():
            if i.column() == 0:
                fn = str(i.model().filePath(i))
                self.selected.add(fn)
                self.add(fn)
                #print "  value:",fn
        #print "klampt_browser: Deselected:"
        for i in deselected.indexes():
            if i.column() == 0:
                fn = str(i.model().filePath(i))
                self.selected.remove(fn)
                self.remove(fn)
                #klampt_browser: print "  value:",fn
        self.refresh()

    def onAutoFitCamera(self):
        if self.autoFitCameraButton.isChecked():
            for (k,item) in self.active.items():
                vis.autoFitViewport(item.program.view,[self.world,item.obj])

    def onLockCamerasToggled(self,on):
        self.glviewportManager.broadcast = on
        if on:
            self.lockCameras()

    def lockCameras(self):
        view0 = self.glviewportManager.views[0].view
        cam0 = view0.camera
        for p in self.glviewportManager.views[1:]:
            cam = p.view.camera
            copyCamera(cam0,cam)

    def timeDriverChanged(self,value):
        u = value * 0.001
        animTrajectoryTime = u*self.glviewportManager.animationDuration
        for (k,item) in self.active.items():
            obj = item.obj
            plugin = item.plugin
            if item.animationBuddy is not None:
                path = item.animationBuddy
                if plugin.getItem(path).animation is None:
                    plugin.animate(path,obj,endBehavior='halt')
                else:
                    anim = plugin.getItem(path).animation
            plugin.pauseAnimation(False)
        self.glviewportManager.setAnimTime(animTrajectoryTime)

    def togglePlay(self,value):
        self.animating = value
        self.glviewportManager.refresh()
        if value:
            for (k,item) in self.active.items():
                obj = item.obj
                plugin = item.plugin
                if item.animationBuddy is not None:
                    plugin.animate(item.animationBuddy,obj,endBehavior='halt')
                plugin.pauseAnimation(False)
            self.glviewportManager.startAnim()
            self.glviewportManager.refresh()
        else:
            #pause
            self.glviewportManager.stopAnim()
            #self.idlesleep(float('inf'))

    def stopPlay(self):
        self.playButton.setChecked(False)
        #revert to no animations
        for (k,item) in self.active.items():
            obj = item.obj
            plugin = item.plugin
            if isinstance(obj,(Trajectory,MultiPath)):
                robotpath = ('world',self.world.robot(0).getName())
                plugin.animate(robotpath,None)
            plugin.pauseAnimation()

    def onEditClicked(self):
        if len(self.active) == 0:
            QtWidgets.QMessageBox.warning(self.splitter,"Invalid item","No item selected, can't edit")
            return
        fn = sorted(self.active.keys())[0]
        def doedit():
            print("klampt_browser: Launching resource.edit",fn,"...")
            try:
                (save,obj) = resource.edit(name=fn,value=self.active[fn].obj,world=self.world)
            except Exception as e:
                print("klampt_browser: Exception raised during resource.edit:",e)
                QtWidgets.QMessageBox.warning(self.splitter,"Editing not available","Unable to edit item of type "+self.active[fn].obj.__class__.__name__)
                return
            if save and obj is not None:
                self.active[fn].obj = obj
                #mark it as modified and re-add it to the visualization
                basename = os.path.basename(fn)
                self.active[fn].plugin.add(basename,obj)
                self.modified.add(fn)
                self.saveButton.setEnabled(True)
        QtCore.QTimer.singleShot(0,doedit)

    def onSaveClicked(self):
        for fn in self.modified:
            if not save(self.active[fn].obj,fn):
                print("klampt_browser: Error saving file",fn)
        self.modified = set()
        self.saveButton.setEnabled(False)
    
    def onCreateIndexChanged(self,item):
        if item == 0: return
        type = create_types[item-1]
        robot = None
        if self.world.numRobots() > 0:
            robot = self.world.robot(0)
        try:
            (save,obj) = resource.edit("untitled",types.make(type,robot),type=type,world=self.world)
        except Exception as e:
            print("klampt_browser: Exception raised during resource.edit():",e)
            QtWidgets.QMessageBox.warning(self.splitter,"Creation not available","Unable to create item of type "+type+", did you remember to add items to the reference world?")
            return
        if obj is not None and save:
            cur_file_path = self.model.filePath(self.view.rootIndex())
            fn = resource.save(obj,type,directory=cur_file_path)
            if fn is not None:
                self.loadedItem(fn,obj)
                #TODO: should we add to selection in tree view?
        self.createComboBox.setCurrentIndex(0)

    def onAddClicked(self):
        if self.world.numIDs() == 0:
            for name in self.selected:
                if name not in self.active: continue
                copyCamera(self.active[name].program.view.camera,self.emptyVisProgram.view.camera)
                break
        todel = []
        for name in self.selected:
            if name not in self.active: continue
            s = self.active[name].obj
            if isinstance(s,(RobotModel,RigidObjectModel,TerrainModel)):
                self.world.add(s.getName(),s)
                self.tempWorld.remove(s)
                todel.append(name)
            elif isinstance(s,WorldModel):
                for i in range(s.numRobots()):
                    self.world.add(s.robot(i).getName(),s.robot(i))
                for i in range(s.numRigidObjects()):
                    self.world.add(s.rigidObject(i).getName(),s.rigidObject(i))
                for i in range(s.numTerrains()):
                    self.world.add(s.terrain(i).getName(),s.terrain(i))
                for k,item in self.active.items():
                    item.plugin.add("world",self.world)
                todel.append(name)
            elif isinstance(s,(TriangleMesh,PointCloud,GeometricPrimitive)):
                t = self.world.makeTerrain(name)
                t.geometry().set(Geometry3D(s))
                todel.append(name)
            elif isinstance(s,Geometry3D):
                t = self.world.makeTerrain(name)
                t.geometry().set(s.clone())
                todel.append(name)
        for name in todel:
            self.remove(name)
        if len(todel) > 0:
            self.refresh()

    def onClearClicked(self):
        self.world = WorldModel()
        self.tempWorld = WorldModel()
        self.active = dict()
        self.visCache = []
        self.emptyVisPlugin.add("world",self.world)
        self.refresh()

    def add(self,fn,openDir=True,warn=True):
        #assert fn not in self.active
        if fn in self.active:
            print("add(): Warning, file",fn,"is already active")
            return
        for i,(cfn,citem) in enumerate(self.visCache):
            if cfn == fn:
                print() 
                print("klampt_browser: PULLED",fn,"FROM CACHE")
                print() 
                self.active[fn] = citem
                return True
        if len(self.active) >= MAX_VIS_ITEMS:
            return
        if os.path.isdir(fn):
            if openDir:
                failures = []
                successes = []
                for f in os.listdir(fn):
                    if f not in ['.','..'] and os.path.splitext(f)[1] != '':
                        if not self.add(os.path.join(fn,f),openDir=False,warn=False):
                            failures.append(f)
                        else:
                            successes.append(f)
                if len(failures) != 0 and len(successes) != 0:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid items","Could not load files "+', '.join(failures)+" as Klamp't elements")
                return True
            else:
                return False
        path,ext = os.path.splitext(fn)
        #print "Extension is",ext
        if ext in world_item_extensions:
            try:
                worldid = self.tempWorld.loadElement(fn)
            except Exception:
                if warn:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid item","Could not load "+fn+" as a Klamp't world element")
                return False
            if worldid < 0:
                if warn:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid item","Could not load "+fn+" as a Klamp't world element")
                return False
            obj = None
            for i in range(self.tempWorld.numRobots()):
                if self.tempWorld.robot(i).getID() == worldid:
                    obj = self.tempWorld.robot(i)
                    break
            for i in range(self.tempWorld.numRigidObjects()):
                if self.tempWorld.rigidObject(i).getID() == worldid:
                    obj = self.tempWorld.rigidObject(i)
                    break
            for i in range(self.tempWorld.numTerrains()):
                if self.tempWorld.terrain(i).getID() == worldid:
                    obj = self.tempWorld.terrain(i)
                    break
            assert obj is not None,"Hmm... couldn't find world id %d in world?"%(worldid,)
            self.loadedItem(fn,obj)
            return True
        try:
            type = loader.filename_to_type(fn)
        except RuntimeError:
            if warn:
                QtWidgets.QMessageBox.warning(self.splitter,"Invalid item","Could not load file "+fn+" as a known Klamp't type")
            return False
        if type == 'xml':
            #try loading a world
            try:
                world = WorldModel()
                res = world.readFile(fn)
                if not res:
                    try:
                        obj = loader.load('MultiPath',fn)
                    except Exception as e:
                        if warn:
                            print("klampt_browser: Trying MultiPath load, got exception",e)
                            import traceback
                            traceback.print_exc()
                            QtWidgets.QMessageBox.warning(self.splitter,"Invalid WorldModel","Could not load "+fn+" as a world XML file")
                        return False
                    self.loadedItem(fn,obj)
                    return True
            except IOError:
                if warn:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid WorldModel","Could not load "+fn+" as a world XML file")
                return False
            self.loadedItem(fn,world)
            return
        elif type == 'json':
            import json
            f = open(fn,'r')
            jsonobj = json.load(f)
            try:
                obj = loader.fromJson(jsonobj)
            except Exception:
                if warn:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid JSON","Could not recognize "+fn+" as a known Klamp't type")
                return False
        else:
            try:
                obj = loader.load(type,fn)
            except Exception as e:
                if warn:
                    QtWidgets.QMessageBox.warning(self.splitter,"Invalid item","Error while loading file "+fn+": "+str(e))
                return False
        self.loadedItem(fn,obj)
        return True

    def loadedItem(self,fn,obj):
        if fn in self.active:
            print("klampt_browser: Re-loaded item",fn,"so I'm first removing it")
            self.remove(fn)
        assert fn not in self.active
        item = ResourceItem(obj)
        self.active[fn] = item
        item.plugin = GLVisualizationPlugin()
        basename = os.path.basename(fn)

        #determine whether it's being animated
        if isinstance(obj,Trajectory) and len(obj.milestones) > 0:
            d = len(obj.milestones[0])
            if self.world.numRobots() > 0 and d == self.world.robot(0).numLinks():
                obj = RobotTrajectory(self.world.robot(0),obj.times,obj.milestones)
                robotpath = ('world',self.world.robot(0).getName())
                item.animationBuddy = robotpath
            elif d == 3:
                item.plugin.add("anim_point",[0,0,0])
                item.animationBuddy = "anim_point"
            elif d == 12:
                item.plugin.add("anim_xform",se3.identity())
                item.animationBuddy = "anim_xform"
            else:
                print("klampt_browser: Can't interpret trajectory of length",d)
        elif isinstance(obj,MultiPath):
            if self.world.numRobots() > 0:
                robotpath = ('world',self.world.robot(0).getName())
                item.animationBuddy = robotpath

        item.plugin.add("world",self.world)
        item.plugin.add(basename,obj)
        item.plugin.addText("label",basename,position=(10,10))
        try:
            type = vis.objectToVisType(obj,self.world)
        except:
            type = 'unknown'
        if type in robot_override_types:
            if self.world.numRobots() > 0:
                path = ('world',self.world.robot(0).getName())
                item.plugin.hide(path)
        item.plugin.initialize()

    def remove(self,fn,openDir=True):
        if os.path.isdir(fn):
            if openDir:
                for f in os.listdir(fn):
                    if f not in ['.','..'] and os.path.splitext(f)[1] != '':
                        self.remove(os.path.join(fn,f),openDir=False)
            return
        if fn not in self.active:
            return
        if fn in self.modified:
            reply = QtWidgets.QMessageBox.question(self, "Unsaved changes", "Would you like to save changes to " + ', '.join(self.modified)+ "?",
                                    QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No);
            if reply == QtWidgets.QMessageBox.Yes:
                save(self.active[fn],fn)
                self.modified.remove(fn)
        s = self.active[fn]
        del self.active[fn]
        if s.program is not None:
            copyCamera(s.program.view.camera,self.emptyVisProgram.view.camera)
        print() 
        print("klampt_browser: ADDING",fn,"TO CACHE")
        print() 
        self.visCache.append((fn,s))
        if len(self.visCache) > MAX_VIS_CACHE:
            self.visCache.pop(0)
        cleartemp = isinstance(s.obj,(RobotModel,RigidObjectModel,TerrainModel))
        if cleartemp:
            for (k,v) in self.active.items():
                if isinstance(v.obj,(RobotModel,RigidObjectModel,TerrainModel)):
                    cleartemp = False
                    break
        if cleartemp:
            if self.tempWorld.numRobots() + self.tempWorld.numRigidObjects() + self.tempWorld.numTerrains() > 10:
                print("klampt_browser: Clearing temp world...")
                self.tempWorld = WorldModel()
                self.visCache = [(fn,s) for (fn,s) in self.visCache if not isinstance(s.obj,(RobotModel,RigidObjectModel,TerrainModel))]
    
    def maxGridItemsChanged(self):
        self.refresh()

    def refresh(self):
        self.glviewportManager.clearViews()
        if len(self.active) == 0:
            self.glviewportManager.addView(self.emptyVisProgram)
        else:
            for k in sorted(self.active.keys()):
                item = self.active[k]
                if item.program is not None:
                    item.program.view.w,item.program.view.h = (640,480)
                    self.glviewportManager.addView(item.program)
                else:
                    #new view
                    self.glviewportManager.addView(item.plugin)
                    item.program = self.glviewportManager.views[-1]
                    if self.autoFitCameraButton.isChecked():
                        vis.autoFitViewport(item.program.view,[self.world,item.obj])
                    else:
                        copyCamera(self.emptyVisProgram.view.camera,item.program.view.camera)
                if len(self.glviewportManager.views) >= self.maxGridItems.value()**2:
                    break
            if self.glviewportManager.broadcast: #locking cameras
                self.lockCameras()
        self.glviewportManager.animationDuration = 0
        for (k,item) in self.active.items():
            obj = item.obj
            if isinstance(obj,(Trajectory,MultiPath)):
                self.glviewportManager.animationDuration = max(self.glviewportManager.animationDuration,obj.duration())
                print("klampt_browser: Setting animation duration to",self.glviewportManager.animationDuration)
        self.glviewportManager.refresh()

def main():
    print("""
===============================================================================
A program to quickly browse Klamp't objects. 

USAGE: %s [item1 item2 ...]

where the given items are world, robot, terrain, object, or geometry files. Run
it without arguments

   %s

for an empty reference world. You may add items to the reference world using
the `Add to World` button.  If you know what items to use in the reference
world, run it with

   %s world.xml

or 

   %s item1 item2 ...

where the items are world, robot, terrain, object, or geometry files.
===============================================================================
"""%(sys.argv[0],sys.argv[0],sys.argv[0],sys.argv[0]))
    #must be explicitly deleted for some reason in PyQt5...
    g_browser = None
    def makefunc(gl_backend):
        global g_browser
        browser = ResourceBrowser(gl_backend)
        g_browser = browser
        dw = QtWidgets.QDesktopWidget()
        x=dw.width()*0.8
        y=dw.height()*0.8
        browser.setFixedSize(x,y)
        for fn in sys.argv[1:]:
            res = browser.world.readFile(fn)
            if not res:
                print("Unable to load model",fn)
                print("Quitting...")
                sys.exit(1)
            print("Added",fn,"to world")
        if len(sys.argv) > 1:
            browser.emptyVisPlugin.add("world",browser.world)
        return browser
    vis.customUI(makefunc)
    vis.setWindowTitle("Klamp't Resource Browser")
    vis.run()
    del g_browser
    return

    #this code below is incorrect...
    app = QtWidgets.QApplication(sys.argv)
    browser = ResourceBrowser()
    for fn in sys.argv[1:]:
        res = browser.world.readFile(fn)
        if not res:
            print("Unable to load model",fn)
            print("Quitting...")
            sys.exit(1)
        print("Added",fn,"to world")
    if len(sys.argv) > 1:
        browser.emptyVisPlugin.add("world",browser.world)
    dw = QtWidgets.QDesktopWidget()
    x=dw.width()*0.8
    y=dw.height()*0.8
    browser.setFixedSize(x,y)
    #browser.splitter.setWindowState(QtCore.Qt.WindowMaximized)
    browser.setWindowTitle("Klamp't Resource Browser")
    browser.show()
    # Start the main loop.
    res = app.exec_()
    return res

if __name__ == '__main__':
    res = main()
    sys.exit(res)
