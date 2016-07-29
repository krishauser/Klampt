import glcommon
import glinit

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

class SelectorEditor(VisualEditorBase):
    def __init__(self,name,value,description,world,robot=None):
        VisualEditorBase.__init__(self,name,value,description,world)
        self.robot = robot
        self.lastClicked = -1
        self.clicked = None
        self.hovered = None
    
    def instructions(self):
        return 'Right-click to toggle selection of robot links / objects in the world.\nKeyboard: < to select previous, > to select next'

    def addDialogItems(self,parent,ui='qt'):
        layout = QHBoxLayout(parent)
        self.clearButton = QPushButton("Clear")
        self.selectAllButton = QPushButton("Select all")
        layout.addWidget(self.clearButton)
        layout.addWidget(self.selectAllButton)
        self.insertButton.clicked.connect(self.clear)
        self.deleteButton.clicked.connect(self.selectAll)

    def clear(self):
        self.value = []
        self.refresh()

    def selectAll(self):
        if self.robot == None:
            #select all ids in the world
            pass
        else:
            self.value = [l for l in range(self.robot.numLinks())]
        self.refresh()        

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        if len(collided) == 0:
            return
        id = collided[0][1].getID()
        if id in self.value:
            self.value.remove(id)
        else:
            self.value.append(id)
        self.lastClicked = id
        self.refresh()

    def click_robot(self,x,y):
        """Helper: returns a list of robot objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for l in range(self.robot.numLinks()):
            (hit,pt) = self.robot.link(l).geometry().rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,l))
        if len(collided) == 0:
            return
        id = collided[0][1]
        if id in self.value:
            self.value.remove(id)
        else:
            self.value.append(id)
        self.lastClicked = id
        self.refresh()


    def mousefunc(self,button,state,x,y):
        if button==1 and state==0:
            if self.robot == None:
                self.click_world(x,y)
            else:
                self.click_robot(x,y)
            return True
        return VisualEditorBase.mousefunc(self,button,state,x,y)
    
    def keyboardfunc(self,c,x,y):
        if c==',' or c=='<':
            if self.lastClicked >= 0:
                self.lastClicked -= 1
            if self.lastClicked >= 0:
                if self.lastClicked not in self.value:
                    self.value.append(self.lastClicked)
                else:
                    self.value.remove(self.lastClicked)
            self.refresh()
            return True
        elif c=='.' or c=='>':
            Nmax = (self.robot.numLinks() if self.robot else self.world.numIDs())
            if self.lastClicked < Nmax:
                self.lastClicked += 1
            if self.lastClicked < Nmax:
                if self.lastClicked not in self.value:
                    self.value.append(self.lastClicked)
                else:
                    self.value.remove(self.lastClicked)
            self.refresh()
            return True

    def display(self):
        #Override display handler to highlight selected links
        if self.world != None:
            for i in xrange(self.world.numTerrains()):
                self.world.terrain(i).drawGL()
            for i in xrange(self.world.numRigidObjects()):
                self.world.rigidObject(i).drawGL()
            for i in xrange(self.world.numRobots()):
                self.world.robot(i).drawGL()
        elif self.robot != None:
            self.robot.drawGL()
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
