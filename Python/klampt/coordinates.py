import so3,se3,vectorops
from collections import defaultdict


class Frame:
    """Represents some coordinate frame in space."""
    def __init__(self,name,worldCoordinates=se3.identity(),
                 parent=None,relativeCoordinates=None):
        self._name = name
        self._parent = parent
        self._worldCoordinates = worldCoordinates
        self._data = None
        if relativeCoordinates == None:
            if parent == None:
                self._relativeCoordinates = worldCoordinates
            else:
                self._relativeCoordinates = se3.mul(se3.inv(parent.worldCoordinates,worldCoordinates))
        else:
            self._relativeCoordinates = relativeCoordinates
    def name(self):
        return self._name
    def data(self):
        return self._data
    def worldCoordinates(self):
        return self._worldCoordinates
    def relativeCoordinates(self):
        return self._relativeCoordinates
    def parent(self):
        return self._parent

class Transform:
    """A transform from one frame (source) to another (destination).  The
    destination may be None, in which case the transform is the world transform
    of the source"""
    def __init__(self,source,destination=None):
        self._source = source
        self._destination = destination
    def source(self):
        return self._source
    def destination(self):
        return self._destination
    def coordinates(self):
        if self._destination==None:
            return self._source.worldCoodinates()
        return se3.mul(sel3.inv(self._destination.worldCoordinates()),self._source.worldCoordinates())
    def toWorld(self):
        return Transform(self.source,None)
    def to(self,frame):
        return Transform(self.source,frame)

class Point:
    """Represents a point in 3D space.  It is attached to a frame, so if the
    frame is changed then its world coordinates will also change."""
    def __init__(self,localCoordinates=[0,0,0],frame=None):
        self._localCoordinates = localCoordinates
        self._frame = frame
    def localCoordinates(self):
        return self._localCoordinates[:]
    def worldCoordinates(self):
        if self._frame ==None:
            return self._localCoordinates[:]
        return se3.apply(self._frame.worldCoordinates(),self._localCoordinates)
    def frame(self):
        return self._frame
    def toWorld(self):
        """Returns a Point representing the same point in space, but
        in the world reference frame"""
        return Point(worldCoordinates(),None)
    def to(self,newframe):
        """Returns a Point representing the same point in space, but
        in a different reference frame"""
        if newframe == None or newframe=='world':
            return self.toWorld()
        newlocal = se3.apply(se3.inv(newframe.worldCoordinates()),self.worldCoordinates())
        return Point(newlocal,newframe)
    def localOffset(self,dir):
        """Offsets this point by a vector in local coordinates"""
        self._localCoordinates = vectorops.add(self._localCoordinates,dir)
    def worldOffset(self,dir):
        """Offsets this point by a vector in world coordinates"""
        if self._frame == None:
            self._localCoordinates = vectorops.add(self._localCoordinates,dir)
        else:
            self._localCoordinates = vectorops.add(so3.apply(so3.inv(self._frame.worldCoordinates()[0]),self._localCoordinates),dir)

class Direction:
    """Represents a directional quantity in 3D space.  It is attached to a
    frame, so if the frame is rotated then its world coordinates will also
    change."""
    def __init__(self,localCoordinates=[0,0,0],frame=None):
        self._localCoordinates = localCoordinates
        self._frame = frame
    def localCoordinates(self):
        return self._localCoordinates[:]
    def worldCoordinates(self):
        if self._frame ==None:
            return self._localCoordinates[:]
        return so3.apply(self._frame.worldCoordinates()[0],self._localCoordinates)
    def frame(self):
        return self._frame
    def toWorld(self):
        """Returns a Direction representing the same direction in space, but
        in the world reference frame"""
        return Direction(worldCoordinates(),None)
    def to(self,newframe):
        """Returns a Direction representing the same direction in space, but
        in a different reference frame"""
        if newframe == None or newframe=='world':
            return self.toWorld()
        newlocal = so3.apply(so3.inv(newframe.worldCoordinates()[0]),self.worldCoordinates())
        return Direction(newlocal,newframe)
    def scale(self,amount):
        """Scales this direction by a scalar amount"""
        self._localCoordinates = vectorops.mul(self._localCoordinates,amount)
    def localOffset(self,dir):
        """Offsets this direction by a vector in local coordinates"""
        self._localCoordinates = vectorops.add(self._localCoordinates,dir)
    def worldOffset(self,dir):
        """Offsets this direction by a vector in world coordinates"""
        if self._frame == None:
            self._localCoordinates = vectorops.add(self._localCoordinates,dir)
        else:
            self._localCoordinates = vectorops.add(so3.apply(so3.inv(self._frame.worldCoordinates()[0]),self._localCoordinates),dir)

class Group:
    """A collection of Frames, Points, Directions, and sub-Groups.
    All groups have a privileged frame called 'root'.
    The default manager is a Group with a privileged frame called 'world'
    which is just an alias for 'root'.

    Subgroup items can be accessed using the syntax [group]:[itemname].
    Subgroups can also be nested.

    Attributes:
    - frames: a map from frame names to Frame objects
    - childLists: a map from frame names to lists of children
    - points: a map from point names to Point objects
    - directions: a map from direction names to Direction objects
    - subgroups: a map from subgroup names to Group objects
    """
    def __init__(self):
        self.destroy()
    def rootFrame(self):
        return self.frames.get('root',None)
    def destroy(self):
        """Call this to destroy a group cleanly"""
        self.frames = {}
        self.childLists = defaultdict(list)
        self.frames['root'] = Frame('root')
        self.points = {}
        self.directions = {}
        self.subgroups = {}
    def setWorldModel(self,worldModel):
        """Sets this group to contain all entities of a world model"""
        for i in xrange(worldModel.numRobots()):
            rgroup = self.addGroup(worldModel.robot(0).getName())
            rgroup.setRobotModel(worldModel.robot(0))
        for i in xrange(worldModel.numRigidObjects()):
            f = self.addFrame(worldModel.rigidObject(0).getName(),worldModel.rigidObject(0).getTransform())
            f.data = worldModel.rigidObject(0)
        for i in xrange(worldModel.numTerrains()):
            f = self.addFrame(worldModel.terrain(0).getName())
            f.data = worldModel.terrain(0)
        return
    def setRobotModel(self,robotModel):
        """Sets this group to contain all links of a robot model"""
        for i in xrange(robotModel.numLinks()):
            f = self.addFrame(robotModel.link(0).getName(),robotModel.link(0).getTransform())
            f.data = robotModel.link(0)
        return
    def setController(self,controller):
        """Given a robotController, sets this group to contain all sensed
        and commanded frames."""
        robot = controller.robot()
        robot.setConfig(controller.getCommandedConfig())
        for i in xrange(robot.numLinks()):
            f = self.addFrame(robot.link(0).getName()+"_commanded",robot.link(0).getTransform())
            f.data = controller
        robot.setConfig(controller.getSensedConfig())
        for i in xrange(robot.numLinks()):
            f = self.addFrame(robot.link(0).getName()+"_sensed",robot.link(0).getTransform())
            f.data = controller
        return
    def setSimBody(self,name,simBody):
        """Sets this group to be attached to a simBody"""
        f = self.addFrame(name,simBody.getTransform())
        f.data = simBody
        return
    def updateFromWorld():
        """For any frames with associated world elements, updates the
        transforms from the world elements."""
        for (n,f) in self.frames.iteritems():
            if f.data == None: continue
            if hasattr(f.data,'getTransform'):
                self.setFrameCoordinates(f,f.data.getTransform(),'world')
    def updateToWorld():
        """For any frames with associated world elements, updates the
        transforms of the world elements.  Note: this does NOT perform inverse
        kinematics!"""
        for (n,f) in self.frames.iteritems():
            if f.data == None: continue
            if hasattr(f.data,'setTransform'):
                f.data.setTransform(*f.worldCoordinates())
    def addFrame(self,name,coordinates,parent=None):
        if name in self.frames:
            raise ValueError("Frame "+name+" already exists")
        if parent==None:
            parent = 'root'
        if isinstance(parent,str):
            parent = self.frames[parent]
        worldCoordinates = se3.mul(parent._worldCoordinates,coordinates)
        self.frames[name] = Frame(name,worldCoordinates,parent,coordinates)
        self.childLists[parent._name].append(self.frames[name])
        return self.frames[name]
    def addPoint(self,name,coordinates=[0,0,0],frame='root'):
        if name in self.frames:
            raise ValueError("Point "+name+" already exists")
        self.points[name] = self.point(coordinates,frame)
        return self.points[name]
    def addDirection(self,name,coordinates=[0,0,0],frame='world'):
        if name in self.frames:
            raise ValueError("Direction "+name+" already exists")
        self.directions[name] = self.direction(coordinates,frame)
        return self.directions[name]
    def addGroup(self,name,group=None,parentFrame='root'):
        """Adds a subgroup to this group. If parentFrame is given,
        then the group is attached relative to the given frame.
        Otherwise, it is assumed attached to the root frame. """
        if group==None:
            group = Group()
        if name in self.subgroups:
            raise ValueError("Subgroup "+name+" already exists")
        self.subgroups[name] = group
        group.frames['root'].parent = self.frame(parentFrame)
        return group
    def deleteFrame(self,name):
        """Deletes the named frame.  All items that refer to this frame
        will be automatically converted to be relative to the root coordinate
        system"""
        assert name != 'root',"Root frame may not be deleted"
        if name not in self.frames:
            raise ValueError("Invalid frame to delete")
        f = self.frames[name]
        f._parent = None
        if f._parent != None:
            self.childLists[f._parent._name].remove(f)
        for (n,p) in self.points.iteritems():
            if p._parent == f:
                p._localCoordinates = p.worldCoordinates()
                p._parent = self.frames['world']
        for (n,p) in self.directions.iteritems():
            if p._parent == f:
                p._localCoordinates = p.worldCoordinates()
                p._parent = self.frames['world']
        for c in self.childLists[name]:
            p._relativeCoordinates = p._worldCoordinates
            p._parent = self.frames['world']
        del self.frames[name]
        del self.childLists[name]
    def deletePoint(self,name):
        del self.points[name]
    def deleteDirection(self,name):
        del self.directions[name]
    def deleteGroup(self,name):
        del self.subgroups[name]
    def setFrameCoordinates(self,name,coordinates,parent='relative'):
        """Sets the coordinates of the frame, given as an se3 element.
        The coordinates can be given either in 'relative' mode, where the
        coordinates are the natural coordinates of the frame relative to
        its parent, or they can be given relative to any other frame in
        this coordinate Manager."""
        f = self.frames[name]
        if parent==None:
            parent = 'root'
        if isinstance(parent,str):
            if parent=='relative':
                parent = f.parent
            else:
                parent = self.frames[parent]
        worldCoordinates = se3.mul(parent._worldCoordinates,coordinates)
        if parent == f.parent:
            f._relativeCoordinates = coordinates
        else:
            f._relativeCoordinates = se3.mul(se3.inv(f.parent._worldCoordinates),worldCoordinates)
        f._worldCoordinates = worldCoordinates
        self.updateDependentFrames(self,f)
    def updateDependentFrames(self,frame):
        """Whenever Frame's world coordinates are updated, call this to update
        the downstream frames. This will be called automatically via
        setFrameCoordinates but not if you change a Frame's coordinates
        manually."""
        for c in self.childLists[frame._name]:
            c._worldCoordinates = se3.mul(f.worldCoordinates,c._relativeCoordinates)
            self.updateDependentFrames(c)
    def frame(self,name):
        """Retrieves a named Frame."""
        if isinstance(name,Frame): return name
        try:
            return self.frames[name]
        except KeyError:
            #try looking through groups
            splits = name.split(":",1)
            if len(splits)==1:
                raise ValueError("Frame "+name+" does not exist")
            if splits[0] not in self.subgroups:
                raise ValueError("Frame "+name+" or subgroup "+splits[0]+" do not exist")
            return self.subgroups[splits[0]].frame(splits[1])
    def getPoint(self,name):
        """Retrieves a named Point."""
        if isinstance(name,Point): return name
        try:
            return self.points[name]
        except KeyError:
            #try looking through groups
            splits = name.split(":",1)
            if len(splits)==1:
                raise ValueError("Point "+name+" does not exist")
            if splits[0] not in self.subgroups:
                raise ValueError("Point "+name+" or subgroup "+splits[0]+" do not exist")
            return self.subgroups[splits[0]].getPoint(splits[1])
    def getDirection(self,name):
        """Retrieves a named Direction."""
        if isinstance(name,Direction): return name
        try:
            return self.directions[name]
        except KeyError:
            #try looking through groups
            splits = name.split(":",1)
            if len(splits)==1:
                raise ValueError("Direction "+name+" does not exist")
            if splits[0] not in self.subgroups:
                raise ValueError("Direction "+name+" or subgroup "+splits[0]+" do not exist")
            return self.subgroups[splits[0]].getDirection(splits[1])
    def toWorld(self,object):
        """Converts a Transform, Point, or Direction to have coordinates
        relative to the world frame."""
        return object.toWorld()
    def to(self,object,frame):
        """Converts a Transform, Point, or Direction to have coordinates
        relative to the given frame 'frame'."""
        return object.to(self.frame(frame))
    def transform(self,sourceFrame,destFrame='world'):
        """Makes a Transform object from the source frame to the destination
        frame. """
        return Transform(self.frame(sourceFrame),self.frame(testFrame))
    def point(self,coordinates=[0,0,0],frame='world'):
        """Makes a Point object with the given local coordinates in the given 
        frame. Does not add it to the list of managed points."""
        return Point(coordinates,self.frame(frame))
    def direction(self,coordinates=[0,0,0],frame='world'):
        """Makes a Direction object with the given local coordinates in the
        given frame. Does not add it to the list of managed points."""
        return Direction(coordinates,self.frame(frame))
    def pointFromWorld(self,worldCoordinates=[0,0,0],frame='world'):
        """Alias for to(point(worldCoordinates,'world'),frame)"""
        f = self.frame(frame)
        local = se3.apply(se3.inv(f._worldCoordinates),worldCoordinates)
        return Point(local,f)
    def directionFromWorld(self,worldCoordinates=[0,0,0],frame='world'):
        """Alias for to(direction(worldCoordinates,'world'),frame)"""
        f = self.frame(frame)
        local = so3.apply(so3.inv(f._worldCoordinates[0]),worldCoordinates)
        return Direction(local,f)


class Manager(Group):
    """A manager of coordinate frames."""
    def __init__(self):
        Group.__init__(self)
        self.frames['world'] = self.frames['root']
    def worldFrame(self):
        return self.frames.get('world',None)
    def destroy(self):
        Group.destroy(self)
    def deleteFrame(self,name):
        assert name != 'world',"World frame may not be deleted"
    def setFrameCoordinates(self,name,coordinates,parent='relative'):
        assert name != 'world',"World frame must stay fixed at identity"
        Group.setFrameCoordinates(self,name,coordinates,parent)


#create defaults so you can just call coordinates.addFrame() etc.
defaultManager = Manager()
def _callfn(name):
    global defaultManager
    return lambda *args:getattr(defaultManager,name)(*args)

destroy = _callfn("destroy")
addFrame = _callfn("addFrame")
addPoint = _callfn("addPoint")
addDirection = _callfn("addDirection")
addGroup = _callfn("addGroup")
deleteFrame = _callfn("deleteFrame")
deletePoint = _callfn("deletePoint")
deleteDirection = _callfn("deleteDirection")
deleteGroup = _callfn("deleteGroup")
setFrameCoordinates = _callfn("setFrameCoordinates")
frame = _callfn("frame")
getPoint = _callfn("getPoint")
getDirection = _callfn("getDirection")
toWorld = _callfn("toWorld")
to = _callfn("to")
transform = _callfn("transform")
point = _callfn("point")
direction = _callfn("direction")
pointFromWorld = _callfn("pointFromWorld")
directionFromWorld = _callfn("directionFromWorld")
