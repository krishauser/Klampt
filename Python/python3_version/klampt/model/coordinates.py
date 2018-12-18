"""A module to help manage coordinate frames and objects attached to them.
Similar to the tf module in ROS.

The coordinates module is set up with a default coordinate manager so that
if you call coordinates.[X], where [X] is a method of Manager, such as
setWorldModel(), addPoint(), addFrame(), etc., then the default Manager
instance gets called.

Power users might create their own Managers, or swap top-level managers
in/out using setManager().
"""

from ..math import so3,se3,vectorops
from ..robotsim import RobotModelLink,RigidObjectModel
from . import ik
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
            if worldCoordinates == None:
                raise ValueError("One of relativeCoordinates or worldCoordinates must be provided")
            if parent == None:
                self._relativeCoordinates = worldCoordinates
            else:
                self._relativeCoordinates = se3.mul(se3.inv(parent.worldCoordinates()),worldCoordinates)
        else:
            self._relativeCoordinates = relativeCoordinates
            if worldCoordinates == None:
                if parent == None:
                    self._worldCoordinates = relativeCoordinates
                else:
                    self._worldCoordinates = se3.mul(parent.worldCoordinates(),relativeCoordinates)
    def name(self):
        """Returns the name of this frame"""
        return self._name
    def data(self):
        """If any data is attached to this frame, returns it"""
        return self._data
    def worldOrigin(self):
        """Returns an element of R^3 denoting the translation of the origin
        of this frame in world coordinates"""
        return self._worldCoordinates[1]
    def relativeOrigin(self):
        """Returns an element of R^3 denoting the translation of the origin
        of this frame relative to its parent"""
        return self._relativeCoordinates[1]
    def worldRotation(self):
        """Returns an element of SO(3) denoting the rotation from this frame
        to world coordinates"""
        return self._worldCoordinates[0]
    def relativeRotation(self):
        """Returns an element of SO(3) denoting the rotation from this frame
        to its parent"""
        return self._relativeCoordinates[0]
    def worldCoordinates(self):
        """Returns an element of SE(3) denoting the transform from this frame
        to world coordinates"""
        return self._worldCoordinates
    def relativeCoordinates(self):
        """Returns an element of SE(3) denoting the transform from this frame
        to its parent"""
        return self._relativeCoordinates
    def parent(self):
        """Returns the parent of the frame, or None if it's given in the world
        frame."""
        return self._parent

class Transform:
    """A transform from one Frame (source) to another (destination).  The
    destination may be None, in which case the transform is the world transform
    of the source.

    The difference between a Transform and a relative Frame (i.e., one with
    a parent) is that a Transform is a sort of "read-only" structure whose
    coordinates change as the frames' coordinates change."""
    def __init__(self,source,destination=None):
        assert isinstance(source,Frame)
        if destination is not None: 
            assert isinstance(destination,Frame)
        self._name = None
        self._source = source
        self._destination = destination
    def source(self):
        """Returns the source Frame"""
        return self._source
    def destination(self):
        """Returns the source Frame"""
        return self._destination
    def coordinates(self):
        """Returns the SE(3) coordinates that transform elements from the
        source to the destination Frame."""
        if self._destination==None:
            return self._source.worldCoordinates()
        return se3.mul(se3.inv(self._destination.worldCoordinates()),self._source.worldCoordinates())
    def translationCoordinates(self):
        """Returns the coordinates of the origin of this frame in R^3, relative
        to its destination"""
        if self._destination==None:
            return self._source.worldOrigin()
        return se3.apply(se3.inv(self._destination.worldCoordinates()),self._source.worldOrigin())
    def rotationCoordinates(self):
        """Returns the SO(3) coordinates that rotate elements from the source
        to the destination Frame"""
        if self._destination==None:
            return self._source.worldRotation()
        return so3.mul(so3.inv(self._destination.worldRotation()),self._source.worldRotation())
    def toWorld(self):
        """Returns a Transform designating the transformation from the
        source frame to the world frame."""
        return Transform(self.source,None)
    def to(self,frame):
        """Returns a Transform designating the transformation from the
        source frame to the given frame."""
        return Transform(self.source,frame)

class Point:
    """Represents a point in 3D space.  It is attached to a frame, so if the
    frame is changed then its world coordinates will also change."""
    def __init__(self,localCoordinates=[0,0,0],frame=None):
        if frame is not None: 
            assert isinstance(frame,Frame)
        self._name = None
        self._localCoordinates = localCoordinates
        self._frame = frame
    def localCoordinates(self):
        """Returns the coordinates of this point in its parent Frame"""
        return self._localCoordinates[:]
    def worldCoordinates(self):
        """Returns the coordinates of this point in the world Frame"""
        if self._frame ==None:
            return self._localCoordinates[:]
        return se3.apply(self._frame.worldCoordinates(),self._localCoordinates)
    def frame(self):
        """Returns the frame to which this Point is attached"""
        return self._frame
    def toWorld(self):
        """Returns a Point representing the same point in space, but
        in the world reference frame"""
        return Point(self.worldCoordinates(),None)
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
        if frame is not None: 
            assert isinstance(frame,Frame)
        self._name = None
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
        return Direction(self.worldCoordinates(),None)
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
        self._name = None
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
        for i in range(worldModel.numRobots()):
            rgroup = self.addGroup(worldModel.robot(i).getName())
            rgroup.setRobotModel(worldModel.robot(i))
        for i in range(worldModel.numRigidObjects()):
            try:
                f = self.addFrame(worldModel.rigidObject(i).getName(),worldCoordinates=worldModel.rigidObject(i).getTransform())
                f._data = worldModel.rigidObject(i)
            except ValueError:
                f = self.addFrame("%s[%d]"%(worldModel.rigidObject(i).getName(),i),worldCoordinates=worldModel.rigidObject(i).getTransform())
                f._data = worldModel.rigidObject(i)
        for i in range(worldModel.numTerrains()):
            try:
                f = self.addFrame(worldModel.terrain(i).getName(),worldCoordinates=se3.identity())
                f._data = worldModel.terrain(i)
            except ValueError:
                f = self.addFrame("%s[%d]"%(worldModel.terrain(i).getName(),i),worldCoordinates=se3.identity())
                f._data = worldModel.terrain(i)
        return
    def setRobotModel(self,robotModel):
        """Sets this group to contain all links of a robot model"""
        root = self.frames['root']
        for i in range(robotModel.numLinks()):
            p = robotModel.link(i).getParent()
            if p >= 0:
                Fp = self.frames[robotModel.link(p).getName()]
            else:
                Fp = root
            f = self.addFrame(robotModel.link(i).getName(),worldCoordinates=robotModel.link(i).getTransform(),parent=Fp)
            f._data = robotModel.link(i)
        return
    def setController(self,controller):
        """Given a robotController, sets this group to contain all sensed
        and commanded frames."""
        root = self.frames['root']
        robot = controller.robot()
        robot.setConfig(controller.getCommandedConfig())
        for i in range(robot.numLinks()):
            if p >= 0:
                Fp = self.frames[robotModel.link(p).getName()+"_commanded"]
            else:
                Fp = root
            f = self.addFrame(robot.link(i).getName()+"_commanded",worldCoordinates=robot.link(i).getTransform(),parent=Fp)
            f._data = (controller,i,'commanded')
        robot.setConfig(controller.getSensedConfig())
        for i in range(robot.numLinks()):
            if p >= 0:
                Fp = self.frames[robotModel.link(p).getName()+"_commanded"]
            else:
                Fp = root
            f = self.addFrame(robot.link(i).getName()+"_sensed",worldCoordinates=robot.link(i).getTransform(),parent=Fp)
            f._data = (controller,i,'sensed')
        return
    def setSimBody(self,name,simBody):
        """Sets this group to be attached to a simBody"""
        f = self.addFrame(name,worldCoordinates=simBody.getTransform())
        f._data = simBody
        return
    def updateFromWorld(self):
        """For any frames with associated world elements, updates the
        transforms from the world elements."""
        for (n,f) in self.frames.items():
            if f._data == None:
                continue
            if hasattr(f._data,'getTransform'):
                worldCoordinates = f._data.getTransform()
                if hasattr(f._data,'getParent'):
                    p = f._data.getParent()
                    if p >= 0:
                        plink = f._data.robot().link(p)
                        parentCoordinates = plink.getTransform()
                        f._relativeCoordinates = se3.mul(se3.inv(parentCoordinates),worldCoordinates)
                    else:
                        f._relativeCoordinates = worldCoordinates
                else:
                    f._relativeCoordinates = worldCoordinates
                f._worldCoordinates = worldCoordinates
                #update downstream non-link items
                for c in self.childLists[f._name]:
                    if c._data == None or not hasattr(c._data,'getTransform'):
                        c._worldCoordinates = se3.mul(f._worldCoordinates,c._relativeCoordinates)
                        self.updateDependentFrames(c)
            if isinstance(f._data,tuple) and isinstance(f._data[0],SimRobotController):
                controller,index,itemtype = f._data
                #TODO: update the frame from the controller data
        for (n,g) in self.subgroups.items():
            g.updateFromWorld()
    def updateToWorld(self):
        """For any frames with associated world elements, updates the
        transforms of the world elements.  Note: this does NOT perform inverse
        kinematics!"""
        for (n,f) in self.frames.items():
            if f.data == None: continue
            if hasattr(f.data,'setTransform'):
                f.data.setTransform(*f.worldCoordinates())
        for (n,g) in self.subgroups.items():
            g.updateToWorld()
    def addFrame(self,name,worldCoordinates=None,parent=None,relativeCoordinates=None):
        """Adds a new named Frame, possibly with a parent.  'parent' may either be a string
        identifying another named Frame in this Group, or it can be a Frame object. (Warning:
        unknown behavior may result from specifying a Frame not in this Group).

        Either worldCoordinates or relativeCoordinates must be given.  If worldCoordinates is given,
        then the frame's initial relative transform is determined by the current coordinates of the
        parent.  If all parameters are left as default, the frame is placed directly at the origin
        of the parent"""
        if name in self.frames:
            raise ValueError("Frame "+name+" already exists")
        if parent==None:
            parent = 'root'
        if isinstance(parent,str):
            parent = self.frames[parent]
        if worldCoordinates == None and relativeCoordinates == None:
            relativeCoordinates = se3.identity()
        self.frames[name] = Frame(name,worldCoordinates=worldCoordinates,parent=parent,relativeCoordinates=relativeCoordinates)
        self.childLists[parent._name].append(self.frames[name])
        return self.frames[name]
    def addPoint(self,name,coordinates=[0,0,0],frame='root'):
        if name in self.points:
            raise ValueError("Point "+name+" already exists")
        res = self.point(coordinates,frame)
        res._name = name
        self.points[name] = res
        return res
    def addDirection(self,name,coordinates=[0,0,0],frame='root'):
        if name in self.direction:
            raise ValueError("Direction "+name+" already exists")
        res = self.direction(coordinates,frame)
        res._name = name
        self.directions[name] = res
        return res
    def addGroup(self,name,group=None,parentFrame='root'):
        """Adds a subgroup to this group. If parentFrame is given,
        then the group is attached relative to the given frame.
        Otherwise, it is assumed attached to the root frame. """
        if group==None:
            group = Group()
        if name in self.subgroups:
            raise ValueError("Subgroup "+name+" already exists")
        group._name = name
        self.subgroups[name] = group
        group.frames['root']._parent = self.frame(parentFrame)
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
        for (n,p) in self.points.items():
            if p._parent == f:
                p._localCoordinates = p.worldCoordinates()
                p._parent = self.frames['root']
        for (n,p) in self.directions.items():
            if p._parent == f:
                p._localCoordinates = p.worldCoordinates()
                p._parent = self.frames['root']
        for c in self.childLists[name]:
            p._relativeCoordinates = p._worldCoordinates
            p._parent = self.frames['root']
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
        its parent, or in 'world' mode, where the coordinates are the
        global world coordinates, or they can be given relative to any
        other frame in this coordinate Group.  If None, this defaults
        to the root frame of this Group."""
        f = self.frame(name)
        if parent==None:
            parent = 'root'
        if isinstance(parent,str):
            if parent=='relative':
                parent = f._parent
            elif parent=='world':
                parent = None
            else:
                parent = self.frames[parent]
        if parent:
            worldCoordinates = se3.mul(parent._worldCoordinates,coordinates)
        else:
            worldCoordinates = coordinates
        if parent == f._parent:
            f._relativeCoordinates = coordinates
        else:
            f._relativeCoordinates = se3.mul(se3.inv(f._parent._worldCoordinates),worldCoordinates)
        f._worldCoordinates = worldCoordinates
        self.updateDependentFrames(f)
    def updateDependentFrames(self,frame):
        """Whenever Frame's world coordinates are updated, call this to update
        the downstream frames. This will be called automatically via
        setFrameCoordinates but not if you change a Frame's coordinates
        manually."""
        for c in self.childLists[frame._name]:
            c._worldCoordinates = se3.mul(frame.worldCoordinates(),c._relativeCoordinates)
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
    def transform(self,sourceFrame,destFrame='root'):
        """Makes a Transform object from the source frame to the destination
        frame. """
        return Transform(self.frame(sourceFrame),self.frame(testFrame))
    def point(self,coordinates=[0,0,0],frame='root'):
        """Makes a Point object with the given local coordinates in the given 
        frame. Does not add it to the list of managed points."""
        return Point(coordinates,self.frame(frame))
    def direction(self,coordinates=[0,0,0],frame='root'):
        """Makes a Direction object with the given local coordinates in the
        given frame. Does not add it to the list of managed points."""
        return Direction(coordinates,self.frame(frame))
    def pointFromWorld(self,worldCoordinates=[0,0,0],frame='root'):
        """Alias for to(point(worldCoordinates,'root'),frame)"""
        f = self.frame(frame)
        local = se3.apply(se3.inv(f._worldCoordinates),worldCoordinates)
        return Point(local,f)
    def directionFromWorld(self,worldCoordinates=[0,0,0],frame='world'):
        """Alias for to(direction(worldCoordinates,'root'),frame)"""
        f = self.frame(frame)
        local = so3.apply(so3.inv(f._worldCoordinates[0]),worldCoordinates)
        return Direction(local,f)
    def listFrames(self,indent=0):
        """Prints all the frames in this group and subgroups"""
        for k,f in self.frames.items():
            if indent > 0:
                print(" "*(indent-1), end=' ')
            if f._parent == None:
                print(k)
            else:
                print(k,"(%s)"%(f._parent._name,))
        for n,g in self.subgroups.items():
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print(n,":")
            g.listFrames(indent+2)
    def listItems(self,indent=0):
        """Prints all the items in this group"""
        if len(self.frames) > 0:
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print("Frames:")
            for k,f in self.frames.items():
                if indent > 0:
                    print(" "*(indent+1), end=' ')
                if f._parent == None:
                    print(k)
                else:
                    print(k,"(%s)"%(f._parent._name,))
        if len(self.points) > 0:
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print("Points:")
            for k in self.points.keys():
                if indent > 0:
                    print(" "*(indent+1), end=' ')
                print(k)
        if len(self.directions) > 0:
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print("Directions:")
            for k in self.directions.keys():
                if indent > 0:
                    print(" "*(indent+1), end=' ')
                print(k)
        if len(self.subgroups) > 0:
            if indent > 0:
                print(" "*(indent-1), end=' ')
            print("Subgroups:")
            for n,g in self.subgroups.items():
                if indent > 0:
                    print(" "*(indent+1), end=' ')
                print(n,":")
                g.listItems(indent+2)

class Manager(Group):
    """A manager of coordinate frames."""
    def __init__(self):
        Group.__init__(self)
        self._name = "world_group"
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
_defaultManager = Manager()
def _callfn(name):
    global _defaultManager
    return lambda *args,**kwargs:getattr(_defaultManager,name)(*args,**kwargs)

def manager():
    """Retrieves the default top-level manager"""
    global _defaultManager
    return _defaultManager

def setManager(manager):
    """Sets the new top-level manager to a new Manager instance, and
    returns the old top-level manager."""
    assert isinstance(manager,Manager),"setManager must be called with a Manager instance"
    global _defaultManager
    res = _defaultManager
    _defaultManager = manager
    return res


destroy = _callfn("destroy")
setWorldModel = _callfn("setWorldModel")
setRobotModel = _callfn("setRobotModel")
setController = _callfn("setController")
setSimBody = _callfn("setSimBody")
updateFromWorld = _callfn("updateFromWorld")
updateToWorld = _callfn("updateToWorld")
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
listFrames = _callfn("listFrames")
listItems = _callfn("listItems")



def _ancestor_with_link(frame):
    """Returns the nearest ancestor of the given frame attached to a robot
    link or rigid object"""
    while frame and (frame._data == None or not isinstance(frame._data,(RobotModelLink,RigidObjectModel))):
        frame = frame._parent
    return frame
    
def ik_objective(obj,target):
    """Returns an IK objective that attempts to fix the given
    klampt.coordinates object 'obj' at given target object 'target'. 

    Arguments:
     - obj: An instance of one of the
       {Point,Direction,Transform,Frame} classes.
     - target: If 'obj' is a Point, Direction, or  Frame objects, this
       must be an object of the same type of 'obj' denoting the target to
       which 'obj' should be fixed.  In other words, the local coordinates
       of 'obj' relative to 'target's parent frame will be equal to 'target's
       local coordinates.
       If obj is a Transform object, this element is an se3 object.
    Return value:
     - An IK objective to be used with the klampt.ik module.

    Since the klampt.ik module is not aware about custom frames, an
    ancestor of the object must be attached to a RobotModelLink or a
    RigidObjectModel, or else None will be returned.  The same goes for target,
    if provided.

    TODO: support lists of objects to fix.

    TODO: support Direction constraints.
    """
    body = None
    coords = None
    ref = None
    if isinstance(obj,Frame):
        assert isinstance(target,Frame),"ik_objective: target must be of same type as obj"
        body = obj
        ref = target.parent()
        coords = target.relativeCoordinates()
    elif isinstance(obj,Transform):
        if ref != None: print("ik_objective: Warning, ref argument passed with Transform object, ignoring")
        body = obj.source()
        ref = obj.destination()
        coords = target
    elif isinstance(obj,(Point,Direction)):
        assert type(target)==type(obj),"ik_objective: target must be of same type as obj"
        body = obj.frame()
        ref = target.frame()
        coords = target.localCoordinates()
    else:
        raise ValueError("Argument to ik_objective must be an object from the coordinates module")
    
    linkframe = _ancestor_with_link(body)
    if linkframe == None:
        print("Warning: object provided to ik_objective is not attached to a robot link or rigid object, returning None")
        return None
    linkbody = linkframe._data

    #find the movable frame attached to ref
    refframe = _ancestor_with_link(ref) if ref != None else None
    refbody = (refframe._data if refframe!=None else None)
    if isinstance(obj,(Frame,Transform)):
        #figure out the desired transform T[linkbody->refbody], given
        #coords = T[obj->ref], T[obj->linkbody], T[ref->refbody]
        #result = (T[ref->refbody] * coords * T[obj->linkbody]^-1)
        if linkframe != body: coords = se3.mul(coords,Transform(linkframe,body).coordinates())
        if refframe != ref: coords = se3.mul(Transform(ref,refframe).coordinates(),coords)
        return ik.objective(linkbody,ref=refbody,R=coords[0],t=coords[1])
    elif isinstance(obj,Point):
        #figure out the local and world points
        local = obj.to(linkframe).localCoordinates()
        world = target.to(refframe).localCoordinates()
        return ik.objective(linkbody,local=[local],world=[world])
    elif isinstance(obj,Direction):
        raise ValueError("Axis constraints are not yet supported in the klampt.ik module")
    return None

    

def ik_fixed_objective(obj,ref=None):
    """Returns an IK objective that attempts to fix the given
    klampt.coordinates object at its current pose.  If ref=None,
    its pose is fixed in world coordinates.  Otherwise, its pose is fixed
    relative to the reference frame ref.

    Arguments:
     - obj: An instance of one of the
       {Point,Direction,Transform,Frame} classes.
     - ref: either None, or a Frame object denoting the reference frame
       to which the object should be fixed.  (If obj is a Transform object,
       its destination frame is used as the reference frame, and this argument
       is ignored.)
    Return value:
     - An IK objective to be used with the klampt.ik module.  For
       Point, Direction, and Frame objects this objective fixes the
       object coordinates relative to the ref frame, or the world if None frame
       is provided.  For Transform objects the source frame is fixed
       relative to the destination frame.

    Since the klampt.ik module is not aware about custom frames, an
    ancestor of the object must be attached to a RobotModelLink or a
    RigidObjectModel, or else None will be returned.  The same goes for ref,
    if provided.

    TODO: support lists of objects to fix.

    TODO: support Direction constraints.
    """
    if isinstance(obj,(Point,Direction)):
        return ik_objective(obj,obj.to(ref))
    elif isinstance(obj,Frame):
        return ik_fixed_objective(Transform(obj,ref))
    elif isinstance(obj,Transform):
        if ref != None: print("ik_fixed_objective: Warning, ref argument passed with Transform object, ignoring")
        return ik_objective(obj,obj.coordinates())
    else:
        raise ValueError("Argument to ik_fixed_objective must be an object from the coordinates module")
