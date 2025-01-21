"""Utilities for inspecting Klamp't objects to retrieve a type string / create
objects from type strings.
"""

from ..model.contact import ContactPoint,Hold
from ..model.trajectory import Trajectory,RobotTrajectory,SO3Trajectory,SE3Trajectory
from ..model.multipath import MultiPath
from ..math import vectorops,so3,se3
from ..robotsim import WorldModel,RobotModel,RobotModelLink,RigidObjectModel,TerrainModel,IKObjective,Geometry3D,TriangleMesh,PointCloud,GeometricPrimitive,ConvexHull,Heightmap,ImplicitSurface,OccupancyGrid
import warnings

_knownTypes = set(['Value','Vector2','Vector3','Matrix3','Point','Rotation','RigidTransform','Vector','Config',
                'IntArray','StringArray',
                'Configs','Trajectory','LinearPath','MultiPath','SE3Trajectory','SO3Trajectory',
                'IKGoal','ContactPoint','Hold',
                'TriangleMesh','PointCloud','ImplicitSurface','OccupancyGrid','GeometricPrimitive','ConvexHull','Heightmap','Geometry3D',
                'WorldModel','RobotModel','RigidObjectModel','TerrainModel'])

_vectorLikeTypes = set(['Vector2','Vector3','Matrix3','Point','Rotation','Vector','Config'])
_arrayLikeTypes = set(['Vector2','Vector3','Matrix3','Point','Rotation','RigidTransform','Vector','Config','IntArray','StringArray','Configs'])
_pathLikeTypes = set(['Configs','Trajectory','LinearPath','MultiPath','SE3Trajectory','SO3Trajectory'])
_geometryTypes = set(['TriangleMesh','PointCloud','ImplicitSurface','OccupancyGrid','GeometricPrimitive','ConvexHull','Heightmap','Geometry3D'])

def known_types():
    """Returns a set of all known Klampt types"""
    global _knownTypes
    return _knownTypes

def object_to_types(object,world=None):
    """Returns a string defining the type of the given Python Klamp't object.
    If multiple types could be associated with it, then it returns a list of all
    possible valid types."""
    if isinstance(object,ContactPoint):
        return 'ContactPoint'
    elif isinstance(object,Hold):
        return 'Hold'
    elif isinstance(object,IKObjective):
        return 'IKGoal'
    elif isinstance(object,SE3Trajectory):
        return ['SE3Trajectory','Trajectory']
    elif isinstance(object,SO3Trajectory):
        return ['SO3Trajectory','Trajectory']
    elif isinstance(object,Trajectory):
        return 'Trajectory'
    elif isinstance(object,MultiPath):
        return 'MultiPath'
    elif isinstance(object,GeometricPrimitive):
        return 'GeometricPrimitive'
    elif isinstance(object,TriangleMesh):
        return 'TriangleMesh'
    elif isinstance(object,PointCloud):
        return 'PointCloud'
    elif isinstance(object,ImplicitSurface):
        return 'ImplicitSurface'
    elif isinstance(object,OccupancyGrid):
        return 'OccupancyGrid'
    elif isinstance(object,ConvexHull):
        return 'ConvexHull'
    elif isinstance(object,Heightmap):
        return 'Heightmap'
    elif isinstance(object,Geometry3D):
        return ['Geometry3D',object.type()]
    elif isinstance(object,WorldModel):
        return 'WorldModel'
    elif isinstance(object,RobotModel):
        return 'RobotModel'
    elif isinstance(object,RigidObjectModel):
        return 'RigidObjectModel'
    elif isinstance(object,TerrainModel):
        return 'TerrainModel'
    #this was here for Geometry3D, but might be mistaken with a SensorModel.
    #elif hasattr(object,'type'):
    #    if callable(object.type):
    #        return object.type()
    #    return object.type
    elif hasattr(object,'__iter__'):
        if len(object)>0 and hasattr(object[0],'__iter__'):
            if isinstance(object[0],str):
                if not all(isinstance(v,str) for v in object):
                    raise ValueError("Mixing string and other items in sequence?")
                return 'StringArray'
            #list of lists or tuples
            if len(object)==2:
                if len(object[0])==9 and len(object[1])==3:
                    #se3
                    return 'RigidTransform'
            allequal = True
            for entry in object:
                if len(entry) != len(object[0]):
                    allequal = False
                    break
            if allequal:
                return 'Configs'
            raise ValueError("Sequence of unequal-length types passed to object_to_types")
        else:
            dtypes = []
            if any(isinstance(v,int) for v in object):
                dtypes.append('int')
            if any(isinstance(v,float) for v in object):
                dtypes.append('float')
            if any(isinstance(v,str) for v in object):
                dtypes.append('str')
            vtypes = []
            if not str in dtypes:
                vtypes.append('Config')
                if not 'float' in dtypes:
                    vtypes.append('IntArray')
                if len(object)==2:
                    #2d point
                    vtypes.append('Vector2')
                elif len(object)==3:
                    #3d point
                    vtypes.append('Vector3')
                elif len(object)==9:
                    #so3 or 9d point?
                    vtypes.append('Matrix3')
                    if vectorops.distance(so3.mul(so3.inv(object),object),so3.identity())<1e-5:
                        vtypes.append('Rotation')
            else:
                vtypes.append("StringArray")
            if len(vtypes)==1:
                return vtypes[0]
            return vtypes
    elif isinstance(object,(int,float)):
        return 'Value'
    else:
        raise ValueError("Unknown object of type %s passed to object_to_types"%(object.__class__.__name__,))

def object_to_type(obj,validTypes=None,world=None):
    """Returns a type string for the Klamp't object obj, restricted
    to the set of validTypes.  If there are multiple interpretations,
    the first type in object_to_types that matches a valid type is
    returned.

    Args:
        obj: A Klamp't-compatible object
        validTypes (set, dict, or None): a set or dict of possible valid types.
            If None, any type is accepted

    Returns:
        str or None: The type of the object, or None if no valid type
        was found
    """
    otypes = object_to_types(obj,world)
    if isinstance(otypes,list):
        if validTypes is None:
            return otypes[0]
        for otype in otypes:
            if otype in validTypes:
                return otype
        return None
    else:
        #only one type
        return otypes


def make(type,object=None):
    """Makes a default instance of the given type.

    Args:
        type (str): the name of the desired type 
        object (optional): If ``type`` is 'Config', 'Configs', 'Vector', or
            'Trajectory', can provide the object for which the new instance
            will be compatible.
    """
    if type == 'Config' or type == 'Vector':
        if isinstance(object,RobotModel):
            return object.getConfig()
        else:
            from . import config
            return config.getConfig(object)
    elif type == 'Configs':
        return [make('Config',object)]
    elif type == 'Trajectory':
        if object is None:
            return Trajectory()
        elif isinstance(object,RobotModel):
            return RobotTrajectory(object,[0.0],make('Configs',object))
        else:
            types = object_to_types(object)
            if types == 'RigidTransform':
                return SE3Trajectory([0.0],make('Configs',object))
            elif 'Matrix3' in types:
                return SO3Trajectory([0.0],make('Configs',object))
            else:
                return Trajectory([0.0],make('Configs',object))
    elif type == 'IKGoal':
        return IKObjective()
    elif type == 'Vector2':
        return [0.,0.]
    elif type == 'Vector3' or type == 'Point':
        return [0.,0.,0.]
    elif type == 'Rotation' or type == 'Matrix3':
        return so3.identity()
    elif type == 'RigidTransform':
        return se3.identity()
    elif type == 'ContactPoint':
        return ContactPoint()
    elif type == 'Hold':
        return Hold()
    elif type == 'LinearPath':
        return Trajectory()
    elif type == 'MultiPath':
        return MultiPath()
    elif type == 'SO3Trajectory':
        return SO3Trajectory()
    elif type == 'SE3Trajectory':
        return SE3Trajectory()
    elif type == 'Value':
        return 0
    elif type == 'Geometry3D':
        return Geometry3D()
    elif type == 'TriangleMesh':
        return TriangleMesh()
    elif type == 'PointCloud':
        return PointCloud()
    elif type == 'GeometricPrimitive':
        p = GeometricPrimitive()
        p.setPoint((0,0,0))
        return p
    elif type in ['VolumeGrid','ImplicitSurface']:
        return ImplicitSurface()
    elif type == 'OccupancyGrid':
        return OccupancyGrid()
    elif type == 'ConvexHull':
        return ConvexHull()
    elif type == 'Heightmap':
        return Heightmap()
    elif type == 'IntArray':
        return [0]
    elif type == 'StringArray':
        return ['']
    elif type == 'WorldModel':
        return WorldModel()
    elif type in ['RobotModel','RigidObjectModel','TerrainModel']:
        if isinstance(object,WorldModel):
            if type == 'RobotModel':
                return object.makeRobot('Untitled')
            elif type == 'RigidObjectModel':
                return object.makeRigidObject('Untitled')
            else:
                return object.makeTerrain('Untitled')
        raise ValueError("Can't make an independent robot, rigid object, or terrain")
    else:
        raise ValueError("Can't make a Klamp't object of type %s"%(type,))


def info(object,world=None) -> dict:
    """Returns a dictionary containing metadata for the object. At a minimum,
    the "type" key will be filled in with the object's type, or None if
    the Klampt type is unknown.
    """
    res = {}
    try:
        otypes = object_to_types(object,world)
    except Exception:
        res["type"] = None
        return res
    otype = otypes
    if isinstance(otypes,list):
        otype = otypes[0]
        res["possible types"] = otypes
    res["type"] = otype
    if otype == "Configs":
        res["items"] = len(object)
        if len(object) > 0:
            res["dofs"] = len(object[0])
    elif otype in _arrayLikeTypes:
        res["items"] = len(object)
    elif otype in ["LinearPath","Trajectory","SO3Trajectory","SE3Trajectory"]:
        res["items"] = len(object.times)
        res["duration"] = object.duration()
        res["start time"] = object.startTime()
        res["end time"] = object.endTime()
        res["length"] = object.length()
        if len(object.milestones) > 0:
            res["dofs"] = len(object.milestones[0])
    elif otype == "IKGoal":
        res["link"]=object.link()
        if object.destLink() >= 0:
            res["destination link"] = object.destLink()
        else:
            res["destination link"] = None
        res["position dimensions constrained"] = object.numPosDims()
        res["rotation dimensions constrained"] = object.numRotDims()
    elif otype == "TriangleMesh":
        res["vertices"] = len(object.vertices)
        res["triangles"] = len(object.indices)
        bmin,bmax = Geometry3D(object).getBBTight()
        res["lower bound"] = bmin
        res["upper bound"] = bmax
    elif otype == "PointCloud":
        res["points"] = len(object.points)
        res["properties"] = object.numProperties()
        bmin,bmax = Geometry3D(object).getBBTight()
        res["lower bound"] = bmin
        res["upper bound"] = bmax
    elif otype == "VolumeGrid":
        res["dims"] = [object.dims[0],object.dims[1],object.dims[2]]
        if len(object.values) > 0:
            res["minimum value"] = min(object.values)
            res["maximum value"] = max(object.values)
        res["lower bound"] = object.bmin
        res["upper bound"] = object.bmax
    elif otype == "Heightmap":
        res["dims"] = [object.heights.shape[0],object.heights.shape[1]]
        if len(object.heights) > 0:
            res["minimum value"] = object.heights.min()
            res["maximum value"] = object.heights.min()
    elif otype == "Geometry3D":
        res["geometry type"] = object.type()
        res["#elements"] = object.numElements()
        bmin,bmax = object.getBBTight()
        res["lower bound"] = bmin
        res["upper bound"] = bmax
        if object.type() == "Group":
            res["elements"] = [info(object.getElement(i)) for i in range(object.numElements())]
    elif otype == 'WorldModel':
        res["#robots"] = object.numRobots()
        res["#rigid objects"] = object.numRigidObjects()
        res["#terrains"] = object.numTerrains()
        if object.numRobots():
            res["robots"] = [info(object.robot(i)) for i in range(object.numRobots())]
        if object.numRigidObjects():
            res["rigid objects"] = [info(object.rigidObject(i)) for i in range(object.numRigidObjects())]
        if object.numTerrains():
            res["terrains"] = [info(object.terrain(i)) for i in range(object.numTerrains())]
    elif otype == 'RobotModel':
        res["name"] = object.getName()
        res["#links"] = object.numLinks()
        res["#drivers"] = object.numDrivers()
        links = [{} for i in range(object.numLinks())]
        for i in range(object.numLinks()):
            links[i]["joint type"] = ("P" if object.link(i).isPrismatic() else "R")
            links[i]["name"] = object.link(i).name
            if not object.link(i).geometry().empty():
                links[i]["geometry"] = info(object.link(i).geometry())
        res["links"] = links
    elif otype in ['RigidObjectModel','TerrainModel']:
        res["name"] = object.getName()
        if not object.geometry().empty():
            res["geometry"] = info(object.geometry())
    return res


def convert(object,dest_type):
    """Converts an object to a semi-equivalent destination type.
    
    Only works for path-like objects (Configs, Trajectory, MultiPath,
    [X]Trajectory) and geometry objects.

    Object data may be referenced in the result.
    """
    global _pathLikeTypes,_geometryTypes, _vectorLikeTypes
    otype = object_to_type(object)
    if otype == dest_type:
        return object
    source_path_like = (otype in _pathLikeTypes)
    dest_path_like = (dest_type in _pathLikeTypes)
    source_geometry = (otype in _geometryTypes)
    dest_geometry = (dest_type in _geometryTypes)
    if otype in ["Config","Vector2","Vector3","Vector"] and dest_path_like:
        #make singleton
        dest = make(dest_type,object)
        if isinstance(dest,list):
            dest.append(object)
        elif dest_type=="MultiPath":
            dest.setTrajectory([object])
        else:
            dest.times.append(0.0)
            dest.milestones.append(object)
        return dest
    if source_path_like and dest_type in _vectorLikeTypes:
        #see if singleton can be extracted
        if otype=='Configs' and len(object)==1:
            return object[1]
        elif otype=='MultiPath':
            if len(object.sections)==1 and len(object.sections[0].configs)==1:
                return object.sections[0].configs[0]
        elif len(object.milestones)==1:
            return object.milestones[1]
        raise ValueError("Can't convert a path-like object with more than one milestone to a milestone type")
    if source_path_like and dest_path_like:
        #path to path
        if otype == 'MultiPath':
            object = object.getTrajectory()
            otype = 'Trajectory'
        
        if otype == "Configs":
            dest = make(dest_type)
            if dest_type == 'MultiPath':
                dest.setTrajectory(object)
            else:
                dest.times = list(range(len(object)))
                dest.milestones = object[:]
            return dest
        if dest_type == "Configs":
            return object.milestones[:]
        else:
            dest = make(dest_type)
            if dest_type == 'MultiPath':
                dest.setTrajectory(object)
            else:
                dest.times = object.times[:]
                dest.milestones = object.miltesones[:]
            return dest
    if source_geometry and dest_geometry:
        #geometry to geometry
        if not isinstance(object,Geometry3D):
            object = Geometry3D(object)
        if dest_type == "Geometry3D":
            return object
        return object.convert(dest_type)
    raise ValueError("Invalid conversion {} -> {}".format(otype,dest_type))


def transfer(object,source_robot,target_robot,link_map=None):
    """Converts a Klampt object that refers to a source robot to an object that
    refers to a target robot, assuming matched link names.

    Args:
        object: any Klampt object. This is only meaningful for int, list of int
            (IntArray), vector (Config), list of vector (Configs),
            Trajectory, IKGoal, RobotModelLink, or SubRobotModel types.
        source_robot (str or RobotModel): the robot for which ``object`` is
            meaningful.  If str, this is the filename of the source robot.
        target_robot (str or RobotModel): the robot that will use the return
            result.  If str, this is the filename of the target robot.
        link_map (dict, optional): if given, maps source link names to target
            link names, or source link indices to target link indices.  This
            can also be a 1-element dict {'*':'prefix:*'} indicating that the
            links of source_robot map to 'prefix:'+[LINK_NAME], or a dict
            {'prefix:*':'*'} indicating the reverse.
    Returns:
        same type as object: the object, with links mapped to the target robot.

    """
    from ..robotsim import WorldModel,RobotModelLink
    from .subrobot import SubRobotModel
    temp_world = None
    if isinstance(source_robot,str):
        temp_world = WorldModel()
        if not temp_world.readFile(source_robot):
            raise ValueError("Couldn't load source robot model "+source_robot)
        source_robot = temp_world.robot(0)
    if isinstance(target_robot,str):
        if temp_world is None:
            temp_world = WorldModel()
        if not temp_world.readFile(target_robot):
            raise ValueError("Couldn't load target robot model "+target_robot)
        target_robot = temp_world.robot(temp_world.numRobots()-1)
    if link_map is None:
        link_map = dict()
        for i in range(source_robot.numLinks()):
            sname = source_robot.link(i).getName()
            tlink = target_robot.link(sname)
            if tlink.index >= 0:
                link_map[i] = tlink.index
        warn_threshold = min(source_robot.numLinks()//2,target_robot.numLinks()//2)
        if len(link_map) < warn_threshold:
            print("klampt.model.types.transfer: warning, auto-detected link map:")
            print(link_map)
            print("Source links:")
            for i in range(source_robot.numLinks()):
                print(" ",source_robot.link(i).getName())
            print("Target links:")
            for i in range(target_robot.numLinks()):
                print(" ",target_robot.link(i).getName())
    if len(link_map)==1:
        items = list(link_map.items())
        if items[0][0] =='*':
            if not items[0][1].endswith('*'):
                if '*' not in items[0][1]:
                    raise NotImplementedError("TODO: match * inside target string")
                raise ValueError("Invalid wildcard match string")
            prefix = items[0][1][:-1]
            link_map = dict()
            for i in range(source_robot.numLinks()):
                sname = source_robot.link(i).getName()
                tlink = target_robot.link(prefix+sname)
                if tlink.index >= 0:
                    link_map[i] = tlink.index
        elif items[0][1] == '*':
            if not items[0][0].endswith('*'):
                if '*' not in items[0][0]:
                    raise NotImplementedError("TODO: match * inside source string")
                raise ValueError("Invalid wildcard match string")
            prefix = items[0][0][:-1]
            link_map = dict()
            for i in range(target_robot.numLinks()):
                tname = target_robot.link(i).getName()
                slink = source_robot.link(prefix+tname)
                if slink.index >= 0:
                    link_map[slink.index] = i
    if len(link_map)==0:
        raise ValueError("The source and target robot have no links in common")
    #convert to indices    
    link_map_indices = dict()
    for (k,v) in link_map.items():
        if isinstance(k,str):
            kindex = source_robot.link(k).index
            if kindex < 0:
                raise ValueError("Invalid source link "+k)
            k = kindex
        elif k < 0 or k >= source_robot.numLinks():
            raise ValueError("Invalid source link {}".format(k))
        if isinstance(v,str):
            vindex = target_robot.link(v).index
            if vindex < 0:
                raise ValueError("Invalid target link "+v)
            v = vindex
        elif k < 0 or k >= target_robot.numLinks():
            raise ValueError("Invalid target link {}".format(k))
        link_map_indices[k]=v
    if isinstance(object,int):
        return link_map_indices.get(object,-1)
    elif isinstance(object,RobotModelLink):
        return target_robot.link(link_map_indices.get(object.index,-1))
    elif isinstance(object,SubRobotModel):
        assert object._robot is source_robot,"Incorrect source robot for transferring SubRobotModel"
        tlinks = transfer(object._links,source_robot,target_robot,link_map_indices)
        if any(v < 0 for v in tlinks):
            raise ValueError("Can't transfer SubRobotModel; one or more links of the source subrobot does not appear in the target robot")
        return SubRobotModel(target_robot,tlinks)

    try:
        otypes = object_to_types(object,temp_world)
        if isinstance(otypes,str):
            otypes = [otypes]
    except ValueError:
        return object
    if 'IntArray' in otypes:
        return [link_map_indices.get(ind,-1) for ind in object]
    elif 'Config' in otypes:
        qres = target_robot.getConfig()
        for i,j in link_map_indices.items():
            qres[j] = object[i]
        return qres
    elif 'Configs' in otypes:
        res = []
        qres = target_robot.getConfig()
        for q in object:
            for i,j in link_map_indices.items():
                qres[j] = q[i]
            res.append([v for v in qres])
        return res
    elif 'IKGoal' in otypes:
        source_link,dest_link = object.link(),object.destLink()
        res = object.copy()
        res.setLinks(source_link,dest_link)
        return res
    elif 'Trajectory' in otypes:
        if isinstance(object,RobotTrajectory):
            return RobotTrajectory(target_robot,object.times,transfer(object.milestones,source_robot,target_robot,link_map_indices))
        return object.constructor(object.times,transfer(object.milestones,source_robot,target_robot,link_map_indices))
    else:
        return object



def _deprecated_func(oldName,newName):
    import sys
    mod = sys.modules[__name__]
    f = getattr(mod,newName)
    def depf(*args,**kwargs):
        warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
        return f(*args,**kwargs)
    depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
    setattr(mod,oldName,depf)

_deprecated_func("knownTypes","known_types")
_deprecated_func("objectToTypes","object_to_types")
