"""Utilities for inspecting Klamp't objects to retrieve a type string / create
objects from type strings.
"""

from ..model.contact import ContactPoint,Hold
from ..model.trajectory import Trajectory,RobotTrajectory,SO3Trajectory,SE3Trajectory
from ..model.multipath import MultiPath
from ..math import vectorops,so3,se3
from ..robotsim import WorldModel,RobotModel,RobotModelLink,RigidObjectModel,TerrainModel,IKObjective,Geometry3D,TriangleMesh,PointCloud,GeometricPrimitive,ConvexHull,VolumeGrid
import warnings

_knownTypes = set(['Value','Vector2','Vector3','Matrix3','Point','Rotation','RigidTransform','Vector','Config',
                'IntArray','StringArray',
                'Configs','Trajectory','LinearPath','MultiPath','SE3Trajectory','SO3Trajectory',
                'IKGoal','ContactPoint','Hold',
                'TriangleMesh','PointCloud','VolumeGrid','GeometricPrimitive','ConvexHull','Geometry3D',
                'WorldModel','RobotModel','RigidObjectModel','TerrainModel'])

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
    elif isinstance(object,VolumeGrid):
        return 'VolumeGrid'
    elif isinstance(object,ConvexHull):
        return 'ConvexHull'
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
    #this was here for Geometry3D, but might be mistaken with a SimRobotSensor.
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
    elif type == 'VolumeGrid':
        return VolumeGrid()
    elif type == 'ConvexHull':
        return ConvexHull()
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
    return None


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
