"""Conversions to and from Numpy objects; makes numerical computations much
more convenient.
"""

import numpy as np
from klampt.math import so3,se3
from ..model import types

SUPPORTED_TYPES = set(['Vector3','Point','Matrix3','Rotation','RigidTransform',
        'Config','Configs','Trajectory',
        'TriangleMesh','PointCloud','VolumeGrid','Geometry3D' ])
"""set of supported types for numpy I/O"""

def to_numpy(obj,type='auto'):
    """Converts a Klamp't object to a numpy array or multiple numpy arrays.

    Supports:

    * lists and tuples
    * RigidTransform: returned as 4x4 homogeneous coordinate transform
    * Matrix3, Rotation: returned as 3x3 matrix. Can't be determined
      with 'auto', need to specify type='Matrix3' or 'Rotation'.
    * Configs
    * Trajectory: returns a pair (times,milestones)
    * TriangleMesh: returns a pair (verts,indices)
    * PointCloud: returns a n x (3+k) array, where k is the # of properties
    * VolumeGrid: returns a triple (bmin,bmax,array)
    * Geometry3D: returns a pair (T,geomdata)

    If you want to get a transformed point cloud or mesh, you can pass in a
    Geometry3D as the obj, and its geometry data type as the type.
    """
    global SUPPORTED_TYPES 
    if type == 'auto':
        otype = types.object_to_type(obj,SUPPORTED_TYPES)
        if otype is None and type=='auto':
            raise ValueError('obj is not a supported type: '+', '.join(otype))
        type = otype
    if type not in SUPPORTED_TYPES:
        raise ValueError(type+' is not a supported type')
    if type == 'RigidTransform':
        return np.array(se3.homogeneous(obj))
    elif type == 'Rotation' or type == 'Matrix3':
        return np.array(so3.matrix(obj))
    elif type == 'Trajectory':
        return np.array(obj.times),np.array(obj.milestones)
    elif type == 'TriangleMesh':
        from klampt import Geometry3D
        if isinstance(obj,Geometry3D):
            tm = obj.getTriangleMesh()
            res = to_numpy(tm,type)
            res = (res[0],res[1].copy())
            R = to_numpy(obj.getCurrentTransform()[0],'Matrix3')
            t = to_numpy(obj.getCurrentTransform()[1],'Vector3')
            return (np.dot(R,res[0])+t,res[1])
        return (obj.getVertices(),obj.getIndices())
    elif type == 'PointCloud':
        from klampt import Geometry3D
        if isinstance(obj,Geometry3D):
            pc = obj.getPointCloud()
            res = to_numpy(pc,type)
            R = to_numpy(obj.getCurrentTransform()[0],'Matrix3')
            t = to_numpy(obj.getCurrentTransform()[1],'Vector3')
            res[:,:3] = np.dot(R,res[:,:3])+t
            return res
        points = obj.getPoints()
        if obj.numProperties() == 0:
            return points
        properties = obj.getAllProperties()
        return np.hstack((points,properties))
    elif type == 'VolumeGrid':
        bmin = np.array(obj.bbox)[:3]
        bmax = np.array(obj.bbox)[3:]
        values = obj.getValues()
        return (bmin,bmax,values)
    elif type == 'Geometry3D':
        if obj.type() == 'PointCloud':
            pc = obj.getPointCloud()
            pcdata = to_numpy(pc,obj.type())
            return to_numpy(obj.getCurrentTransform(),'RigidTransform'),pcdata
        elif obj.type() == 'TriangleMesh':
            mesh = obj.getTriangleMesh()
            meshdata = to_numpy(mesh,obj.type())
            meshdata = (meshdata[0].copy(),meshdata[1].copy())
            return to_numpy(obj.getCurrentTransform(),'RigidTransform'),meshdata
        elif obj.type() == 'VolumeGrid':
            grid = obj.getVolumeGrid()
            griddata = to_numpy(grid,obj.type())
            griddata = (griddata[0],griddata[1],griddata[2].copy())
            return to_numpy(obj.getCurrentTransform(),'RigidTransform'),griddata
        elif obj.type() == 'Group':
            arrays = []
            for i in range(obj.numElements()):
                arrays.append(to_numpy(obj.getElement(i),'Geometry3D'))
            return to_numpy(obj.getCurrentTransform(),'RigidTransform'),arrays
    else:
        return np.array(obj)


def from_numpy(obj,type='auto',template=None):
    """Converts a numpy array or multiple numpy arrays to a Klamp't object.

    Supports:

    * lists and tuples
    * RigidTransform: accepts a 4x4 homogeneous coordinate transform
    * Matrix3, Rotation: accepts a 3x3 matrix.
    * Configs
    * Trajectory: accepts a pair (times,milestones)
    * TriangleMesh: accepts a pair (verts,indices)
    * PointCloud: accepts a n x (3+k) array, where k is the # of properties
    * VolumeGrid: accepts a triple (bmin,bmax,array)
    * Geometry3D: accepts a pair (T,geomdata)
    """
    global SUPPORTED_TYPES 
    if type == 'auto' and template is not None:
        otype = types.object_to_types(template)
        if isinstance(otype,(list,tuple)):
            for t in otype:
                if t in SUPPORTED_TYPES:
                    type = t
                    break
            if type == 'auto':
                raise ValueError('obj is not a supported type: '+', '.join(otype))
        else:
            type = otype
    if type == 'auto':
        if isinstance(obj,(tuple,list)):
            if all(isinstance(v,np.ndarray) for v in obj):
                if len(obj)==2:
                    if len(obj[0].shape) == 1 and len(obj[1].shape) == 2:
                        type = 'Trajectory'
                    elif len(obj[0].shape) == 2 and len(obj[1].shape) == 2 and obj[0].shape[1] == 3 and obj[1].shape[1] == 3:
                        type = 'TriangleMesh'
                if len(obj)==3:
                    if obj[0].shape == (3,) and obj[1].shape == (3,):
                        type = 'VolumeGrid'
                if type == 'auto':
                    raise ValueError("Can't auto-detect type of list of shapes"+', '.join(str(v.shape) for v in obj))
            else:
                if isinstance(obj[0],np.ndarray) and obj[0].shape == (4,4):
                    type = 'Geometry3D'
                else:
                    raise ValueError("Can't auto-detect type of irregular list")
        else:
            assert isinstance(obj,np.ndarray),"Can only convert lists, tuples, and arrays from numpy"
            if obj.shape == (3,3):
                type = 'Matrix3'
            elif obj.shape == (4,4):
                type = 'RigidTransform'
            elif len(obj.shape) == 1:
                type = 'Config'
            else:
                raise ValueError("Can't auto-detect type of matrix of shape "+str(obj.shape))
    if type not in SUPPORTED_TYPES:
        raise ValueError(type+' is not a supported type')
    if type == 'RigidTransform':
        return se3.from_homogeneous(obj)
    elif type == 'Rotation' or type == 'Matrix3':
        return so3.from_matrix(obj)
    elif type == 'Trajectory':
        assert len(obj)==2,"Trajectory format is (times,milestones)"
        times = obj[0].tolist()
        milestones = obj[1].tolist()
        if template is not None:
            return template.constructor()(times,milestones)
        from klampt.model.trajectory import Trajectory
        return Trajectory(times,milestones)
    elif type == 'TriangleMesh':
        from klampt import TriangleMesh
        res = TriangleMesh()
        res.setVertices(obj[0].astype(float))
        res.setIndices(obj[1].astype(np.int32))
        return res
    elif type == 'PointCloud':
        from klampt import PointCloud
        assert len(obj.shape) == 2,"PointCloud array must be a 2D array"
        assert obj.shape[1] >= 3,"PointCloud array must have at least 3 values"
        #points = obj[:,:3]
        #properties = obj[:,3:]
        numproperties = obj.shape[1]-3
        res = PointCloud()
        #res.setPoints(points)
        res.setPointsAndProperties(obj.astype(float))
        if template is not None:
            if len(template.propertyNames) != numproperties:
                raise ValueError("Template object doesn't have the same properties as the numpy object")
            res.propertyNames.resize(len(template.propertyNames))
            for i in range(len(template.propertyNames)):
                res.propertyNames[i] = template.propertyNames[i]
        else:
            for i in range(numproperties):
                res.propertyNames.append('property %d'%(i+1))
        #if len(res.propertyNames) > 0:
        #    res.properties.resize(len(res.propertyNames)*points.shape[0])
        #if obj.shape[1] >= 3:
        #    res.setProperties(properties)
        return res
    elif type == 'VolumeGrid':
        from klampt import VolumeGrid
        assert len(obj) == 3,"VolumeGrid format is (bmin,bmax,values)"
        assert len(obj[2].shape) == 3,"VolumeGrid values must be a 3D array"
        bmin = obj[0]
        bmax = obj[1]
        values = obj[2]
        res = VolumeGrid()
        res.bbox.append(bmin[0])
        res.bbox.append(bmin[1])
        res.bbox.append(bmin[2])
        res.bbox.append(bmax[0])
        res.bbox.append(bmax[1])
        res.bbox.append(bmax[2])
        res.dims.append(values.shape[0])
        res.dims.append(values.shape[1])
        res.dims.append(values.shape[2])
        res.setValues(values.astype(float))
        return res
    elif type == 'Group':
        from klampt import Geometry3D
        res = Geometry3D()
        assert isinstance(obj,(list,tuple)),"Group format is a list or tuple of Geometry3D's"
        for i in range(len(obj)):
            res.setElement(i,from_numpy(obj[i],'Geometry3D'))
        return res
    elif type == 'Geometry3D':
        from klampt import Geometry3D
        if not isinstance(obj,(list,tuple)) or len(obj) != 2:
            raise ValueError("Geometry3D must be a (transform,geometry) tuple")
        T = from_numpy(obj[0],'RigidTransform')
        geomdata = obj[1]
        subtype = None
        if template is not None:
            subtype = template.type()
            if subtype == 'PointCloud':
                g = Geometry3D(from_numpy(geomdata,subtype,template.getPointCloud()))
            else:
                g = Geometry3D(from_numpy(geomdata,subtype))
            g.setCurrentTransform(*T)
            return g
        subtype = 'Group'
        if all(isinstance(v,np.ndarray) for v in geomdata):
            if len(geomdata)==2:
                if len(geomdata[0].shape) == 1 and len(geomdata[1].shape) == 2:
                    subtype = 'Trajectory'
                elif len(geomdata[0].shape) == 2 and len(geomdata[1].shape) == 2 and geomdata[0].shape[1] == 3 and geomdata[1].shape[1] == 3:
                    subtype = 'TriangleMesh'
            if len(geomdata)==3:
                if geomdata[0].shape == (3,) and geomdata[1].shape == (3,):
                    subtype = 'VolumeGrid'
        g = Geometry3D(from_numpy(obj,subtype))
        g.setCurrentTransform(*T)
        return g
    else:
        return obj.flatten()

from .loader import _DeprecatedList
supportedTypes = _DeprecatedList("supportedTypes","SUPPORTED_TYPES",SUPPORTED_TYPES)
