"""Utilities for creating geometric primitives (and world entities made out
of them).
"""

from klampt import Geometry3D,GeometricPrimitive,Mass
from klampt.math import vectorops

def box(width,depth,height,center=None,R=None,t=None,world=None,name=None,mass=float('inf'),type='TriangleMesh'):
    """Makes a box with dimensions width x depth x height.  The box is centered
    at (0,0,0) by default.

    Args:
        width,depth,height (float): x,y,z dimensions of the box
        center (list of 3 floats, optional): if None (typical),
            the *geometry* of the box is centered at 0. Otherwise,
            the *geometry* of the box is shifted relative to the
            box's local coordinate system.
        R,t (se3 transform, optional): if given, the box's world coordinates
            will be rotated and shifted by this transform.
        world (WorldModel, optional): If given, then the box will be a
            RigidObjectModel or TerrainModel will be created in this world
        name (str, optional): If world is given, this is the name of the object. 
            Default is 'box'.
        mass (float, optional): If world is given and this is inf, then a
            TerrainModel will be created. Otherwise, a RigidObjectModel
            will be created with automatically determined inertia.
        type (str, optional): the geometry type.  Defaults to 'TriangleMesh',
            but also 'GeometricPrimitive' and 'VolumeGrid' are accepted.

    Returns:
        Geometry3D, RigidObjectModel, or TerrainModel: A representation
        of the box.  If a world is given, then either a RigidObjectModel
        or TerrainModel is added to the world and returned.
    """
    if center is None:
        center = [0,0,0]
    prim = GeometricPrimitive()
    prim.setAABB([center[0]-width*0.5,center[1]-depth*0.5,center[2]-height*0.5],[center[0]+width*0.5,center[1]+depth*0.5,center[2]+height*0.5])
    geom = Geometry3D(prim)
    if type != 'GeometricPrimitive':
        geom = geom.convert(type)
    if world is None:
        if R is not None and t is not None:
            geom.setCurrentTransform(R,t)
        return geom

    #want a RigidObjectModel or TerrainModel
    if name is None:
        name = 'box'
    if mass != float('inf'):
        bmass = Mass()
        bmass.setMass(mass)
        bmass.setCom(center)
        bmass.setInertia([mass*(depth**2+height**2)/12,mass*(width**2+height**2)/12,mass*(width**2+height**2)/12])
        robj = world.makeRigidObject(name)
        robj.geometry().set(geom)
        robj.setMass(bmass)
        if R is not None and t is not None:
            robj.setTransform(R,t)
        return robj
    else:
        tobj = world.makeTerrain(name)
        if R is not None and t is not None:
            geom.transform(R,t)
        tobj.geometry().set(geom)
        return tobj

def sphere(radius,center=None,R=None,t=None,world=None,name=None,mass=float('inf'),type='TriangleMesh'):
    """Makes a sphere with the given radius

    Args:
        radius (float): radius of the sphere
        center (list of 3 floats, optional): if None (typical), the *geometry*
            of the sphere is centered at 0. Otherwise, the *geometry* of
            the sphere is shifted relative to the sphere's local coordinate system.
        R,t (se3 transform, optional): if given, the sphere's world coordinates
            will be rotated and shifted by this transform.
        world (WorldModel, optional): If given, then the sphere will be a
            RigidObjectModel or TerrainModel will be created in this world
        name (str, optional): If world is given, this is the name of the object. 
            Default is 'sphere'.
        mass (float, optional): If world is given and this is inf, then a
            TerrainModel will be created. Otherwise, a RigidObjectModel
            will be created with automatically determined inertia.
        type (str, optional): the geometry type.  Defaults to 'TriangleMesh',
            but also 'GeometricPrimitive' and 'VolumeGrid' are accepted.

    Returns:
        Geometry3D, RigidObjectModel, or TerrainModel: A representation
        of the sphere.  If a world is given, then either a RigidObjectModel
        or TerrainModel is added to the world and returned.
    """
    if center is None:
        center = [0,0,0]
    prim = GeometricPrimitive()
    prim.setSphere(center,radius)
    geom = Geometry3D(prim)
    if type != 'GeometricPrimitive':
        geom = geom.convert(type)
    if world is None:
        if R is not None and t is not None:
            geom.setCurrentTransform(R,t)
        return geom

    #want a RigidObjectModel or TerrainModel
    if name is None:
        name = 'sphere'
    if mass != float('inf'):
        bmass = Mass()
        bmass.setMass(mass)
        bmass.setCom(center)
        bmass.setInertia([0.4*mass*radius**2]*3)
        robj = world.makeRigidObject(name)
        robj.geometry().set(geom)
        robj.setMass(bmass)
        if R is not None and t is not None:
            robj.setTransform(R,t)
        return robj
    else:
        tobj = world.makeTerrain(name)
        if R is not None and t is not None:
            geom.transform(R,t)
        tobj.geometry().set(geom)
        return tobj

def bbox(bmin,bmax,R=None,t=None,world=None,name=None,mass=float('inf'),type='TriangleMesh'):
    """Makes a box from bounds [bmin,bmax].

    Args:
        bmin (list of 3 floats): the lower corner of the box
        center (list of 3 floats): the upper corner of the box
        R,t (se3 transform, optional): if given, the box's world coordinates
            will be rotated and shifted by this transform.
        world (WorldModel, optional): If given, then the box will be a
            RigidObjectModel or TerrainModel will be created in this world
        name (str, optional): If world is given, this is the name of the object. 
            Default is 'box'.
        mass (float, optional): If world is given and this is inf, then a
            TerrainModel will be created. Otherwise, a RigidObjectModel
            will be created with automatically determined inertia.
        type (str, optional): the geometry type.  Defaults to 'TriangleMesh',
            but also 'GeometricPrimitive' and 'VolumeGrid' are accepted.

    Returns:
        Geometry3D, RigidObjectModel, or TerrainModel: A representation
        of the box.  If a world is given, then either a RigidObjectModel
        or TerrainModel is added to the world and returned.
    """
    w,d,h = vectorops.sub(bmax,bmin)
    center = vectorops.interpolate(bmin,bmax,0.5)
    return box(w,d,h,center,R,t,world,name,mass,type)
