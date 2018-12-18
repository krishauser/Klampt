from klampt import *

def box(width,depth,height,center=None,R=None,t=None,world=None,name=None,mass=float('inf'),type='TriangleMesh'):
    """Makes a box with dimensions width x depth x height. 

    Parameters:
    - width,depth,height: x,y,z dimensions of the box
    - center: if None (typical), the *geometry* of the box is centered at 0. Otherwise, the *geometry* of
      the box is shifted relative to the box's local coordinate system.
    - R,t: if given, the box's world coordinates will be rotated and shifted by this transform.
    - world: If given, then a RigidObjectModel or TerrainModel will be created in this world
    - name: If world is given, this is the name of the object.  Default 'box'.
    - mass: If world is given and this is inf, then a TerrainModel will be created. Otherwise, a RigidObjectModel
      will be created with automatically determined inertia.
    - type: the geometry type.  Defaults to 'TriangleMesh', but also 'GeometricPrimitive' and 'VolumeGrid' are reasonable.

    Returns either a Geometry3D, RigidObjectModel, or TerrainModel.  In the latter two cases, the box is
    added to the world.
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
        bmass.setInertia([width/12,depth/12,height/12])
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

    Parameters:
    - radius: radius of the sphere
    - center: if None (typical), the *geometry* of the sphere is centered at 0. Otherwise, the *geometry* of
      the sphere is shifted relative to the sphere's local coordinate system.
    - R,t: if given, the spheres's world coordinates will be rotated and shifted by this transform.
    - world: If given, then a RigidObjectModel or TerrainModel will be created in this world
    - name: If world is given, this is the name of the object.  Default 'sphere'.
    - mass: If world is given and this is inf, then a TerrainModel will be created. Otherwise, a RigidObjectModel
      will be created with automatically determined inertia.
    - type: the geometry type.  Defaults to 'TriangleMesh', but also 'GeometricPrimitive' and 'VolumeGrid' are reasonable.

    Returns either a Geometry3D, RigidObjectModel, or TerrainModel.  In the latter two cases, the sphere is
    added to the world.
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
        bmass.setInertia([width/12,depth/12,height/12])
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
