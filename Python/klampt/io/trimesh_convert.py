import trimesh
import trimesh.visual
from klampt import Geometry3D,TriangleMesh,WorldModel,Appearance
from klampt.math import se3
from typing import Union,Optional
import numpy as np

def to_trimesh(geom : Union[TriangleMesh,Geometry3D,WorldModel], appearance:Optional[Appearance]=None) -> Union[trimesh.Trimesh,trimesh.Scene]:
    """Converts a Klampt Geometry3D, TriangleMesh, or WorldModel to a Trimesh
    object or Scene.

    Args:
        geom (Union[TriangleMesh,Geometry3D,WorldModel]): the input geometry 
            or scene.
        appearance (Appearance): the appearance of the input geometry.

    Returns:
        trimesh.TriangleMesh or trimesh.Scene: the output trimesh object or
        scene.
    """
    if isinstance(geom,TriangleMesh):
        res = trimesh.Trimesh(vertices=geom.getVertices(),faces=geom.getIndices())
        if appearance is not None:
            col = appearance.getColor()
            res.visual = trimesh.visual.ColorVisuals(res,face_colors=col)
        return res
    elif isinstance(geom,Geometry3D):
        if geom.type() != "TriangleMesh":
            raise ValueError("Can only convert TriangleMesh objects to Trimesh")
        return to_trimesh(geom.getTriangleMesh(), appearance)
    elif isinstance(geom,WorldModel):
        res = trimesh.Scene()
        for i in geom.numRobots():
            r = geom.robot(i)
            for j in r.numLinks():
                lj = r.link(j)
                g = lj.geometry()
                if g.numElements() > 0:
                    tg = to_trimesh(g)
                    if lj.getParent() >= 0:
                        lp = r.link(lj.getParent())
                        parent = r.name + ' ' + lp.name
                        Tp = se3.ndarray(se3.mul(se3.inv(lp.getTransform()),lj.getTransform()))
                    else:
                        parent = None
                        Tp = se3.ndarray(lj.getTransform())
                    res.add_geometry(tg,node_name=r.name + ' '+lj.name,parent_node_name=parent,
                                     transform=Tp)
        for i in geom.numRigidObjects():
            r = geom.rigidObject(i)
            g = r.geometry()
            if g.numElements() > 0:
                tg = to_trimesh(g)
                res.add_geometry(tg,node_name=r.getName(),transform=se3.ndarray(r.getTransform()))
        for i in geom.numTerrains():
            r = geom.terrain(i)
            g = r.geometry()
            if g.numElements() > 0:
                tg = to_trimesh(g)
                res.add_geometry(tg,node_name=r.getName(),transform=se3.ndarray(se3.identity()))
    else:
        raise ValueError("Invalid input type")

def from_trimesh(mesh : Union[trimesh.Trimesh,trimesh.Scene],flatten=False) -> Union[TriangleMesh,WorldModel]:
    """Converts a Trimesh object to a Klampt TriangleMesh, or a Trimesh Scene
    to a WorldModel. 

    Args:
        mesh (trimesh.Trimesh or trimesh.Scene): the input trimesh object or
            Scene.
        flatten (bool): if True, flattens a scene into a TriangleMesh object.

    Returns:
        TriangleMesh or WorldModel: the output Klampt TriangleMesh object or
        WorldModel scene.  Scenes include all geometries as rigidObjects.
    """
    if isinstance(mesh,trimesh.Trimesh):
        m = TriangleMesh()
        m.setVertices(mesh.vertices.astype(np.float32))
        m.setIndices(mesh.faces.astype(np.int32))
        return m
    elif isinstance(mesh,trimesh.Scene):
        res = WorldModel()
        for node_name in mesh.graph.nodes_geometry:
            # which geometry does this node refer to
            transform, geometry_name = mesh.graph[node_name]

            # get the actual potential mesh instance
            geometry = mesh.geometry[geometry_name]
            robj = res.makeRigidObject(node_name)
            m = from_trimesh(geometry)
            assert isinstance(m,TriangleMesh),"Hmm... geometry has type other than TriangleMesh? {}".format(geometry.__class__.__name__)
            robj.geometry().set(Geometry3D(m))
            robj.setTransform(*se3.from_ndarray(transform))
            if isinstance(geometry.visual,trimesh.visual.ColorVisuals):
                robj.appearance().setColors(Appearance.FACES,np.array(geometry.visual.face_colors))
        if flatten:
            if res.numRigidObjects() == 1:
                return res.rigidObject(0).geometry().getTriangleMesh()
            from ..model import geometry
            return geometry.merge(*[res.rigidObject(i).geometry() for i in range(res.numRigidObjects())])
        return res

