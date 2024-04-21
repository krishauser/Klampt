import trimesh
from klampt import Geometry3D,TriangleMesh
from typing import Union

def to_trimesh(geom : Union[TriangleMesh,Geometry3D]) -> trimesh.Trimesh:
    """Converts a Klampt Geometry3D or TriangleMesh to a Trimesh object.

    Args:
        geom (Union[TriangleMesh,Geometry3D]): the input geometry

    Returns:
        trimesh.TriangleMesh: the output trimesh object
    """
    if isinstance(geom,TriangleMesh):
        return trimesh.Trimesh(vertices=geom.getVertices(),faces=geom.getIndices())
    elif isinstance(geom,Geometry3D):
        if geom.type() != "TriangleMesh":
            raise ValueError("Can only convert TriangleMesh objects to Trimesh")
        return to_trimesh(geom.getTriangleMesh())
    else:
        raise ValueError("Invalid input type")

def from_trimesh(mesh : trimesh.Trimesh) -> TriangleMesh:
    """Converts a Trimesh object to a Klampt TriangleMesh.  You can wrap this
    in a Geometry3D object if you need it, i.e., 
    ``Geometry3D(from_trimesh(mesh))``.

    Args:
        mesh (trimesh.Trimesh): the input trimesh object

    Returns:
        TriangleMesh: the output Klampt TriangleMesh object
    """
    res = TriangleMesh()
    res.setVertices(mesh.vertices)
    res.setIndices(mesh.faces)
    return res

