import trimesh
import trimesh.visual
from klampt import Geometry3D,TriangleMesh,WorldModel,Appearance
from klampt.math import so3,se3
from typing import Union,Optional,Tuple
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
        res = trimesh.Trimesh(vertices=geom.vertices,faces=geom.indices)
        if appearance is not None:
            col = appearance.getColor()
            #TODO: textured objects
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

def _set_texture(a : Appearance, img):
    mode = img.mode
    if mode == 'P':
        img = img.convert('RGB')
        _set_texture(a,img)
        return
    img = np.array(img)
    #print("IMAGE",img.shape)
    #print("IMAGE MODE",mesh.visual.material.baseColorTexture.mode)
    if len(img.shape) == 3:
        if mode == 'RGB':
            a.setTexture2D('rgb8',img)
        elif mode == 'RGBA':
            a.setTexture2D('rgba8',img)
        elif mode.startswith('L'):
            a.setTexture2D('l8',img[:,:,0])
        else:
            print("from_trimesh: Can't handle texture mode",mode)
    elif len(img.shape) == 2:
        if mode == 'L':
            a.setTexture2D('l8',img)
        else:
            print("from_trimesh: Can't handle texture mode",mode)

def from_trimesh(mesh : Union[trimesh.Trimesh,trimesh.Scene],flatten=False) -> Union[Tuple[TriangleMesh,Appearance],WorldModel]:
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
        if mesh.visual.kind is None:
            a = Appearance()
            a.setColor(*[a/255.0 for a in mesh.visual.face_colors[0]])
        elif mesh.visual.kind == 'face':
            a = Appearance()
            a.setColors(Appearance.FACES,np.array(mesh.visual.face_colors)/255.0)
        elif mesh.visual.kind == 'vertex':
            a = Appearance()
            a.setColors(Appearance.VERTICES,np.array(mesh.visual.vertex_colors)/255.0)
        elif mesh.visual.kind == 'texture':
            if isinstance(mesh.visual.material,trimesh.visual.material.PBRMaterial):
                a = Appearance()
                if mesh.visual.material.baseColorFactor is not None:
                    a.setColor(*[a/255.0 for a in mesh.visual.material.baseColorFactor])
                else:
                    a.setColor(1,1,1,1)
                if mesh.visual.material.emissiveFactor is not None:
                    c = [a/255.0 for a in mesh.visual.material.emissiveFactor]
                    if len(c)==3:
                        c.append(1)
                    a.setColor(Appearance.EMISSIVE,*c)
                if mesh.visual.material.metallicFactor is not None and mesh.visual.material.roughnessFactor is not None:
                    #print("Shininess",mesh.visual.material.metallicFactor,mesh.visual.material.roughnessFactor)
                    a.setShininess(mesh.visual.material.metallicFactor,mesh.visual.material.roughnessFactor)
                if mesh.visual.material.baseColorTexture is not None:
                    _set_texture(a,mesh.visual.material.baseColorTexture)
                    a.setTexcoords2D(mesh.visual.uv)
            elif isinstance(mesh.visual.material,trimesh.visual.material.SimpleMaterial):
                a = Appearance()
                a.setColor(*[a/255.0 for a in mesh.visual.material.diffuse])
                if mesh.visual.material.ambient is not None:
                    c = [a/255.0 for a in mesh.visual.material.emissiveFactor]
                    if len(c)==3:
                        c.append(1)
                    a.setColor(Appearance.EMISSIVE,*c)
                if mesh.visual.material.specular is not None:
                    c = [a/255.0 for a in mesh.visual.material.specular]
                    if len(c)==3:
                        c.append(1)
                    a.setColor(Appearance.SPECULAR,*c)
                if mesh.visual.material.image is not None:
                    _set_texture(a,mesh.visual.material.image)
                    a.setTexcoords2D(mesh.visual.uv)
            else:
                print("from_trimesh: Can't handle material type",mesh.visual.material.__class__.__name__)
                a = Appearance()
        return m,a
    elif isinstance(mesh,trimesh.Scene):
        res = WorldModel()
        for node_name,transform_geom in mesh.graph.to_flattened().items():
            # which geometry does this node refer to
            transform = transform_geom['transform']
            geometry_name = transform_geom['geometry']
            if geometry_name is None:
                continue
            geometry = mesh.geometry[geometry_name]

            robj = res.makeRigidObject(node_name)
            m,a = from_trimesh(geometry)
            assert isinstance(m,TriangleMesh),"Hmm... geometry has type other than TriangleMesh? {}".format(geometry.__class__.__name__)
            robj.geometry().set(Geometry3D(m))
            robj.appearance().set(a)
            robj.appearance().refresh()
            #print(node_name,transform)
            R,t = se3.from_ndarray(transform)
            #somehow it's better not to transform?
            # u,s,vh = np.linalg.svd(so3.ndarray(R))
            # R = so3.from_ndarray(np.dot(u,vh))
            # print("  Rotation component",R)
            # print("  Translation component",t)
            # print("  Scaling component",s)
            # print("  Color",a.getColor())
            # robj.geometry().transform(so3.from_ndarray(np.diag(s)),[0,0,0])
            robj.setTransform(R,t)
        if flatten:
            geoms = []
            apps = []
            for i in range(res.numRigidObjects()):
                geoms.append(res.rigidObject(i).geometry())
                apps.append(res.rigidObject(i).appearance())
            if len(geoms) == 1:
                return geoms[0],apps[0]
            from ..model import geometry
            #merge geometries and apperances
            print("from_trimesh: Warning: merging appearances is not perfect, textures are ignored")
            merged_app = Appearance()
            face_colors = []
            for g,a in zip(geoms,apps):
                c = a.getColor()
                face_colors += [c]*g.numElements()
            merged_app.setColors(Appearance.FACES,np.array(face_colors,dtype=np.float32))
            return geometry.merge(*geoms),merged_app
        return res

