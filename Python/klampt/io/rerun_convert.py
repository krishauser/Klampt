"""Conversions to and from the Rerun.io visualization library

Rerun can be installed using pip as follows::

    pip install rerun-sdk

"""

import rerun as rr
import numpy as np
import klampt
from klampt.math import so3,se3,vectorops
from klampt.model.trajectory import Trajectory, SE3Trajectory
from klampt.model.typing import Rotation, RigidTransform
from typing import Union

def trimesh_to_rr(mesh : klampt.TriangleMesh, appearance : klampt.Appearance = None) -> rr.Mesh3D:
    vertex_colors = None
    vertex_texcoords = None
    albedo_texture = None
    albedo_factor = None
    normals = mesh.vertexNormals()
    if appearance is not None:
        vertex_colors = appearance.getColors(klampt.Appearance.VERTICES)
        if vertex_colors.shape[0] == 1:  #no vertex colors
            if appearance.getColor() is not None:
                # uniform color
                vertex_colors = np.array([appearance.getColor()]*len(mesh.vertices)) 
        fmt = appearance.getTexture2D_format()
        if fmt:
            try:
                vertex_texcoords = appearance.getTexcoords2D()
            except Exception:
                vertex_texgen = appearance.getTexgenMatrix()
                if vertex_texgen.shape[0] != 2:
                    raise ValueError("Texture gen matrix must be 2x4")
                v = mesh.vertices
                v4 = np.hstack((v,np.ones((v.shape[0],1))))
                vertex_texcoords = np.dot(v4,vertex_texgen.T)
            albedo_texture = appearance.getTexture2D_channels()
            #only RGB / RGBA are accepted
            rr_fmt = rr.ColorModel.RGBA
            if fmt.startswith('rgba'):
                pass
            elif fmt.startswith('rgb'):
                rr_fmt = rr.ColorModel.RGB
            elif fmt.startswith('bgra'):
                rr_fmt = rr.ColorModel.RGBA
                albedo_texture[:,:,:3] = albedo_texture[:,:,:3][:,:,-1]
            elif fmt.startswith('bgr'):
                rr_fmt = rr.ColorModel.RGB
                albedo_texture[:,:,:3] = albedo_texture[:,:,-1]
            else:
                rr_fmt = rr.ColorModel.RGB
                albedo_texture = np.stack([albedo_texture[:,:,0]]*3,axis=2)
            albedo_factor = vertex_colors[0]
            vertex_colors = None
    return rr.Mesh3D(vertex_positions = mesh.getVertices(),
                     triangle_indices = mesh.getIndices(),
                     vertex_normals = normals,
                     vertex_colors=vertex_colors,
                     vertex_texcoords=vertex_texcoords,
                     albedo_texture=albedo_texture,
                     albedo_factor=albedo_factor)

def point_cloud_to_rr(pc : klampt.PointCloud, appearance : klampt.Appearance = None) -> rr.Points3D:
    point_colors = None
    if appearance is not None:
        try:
            point_colors = appearance.getColors(klampt.Appearance.VERTICES)
        except Exception:
            if appearance.getColor() is not None:
                point_colors = np.array([appearance.getColor()]*len(pc.points))
    pc_colors = pc.getColors(('r','g','b'))
    if point_colors is None:
        point_colors = pc_colors
    elif pc_colors is not None:
        point_colors = np.multiply(point_colors,pc_colors)
    return rr.Points3D(positions = pc.points, colors = point_colors)

def geom_to_rr(geom : klampt.Geometry3D, appearance : klampt.Appearance = None) -> Union[rr.Mesh3D,rr.Points3D]:
    """Returns a rerun archetype corresponding to the given geometry.
    
    Note that the rerun object is in the geometry's local frame and will need
    its transform updated.
    """
    if geom.empty():
        return
    if geom.type() == 'TriangleMesh':
        return trimesh_to_rr(geom.getTriangleMesh(),appearance)
    elif geom.type() == 'PointCloud':
        return point_cloud_to_rr(geom.getPointCloud(),appearance)
    else:
        tmp = geom.convert('TriangleMesh')
        return trimesh_to_rr(tmp.getTriangleMesh(),appearance)

def transform_to_rr(T : RigidTransform) -> rr.Transform3D:
    rot = rr.RotationAxisAngle(*so3.axis_angle(T[0]))
    return rr.Transform3D(translation = T[1], rotation = rot)

def point_to_rr(pt : list, radius = None, color = None) -> rr.Points3D:
    if radius is None:
        radii = None
    else:
        radii = [radius]
    if color is None:
        colors = None
    else:
        colors = [color]
    return rr.Points3D([pt], radii=radii, colors = colors)

def camera_to_rr(camera : klampt.SensorModel) -> rr.Pinhole:
    if camera.type() != 'CameraSensor':
        raise ValueError("Can only convert CameraSensor types to rerun")
    from klampt.model.sensing import camera_to_intrinsics
    vals = camera_to_intrinsics(camera,'json')
    w = int(camera.getSetting('xres'))
    h = int(camera.getSetting('yres'))
    return rr.Pinhole(width=w,height=h, focal_length = vals['fx'], principal_point = [vals['cx'],vals['cy']])

def traj_to_rr(traj : Trajectory, radius = None, color = None) -> Union[rr.LineStrips2D,rr.LineStrips3D]:
    """Converts a klampt.Trajectory to a rerun line strips."""
    if len(traj.milestones) == 0:
        return None
    if radius is None:
        radii = None
    else:
        radii = [radius]
    if color is None:
        colors = None
    else:
        colors = [color]
    if len(traj.milestones[0]) == 2:
        return rr.LineStrips2D(strips=[traj.milestones],radii=radii,colors=colors)
    elif len(traj.milestones[0]) == 3:
        return rr.LineStrips3D(strips=[traj.milestones],radii=radii,colors=colors)
    else:
        raise ValueError("Trajectory must be 2D or 3D")

def log_rr_geometry(geom : klampt.Geometry3D, name : str, appearance : klampt.Appearance = None, hide_label = False, color = None):
    geom_rr = geom_to_rr(geom,appearance)
    if geom_rr is not None:
        if not hide_label:
            geom_rr.labels = [name] 
        if color is not None:
            geom_rr.vertex_colors = [color]*len(geom_rr.vertex_positions)
        rr.log(name,geom_rr,static=True)
        update_rr_transform(geom.getCurrentTransform(),name)

def log_rr_robot(robot : klampt.RobotModel, prefix : str = '', hide_labels = False, color = None):
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    for i in range(robot.numLinks()):
        name = prefix+robot.name + '/' + robot.link(i).name
        geom = geom_to_rr(robot.link(i).geometry(), robot.link(i).appearance())
        if geom is not None:
            if not hide_labels:
                geom.labels = [robot.link(i).name] 
            if color is not None:
                geom.vertex_colors = [color]*len(geom.vertex_positions)
            rr.log(name,geom,static=True)
            update_rr_transform(robot.link(i).getTransform(),name)

def log_rr_terrain(terrain : klampt.TerrainModel, prefix : str = '', hide_label = False, color = None):
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    name = prefix+terrain.name
    geom = geom_to_rr(terrain.geometry(),terrain.appearance())
    if geom is not None:
        if not hide_label:
            geom.labels = [terrain.name] 
            if color is not None:
                geom.vertex_colors = [color]*len(geom.vertex_positions)
        rr.log(name,geom,static=True)

def log_rr_rigid_object(rigid_object : klampt.RigidObjectModel, prefix : str = '', hide_label = False, color = None):
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    name = prefix+rigid_object.name
    geom = geom_to_rr(rigid_object.geometry(),rigid_object.appearance())
    if geom is not None:
        if not hide_label:
            geom.labels = [rigid_object.name] 
        if color is not None:
            geom.vertex_colors = [color]*len(geom.vertex_positions)
        rr.log(name,geom,static=True)
        update_rr_transform(rigid_object.getTransform(),name)

def log_rr_transform(T : RigidTransform, name : str, length = 0.1, hide_label = False):
    """Logs a new transform to the rerun log."""
    labels = None
    if not hide_label:
        labels = [name,None,None]
    rr.log(name,
        rr.Arrows3D(
            vectors=[[length, 0, 0], [0, length, 0], [0, 0, length]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            labels=labels
        ),static=True)
    update_rr_transform(T, name)

def log_rr_world(world : klampt.WorldModel, prefix : str = '', hide_labels = False):
    """Logs all elements in a world to a rerun log."""
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    for i in range(world.numRobots()):
        log_rr_robot(world.robot(i), prefix, hide_labels)
    for i in range(world.numTerrains()):
        log_rr_terrain(world.terrain(i), prefix, hide_labels)
    for i in range(world.numRigidObjects()):
        log_rr_rigid_object(world.rigidObject(i), prefix, hide_labels)

def log_rr_trajectory(traj : Union[Trajectory,SE3Trajectory], name : str, color = None, hide_label = False):
    """Logs a trajectory to the rerun log."""
    if isinstance(traj, SE3Trajectory):
        log_rr_trajectory(traj.getPositionTrajectory(), name, hide_label)
        #show milestones
        spacing = 1 if len(traj.milestones) < 10 else len(traj.milestones)//10
        for i in range(0,len(traj.times),spacing):
            T = traj.to_se3(traj.milestones[i])
            log_rr_transform(T,name + '/T' + str(i))
    else:
        lines = traj_to_rr(traj, color=color)
        if not hide_label:
            lines.labels = [name]
        rr.log(name,lines,static=True)

def update_rr_transform(T : RigidTransform, name : str):
    """Updates the transform of a single entity in the log"""
    rr.log(name, transform_to_rr(T))

def update_rr_robot(robot : klampt.RobotModel, prefix : str = ''):
    """Updates the transforms of all links in the robot to the rerun log.
    """
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    for j in range(robot.numLinks()):
        if robot.link(j).geometry().empty():
            continue
        T = robot.link(j).getTransform()
        update_rr_transform(T, prefix + robot.name + '/' + robot.link(j).name)

def update_rr_rigid_object(rigid_object : klampt.RigidObjectModel, prefix : str = ''):
    """Updates the transform of a single rigid object in the log"""
    if rigid_object.geometry().empty():
        return
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    T = rigid_object.getTransform()
    update_rr_transform(T, prefix + rigid_object.getName())

def update_rr_world(world : klampt.WorldModel, prefix : str = ''):
    """Updates the transforms of all movable objects in the world to the
    rerun log.
    """
    if prefix != '' and not prefix.endswith('/'):
        prefix += '/'
    for i in range(world.numRobots()):
        update_rr_robot(world.robot(i), prefix)
    for i in range(world.numRigidObjects()):
        update_rr_rigid_object(world.rigidObject(i), prefix)