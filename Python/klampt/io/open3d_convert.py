"""Conversions to and from the Open3D library.

Open3D is useful for doing various point cloud processing routines.
It can be installed using pip as follows::

    pip install open3d-python

"""

import open3d
from klampt import PointCloud,TriangleMesh,VolumeGrid,Geometry3D
from ..math import se3
from ..model import geometry

def to_open3d(obj):
    """Converts Klamp't geometry to an open3d geometry.

    Geometry3D objects are converted applying the current transform.

    If the VolumeGrid is considered to be an occupancy grid (all values
    between 0 and 1), then it is converted to a VoxelGrid containing a
    voxel for any positive items.  If it is considered to be a SDF,
    then it contains a voxel of any non-positive items.  There may be
    issues converting from VolumeGrids whose cells are non-cubic.
    """
    if isinstance(obj,PointCloud):
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(obj.points)
        #TODO: other properties
        colors = geometry.point_cloud_colors(obj,('r','g','b'))
        if colors is not None:
            for i in range(len(obj.points)):
                pc.colors.append(colors[i])
        return pc
    elif isinstance(obj,TriangleMesh):
        m = open3d.geometry.TriangleMesh()
        m.vertices = open3d.utility.Vector3dVector(obj.vertices)
        m.triangles = open3d.utility.Vector3iVector(obj.indices)
        return m
    elif isinstance(obj,VolumeGrid):
        import numpy as np

        origin = obj.bmin
        dims = obj.values.shape
        cx = (obj.bmax[0]-obj.bmin[0])/dims[0]
        cy = (obj.bmax[1]-obj.bmin[1])/dims[1]
        cz = (obj.bmax[2]-obj.bmin[2])/dims[2]
        voxel_size = pow(cx*cy*cz,1.0/3.0)
        
        values = obj.values
        vrange = np.min(values),np.min(values)
        if vrange[0] < 0 or vrange[1] > 1:
            #treat as SDF
            indices = np.nonzero(values <= 0)
        else:
            #treat as occupancy grid
            indices = np.nonzero(values > 0.5)

        pc = open3d.geometry.PointCloud()
        for i in indices[0]:
            cell = np.array([i//(dims[2]*dims[1]), (i//dims[2]) % dims[1], i%dims[2]])
            pt = (cell + 0.5)*voxel_size + origin
            pc.points.append(pt)
        vg = open3d.geometry.VoxelGrid()
        vg.create_from_point_cloud(pc,voxel_size)
        return vg
        #deprecated in open3d 0.8.0
        #return open3d.geometry.create_surface_voxel_grid_from_point_cloud(pc,voxel_size)
    elif isinstance(obj,Geometry3D):
        if obj.type() == 'PointCloud':
            pc = obj.copy().getPointCloud()
            pc.transform(*obj.getCurrentTransform())
            return to_open3d(pc)
        elif obj.type() == 'TriangleMesh':
            m = obj.copy().getTriangleMesh()
            m.transform(*obj.getCurrentTransform())
            return to_open3d(m)
        else:
            raise ValueError("Can't convert Geometry3D of type "+obj.type()+" yet")
    raise TypeError("Invalid type")

def from_open3d(obj):
    """Converts open3d geometry to a Klamp't geometry.
    """
    if isinstance(obj,open3d.geometry.PointCloud):
        import numpy as np
        pc = PointCloud()
        pc.points = np.asarray(obj.points)
        if obj.has_colors():
            geometry.point_cloud_set_colors(pc,np.asarray(obj.colors),('r','g','b'),'rgb')
        #TODO: other properties
        return pc
    elif isinstance(obj,open3d.geometry.TriangleMesh):
        import numpy as np
        m = TriangleMesh()
        m.vertices = np.asarray(obj.vertices)
        m.indices = np.asarray(obj.triangles)
        return m
    elif isinstance(obj,open3d.geometry.VoxelGrid):
        grid = VolumeGrid()
        import numpy as np
        if hasattr(obj, 'voxels'):
            # open3d <= 0.9.0
            occupied = np.array(obj.voxels,dtype=np.int32)
        else:
            occupied = np.array([v.grid_index for v in obj.get_voxels()],dtype=np.int32)
        if len(occupied) == 0:
            #empty grid?
            return grid
        imin = np.min(occupied,axis=0)
        imax = np.max(occupied,axis=0)
        assert imin.shape == (3,)
        assert imax.shape == (3,)
        #TODO: fix this up
        import warnings
        warnings.warn("from_open3d: VoxelGrid conversion may not be correct")
        bmin = obj.origin + (imin-1)*obj.voxel_size
        bmax = obj.origin + (imax+2)*obj.voxel_size
        grid.bmin = bmin
        grid.bmax = bmax
        grid.values = occupied.astype(float)
        return grid
    raise TypeError("Invalid type")
