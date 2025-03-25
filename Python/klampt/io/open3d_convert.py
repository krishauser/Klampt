"""Conversions to and from the Open3D library.

Open3D is useful for doing various point cloud processing routines.
It can be installed using pip as follows::

    pip install open3d-python

"""

import open3d
from klampt import PointCloud,TriangleMesh,ImplicitSurface,OccupancyGrid,Geometry3D

def to_open3d(obj):
    """Converts Klamp't geometry to an open3d geometry.

    Geometry3D objects are converted applying the current transform.

    OccupancyGrids are is converted to a VoxelGrid containing a
    voxel for any positive items.  ImplicitSurfaces are first converted
    to OccupancyGrid type first. 
    """
    if isinstance(obj,PointCloud):
        pc = open3d.geometry.PointCloud()
        pc.points = open3d.utility.Vector3dVector(obj.points)
        #TODO: other properties
        colors = obj.getColors(('r','g','b'))
        if colors is not None:
            for i in range(len(obj.points)):
                pc.colors.append(colors[i])
        return pc
    elif isinstance(obj,TriangleMesh):
        m = open3d.geometry.TriangleMesh()
        m.vertices = open3d.utility.Vector3dVector(obj.vertices)
        m.triangles = open3d.utility.Vector3iVector(obj.indices)
        return m
    elif isinstance(obj,OccupancyGrid):
        import numpy as np

        origin = obj.bmin
        dims = obj.values.shape
        cx = (obj.bmax[0]-obj.bmin[0])/dims[0]
        cy = (obj.bmax[1]-obj.bmin[1])/dims[1]
        cz = (obj.bmax[2]-obj.bmin[2])/dims[2]
        voxel_size = pow(cx*cy*cz,1.0/3.0)
        
        indices = np.nonzero(obj.values > obj.occupancyThreshold)

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
    elif isinstance(obj,ImplicitSurface):
        import numpy as np

        origin = obj.bmin
        dims = obj.values.shape
        cx = (obj.bmax[0]-obj.bmin[0])/dims[0]
        cy = (obj.bmax[1]-obj.bmin[1])/dims[1]
        cz = (obj.bmax[2]-obj.bmin[2])/dims[2]
        voxel_size = pow(cx*cy*cz,1.0/3.0)
        
        indices = np.nonzero(obj.values <= 0)

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
            pc.setColors(np.asarray(obj.colors),('r','g','b'))
        #TODO: other properties
        return pc
    elif isinstance(obj,open3d.geometry.TriangleMesh):
        import numpy as np
        m = TriangleMesh()
        m.vertices = np.asarray(obj.vertices)
        m.indices = np.asarray(obj.triangles)
        return m
    elif isinstance(obj,open3d.geometry.VoxelGrid):
        import numpy as np
        grid = OccupancyGrid()
        grid.bmin = obj.get_min_bound()
        grid.bmax = obj.get_max_bound()
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
        assert imin[0] >= 0 and imin[1] >= 0 and imin[2] >= 0
        mask = np.zeros((imax[0]+1,imax[1]+1,imax[2]+1),dtype=float)
        mask[occupied[:,0],occupied[:,1],occupied[:,2]] = 1.0
        grid.values = mask
        return grid
    raise TypeError("Invalid type")
