"""Conversions to and from the Open3D library.

Open3D is useful for doing various point cloud processing routines.
It can be installed using pip as follows::

    pip install open3d-python

"""

import open3d
from klampt import PointCloud,TriangleMesh,VolumeGrid,Geometry3D
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
        pc.points = open3d.utility.Vector3dVector(obj.getPoints())
        #TODO: other properties
        colors = geometry.point_cloud_colors(obj,('r','g','b'))
        if colors is not None:
            for i in range(obj.numPoints()):
                pc.colors.append(colors[i])
        return pc
    elif isinstance(obj,TriangleMesh):
        m = open3d.geometry.TriangleMesh()
        m.vertices = open3d.utility.Vector3dVector(obj.getVertices())
        m.triangles = open3d.utility.Vector3iVector(obj.getIndices())
        return m
    elif isinstance(obj,VolumeGrid):
        import numpy as np

        origin = np.array([obj.bbox[0],obj.bbox[1],obj.bbox[2]])
        cx = (obj.bbox[3]-obj.bbox[0])/obj.dims[0]
        cy = (obj.bbox[4]-obj.bbox[1])/obj.dims[1]
        cz = (obj.bbox[5]-obj.bbox[2])/obj.dims[2]
        voxel_size = pow(cx*cy*cz,1.0/3.0)
        
        values = obj.getValues()
        vrange = np.min(values),np.min(values)
        if vrange[0] < 0 or vrange[1] > 1:
            #treat as SDF
            indices = np.nonzero(values <= 0)
        else:
            #treat as occupancy grid
            indices = np.nonzero(values > 0.5)

        pc = open3d.geometry.PointCloud()
        for i in indices[0]:
            cell = np.array([i//(obj.dims[2]*obj.dims[1]), (i//obj.dims[2]) % obj.dims[1], i%obj.dims[2]])
            pt = (cell + 0.5)*voxel_size + origin
            pc.points.append(pt)
        return open3d.geometry.create_surface_voxel_grid_from_point_cloud(pc,voxel_size)
    elif isinstance(obj,Geometry3D):
        if obj.type() == 'PointCloud':
            pc = obj.getPointCloud()
            pc.transform(*obj.getCurrentTransform())
            return to_open3d(pc)
        elif obj.type() == 'TriangleMesh':
            m = obj.getTriangleMesh()
            m.transform(*obj.getCurrentTransform())
            return to_open3d(m)
    raise TypeError("Invalid type")

def from_open3d(obj):
    """Converts open3d geometry to a Klamp't geometry.
    """
    if isinstance(obj,open3d.geometry.PointCloud):
        import numpy as np
        pc = PointCloud()
        pc.setPoints(np.asarray(obj.points))
        if obj.has_colors():
            geometry.point_cloud_set_colors(pc,np.asarray(obj.colors).T,('r','g','b'),'rgb')
        #TODO: other properties
        return pc
    elif isinstance(obj,open3d.geometry.TriangleMesh):
        import numpy as np
        m = TriangleMesh()
        m.setVertices(np.asarray(obj.vertices))
        m.setIndices(np.asarray(obj.triangles))
        return m
    elif isinstance(obj,open3d.geometry.VoxelGrid):
        grid = VolumeGrid()
        import numpy as np
        if hasattr(obj, 'voxels'):
            # open3d <= 0.9.0
            occupied = np.array(obj.voxels,dtype=np.int32)
        else:
            occupied = np.array([v.grid_index for v in obj.get_voxels()],dtype=np.int32)
        imin = np.min(occupied,axis=0)
        imax = np.max(occupied,axis=0)
        assert imin.shape == (3,)
        assert imax.shape == (3,)
        bmin = obj.origin + (imin-1)*obj.voxel_size
        bmax = obj.origin + (imax+2)*obj.voxel_size
        grid.bbox.append(bmin[0])
        grid.bbox.append(bmin[1])
        grid.bbox.append(bmin[2])
        grid.bbox.append(bmax[0])
        grid.bbox.append(bmax[1])
        grid.bbox.append(bmax[2])
        grid.dims.append(imax[0]-imin[0]+3)
        grid.dims.append(imax[1]-imin[1]+3)
        grid.dims.append(imax[2]-imin[2]+3)
        grid.setValues(occupied.astype(float))
        return grid
    raise TypeError("Invalid type")
