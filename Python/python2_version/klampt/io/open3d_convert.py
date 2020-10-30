"""Conversions to and from the Open3D library.

Open3D is useful for doing various point cloud processing routines.
It can be installed using pip as follows:

   pip install open3d-python

"""

import open3d
from klampt import PointCloud,TriangleMesh,VolumeGrid,Geometry3D

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
        for i in xrange(obj.numPoints()):
            k = i*3
            pc.points.append((obj.vertices[k],obj.vertices[k+1],obj.vertices[k+2]))
        #TODO: other properties
        return pc
    elif isinstance(obj,TriangleMesh):
        m = open3d.geometry.TriangleMesh()
        for i in xrange(len(obj.vertices)//3):
            k = i*3
            m.vertices.append((obj.vertices[k],obj.vertices[k+1],obj.vertices[k+2]))
        for i in xrange(len(obj.indices)//3):
            k = i*3
            m.triangles.append((obj.indices[k],obj.indices[k+1],obj.indices[k+2]))
        return m
    elif isinstance(obj,VolumeGrid):
        import numpy as np

        origin = np.array([obj.bbox[0],obj.bbox[1],obj.bbox[2]])
        cx = (obj.bbox[3]-obj.bbox[0])/obj.dims[0]
        cy = (obj.bbox[4]-obj.bbox[1])/obj.dims[1]
        cz = (obj.bbox[5]-obj.bbox[2])/obj.dims[2]
        voxel_size = pow(cx*cy*cz,1.0/3.0)
        
        values = np.array(obj.values)
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
        pc = PointCloud()
        for p in obj.points:
            pc.vertices.append(p[0])
            pc.vertices.append(p[1])
            pc.vertices.append(p[2])
        #TODO: other properties
        return pc
    elif isinstance(obj,open3d.geometry.TriangleMesh):
        m = TriangleMesh()
        for p in obj.vertices:
            m.vertices.append(p[0])
            m.vertices.append(p[1])
            m.vertices.append(p[2])
        for i in obj.triangles:
            m.indices.append(int(i[0]))
            m.indices.append(int(i[1]))
            m.indices.append(int(i[2]))
        return m
    elif isinstance(obj,open3d.geometry.VoxelGrid):
        grid = VolumeGrid()
        import numpy as np
        occupied = np.array(obj.voxels,dtype=np.int32)
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
        grid.values.resize(grid.dims[0]*grid.dims[1]*grid.dims[2],-1.0)
        for cell in occupied:
            grid.set(int(cell[0]-imin[0]+1),int(cell[1]-imin[1]+1),int(cell[2]-imin[2]+1),1.0)
        return grid
    raise TypeError("Invalid type")
