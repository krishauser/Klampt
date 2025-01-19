"""Colorize an object to show heatmaps, false color images, etc.
"""

from ..robotsim import *
from ..math import vectorops
from typing import Optional, Union, List, Tuple, Callable
from klampt.model.typing import Vector3 
try:
    import numpy as np
except Exception:
    HAVE_NUMPY = False

def colorize(object : Union[Geometry3D,PointCloud,TriangleMesh,Heightmap,Appearance],
             value : Union[str,list],
             colormap : Optional[str]=None,
             feature : Optional[str]=None,
             vrange : Optional[Tuple[float,float]]=None,
             lighting : Optional[Union[Vector3,Callable]]=None):
    """Colorizes an object according to some value.  Useful for making
    heatmaps, false color images, etc.  Can only be used with point clouds
    and triangle meshes.

    Examples:

    - ``colorize(point_cloud,'z','plasma')``: sets the point cloud to a rainbow
      colorization (the 'plasma' colormap in Matplotlib) depending on z.

    - ``colorize(trimesh,'nz')``: colorizes the trimesh according to the z
      component of each triangle normal.

    - ``colorize(trimesh,[v1_val,v2_val,...,vm_val])``: colorizes each vertex
      of the triangle mesh by the default colormap. (here m is the # of
      vertices)

    - ``colorize(trimesh,[t1_rgb,t2_rgb,...,tn_rgb])``: colorizes each triangle
      of the triangle mesh to an assigned color. (here n is the # of
      triangles)

    - ``colorize(trimesh,'index','random',feature='vertices')``: assigns random
      vertex colors.

    - ``colorize(trimesh,segments,'random')``: if clusters is a list of segment
      IDs (len(segments) = # triangles), this assigns a random color to each
      segment.

    - ``colorize(trimesh,segments,'random',lighting=[0,0,-1])``: similar to
      above, but each triangle is shaded as though the scene was lighted by a
      downward-facing directional light (noonday sun).

    - ``colorize(heightmap,'z','plasma')``: sets the heightmap to a rainbow
      colorization (the 'plasma' colormap in Matplotlib) depending on z.

    - ``colorize(heightmap,'segment','random')``: if heightmap has a property
      named 'segment' mapping each point to a segment id, this assigns a
      random color to each segment.

    Arguments:
        object: either an object with both an ``appearance()`` and
            ``geometry()`` method, a :class:`~klampt.robotsim.Geometry3D`,
            a :class:`~klampt.robotsim.PointCloud`, a
            :class:`~klampt.robotsim.Heightmap`, or an
            :class:`~klampt.robotsim.Appearance`.

            - In the first case, the associated appearance is updated.
            - For Geometry3Ds, the return value is an Appearance that can be
              used via Appearance.drawGL(object).
            - For PointClouds, color attributes are added as an 'rgb' or 'rgba'
              channel, if color information is not already present.
            - For Heightmaps, color attributes are added to the `color`
              attribute.
            - For Appearances, the face/vertex colors are assigned. Note that
              ``feature`` cannot be "auto".

        value (str or list): a named feature of the geometry or a list of
            values or RGB/RGBA colors. Valid named features include:

            - 'p' or 'position': assigns RGB color by position
            - 'n' or 'normal': assigns RGB color by normal (range [-1,1] if
              vrange=None)
            - 'x': assigns value by position x 
            - 'y': assigns value by position y
            - 'z': assigns value by position z
            - 'nx': assigns value by normal x (range [-1,1] if vrange=None)
            - 'ny': assigns value by normal y (range [-1,1] if vrange=None)
            - 'nz': assigns value by normal x (range [-1,1] if vrange=None)
            - 'index': assigns value by feature index
            - 'height': assigns value by heightmap height
            - Any point cloud / heightmap property name: assigns value by
              property value.

            If this is a list, its length should equal the number of features
            (either vertices or faces) in the object's geometry.
        colormap (str, optional): either a Matplotlib colormap identifier or
            'random'.  By default, 'viridis' is used.  If 'random' is used,
            a random color is assigned to each unique value.
        feature (str, optional): if this cannot determine whether to use the
            vertices or faces of a triangle mesh, ``feature`` indicates which
            feature to use (either 'vertices' or 'faces').
        vrange (pair, optional): if values are 1D, this maps the value range
            to u=(v-vrange[0])/(vrange[1]-vrange[0]) before passing to the
            colormap.  If not provided, the range is determined automatically
            by the min/max of values.
        lighting (3-list or callable, optional): if a 3-list, this is
            interpreted as the direction of a directional light. The items are
            shaded as though the light were illuminating the scene.  If it's a
            callable, this is a function ``f(p,n)`` of the position and normal
            of a point or triangle, which should return a value in the range
            [0,1] indicating the overall brightness of the item.

    """
    if colormap is None:
        colormap = 'viridis'
    geometry = None
    geometrydata = None
    appearance = None
    try:
        geometry = object.geometry()
    except Exception:
        geometry = object
    if isinstance(geometry,Geometry3D):
        type = geometry.type()
        if geometry.empty():
            return
        if type == 'TriangleMesh':
            geometrydata = geometry.getTriangleMesh()
        elif type == 'PointCloud':
            geometrydata = geometry.getPointCloud()
        elif type == 'Heightmap':
            geometrydata = geometry.getHeightmap()
        elif type == 'Group':
            raise NotImplementedError("Can't colorize group objects yet")
        else:
            raise NotImplementedError("Invalid object type, can only colorize TriangleMesh, PointCloud, and Heightmap objects")
    elif isinstance(geometry,PointCloud):
        geometrydata = geometry
    elif isinstance(geometry,Heightmap):
        geometrydata = geometry
    elif isinstance(geometry,TriangleMesh):
        geometrydata = geometry

    if isinstance(object,Appearance):
        appearance = object
        geometry = None
        geometrydata = None
    else:
        if geometrydata is None:
            raise ValueError("Need to provide an object with TriangleMesh or PointCloud data")
    if isinstance(geometrydata,PointCloud):
        feature = 'vertices'

    try:
        appearance = object.appearance()
    except Exception:
        if isinstance(geometrydata,PointCloud):  #put result directly into pointcloud
            appearance = geometrydata
        elif isinstance(geometrydata,Heightmap):  #put result directly into heightmap
            appearance = geometrydata
        else:
            appearance = Appearance()

    #figure out if the number of faces is appropriate
    if feature not in ['vertices','faces',None]:
        raise ValueError("Invalid feature specified")
    N = 0
    if feature == 'vertices':
        if isinstance(geometrydata,PointCloud):
            N = len(geometrydata.points)
        elif isinstance(geometrydata,Heightmap):
            N = geometrydata.heights.shape[0]*geometrydata.heights.shape[1]
        else:
            N = len(geometrydata.vertices)
    elif feature == 'faces':
        if not isinstance(geometrydata,TriangleMesh):
            raise ValueError("Triangle meshes are the only geometries that can use the 'faces' feature")
        N = len(geometrydata.indices)

    #check if values needs transforming
    if isinstance(value,str):
        if feature == None:
            if isinstance(geometrydata,Heightmap):
                feature = 'vertices'
                N = geometrydata.heights.shape[0]*geometrydata.heights.shape[1]
            elif not isinstance(colormap,str) or colormap != 'random' or isinstance(geometrydata,PointCloud):
                feature = 'vertices'
                N = len(geometrydata.vertices)
            else:
                #random face colors
                feature = 'faces'
                N = len(geometrydata.indices)
                value = 'index'
        if geometrydata is None:
            raise ValueError("Can't assign colors based on a named value to an Appearance")
        if value == 'index':
            value = list(range(N))
        else:
            vname = value
            found = False
            if isinstance(geometrydata,PointCloud):
                if vname == 'nx':
                    vname = 'normal_x'
                elif vname == 'ny':
                    vname = 'normal_y'
                elif vname == 'nz':
                    vname = 'normal_z'
                i = geometrydata.propertyIndex(vname)
                if i >= 0:
                    found = True
                    value = geometrydata.properties[:,i]
            elif isinstance(geometrydata,Heightmap):
                if value == 'height' or value == 'z':
                    found = True
                    value = geometrydata.getHeights().flatten()
                else:
                    i = geometry.propertyIndex(value)
                    if i >= 0:
                        found = True
                        value = geometrydata.getProperties(i).flatten()
            if found:
                assert value is not None
                assert len(value) == N,"Feature is "+feature+" with length "+str(N)+" but extracted values of size "+str(len(value))
                assert len(np.asarray(value).shape)==1,"Invalid value shape "+str(np.asarray(value).shape)
    else:
        if not hasattr(value,'__iter__'):
            raise ValueError("Provided an invalid value "+str(value))
        
        #it's a list
        if feature == None:
            if isinstance(geometrydata,PointCloud):
                feature = 'vertices'
                N = len(geometrydata.points)
            elif isinstance(geometrydata,Heightmap):
                feature = 'height'
                N = geometrydata.heights.shape[0]*geometrydata.heights.shape[1]
            else:
                assert isinstance(geometrydata,TriangleMesh)
                if len(value) == len(geometrydata.indices):
                    feature = 'faces'
                    N = len(geometrydata.indices)
                elif len(value) == len(geometrydata.vertices):
                    feature = 'vertices'
                    N = len(geometrydata.vertices)
                else:
                    raise ValueError("The number of values (%d) does not match the number of vertices (%d) or faces (%d) of the mesh"%(len(value),len(geometrydata.vertices)),len(geometrydata.indices))
        if len(value) != N:
            raise ValueError("The number of values does not match the number of features: %d != %d"%(len(value),N))
        if len(np.asarray(value).shape)!=1:
            raise ValueError("Invalid value shape "+str(np.asarray(value).shape))

    shading = None
    if isinstance(value,str) or lighting is not None:
        #need positions / normals -- compute them for the indicated features
        if isinstance(geometrydata,PointCloud):
            positions = geometrydata.points
        elif isinstance(geometrydata,Heightmap):
            if isinstance(geometry,Geometry3D):
                positions = geometry.convert('PointCloud').getPointCloud().points
            else:
                positions = Geometry3D(geometrydata).convert('PointCloud').getPointCloud().points
        else:
            assert isinstance(geometrydata,TriangleMesh)
            positions = geometrydata.vertices
        normals = None
        if lighting is not None or value in ['n','normal','nx','ny','nz']:
            if isinstance(geometrydata,PointCloud):
                #get normals from point cloud
                from ..model.geometry import point_cloud_normals
                normals = np.asarray(point_cloud_normals(geometrydata,estimation_viewpoint=[0,0,0]))
            elif isinstance(geometrydata,Heightmap):
                #get normals from point cloud
                raise NotImplementedError("Can't get normals from heightmaps yet")
            else:
                assert isinstance(geometrydata,TriangleMesh)
                if feature == 'vertices':
                    normals = geometrydata.vertexNormals(geometrydata)
                else:
                    normals = geometrydata.triangleNormals(geometrydata)
        if feature == 'faces':
            assert isinstance(geometrydata,TriangleMesh)
            if lighting is not None or value in ['position','x','y','z']:
                #compute positions = triangle centroids
                tris = geometrydata.indices
                A = positions[tris[:,0]]
                B = positions[tris[:,1]]
                C = positions[tris[:,2]]
                positions = (A+B+C)/3
        
        if isinstance(value,str):
            if value == 'p' or value == 'position':
                pmin = positions.min(axis=0)
                pmax = positions.max(axis=0)
                assert pmin.shape == (3,)
                dim = (pmax-pmin).max()
                value = (positions - pmin[np.newaxis,:])/dim
            elif value == 'x':
                value = positions[:,0]
            elif value == 'y':
                value = positions[:,1]
            elif value == 'z':
                value = positions[:,2]
            else:
                if value not in ['n','normal','nx','ny','nz']:
                    raise ValueError("Invalid named value "+value)
                vrange = [-1,1]
                if value == 'n' or value == 'normal':
                    value = (normals + 1.0)*0.5
                elif value == 'nx':
                    value = normals[:,0]
                elif value == 'ny':
                    value = normals[:,1]
                elif value == 'nz':
                    value = normals[:,2]
                else:
                    assert False,"Code should never be reached"
            assert value is not None
            assert len(value) == N,"Got the wrong values? feature = %s, positions size %d, value size %d, N=%d"%(feature,positions.shape[0],len(value),N)

        if lighting is not None:
            if hasattr(lighting,'__iter__'):
                if len(lighting) != 3:
                    raise ValueError("Lighting vector needs to be a 3-vector")
                shading = -np.dot(normals,np.asarray(lighting))
                shading[shading < 0] = 0
                shading[shading > 1] = 1
                shading = 0.75*shading + 0.25
                assert np.all(shading >= 0) and np.all(shading <= 1)
            else:
                if not callable(lighting):
                    raise ValueError("Lighting argument needs to be a 3-vector or callable")
                shading = np.array([lighting(p,n) for p,n in zip(positions,normals)])

    #now map values to colors
    colors = None
    if colormap == 'random':
        vunique,vinds = np.unique(value,return_inverse=True,axis=0)
        if len(vunique) < len(value):
            cmap = np.random.rand(len(vunique),3).astype(np.float32)
            colors = cmap[vinds]
        else:
            colors = np.random.rand(N,3).astype(np.float32)
    elif hasattr(value[0],'__iter__'):
        colors = np.array(value,dtype=np.float32)
        if colors.shape[1] not in [3,4]:
            raise ValueError("Value array must be a 1-D list, Nx3 array, or Nx4 array")
    else:
        #assign a colormap
        value = np.asarray(value)
        if vrange is None:
            vrange = (value[np.isfinite(value)].min(),value[np.isfinite(value)].max())
        if vrange[0] == vrange[1]:
            val = vrange[0]
            vrange = (val - 0.5,val + 0.5)
        from matplotlib import cm
        cm_interpolator = cm.get_cmap(colormap)
        value = np.asarray(value)
        interp = (value - vrange[0])*(1.0/(vrange[1]-vrange[0]))
        colors = cm_interpolator(interp).astype(np.float32)

    if shading is not None:
        colors[:,:3] = shading[:,np.newaxis]*colors[:,:3]

    #finally, assign colors to object
    if isinstance(appearance,PointCloud):
        #set the point cloud colors directly
        hascolor = -1
        prop = None
        for i in range(appearance.numProperties()):
            if appearance.getPropertyName(i) in ['rgb','rgba','r']:
                hascolor = i
                prop = appearance.getPropertyName(i)
                break
        if hascolor < 0:
            hascolor = appearance.numProperties()
            if colors.shape[1] == 3:
                prop = 'rgb'
                appearance.addProperty(prop)
            else:
                prop = 'rgba'
                appearance.addProperty(prop)
        if prop == 'r':
            assert appearance.getPropertyName(hascolor+1) == 'g'
            assert appearance.getPropertyName(hascolor+2) == 'b'
            appearance.properties[:,hascolor] = colors[:,0]
            appearance.properties[:,hascolor+1] = colors[:,1]
            appearance.properties[:,hascolor+2] = colors[:,2]
        elif prop == 'rgb':
            r = np.rint(colors[:,0]*255.0).astype(np.uint32)
            g = np.rint(colors[:,1]*255.0).astype(np.uint32)
            b = np.rint(colors[:,2]*255.0).astype(np.uint32)
            rgb = np.bitwise_or.reduce((np.left_shift(r,16),np.left_shift(g,8),b))
            appearance.properties[:,hascolor] = rgb
        elif prop == 'rgba':
            r = np.rint(colors[:,0]*255.0).astype(np.uint32)
            g = np.rint(colors[:,1]*255.0).astype(np.uint32)
            b = np.rint(colors[:,2]*255.0).astype(np.uint32)
            if colors.shape[1] == 3:
                a = np.full(colors.shape[0],255,dtype=np.uint32)
            else:
                a = np.rint(colors[:,3]*255.0).astype(np.uint32)
            rgba = np.bitwise_or.reduce((np.left_shift(r,16),np.left_shift(g,8),
                                        np.left_shift(a,24),b))
            appearance.properties[:,hascolor] = rgba
        if isinstance(geometry,Geometry3D):
            #write it back to the geometry
            geometry.setPointCloud(appearance)
    elif isinstance(appearance,Heightmap):
        colors = colors.reshape((appearance.heights.shape[0],appearance.heights.shape[1],colors.shape[1]))
        appearance.setColorImage(colors.swapaxes(0,1))
        if isinstance(geometry,Geometry3D):
            #write it back to the geometry
            geometry.setHeightmap(appearance)
    else:
        assert isinstance(appearance,Appearance)
        #assign appearance features
        temp = Appearance()
        if feature == 'vertices':
            temp.setColors(Appearance.VERTICES,colors)
        else:
            temp.setColors(Appearance.FACES,colors)
        appearance.set(temp)
        appearance.refresh()

    if object is geometry:
        return appearance
    return object
