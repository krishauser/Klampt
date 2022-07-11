"""Colorize an object to show heatmaps, false color images, etc.
"""


from OpenGL.raw.GL.VERSION.GL_1_1 import GL_NONE
from ..robotsim import *
from ..math import vectorops
try:
    import numpy as np
except Exception:
    HAVE_NUMPY = False

def colorize(object,value,colormap=None,feature=None,vrange=None,lighting=None):
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


    Arguments:
        object: either an object with both an ``appearance()`` and
            ``geometry()`` method, a :class:`~klampt.robotsim.PointCloud`, a
            :class:`~klampt.robotsim.Geometry3D`, or an
            :class:`~klampt.robotsim.Appearance`.

            - In the first case, the associated appearance is updated.
            - For PointClouds, color attributes are added as an 'rgb' or 'rgba'
              channel, if color information is not already present.
            - For Geometry3Ds, the return value is an Appearance that can be
              used via Appearance.drawGL(object).
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
            - Any point cloud feature name: assigns value by feature.

            If this is a list, its length should equal the number of features
            (either vertices or faces) in the object's geometry.
        colormap (str, optional): either a Matplotlib colormap identifier or
            'random'.  By default, 'viridis' is used.  If 'random' is used,
            the value is discarded and instead random colors are assigned.
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
        elif type == 'Group':
            raise NotImplementedError("Can't colorize group objects yet")
        else:
            raise NotImplementedError("Invalid object type, can only colorize TriangleMesh and PointCloud objects")
    elif isinstance(geometry,PointCloud):
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
        else:
            appearance = Appearance()

    #figure out if the number of faces is appropriate
    if feature not in ['vertices','faces',None]:
        raise ValueError("Invalid feature specified")
    N = 0
    if feature == 'vertices':
        N = len(geometrydata.vertices)//3
    elif feature == 'faces':
        if not isinstance(geometrydata,TriangleMesh):
            raise ValueError("Triangle meshes are the only geometries that can use the 'faces' feature")
        N = len(geometrydata.indices)//3

    #check if values needs transforming
    if isinstance(value,str):
        if feature == None:
            if not isinstance(colormap,str) or colormap != 'random' or isinstance(geometrydata,PointCloud):
                feature = 'vertices'
                N = len(geometrydata.vertices)//3
            else:
                #random face colors
                feature = 'faces'
                N = len(geometrydata.indices)//3
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
                for i in range(geometrydata.numProperties()):
                    if value == geometrydata.propertyNames[i]:
                        found = True
                        value = geometrydata.getProperties(i)
                        break
            if found:
                assert value is not None
                assert len(value) == N,"Feature is "+feature+" with length "+str(N)+" but extracted values of size "+str(len(value))
    else:
        if not hasattr(value,'__iter__'):
            raise ValueError("Provided an invalid value "+str(value))

        #it's a list
        if feature == None:
            if isinstance(geometrydata,PointCloud):
                feature = 'vertices'
                N = geometrydata.numPoints()
            else:
                if len(value)*3 == len(geometrydata.indices):
                    feature = 'faces'
                    N = len(geometrydata.indices)//3
                elif len(value)*3 == len(geometrydata.vertices):
                    feature = 'vertices'
                    N = len(geometrydata.vertices)//3
                else:
                    raise ValueError("The number of values (%d) does not match the number of vertices (%d) or faces (%d) of the mesh"%(len(value),len(geometrydata.vertices)//3),len(geometrydata.indices)//3)
        if len(value) != N:
            raise ValueError("The number of values does not match the number of features: %d != %d"%(len(value),N))

    shading = None
    if isinstance(value,str) or lighting is not None:
        #need positions / normals -- compute them for the indicated features
        if isinstance(geometrydata,PointCloud):
            positions = geometrydata.getPoints()
        else:
            positions = geometrydata.getVertices()
        normals = None
        if lighting is not None or value in ['n','normal','nx','ny','nz']:
            if isinstance(geometrydata,PointCloud):
                #get normals from point cloud
                from ..model import geometry
                normals = np.asarray(geometry.point_cloud_normals(geometrydata,estimation_viewpoint=[0,0,0]))
            else:
                if feature == 'vertices':
                    #compute normals by averaging triangle vertices
                    normals = np.zeros((N,3))
                    for i in range(0,len(geometrydata.indices),3):
                        a,b,c = geometrydata.indices[i],geometrydata.indices[i+1],geometrydata.indices[i+2]
                        n = vectorops.cross(positions[b]-positions[a],positions[c]-positions[a])
                        n = np.array(vectorops.unit(n))
                        normals[a] += n
                        normals[b] += n
                        normals[c] += n
                    for i in range(normals.shape[0]):
                        l = np.linalg.norm(normals[i])
                        if l > 0:
                            normals[i] *= 1.0/l
                else:
                    indices = geometrydata.getIndices()
                    a = positions[indices[:,0]]
                    b = positions[indices[:,1]]
                    c = positions[indices[:,2]]
                    normals = np.cross(b-a,c-a)
                    norms = np.linalg.norm(normals,axis = 1)
                    mask = norms!=0
                    normals[mask] = normals[mask]/norms[mask].reshape(-1,1)
                    normals = normals/np.linalg.norm(normals,axis = 1).reshape(-1,1)
        if feature == 'faces':
            if lighting is not None or value in ['position','x','y','z']:
                #compute positions = triangle centroids
                assert not isinstance(geometrydata,PointCloud)
                tris = geometrydata.getIndices()
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
        colors = np.random.rand(N,3).astype(np.float32)
    elif hasattr(value[0],'__iter__'):
        colors = np.array(value,dtype=np.float32)
        if colors.shape[1] not in [3,4]:
            raise ValueError("Value array must be a 1-D list, Nx3 array, or Nx4 array")
    else:
        #assign a colormap
        if vrange is None:
            vrange = (min(value),max(value))
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
            if appearance.propertyNames[i] in ['rgb','rgba','r']:
                hascolor = i
                prop = appearance.propertyNames[i]
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
            assert appearance.propertyNames[hascolor+1] == 'g'
            assert appearance.propertyNames[hascolor+2] == 'b'
            appearance.setProperties(hascolor,colors[:,0])
            appearance.setProperties(hascolor+1,colors[:,1])
            appearance.setProperties(hascolor+2,colors[:,2])
        elif prop == 'rgb':
            r = np.rint(colors[:,0]*255.0).astype(np.uint32)
            g = np.rint(colors[:,1]*255.0).astype(np.uint32)
            b = np.rint(colors[:,2]*255.0).astype(np.uint32)
            rgb = np.bitwise_or.reduce((np.left_shift(r,16),np.left_shift(g,8),b))
            appearance.setProperties(hascolor,rgb)
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
            appearance.setProperties(hascolor,rgba)
        if isinstance(geometry,Geometry3D):
            #write it back to the geometry
            geometry.setPointCloud(appearance)
    else:
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
