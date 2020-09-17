from ..robotsim import *
from ..math import vectorops
try:
    import numpy as np
except Exception:
    HAVE_NUMPY = False

def colorize(object,value,colormap=None,feature=None,vrange=None):
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


    Arguments:
        object: either an object with both an ``appearance()`` and
            ``geometry()`` method, a :class:`~klampt.robotsim.PointCloud`, a
            :class:`~klampt.robotsim.Geometry3D`, or an
            :class:`~klampt.robotsim.Appearance`.

            - In the first case, the associated appearance is updated.
            - For PointClouds, the 'color' attribute is added, if not already
              present.
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
    if feature == 'vertices':
        N = len(geometrydata.vertices)//3
    elif feature == 'faces':
        if not isinstance(geometrydata,TriangleMesh):
            raise ValueError("Triangle meshes are the only geometries that can use the 'faces' feature")
        N = len(geometrydata.indices)//3

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
                for i in range(geometrydata.numProperties()):
                    if value == geometrydata.propertyNames[i]:
                        found = True
                        value = geometrydata.getProperties(i)
                        break
            if not found:
                positions = np.array(geometrydata.vertices)
                positions = positions.reshape((positions.shape[0]//3,3))
                if feature == 'vertices':
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
                        if isinstance(geometrydata,PointCloud):
                            raise ValueError("Invalid named value "+value)
                        vrange = [-1,1]
                        vnormals = np.zeros((N,3))
                        for i in range(0,len(geometrydata.indices),3):
                            a,b,c = geometrydata.indices[i],geometrydata.indices[i+1],geometrydata.indices[i+2]
                            n = vectorops.cross(positions[b]-positions[a],positions[c]-positions[a])
                            n = np.array(vectorops.unit(n))
                            vnormals[a] += n
                            vnormals[b] += n
                            vnormals[c] += n
                        for i in range(vnormals.shape[0]):
                            l = np.linalg.norm(vnormals[i])
                            if l > 0:
                                vnormals[i] *= 1.0/l
                        if value == 'n' or value == 'normal':
                            value = (vnormals + 1.0)*0.5
                        elif value == 'nx':
                            value = vnormals[:,0]
                        elif value == 'ny':
                            value = vnormals[:,1]
                        elif value == 'nz':
                            value = vnormals[:,2]
                        else:
                            assert False,"Code should never be reached"
                    assert value is not None
                else:
                    assert not isinstance(geometrydata,PointCloud)
                    tris = np.array(geometrydata.indices,dtype=np.uint32)
                    tris = tris.reshape((tris.shape[0]//3,3))
                    if value in ['p','position','x','y','z']:
                        tpositions = np.zeros((N,3))
                        for i,t in enumerate(tris):
                            tpositions[i] = np.average(positions[t,:],axis=0)
                        if value == 'p' or value == 'position':
                            pmin = tpositions.min(axis=0)
                            pmax = tpositions.max(axis=0)
                            assert pmin.shape == (3,)
                            dim = (pmax-pmin).max()
                            value = (tpositions - pmin[np.newaxis,:])/dim
                        elif value == 'x':
                            value = tpositions[:,0]
                        elif value == 'y':
                            value = tpositions[:,1]
                        elif value == 'z':
                            value = tpositions[:,2]
                        else:
                            assert False,"Code should never be reached"
                    else:
                        if value not in ['n','normal','nx','ny','nz']:
                            raise ValueError("Invalid named value "+value)
                        vrange = [-1,1]
                        normals = np.zeros((N,3))
                        for i in range(0,len(geometrydata.indices),3):
                            a,b,c = geometrydata.indices[i],geometrydata.indices[i+1],geometrydata.indices[i+2]
                            n = vectorops.cross(positions[b]-positions[a],positions[c]-positions[a])
                            normals[i//3] = np.array(vectorops.unit(n))
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
            else:
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

    #now map values to colors
    colors = None
    if colormap == 'random':
        colors = np.random.rand(N,3)
    elif hasattr(value[0],'__iter__'):
        colors = np.asarray(value)
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
        colors = cm_interpolator(interp)

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
            appearance.setProperties(hascolor,colors[:,0].tolist())
            appearance.setProperties(hascolor+1,colors[:,1].tolist())
            appearance.setProperties(hascolor+2,colors[:,2].tolist())
        elif prop == 'rgb':
            r = colors[:,0]*255.0
            g = colors[:,1]*255.0
            b = colors[:,2]*255.0
            rgb = np.bitwise_or(np.bitwise_or(np.left_shift(r.astype(np.uint8),16),
                                        np.left_shift(g.astype(np.uint8),8)),
                                        b.astype(np.uint8))
            appearance.setProperties(hascolor,rgb.tolist())
        elif prop == 'rgba':
            r = colors[:,0]*255.0
            g = colors[:,1]*255.0
            b = colors[:,2]*255.0
            if colors.shape[1] == 3:
                a = np.full(colors.shape[0],255.0)
            else:
                a = colors[:,3]*255.0
            rgba = np.bitwise_or(np.bitwise_or(np.left_shift(r.astype(np.uint32),16),
                                    np.left_shift(g.astype(np.uint32),8)),
                                np.bitwise_or(np.left_shift(a.astype(np.uint32),24),
                                    b.astype(np.uint32)))
            appearance.setProperties(hascolor,rgba.tolist())
        if isinstance(geometry,Geometry3D):
            #write it back to the geometry
            geometry.setPointCloud(appearance)
    else:
        #assign appearance features
        have_alpha = (colors.shape[1]==4)
        temp = Appearance()
        if feature == 'vertices':
            temp.setColors(Appearance.VERTICES,colors.flatten(),have_alpha)
        else:
            temp.setColors(Appearance.FACES,colors.flatten(),have_alpha)
        appearance.set(temp)
        appearance.refresh()

    if object is geometry:
        return appearance
    return object
