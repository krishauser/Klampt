"""Functions for loading/saving Klampt' objects to disk in the same format
as the Klampt C++ bindings / JSON formats.

Can read/write objects using the general purpose reader/writer functions
'read(type,text)' and 'write(x,type)'.

Can load/save objects using the general purpose loader/saver functions
'load(type,fileName)' and 'save(x,type,fileName)'.  The load functions also
support URLs

Json serialization/deserialization are handled using the :func:`to_json` and
:func:`from_json` functions.

Module summary
==============

High level interface
--------------------

.. autosummary::
    write
    read
    load
    save
    to_json
    from_json
    EXTENSION_TO_TYPES
    TYPE_TO_EXTENSIONS
    UNSUPPORTED_JSON_TYPES
    filename_to_types
    filename_to_type

Implementation
--------------

.. autosummary::
    read_Vector
    write_Vector
    read_Vector_raw
    write_Vector_raw
    read_VectorList
    write_VectorList
    read_Matrix
    write_Matrix
    read_Matrix3
    write_Matrix3
    read_IntArray
    read_StringArray
    read_so3
    write_so3
    read_se3
    write_se3
    read_IKObjective
    write_IKObjective
    read_ContactPoint
    write_ContactPoint
    read_Hold
    write_Hold
    load_WorldModel
    load_Geometry3D
    load_Trajectory
    load_MultiPath
    load_dynamic_xml

"""
from ..robotsim import *
from ..math import so3,vectorops
from ..model.contact import ContactPoint, Hold
from ..model.trajectory import Trajectory,HermiteTrajectory,SO3Trajectory,SE3Trajectory
from ..model import types
import os
import warnings

EXTENSION_TO_TYPES = {'.config':['Config'],
                   '.configs':['Configs'],
                   '.tri':['Geometry3D','TriangleMesh'],
                   '.off':['Geometry3D','TriangleMesh'],
                   '.stl':['Geometry3D','TriangleMesh'],
                   '.ply':['Geometry3D','TriangleMesh'],
                   '.wrl':['Geometry3D','TriangleMesh'],
                   '.dae':['Geometry3D','TriangleMesh'],
                   '.poly':['Geometry3D','TriangleMesh'],
                   '.geom':['Geometry3D','GeometricPrimitive'],
                   '.pcd':['Geometry3D','PointCloud'],
                   '.vector3':['Vector3'],
                   '.matrix3':['Matrix3'],
                   '.ikgoal':['IKGoal'],
                   '.xform':['RigidTransform'],
                   '.path':['Trajectory','LinearPath','SE3Trajectory','SO3Trajectory'],
                   '.hold':['Hold'],
                   '.stance':['Stance'],
                   '.grasp':['Grasp'],
                   '.rob':['RobotModel'],
                   '.urdf':['RobotModel'],
                   '.obj':['Geometry3D','RigidObjectModel','TriangleMesh'],
                   '.env':['TerrainModel'],
                   '.xml':['WorldModel','MultiPath']
                   }
"""dict mapping file extensions to lists of compatible Klampt types."""

UNSUPPORTED_JSON_TYPES = ['Geometry3D','TriangleMesh','PointCloud','GeometricPrimitive','VolumeGrid',
    'RobotModel','RigidObjectModel','TerrainModel','WorldModel']
"""List of Klampt types that cannot currently be exported to JSON"""

TYPE_TO_EXTENSIONS = dict()
"""dict mapping Klamp't types to lists of compatible file extensions."""
for (k,v) in list(EXTENSION_TO_TYPES.items()):
    for t in v:
        if t in TYPE_TO_EXTENSIONS:
            TYPE_TO_EXTENSIONS[t].append(k)
        else:
            TYPE_TO_EXTENSIONS[t] = [k]


def filename_to_types(name):
    """Returns the Klampt types possibly represented by the given filename's
    extension.
    """
    fileName, fileExtension = os.path.splitext(name)
    fileExtension = fileExtension.lower()
    if fileExtension in EXTENSION_TO_TYPES:
        return EXTENSION_TO_TYPES[fileExtension]
    else:
        raise RuntimeError("Cannot determine type of object from filename "+name)

def filename_to_type(name):
    """Returns one Klampt type represented by the given filename's
    extension.

    If the file is a dynamic type (.xml or .json), just 'xml' or 'json' is
    returned because the type will need to be determined after parsing the
    file.

    If the type is ambiguous (like .obj), the first type in EXTENSION_TO_TYPES
    is returned.

    Returns:
        str: The Klamp't type
    """
    fileName, fileExtension = os.path.splitext(name)
    fileExtension = fileExtension.lower()
    if fileExtension == '.xml':
        return 'xml'  #dynamic loading
    elif fileExtension == '.json':
        return 'json'  #dynamic loading
    elif fileExtension in EXTENSION_TO_TYPES:
        ftypes = EXTENSION_TO_TYPES[fileExtension]
        if len(ftypes) > 1 and fileExtension not in ['.path'] and (ftypes[0] != 'Geometry3D' and len(ftypes) > 2):
            warnings.warn("loader.filename_to_type(): filename {} is ambiguous, matches types {}".format(name,', '.join(ftypes)))
        return ftypes[0]
    else:
        raise RuntimeError("Cannot determine type of object from filename "+name)

def write_Vector(q):
    """Writes a vector to a string in the length-prepended format 'n v1 ... vn'"""
    return str(len(q))+'\t'+' '.join(str(v) for v in q)

def read_Vector(text):
    """Reads a length-prepended vector from a string 'n v1 ... vn'"""
    items = text.split()
    if len(items) == 0:
        raise ValueError("Empty text")
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return [float(v) for v in items[1:]]

def write_Vector_raw(x):
    """Writes a vector to a string in the raw format 'v1 ... vn'"""
    return ' '.join(str(xi) for xi in x)

def read_Vector_raw(text):
    """Reads a vector from a raw string 'v1 ... vn'"""
    items = text.split()
    return [float(v) for v in items]


def write_VectorList(x):
    """Writes a list of vectors to string"""
    return '\n'.join(write_Vector(xi) for xi in x)


def read_VectorList(text):
    """Reads a list of endline-separated vectors from a string"""
    items = text.split()
    vectors = []
    pos = 0
    while pos < len(items):
        n = int(items[pos])
        vectors.append([float(v) for v in items[pos+1:pos+1+n]])
        pos += 1+n
    return vectors


def write_Matrix(x):
    """Writes a matrix to a string in the format
    m n
    x11 x12 ... x1n
    ...
    xm1 xm2 ... xmn
    """
    return '\n'.join([str(len(x))+' '+str(len(x[0]))]+[write_Vector_raw(xi) for xi in x])

def read_Matrix(text):
    """Reads a matrix from a string in the format
    m n
    x11 x12 ... x1n
    ...
    xm1 xm2 ... xmn
    """
    items = text.split()
    if len(items) < 2: raise ValueError("Invalid matrix string")
    m,n = int(items[0]),int(items[1])
    if len(items) != 2 + m*n:
        raise ValueError("Invalid number of matrix elements, should be %d, instead got %d"%(m*n,len(items)-2))
    k = 2
    x = []
    for i in range(m):
        x.append([float(v) for v in items[k:k+n]])
        k += n
    return x

def write_so3(x):
    """Writes an so3 element, i.e., rotation matrix, to string in the same
    format as written to by Klampt C++ bindings (row major)."""
    assert len(x)==9,"Argument must be an so3 element"
    return '\t'.join([' '.join([str(mij) for mij in mi ]) for mi in so3.matrix(x)])

def read_so3(text):
    """Reads an so3 element, i.e., rotation matrix, from string in the same
    format as written to by Klampt C++ bindings (row major)."""
    items = text.split()
    if len(items) != 9: raise ValueError("Invalid element of SO3, must have 9 elements")
    return so3.inv([float(v) for v in items])


def write_se3(x):
    """Writes an se3 element, i.e., rigid transformation, to string in the
    same format as written to by Klampt C++ bindings (row major R, followed by
    t)."""
    return write_so3(x[0])+'\t'+write_Vector_raw(x[1])

def read_se3(text):
    """Reads an se3 element, i.e., rigid transformation, to string in the
    same format as written to by Klampt C++ bindings (row major R, followed by
    t)."""
    items = text.split()
    if len(items) != 12: raise ValueError("Invalid element of SE3, must have 12 elements")
    return (so3.inv([float(v) for v in items[:9]]),[float(v) for v in items[9:]])

def write_Matrix3(x):
    """Writes a 3x3 matrix to a string"""
    return write_so3(so3.from_matrix(x))

def read_Matrix3(text):
    """Reads a 3x3 matrix from a string"""
    return so3.matrix(read_so3(text))

def write_ContactPoint(cp):
    """Writes a contact point's members x,n,kFriction"""
    return ' '.join(str(v) for v in cp.tolist())

def read_ContactPoint(text):
    """Reads a contact point from a string 'x1 x2 x3 n1 n2 n3 kFriction'"""
    items = text.split()
    if len(items)!=7:
        raise ValueError("Invalid number of items, should be 7")
    return ContactPoint([float(v) for v in items[0:3]],[float(v) for v in items[3:6]],float(items[6]))

def write_ContactPoint(cp):
    """Writes a contact point to a string 'x1 x2 x3 n1 n2 n3 kFriction'"""
    return ' '.join([str(v) for v in list(cp.x)+list(cp.n)+[cp.kFriction]]) 
    
def read_IKObjective(text):
    """Reads an IKObjective from a string in the Klamp't native format
    
    ``link destLink posConstraintType [pos constraint items] ...
    rotConstraintType [rot constraint items]``
    
    where link and destLink are integers, posConstraintType is one of

    * N: no constraint
    * P: position constrained to a plane
    * L: position constrained to a line
    * F: position constrained to a point

    and rotConstraintType is one of

    * N: no constraint
    * T: two-axis constraint (not supported)
    * A: rotation constrained about axis 
    * F: fixed rotation

    The [pos constraint items] contain a variable number of whitespace-
    separated items, dependending on posConstraintType:

    * N: 0 items
    * P: the local position xl yl zl, world position x y z on the plane, and
      plane normal nx,ny,nz
    * L: the local position xl yl zl, world position x y z on the line, and
      line axis direction nx,ny,nz
    * F: the local position xl yl zl and world position x y z

    The [rot constraint items] contain a variable number of whitespace-
    separated items, dependending on rotConstraintType:

    * N: 0 items
    * T: not supported
    * A: the local axis xl yl zl and the world axis x y z
    * F: the world rotation matrix, in moment (aka exponential map) form
      mx my mz (see so3.from_moment())

    """
    items = text.split()
    if len(items) < 4:
        raise ValueError("Not enough items to unpack")
    link = int(items[0])
    destlink = int(items[1])
    ind = 2
    posType = None
    posLocal = None
    posWorld = None
    posDirection = None
    rotType = None
    rotWorld = None
    rotAxis = None
    #read pos constraint
    posType = items[ind]
    ind += 1
    if posType=='N':
        #no constraint
        pass
    elif posType=='P' or posType=='L':
        posLocal = items[ind:ind+3]
        ind += 3
        posWorld = items[ind:ind+3]
        ind += 3
        posDirection = items[ind:ind+3]
        ind += 3
    elif posType=='F':
        posLocal = items[ind:ind+3]
        ind += 3
        posWorld = items[ind:ind+3]
        ind += 3
    else:
        raise ValueError("Invalid pos type "+posType+", must be N,P,L or F")
    rotType = items[ind]
    ind += 1
    if rotType=='N':
        #no constraint
        pass
    elif rotType=='T' or rotType=='A':
        rotAxis = items[ind:ind+3]
        ind += 3
        rotWorld = items[ind:ind+3]
        ind += 3
    elif rotType=='F':
        rotWorld = items[ind:ind+3]
        ind += 3
    else:
        raise ValueError("Invalid rot type "+rotType+", must be N,T,A or F")

    if posLocal: posLocal = [float(v) for v in posLocal]
    if posWorld: posWorld = [float(v) for v in posWorld]
    if posDirection: posDirection = [float(v) for v in posDirection]
    if rotAxis: rotAxis = [float(v) for v in rotAxis]
    if rotWorld: rotWorld = [float(v) for v in rotWorld]
    
    obj = IKObjective()
    obj.setLinks(link,destlink);
    if posType=='N':
        obj.setFreePosConstraint()
    elif posType=='F':
        obj.setFixedPosConstraint(posLocal,posWorld)
    elif posType=='P':
        obj.setPlanePosConstraint(posLocal,posDirection,vectorops.dot(posDirection,posWorld))
    else:
        obj.setLinearPosConstraint(posLocal,posWorld,posDirection)
    if rotType == 'N':
        obj.setFreeRotConstraint()
    elif rotType == 'F':
        #fixed constraint
        R = so3.from_moment(rotWorld)
        obj.setFixedRotConstraint(R)
    elif rotType == 'A':
        obj.setAxialRotConstraint(rotAxis,rotWorld)
    else:
        raise NotImplementedError("Two-axis rotational constraints not supported")
    return obj

def write_IKObjective(obj):
    return obj.saveString()

def read_Hold(text):
    """Loads a Hold from a string"""
    lines = parse_lines(text)
    if lines[0] != 'begin hold':
        raise ValueError('Invalid hold begin text')
    if lines[-1] != 'end':
        raise ValueError('Invalid hold end text')
    h = Hold()
    posLocal = None
    posWorld = None
    localPos0 = None
    rotAxis = None
    rotWorld = None
    iktype = 0
    for l in lines[1:-1]:
        items = l.split()
        if items[1] != '=':
            raise ValueError("Invalid line format")
        if items[0] == 'link':
            h.link = int(items[2])
        elif items[0] == 'contacts':
            ind = 2
            while ind < len(items):
                h.contacts.append(read_ContactPoint(' '.join(items[ind:ind+7])))
                ind += 7
        elif items[0] == "position":
            posLocal = [float(v) for v in items[2:5]]
            posWorld = [float(v) for v in items[5:8]]
        elif items[0] == "axis":
            rotAxis = [float(v) for v in items[2:5]]
            rotWorld = [float(v) for v in items[5:8]]
        elif items[0] == "rotation":
            rotWorld = [float(v) for v in items[2:5]]
        elif items[0] == "ikparams":
            localPos0 = [float(v) for v in items[2:5]]
            rotWorld = [float(v) for v in items[5:8]]
            iktype = 1
        else:
            raise ValueError("Invalid item "+items[0])
    if iktype == 0:
        h.ikConstraint = IKObjective()
        if posLocal==None:
            raise ValueError("Hold must have some point constraint")
        if rotWorld==None:
            h.ikConstraint.setFixedPoint(h.link,posLocal,posWorld)
        elif rotAxis==None:
            R = so3.from_moment(rotWorld)
            t = vectorops.sub(posWorld,so3.apply(R,posLocal))
            h.ikConstraint.setFixedTransform(h.link,R,t)
        else:
            raise NotImplementedError("Can't do axis rotation yet")
            h.ikConstraint.setAxisRotation(h.link,posLocal,posWorld,localAxis,worldAxis)
    else:
        raise NotImplementedError("other ik specifications not done yet")
        h.setupIKConstraint(localPos0,rotWorld)
    return h
       
def write_Hold(h):
    """Writes a Hold to a string"""
    text = "begin hold\n"
    text += "  link = "+str(h.link)+"\n"
    text += "  contacts = ";
    text += " \\\n    ".join([write_ContactPoint(c) for c in h.contacts])
    text += "\n"
    localPos, worldPos = h.ikConstraint.getPosition()
    text += "  position = "+" ".join(str(v) for v in localPos)+"  \\\n"
    text += "    "+" ".join(str(v) for v in worldPos)+"\n"
    #TODO: write ik constraint rotation
    if h.ikConstraint.numRotDims()==3:
        #fixed rotation
        m = so3.moment(h.ikConstraint.getRotation())
        text += "rotation = "+" ".join(str(v) for v in m)+"\n"
    elif h.ikConstraint.numRotDims()==1:
        locAxis,worldAxis = h.ikConstraint.getRotationAxis()
        text += "axis = "+" ".join(str(v) for v in locAxis)+"  \\\n"
        text += "    "+" ".join(str(v) for v in worldAxis)+"\n"
    text += "end"
    return text

def write_GeometricPrimitive(g):
    return g.saveString()

def read_GeometricPrimitive(text):
    g = GeometricPrimitive()
    if not g.loadString(text):
        raise RuntimeError("Error reading GeometricPrimitive from string")
    return g

def read_IntArray(text):
    """Reads a length-prepended vector from a string 'n v1 ... vn'"""
    items = text.split()
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return [int(v) for v in items[1:]]

def read_StringArray(text):
    """Reads a length-prepended vector from a string 'n v1 ... vn'"""
    items = text.split()
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return items[1:]

def parse_lines(text):
    """Returns a list of lines from the given text.  Understands end-of-line escapes '\\n'"""
    lines = text.strip().split('\n')
    esclines = []
    esc = False
    for l in lines:
        if esc:
            esclines[-1] = esclines[-1]+l
        else:
            esclines.append(l)
        if len(l)>0 and l[-1]=='\\':
            esclines[-1] = esclines[-1][:-1]
            esc = True
        else:
            esc = False
    return esclines





readers = {'Config':read_Vector,
           'Vector':read_Vector,
           'Configs':read_VectorList,
           'Vector3':read_Vector_raw,
           'Matrix':read_Matrix,
           'Matrix3':read_Matrix3,
           'Rotation':read_so3,
           'RigidTransform':read_se3,
           'IKObjective':read_IKObjective,
           'IKGoal':read_IKObjective,
           'Hold':read_Hold,
           'GeometricPrimitive':read_GeometricPrimitive,
           'IntArray':read_IntArray,
           'StringArray':read_StringArray,
           }

writers = {'Config':write_Vector,
           'Vector':write_Vector,
           'Configs':write_VectorList,
           'Vector3':write_Vector_raw,
           'Matrix':write_Matrix,
           'Matrix3':write_Matrix3,
           'Rotation':write_so3,
           'RigidTransform':write_se3,
           'IKObjective':write_IKObjective,
           'IKGoal':write_IKObjective,
           'Hold':write_Hold,
           'GeometricPrimitive':write_GeometricPrimitive,
           'IntArray':write_Vector,
           'StringArray':write_Vector,
           }

def write(obj,type):
    """General-purpose write of an arbitrary Klampt object to a str.

    Args:
        obj: A Klampt object
        type (str): Either the Klamp't type, 'json', or 'auto'. If 'auto', the
            type will be auto-detected from the object.

    Returns:
        str: The encoding of the object.
    """
    global writers
    if type == 'auto':
        type = types.object_to_type(obj,writers)
        if type is None:
            raise ValueError("Can't determine a writable type for object of type "+obj.__class__.__name__)
    elif type == 'json':
        import json
        return json.dumps(to_json(obj))
    if type not in writers:
        raise ValueError("Writing of objects of type "+type+" not supported")
    return writers[type](obj)

def read(type,text):
    """General-purpose read of an arbitrary Klampt object from a str.

    Args:
        type (str): Either the Klamp't type, or 'json'. Future versions may
            support 'xml' but this is not supported right now.  'auto' may
            not be specified.
        text (str): A string containing the object data.

    Returns:
        Klamp't object
    """
    global readers
    if type == 'json':
        import json
        jsonobj = json.loads(text)
        return from_json(jsonobj)
    if type not in readers:
        raise ValueError("Reading of objects of type "+type+" not supported")
    return readers[type](text)

def load_WorldModel(fn):
    w = WorldModel()
    if not w.loadFile(fn):
        raise IOError("Error reading WorldModel from "+fn)
    return w

def load_Geometry3D(fn):
    g = Geometry3D()
    if not g.loadFile(fn):
        raise IOError("Error reading Geometry3D from "+fn)
    return g

def load_Trajectory(fn):
    value = Trajectory()
    value.load(fn)
    return value

def load_MultiPath(fn):
    from ..model import multipath
    value = multipath.MultiPath()
    value.load(fn)
    return value

def load_dynamic_xml(fn):
    #XML types may only be a WorldModel or MultiPath
    value = WorldModel()
    res = value.readFile(fn)
    if res:
        return value
    try:
        return load_MultiPath(fn)
    except Exception as e:
        raise

loaders = {'Trajectory':load_Trajectory,
           'LinearPath':load_Trajectory,
           'MultiPath':load_MultiPath,
           'Geometry3D':load_Geometry3D,
           'WorldModel':load_WorldModel,
           'xml':load_dynamic_xml
           }

savers = {'Trajectory':lambda x,fn:x.save(fn),
          'LinearPath':lambda x,fn:x.save(fn),
          'MultiPath':lambda x,fn:x.save(fn),
          'Geometry3D':lambda x,fn:x.save(fn),
          'WorldModel':lambda x,fn:x.writeFile(fn),
          }

def save(obj,type,fn):
    """General-purpose save of an arbitrary Klampt object to a file.

    This also works with RobotModel, RigidObjectModel, and TerrainModel
    (which don't work with load).

    Args:
        obj: a Klamp't object.
        type (str): the Klampt type, 'json', or 'auto'
        fn (str): a file name

    Returns:
        bool: True if successful.
    """
    global savers,writers

    if hasattr(obj,'saveFile'):
        return obj.saveFile(fn)

    if type == 'auto':
        savers_and_writers = list(savers.keys()) + list(writers.keys())
        type = types.object_to_type(obj,savers_and_writers)
        if type is None:
            raise ValueError("Can't determine a savable type for object of type "+obj.__class__.__name__)
    elif type == 'json':
        import json
        with open(fn,'w') as f:
            json.dump(to_json(obj),f)
        return True
    if type in savers:
        return savers[type](obj,fn)
    elif type in writers:
        with open(fn,'w') as f:
            f.write(writers[type](obj)+'\n')
        return True
    else:
        raise ValueError("Saving of type "+type+" is not supported")


def load(type,fn):
    """General-purpose load of an arbitrary Klampt object from a file
    or URL.

    An exception is raised if there is an error loading or
    parsing the file.  Possible exception types include IOError,
    ValueError, and HTTPError.

    Args:
        type (str): a Klamp't type, 'json', or 'auto'
        fn (str): a filename.

    Returns: 
        Klamp't object
    """
    if type == 'auto':
        type = filename_to_type(fn)
    
    global loaders,readers
    cppurl = False
    if type == 'WorldModel' or (type == 'Geometry3D' and fn.find('ros://') >= 0):  #these two types handle URLs in C++ API
        cppurl = True
    if not cppurl and fn.find('://') >= 0:
        import urllib.request, urllib.error, urllib.parse
        src = None
        data = None
        try:
            src = urllib.request.urlopen(fn)
            data = src.read()
            print("klampt.io.loader.load(): Download %s HTTP response code %s, size %d bytes"%(fn,src.getcode(),len(data)))
        finally:
            if src:
                src.close()
        if type in loaders:
            #need to write to temporary file
            import os
            import tempfile
            local_filename = None
            fileName, suffix = os.path.splitext(fn)
            with tempfile.NamedTemporaryFile(delete=False,suffix=suffix) as tmp_file:
                local_filename = tmp_file.name
                print("klampt.io.loader.load(): saving data to temp file",local_filename)
                tmp_file.write(data)
                tmp_file.flush()
            res = loaders[type](local_filename)
            os.remove(local_filename)
        elif type in readers or type == 'json':
            res = readers[type](data)
        else:
            raise ValueError("Loading of type "+type+" is not supported")
        return res

    if type in loaders:
        return loaders[type](fn)
    elif type in readers or type == 'json':
        text = None
        with open(fn,'r') as f:
            text = ''.join(f.readlines())
        return read(type,text)
    else:
        raise ValueError("Loading of type "+type+" is not supported")



def to_json(obj,type='auto'):
    """Converts from a Klamp't object to a JSON-compatible structure.

    The resulting structure can be converted to a JSON string using 
    ``json.dumps()`` in the Python builtin ``json`` module.

    Not all objects are supported yet, notably geometry-related objects and
    world entities.
    
    Args:
        obj: A Klamp't object.
        type (str, optional): the type of the object (see
            :mod:`~klampt.model.types`) If 'auto' (default), the type of the
            object is inferred automatically.
    
    """
    if type == 'auto':
        if isinstance(obj,(list,tuple)):
            if all([isinstance(v,(bool,int,float)) for v in obj]):
                type = 'Vector'
            else:
                if len(obj)==2 and len(obj[0])==9 and len(obj[1])==3:
                    type = 'RigidTransform'
                else:
                    isconfigs = True
                    for item in obj:
                        if not all([isinstance(v,(bool,int,float)) for v in item]):
                            isconfigs = False
                    if isconfigs:
                        type = 'Configs'
                    else:
                        raise TypeError("Could not parse object "+str(obj))
        elif isinstance(obj,(bool,int,float,str)):
            type = 'Value'
        elif obj.__class__.__name__ in ['ContactPoint','IKObjective','Trajectory','MultiPath','GeometricPrimitive','TriangleMesh','ConvexHull','PointCloud','VolumeGrid','Geometry3D']:
            type = obj.__class__.__name__
        elif isinstance(obj,Trajectory):   #some subclasses of Trajectory may be used here too
            type = obj.__class__.__name__
        else:
            raise TypeError("Unknown object of type "+obj.__class__.__name__)

    if type in ['Config','Configs','Vector','Matrix','Vector2','Vector3','Matrix3','Point','Rotation','Value','IntArray','StringArray']:
        return obj
    elif type == 'RigidTransform':
        return obj
    elif type == 'ContactPoint':
        return {'x':obj.x,'n':obj.n,'kFriction':obj.kFriction}
    elif type == 'Trajectory' or type == 'LinearPath':
        return {'times':obj.times,'milestones':obj.milestones}
    elif type.endswith('Trajectory'):
        return {'type':type,'times':obj.times,'milestones':obj.milestones}
    elif type == 'IKObjective' or type == 'IKGoal':
        res = {'type':type,'link':obj.link()}
        if obj.destLink() >= 0:
            res['destLink'] = obj.destLink()
        if obj.numPosDims()==3:
            res['posConstraint']='fixed'
            res['localPosition'],res['endPosition']=obj.getPosition()
        elif obj.numPosDims()==2:
            res['posConstraint']='linear'
            res['localPosition'],res['endPosition']=obj.getPosition()
            res['direction']=obj.getPositionDirection()
        elif obj.numPosDims()==1:
            res['posConstraint']='planar'
            res['localPosition'],res['endPosition']=obj.getPosition()
            res['direction']=obj.getPositionDirection()
        else:
            #less verbose to just eliminate this 
            #res['posConstraint']='free'
            pass
        if obj.numRotDims()==3:
            res['rotConstraint']='fixed'
            res['endRotation']=so3.moment(obj.getRotation())
        elif obj.numRotDims()==2:
            res['rotConstraint']='axis'
            res['localAxis'],res['endRotation']=obj.getRotationAxis()
        elif obj.numRotDims()==1:
            raise NotImplementedError("twoaxis constraints are not implemented in Klampt")
        else:
            #less verbose to just eliminate this
            #res['rotConstraint']='free'
            pass
        return res
    elif type == 'TriangleMesh':
        inds = list(obj.indices)
        inds = [inds[i*3:i*3+3] for i in range(len(inds)//3)]
        verts = list(obj.vertices)
        verts = [verts[i*3:i*3+3] for i in range(len(verts)//3)]
        return {'type':type,'indices':inds,'vertices':verts}
    elif type == 'PointCloud':
        verts = list(obj.vertices)
        verts = [verts[i*3:i*3+3] for i in range(len(verts)//3)]
        res = {'type':type,'vertices':verts}
        propNames = list(obj.propertyNames)
        if len(propNames) > 0:
            res['propertyNames'] = propNames
        if len(verts) * len(propNames) > 0:
            n = len(verts)
            k = len(propNames)
            props = list(obj.properties)
            props = [props[i*k:i*k+k] for i in range(n)]
            res['properties'] = props
        #TODO: settings
        return res
    elif type == 'VolumeGrid':
        res = {'type':type}
        res['bmin'] = [obj.bbox[i] for i in range(3)]
        res['bmax'] = [obj.bbox[i] for i in range(3,6)]
        res['dims'] = list(obj.dims)
        res['values'] = list(obj.values)
        return res
    elif type == 'ConvexHull':
        points = [[obj.points[i],obj.points[i+1],obj.points[i+2]] for i in range(0,len(obj.points),3)]
        return {'type':type,'points':points}
    elif type == 'Geometry3D':
        data = None
        gtype = obj.type()
        if gtype == 'GeometricPrimitive':
            data = to_json(obj.getGeometricPrimitive(),gtype)
        elif gtype == 'TriangleMesh':
            data = to_json(obj.getTriangleMesh(),gtype)
        elif gtype == 'PointCloud':
            data = to_json(obj.getPointCloud(),gtype)
        elif gtype == 'ConvexHull':
            data = to_json(obj.getConvexHull(),gtype)
        elif gtype == 'VolumeGrid':
            data = to_json(obj.getVolumeGrid(),gtype)
        elif gtype == 'Group':
            data = [to_json(obj.getElement(i)) for i in range(obj.numElements())]
        return {'type':type,'datatype':gtype,'data':data}
    elif type in writers:
        return {'type':type,'data':write(obj,type)}
    else:
        raise ValueError("Unknown or unsupported type "+type)


def from_json(jsonobj,type='auto'):
    """Converts from a JSON structure to a Klamp't object of the appropriate
    type. 

    Note: a JSON structure can be created from a JSON string using the
    ``json.loads()`` function in the Python builtin ``json`` module.

    Not all objects are supported yet, notably geometry-related objects and
    world entities.

    Args:
        jsonobj: A JSON structure (i.e., one coming from :func:`to_json`)
        type (str, optional): the type of the object (see
            :mod:`~klampt.model.types`) If 'auto' (default), the type of the
            object is inferred automatically.

    """
    if type == 'auto':
        if isinstance(jsonobj,(list,tuple)):
            return jsonobj
        elif isinstance(jsonobj,(bool,int,float,str)):
            return jsonobj
        elif isinstance(jsonobj,dict):
            if 'type' in jsonobj:
                type = jsonobj["type"]
            elif 'times' in jsonobj and 'milestones' in jsonobj:
                type = 'Trajectory'
            elif 'x' in jsonobj and 'n' in jsonobj and 'kFriction' in jsonobj:
                type = 'ContactPoint'
        else:
            raise TypeError("Unknown JSON object of type "+jsonobj.__class__.__name)

    if type in ['Config','Configs','Vector','Matrix','Matrix3','Rotation','Value','IntArray','StringArray']:
        return jsonobj
    elif type == 'RigidTransform':
        return jsonobj
    elif type == 'ContactPoint':
        return ContactPoint(jsonobj['x'],jsonobj['n'],jsonobj['kFriction'])
    elif type == 'Trajectory' or type == 'LinearPath':
        return Trajectory(jsonobj['times'],jsonobj['milestones'])
    elif type == 'HermiteTrajectory':
        return HermiteTrajectory(jsonobj['times'],jsonobj['milestones'])
    elif type == 'SO3Trajectory':
        return SO3Trajectory(jsonobj['times'],jsonobj['milestones'])
    elif type == 'SE3Trajectory':
        return SO3Trajectory(jsonobj['times'],jsonobj['milestones'])
    elif type == 'IKObjective' or type == 'IKGoal':
        link = jsonobj['link']
        destlink = jsonobj['destLink'] if 'destLink' in jsonobj else -1
        posConstraint = 'free'
        rotConstraint = 'free'
        localPosition = endPosition = None
        direction = None
        endRotation = None
        localAxis = None
        if 'posConstraint' in jsonobj:
            posConstraint = jsonobj['posConstraint']
        if 'rotConstraint' in jsonobj:
            rotConstraint = jsonobj['rotConstraint']
        if posConstraint == 'planar' or posConstraint == 'linear':
            direction = jsonobj['direction']
        if posConstraint != 'free':
            localPosition = jsonobj['localPosition']
            endPosition = jsonobj['endPosition']
        if rotConstraint != 'free':
            endRotation = jsonobj['endRotation']
        if rotConstraint == 'axis' or rotConstraint == 'twoaxis':
            localAxis = jsonobj['localAxis']
        if posConstraint == 'free' and rotConstraint == 'free':
            #empty
            return IKObjective()
        elif posConstraint != 'fixed':
            raise NotImplementedError("Can't do non-fixed position constraints yet in Python API")
        if rotConstraint == 'twoaxis':
            raise NotImplementedError("twoaxis constraints are not implemented in Klampt")
        if rotConstraint == 'free':
            obj = IKObjective()
            if destlink >= 0:
                obj.setRelativePoint(link,destlink,localPosition,endPosition)
            else:
                obj.setFixedPoint(link,localPosition,endPosition)
            return obj
        elif rotConstraint == 'axis':
            obj = IKObjective()
            h = 0.1
            lpts = [vectorops.madd(localPosition,localAxis,-h),vectorops.madd(localPosition,localAxis,h)]
            wpts = [vectorops.madd(endPosition,endRotation,-h),vectorops.madd(endPosition,endRotation,h)]
            if destlink >= 0:
                obj.setRelativePoints(link,destlink,lpts,wpts)
            else:
                obj.setFixedPoints(link,lpts,wpts)
            return obj
        elif rotConstraint == 'fixed':
            obj = IKObjective()
            R = so3.from_moment(endRotation)
            t = vectorops.sub(endPosition,so3.apply(R,localPosition))
            obj.setFixedTransform(link,R,t)
            return obj
        else:
            raise ValueError("Invalid IK rotation constraint "+rotConstraint)
    elif type == 'TriangleMesh':
        inds = sum(jsonobj['indices'],[])
        verts = sum(jsonobj['vertices'],[])
        mesh = TriangleMesh()
        mesh.indices.resize(len(inds))
        mesh.vertices.resize(len(verts))
        for i,v in enumerate(inds):
            mesh.indices[i] = v
        for i,v in enumerate(verts):
            mesh.vertices[i] = v
        return mesh
    elif type == 'PointCloud':
        pc = PointCloud()
        verts = sum(jsonobj['vertices'],[])
        pc.vertices.resize(len(verts))
        for i,v in enumerate(verts):
            pc.vertices[i] = v
        if 'propertyNames' in jsonobj:
            propNames = jsonobj['propertyNames']
            pc.propertyNames.resize(len(propNames))
            for i,v in enumerate(propNames):
                pc.propertyNames[i] = v
            if 'properties' in jsonobj:
                props = sum(jsonobj['properties'])
                pc.properties.resize(len(props))
                for i,v in enumerate(props):
                    pc.properties[i] = v
        #TODO: settings
        return pc
    elif type == 'VolumeGrid':
        vg = VolumeGrid()
        bbox = jsonobj['bmin'] + jsonobj['bmax']
        vg.bbox.resize(6)
        for i,v in enumerate(bbox):
            vg.bbox[i] = v
        vg.dims.resize(3)
        for i,v in enumerate(jsonobj['dims']):
            vg.dims[i] = v
        values = jsonobj['values']
        vg.values.resize(len(values))
        for i,v in enumerate(values):
            vg.values[i] = v
        return vg
    elif type == 'ConvexHull':
        ch = ConvexHull()
        points = sum(jsonobj['points'])
        ch.points.resize(len(points))
        for i,v in enumerate(points):
            ch.points[i] = v
        return ch
    elif type == 'Geometry3D':
        gtype = jsonobj['datatype']
        if gtype == '':
            return Geometry3D()
        return Geometry3D(from_json(jsonobj['data'],gtype))
    elif type in readers:
        return read(type,jsonobj["data"])
    else:
        raise ValueError("Unknown or unsupported type "+type)

def _deprecated_func(oldName,newName):
    import sys
    mod = sys.modules[__name__]
    f = getattr(mod,newName)
    def depf(*args,**kwargs):
        warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
        return f(*args,**kwargs)
    depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
    setattr(mod,oldName,depf)

_deprecated_func("filenameToType","filename_to_type")
_deprecated_func("filenameToTypes","filename_to_type")
_deprecated_func("toJson","to_json")
_deprecated_func("fromJson","from_json")

from collections import UserDict, UserList
class _DeprecatedDict(dict):
    def __init__(self,oldname,newname,*args,**kwargs):
        UserDict.__init__(self,*args,**kwargs)
        self._oldname = oldname
        self._newname = newname
        self._warned = False
    def __getitem__(self,key):
        if not self._warned:
            self._warned = True
            warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(self._oldname,self._newname),DeprecationWarning)
        return UserDict.__getitem__(self,key)

class _DeprecatedList(UserList):
    def __init__(self,oldname,newname,*args,**kwargs):
        UserList.__init__(self,*args,**kwargs)
        self._oldname = oldname
        self._newname = newname
        self._warned = False
    def __getitem__(self,key):
        if not self._warned:
            self._warned = True
            warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(self._oldname,self._newname),DeprecationWarning)
        return UserList.__getitem__(self,key)
    def __contains__(self,key):
        if not self._warned:
            self._warned = True
            warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(self._oldname,self._newname),DeprecationWarning)
        return UserList.__contains__(self,key)


extensionToTypes = _DeprecatedDict("extensionToTypes","EXTENSION_TO_TYPES",EXTENSION_TO_TYPES)
typeToExtensions = _DeprecatedDict("typeToExtensions","TYPE_TO_EXTENSIONS",TYPE_TO_EXTENSIONS)
unsupportedJsonTypes = _DeprecatedList("unsupportedJsonTypes","UNSUPPORTED_JSON_TYPES",UNSUPPORTED_JSON_TYPES)
