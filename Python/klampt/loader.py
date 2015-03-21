"""Functions for loading/saving Klampt' objects to disk in the same format
as the Klampt C++ bindings / JSON formats.

Can read/write objects using the general purpose reader/writer functions
'read(type,text)' and 'write(x,type)'.

Can load/save objects using the general purpose loader/saver functions
'load(type,fileName)' and 'save(x,type,fileName)'.

Json serialization/deserialization are handled using the toJson and fromJson
functions.
"""
from robotsim import *
import so3,vectorops
from contact import ContactPoint
from trajectory import Trajectory
import hold


def writeVector(q):
    """Writes a vector to text in the length-prepended format 'n v1 ... vn'"""
    return str(len(q))+'\t'+' '.join(str(v) for v in q)

def readVector(text):
    """Reads a length-prepended vector from text 'n v1 ... vn'"""
    items = text.split()
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return [float(v) for v in items[1:]]

def writeVectorRaw(x):
    """Writes a vector to text in the raw format 'v1 ... vn'"""
    return ' '.join(str(xi) for xi in x)

def readVectorRaw(text):
    """Reads a vector from raw text 'v1 ... vn'"""
    items = text.split()
    return [float(v) for v in items]


def writeVectorList(x):
    """Writes a list of vectors to string"""
    return '\n'.join(loader.writeVector(xi) for xi in x)


def readVectorList(text):
    """Reads a list of endline-separated vectors from a string"""
    items = text.split()
    vectors = []
    pos = 0
    while pos < len(items):
        n = int(items[pos])
        vectors.append([float(v) for v in items[pos+1:pos+1+n]])
        pos += 1+n
    return vectors


def writeMatrix(x):
    """Writes a matrix to text in the format
    m n
    x11 x12 ... x1n
    ...
    xm1 xm2 ... xmn
    """
    return '\n'.join([str(len(x))+' '+str(len(x[0]))]+[writeVectorRaw(xi) for xi in x])

def readMatrix(text):
    """Reads a matrix from text in the format
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
    for i in xrange(m):
        x.append([float(v) for v in items[k:k+n]])
        k += n
    return x

def writeSo3(x):
    """Writes an so3 element, i.e., rotation matrix, to string in the same
    format as written to by Klampt C++ bindings (row major)."""
    assert len(x)==9,"Argument must be an so3 element"
    return '\t'.join([' '.join([str(mij) for mij in mi ]) for mi in so3.matrix(x)])

def readSo3(text):
    """Reads an so3 element, i.e., rotation matrix, from string in the same
    format as written to by Klampt C++ bindings (row major)."""
    items = text.split()
    if len(items) != 9: raise ValueError("Invalid element of SO3, must have 9 elements")
    return so3.inv([float(v) for v in items])


def writeSe3(x):
    """Writes an se3 element, i.e., rigid transformation, to string in the
    same format as written to by Klampt C++ bindings (row major R, followed by
    t)."""
    return writeSo3(x[0])+'\t'+writeVectorRaw(x[1])

def readSe3(text):
    """Reads an se3 element, i.e., rigid transformation, to string in the
    same format as written to by Klampt C++ bindings (row major R, followed by
    t)."""
    items = text.split()
    if len(items) != 12: raise ValueError("Invalid element of SE3, must have 12 elements")
    return (so3.inv([float(v) for v in items[:9]]),[float(v) for v in items[9:]])

def writeMatrix3(x):
    """Writes a 3x3 matrix from text"""
    return writeSo3(so3.from_matrix(text))

def readMatrix3(text):
    """Reads a 3x3 matrix from text"""
    return so3.matrix(readSo3(text))

def writeContactPoint(cp):
    """Writes a contact point's members x,n,kFriction"""
    return ' '.join(str(v) for v in (cp.x+cp.n+[cp.kFriction]))

def readContactPoint(text):
    """Reads a contact point from text 'x1 x2 x3 n1 n2 n3 kFriction'"""
    items = text.split()
    if len(items)!=7:
        raise ValueError("Invalid number of items, should be 7")
    return ContactPoint([float(v) for v in items[0:3]],[float(v) for v in items[3:6]],float(items[6]))

def writeContactPoint(cp):
    """Writes a contact point to text 'x1 x2 x3 n1 n2 n3 kFriction'"""
    return ' '.join([str(v) for v in cp.x+cp.n+[cp.kFriction]]) 
    
def readIKObjective(text):
    """Reads an IKObjective from text"""
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
    if posType != 'F':
        raise NotImplementedError("Python API can only do point and fixed IK goals")
    obj = IKObjective()
    if rotType == 'N':
        #point constraint
        obj.setRelativePoint(link,destlink,[float(v) for v in posLocal],[float(v) for v in posWorld])
    elif rotType == 'F':
        #fixed constraint
        R = so3.from_moment([float(v) for v in rotWorld])
        t = vectorops.sub([float(v) for v in posWorld],so3.apply(R,[float(v) for v in posLocal]))
        obj.setRelativeTransform(link,destlink,R,t)
    elif rotType == 'A':
        #TODO: rotational axis constraints actually can be set in the python API
        raise NotImplementedError("Axis rotations not yet implemented")
    else:
        raise NotImplementedError("Two-axis rotational constraints not supported")
    return obj

def readHold(text):
    """Loads a Hold from a string"""
    lines = parseLines(text)
    if lines[0] != 'begin hold':
        raise ValueError('Invalid hold begin text')
    if lines[-1] != 'end':
        raise ValueError('Invalid hold end text')
    h = hold.Hold()
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
                h.contacts.append(readContactPoint(' '.join(items[ind:ind+7])))
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
            R = momentToMatrix(rotWorld)
            t = vectorops.sub(posWorld,so3.apply(R,posLocal))
            h.ikConstraint.setFixedTransform(h.link,R,t)
        else:
            raise NotImplementedError("Can't do axis rotation yet")
            h.ikConstraint.setAxisRotation(h.link,posLocal,posWorld,localAxis,worldAxis)
    else:
        raise NotImplementedError("other ik specifications not done yet")
        h.setupIKConstraint(localPos0,rotWorld)
    return h
       
def writeHold(h):
    """Writes a Hold to a string"""
    text = "begin hold\n"
    text += "link = "+str(h.link)+"\n"
    text += "contacts = ";
    text += " \\\n    ".join([writeContactPoint(c) for c in h.contacts])
    text += "\n"
    localPos, worldPos = h.ikConstraint.getPosition()
    text += "position = "+" ".join(str(v) for v in localPos)+"  \\\n"
    text += "    "+" ".join(str(v) for v in worldPos)+"\n"
    #TODO: write ik constraint rotation
    if h.ikConstraint.numRotDims()==3:
        #fixed rotation
        m = so3.moment(h.ikConstraint.getRotation())
        text += "rotation = "+" ".join(str(v) for v in m)+"\n"
    elif h.ikConstraint.numRotDims()==1:
        locAxis,worldAxis = h.ikConstraint.getRotationAxis()
        text += "axis = "+" ".join(str(v) for v in locAxis)+"  \\\n"
        text += "    "+" ".join(str(v) for v in worldaxis)+"\n"
    text += "end"
    return text

def writeGeometricPrimitive(g):
    return g.saveString()

def readGeometricPrimitive(text):
    g = GeometricPrimitive()
    if not g.loadString(text):
        raise RuntimeError("Error reading GeometricPrimitive from string")
    return g

def parseLines(text):
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





readers = {'Config':readVector,
           'Vector':readVector,
           'Configs':readVectorList,
           'Vector3':readVectorRaw,
           'Matrix':readMatrix,
           'Matrix3':readMatrix3,
           'RotationMatrix':readSo3,
           'RigidTransform':readSe3,
           'IKObjective':readIKObjective,
           'Hold':readHold,
           'GeometricPrimitive':readGeometricPrimitive,
           }

writers = {'Config':writeVector,
           'Vector':writeVector,
           'Configs':writeVectorList,
           'Vector3':writeVectorRaw,
           'Matrix':writeMatrix,
           'Matrix3':writeMatrix3,
           'RotationMatrix':writeSo3,
           'RigidTransform':writeSe3,
           'Hold':writeHold,
           'GeometricPrimitive':writeGeometricPrimitive,
           }


def write(obj,type):
    """General-purpose write"""
    if type not in writers:
        raise RuntimeError("Writing of objects of type "+type+" not supported")
    return writers[type](obj)

def read(type,text):
    """General-purpose read"""
    if type not in readers:
        raise RuntimeError("Reading of objects of type "+type+" not supported")
    return readers[type](text)

def loadGeometry3D(fn):
    g = Geometry3D()
    if not g.loadFile(fn):
        raise RuntimeError("Error reading Geometry3D from "+fn)
    return g

def loadTrajectory(fn):
    value = trajectory.Trajectory()
    value.load(fn)
    return value

def loadMultiPath(fn):
    value = multipath.MultiPath()
    value.load(fn)
    return value

loaders = {'Trajectory':loadTrajectory,
           'MultiPath':loadMultiPath,
           'Geometry3D':loadGeometry3D,
           }

savers = {'Trajectory':lambda x,fn:x.save(fn),
           'MultiPath':lambda x,fn:x.save(fn),
          'Geometry3D':lambda x,fn:x.saveFile(fn),
          }

def save(obj,type,fn):
    """General-purpose save"""
    if type in savers:
        return savers[type](obj,fn)
    elif type in writers:
        f = open(fn,'w')
        f.write(writers[type](obj)+'\n')
        f.close()
        return True
    else:
        raise RuntimeError("Saving of type "+type+" is not supported")


def load(type,fn):
    """General-purpose load"""
    
    if type in loaders:
        return loaders[type](fn)
    elif type in readers:
        f = open(fn,'r')
        res = readers[type](''.join(f.readlines()))
        f.close()
        return res
    else:
        raise RuntimeError("Loading of type "+type+" is not supported")



def toJson(obj,type='auto'):
    """Converts from a Klamp't object to a structure that can be converted
    to a JSON string (e.g., from json.dumps()).  If 'type' is not provided,
    this attempts  to infer the object type automatically.
    
    Not all objects are supported yet.
    """
    if type == 'auto':
        if isinstance(obj,(list,tuple)):
            if all([isinstance(v,(bool,int,float)) for v in obj]):
                type = 'Vector'
            else:
                if len(obj)==2 and len(obj[0])==9 and len(obj[1])==3:
                    type = 'RigidTransform'
                else:
                    isconfigs = true
                    for item in obj:
                        if not all([isinstance(v,(bool,int,float)) for v in item]):
                            isconfigs = False
                    if isconfigs:
                        type = 'Configs'
                    else:
                        raise RuntimeError("Could not parse object "+str(obj))
        elif isinstance(obj,(bool,int,float,str,unicode)):
            type = 'Value'
        elif isinstance(obj,ContactPoint):
            type = 'ContactPoint'
        elif isinstance(obj,IKObjective):
            type = 'IKObjective'
        elif isinstance(obj,Trajectory):
            type = 'Trajectory'
        else:
            raise RuntimeError("Unknown object of type "+obj.__class__.__name)

    if type=='Config' or type=='Configs' or type=='Vector' or type == 'Matrix' or type == 'RotationMatrix' or type == 'Value':
        return obj
    elif type == 'RigidTransform':
        return obj
    elif type == 'ContactPoint':
        return {'x':obj.x,'n':obj.n,'kFriction':kFriction}
    elif type == 'Trajectory':
        return {'times':obj.times,'milestones':obj.milestones}
    elif type in writers:
        return {'type':type,'data':write(obj,type)}
    else:
        raise RuntimeError("Unknown or unsupported type "+type)


def fromJson(jsonobj,type='auto'):
    """Converts from a JSON object (e.g., from json.loads()) to a Klamp't
    object of the appropriate type.  If 'type' is not provided, this attempts
    to infer the object type automatically."""
    if type == 'auto':
        if isinstance(jsonobj,(list,tuple)):
            return jsonobj
        elif isinstance(jsonobj,(bool,int,float,str,unicode)):
            return jsonobj
        elif isinstance(jsonobj,dict):
            if 'type' in jsonobj:
                type = jsonobj["type"]
            elif 'times' in jsonobj and 'milestones' in jsonobj:
                type = 'Trajectory'
            elif 'x' in jsonobj and 'n' in jsonobj and 'kFriction' in jsonobj:
                type = 'ContactPoint'
        else:
            raise RuntimeError("Unknown JSON object of type "+jsonobj.__class__.__name)

    if type=='Config' or type=='Configs' or type=='Vector' or type == 'Matrix' or type == 'RotationMatrix' or type == 'Value':
        return jsonobj
    elif type == 'RigidTransform':
        return jsonobj
    elif type == 'ContactPoint':
        return ContactPoint(jsonobj['x'],jsonobj['n'],jsonobj['kFriction'])
    elif type == 'Trajectory':
        return Trajectory(jsonobj['times'],jsonobj['milestones'])
    elif type == 'IKObjective':
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
            endRotation = jsonObject['endRotation']
        if rotConstraint == 'axis' or rotConstraint == 'twoaxis':
            localAxis = jsonObject['localAxis']
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
            lpts = [localPosition,vectorops.madd(localPosition,localAxis,h)]
            wpts = [endPosition,vectorops.madd(endPosition,endRotation,h)]
            if destlink >= 0:
                obj.setRelativePoints(link,destlink,lpts,wpts)
            else:
                obj.setFixedPoints(link,lpts,wpts)
            return obj
        elif rotConstraint == 'fixed':
            obj = IKObjective()
            R = so3.from_moment(endRotation)
            t = vectorops.sub(endPosition,so3.apply(R,localPosition))
            obj.setFixedTransform(R,t)
            return obj
        else:
            raise RuntimeError("Invalid IK rotation constraint "+rotConstraint)
    elif type in readers:
        return read(type,jsonobj["data"])
    else:
        raise RuntimeError("Unknown or unsupported type "+type)
