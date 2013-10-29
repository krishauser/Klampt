from robotsim import *
from contact import ContactPoint
import so3
import vectorops

def writeVector(q):
    """Writes a vector to text 'n v1 ... vn'"""
    return str(len(q))+'\t'+' '.join(str(v) for v in q)

def readVector(text):
    """Reads a vector from text 'n v1 ... vn'"""
    items = text.split()
    if int(items[0])+1 != len(items):
        raise ValueError("Invalid number of items")
    return [float(v) for v in items[1:]]

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
    
def momentToMatrix(m):
    """Converts an exponential map rotation represenation m to a matrix R"""
    angle = vectorops.norm(m)
    axis = vectorops.div(m,angle)
    return so3.rotation(axis,angle)

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
        R = momentToMatrix([float(v) for v in rotWorld])
        t = vectorops.sub([float(v) for v in posWorld],so3.apply(R,[float(v) for v in posLocal]))
        obj.setRelativeTransform(link,destlink,R,t)
    elif rotType == 'A':
        #TODO: rotational axis constraints actually can be set in the python API
        raise NotImplementedError("Axis rotations not yet implemented")
    else:
        raise NotImplementedError("Two-axis rotational constraints not supported")
    return obj

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
