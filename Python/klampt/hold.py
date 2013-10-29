"""This module helps load and read holds.  Defines
the Hold class, and the functions readHold(text) and writeHold(h).
"""

from robot import vectorops
from robot import so3
from robot.loader import *

class Hold:
    """A Hold, similar to the Hold class in the C++ RobotSim library."""
    def __init__(self):
        self.link = None
        self.ikConstraint = None
        self.contacts = []

def readHold(text):
    lines = parseLines(text)
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

