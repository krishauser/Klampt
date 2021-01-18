"""This module defines the MultiPath class, and methods for loading and
saving multipaths from xml files.
"""

from ..model.contact import Hold
from ..math import vectorops
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
from xml.dom import minidom
import math
import bisect
import warnings

class MultiPath:
    """A sophisticated path representation that allows timed/untimed paths, attached
    velocity information, as well as making and breaking contact. 

    Primarily, a MultiPath consists of a list of Sections, each of which is a path or
    timed trajectory along a fixed stance.

    A Section can either contain the Holds defining its stance, or its stance could be
    defined by indices into the holdSet member of MultiPath.  If all Sections have the
    latter structure it is known as an "aggregated" MultiPath.

    Attributes:
        sections (list of Sections): the segments of the multipath, each operating over a
            fixed stance.
        settings (dict mapping str to str): an unstructured propery map 'settings' which
            can contain metadata about which robot this path applies to, how the path
            was created, etc.
        holdSet (dict mapping int to Hold): a set of indexed Holds, which can be referred
            to inside Section objects.

    """
    
    class Section:
        """Contains a path or time-parameterized trajectory, as well as a
        list of holds and ik constraints
        
        If the times member is set, this is time parameterized.  Otherwise,
        it is just a path.

        Attributes:
            settings (dict mapping str to str): an unstructured property map.
            configs (list of lists of floats): a list of N configurations along the
                path section.
            velocities (list of lists of floats, optional): a list of N joint
                velocities along the path section.
            times (list of floats, optional) a list of N times for each configuration
                along the path section.
            holds (list of Holds, optional): the set of Holds that this section
                is required to meet.
            holdIndices (list of ints, optional): the set of Holds that this section
                is required to meet, indexed into MultiPath.holdSet.
            ikObjectives (list of IKObjectives, optional): the set of extra IKObjectives
                that this section is required to meet.
        """
        def __init__(self):
            self.settings = {}
            self.configs = []
            self.velocities = None
            self.times = None
            self.holds = []
            self.holdIndices = []
            self.ikObjectives = []

    def __init__(self):
        self.sections = []
        self.settings = {}
        self.holdSet = dict()

    def numSections(self):
        return len(self.sections)

    def startConfig(self):
        return self.sections[0].configs[0]

    def endConfig(self):
        return self.sections[-1].configs[-1]

    def startTime(self):
        if len(self.sections)==0 or self.sections[0].times==None: return 0
        return self.sections[0].times[0]

    def endTime(self):
        """Returns the final time parameter"""
        if len(self.sections)==0: return 0
        if self.sections[-1].times==None: return sum(len(s.configs)-1 for s in self.sections)
        return self.sections[-1].times[-1]

    def duration(self):
        return self.endTime()-self.startTime()

    def hasTiming(self):
        """Returns true if the multipath is timed"""
        return self.sections[0].times is not None

    def checkValid(self):
        """Checks for validity of the path"""
        if self.hasTiming():
            t0 = self.startTime()
            for i,s in enumerate(self.sections):
                if s.times is None:
                    raise ValueError("MultiPath section 0 is timed but section "+str(i)+" is not timed")
                if len(s.times) != len(s.configs):
                    raise ValueError("MultiPath section "+str(i)+" has invalid number of times")
                for t in s.times:
                    if t < t0:
                        raise ValueError("MultiPath section "+str(i)+" times are not monotonically increasing")
                    t0 = t
        else:
            for i,s in enumerate(self.sections):
                if s.times is not None:
                    raise ValueError("MultiPath section 0 is not timed but section "+str(i)+" is timed")
            for i,s in enumerate(self.sections):
                if len(s.configs) <= 1:
                    raise ValueError("Section "+str(i)+" has 0 or 1 configuration, timing may be messed up")
        return True

    def isContinuous(self):
        """Returns true if all the sections are continuous (i.e., the last config of each
        section matches the start config of the next)."""
        for i in range(len(self.sections)-1):
            if self.sections[i].configs[-1] != self.sections[i+1].configs[0]:
                return False
        return True

    def getSectionTiming(self,section):
        """Returns a pair (tstart,tend) giving the timing of the section"""
        assert section >= 0 and section < len(self.sections)
        if self.hasTiming():
            return self.sections[section].times[0],self.sections[section].times[-1]
        t0 = 0
        for i in range(section):
            t0 += len(self.sections[i].configs)-1
        return (t0,t0+len(self.sections[section].configs))

    def getStance(self,section):
        """Returns the list of Holds that the section should satisfy"""
        res = self.sections[section].holds[:]
        res+=[self.holdSet[ind] for ind in self.sections[section].holdIndices]
        for g in self.sections[section].ikObjectives:
            h=Hold()
            h.link = g.link()
            h.ikConstraint = g
            res.append(h)
        return res

    def getIKProblem(self,section):
        """Returns the set of IKObjectives that the section should satisfy"""
        res = [h.ikConstraint for h in self.sections[section].holds]
        res += [self.holdSet[ind].ikConstraint for ind in self.sections[section].holdIndices]
        res += self.sections[section].ikObjectives
        return res

    def aggregateHolds(self,holdSimilarityThreshold=None):
        """Aggregates holds from all sections to the global holdSet variable, and converts
        sections to use holdIndices.  If holdSimilarityThreshold is not None, then sections'
        holds that are the same within the given tolerance are combined into one hold."""
        def sameHold(h1,h2,tol):
            from ..math import vectorops,so3
            if h1.link != h2.link: return False
            if len(h1.contacts) != len(h2.contacts): return False
            if h1.ikConstraint.numPosDims() != h2.ikConstraint.numPosDims(): return False
            if h1.ikConstraint.numRotDims() != h2.ikConstraint.numRotDims(): return False
            if h1.ikConstraint.numPosDims() == 3:
                xl1,xw1 = h1.ikConstraint.getPosition()
                xl2,xw2 = h2.ikConstraint.getPosition()
                if vectorops.distanceSquared(xl1,xl2) > tol**2 or vectorops.distanceSquared(xw1,xw2) > tol**2:
                    return False
            elif h1.ikConstraint.numPosDims() != 0:
                raise NotImplementedError("Distance detection for non-point or free holds")
            if h1.ikConstraint.numPosDims() == 3:
                R1 = h1.ikConstraint.getRotation()
                R2 = h2.ikConstraint.getRotation()
                if so3.distance(R1,R2) > tol:
                    return False
            for (c1,c2) in zip(h1.contacts,h2.contacts):
                if vectorops.distanceSquared(c1,c2) > tol**2:
                    return False
            return True

        holdsByLink = dict()
        for s in self.sections:
            for h in s.holds:
                found = False
                if holdSimilarityThreshold is not None:
                    for oldHold,index in holdsByLink.get(h.link,[]):
                        if sameHold(h,oldHold,holdSimilarityThreshold):
                            s.holdIndices.append(index)
                            found = True
                            break
                if not found:
                    s.holdIndices.append(len(self.holdSet))
                    self.holdSet[len(self.holdSet)] = h
                    if holdSimilarityThreshold is not None:
                        holdsByLink.setdefault(h.link,[]).append((h,s.holdIndices[-1]))
            s.holds = []

    def deaggregateHolds(self):
        """De-aggregates holds from the global holdSet variable into the sections' holds."""
        for s in self.sections:
            for h in s.holdIndices:
                s.holds.append(self.holdSet[h])
            s.holdIndices = []
        self.holdSet = dict()

    def setConfig(self,section,configIndex,q,v=None,t=None,maintainContinuity=True):
        """Sets a configuration along the path, maintaining continuity if maintainContinuity is true.
        Equivalent to self.sections[section].configs[configIndex] = q except that adjacent sections'
        configurations are also modified."""
        assert section >= 0 and section < len(self.sections)
        if configIndex < 0:
            configIndex = len(self.sections[section].configs)-1
        self.sections[section].configs[configIndex] = q
        if v is not None:
            assert self.sections[section].velocities is not None
            self.sections[section].velocities[configIndex] = v
        if t is not None:
            assert self.sections[section].times is not None
            self.sections[section].times[configIndex] = t
        if maintainContinuity:
            section0 = section
            configIndex0 = configIndex
            while section > 0 and configIndex == 0:
                section = section - 1
                configIndex = len(self.sections[section].configs)-1
                self.setConfig(section,configIndex,q,v,t,False)
            section = section0
            configIndex = configIndex0
            while section + 1 < len(self.sections) and configIndex + 1 == len(self.sections[section].configs):
                section = section + 1
                configIndex = 0
                self.setConfig(section,configIndex,q,v,t,False)

    def concat(self,path):
        """Appends the path, making sure times and holds are appropriately set"""
        newSections = path.sections[:]
        dt = 0.0
        if path.hasTiming() and len(self.sections)>0:
            #shift timing
            assert(self.hasTiming())
            dt = self.endTime()
            for s in newSections:
                s.times = [t+dt for t in s.times]
        #rename global hold set
        newholds = path.holdSet.copy()
        namemap = dict()
        for (name,h) in path.holdSet.items():
            if name in self.holdSet:
                found = False
                for k in range(2,1000):
                    newname = str(name)+"("+str(k)+")"
                    if newname not in self.holdSet:
                        namemap[name] = newname
                        found = True
                if not found:
                    raise ValueError("Unable to merge hold name "+name)
                newholds[newname] = newholds[name]
                del newholds[name]

        for s in newSections:
            inds = []
            for h in s.holdIndices:
                if h in namemap:
                    inds.append(namemap[h])
                else:
                    inds.append(h)
            s.holdIndices = inds
        self.sections += newSections
        self.holdSet.update(newholds)

    def save(self,fn):
        """Saves this  multipath to an xml file."""
        tree = self.saveXML()
        f = open(fn,'w')
        f.write('<?xml version="1.0"?>\n')
        f.write(_prettify(tree.getroot()))
        #tree.write(fn,pretty_print=True)
        f.close()

    def load(self,fn):
        """Loads this  multipath from a multipath xml file."""
        tree = ET.parse(fn)
        return self.loadXML(tree)

    def saveXML(self):
        """Saves this multipath to a multipath xml tree (ElementTree)"""
        from ..io import loader
        root = ET.Element("multipath")
        root.attrib = self.settings
        for sec in self.sections:
            xs = ET.Element("section")
            root.append(xs)
            xs.attrib = sec.settings
            for ikgoal in sec.ikObjectives:
                xik = ET.SubElement(xs,"ikgoal")
                if hasattr(ikgoal,'text'):
                    xik.text = ikgoal.text
                else:
                    xik.text = loader.writeIKObjective(ikgoal)
            for h in sec.holds:
                xh = ET.SubElement(xs,"hold")
                xh.text = loader.writeHold(h)
            for h in sec.holdIndices:
                xh = ET.SubElement(xs,"hold")
                if isinstance(h,int):
                    xh.set("index",str(h))
                else:
                    xh.set("name",str(h))
            for i in range(len(sec.configs)):
                xm = ET.Element("milestone")
                xs.append(xm)
                xm.set("config",loader.writeVector(sec.configs[i]))
                if sec.times != None:
                    xm.set("time",str(sec.times[i]))
                if sec.velocities != None:
                    xm.set("velocity",loader.writeVector(sec.velocities[i]))
        for hkey,h in self.holdSet.items():
            xh = ET.Element("hold")
            root.append(xh)
            if not isinstance(hkey,int):
                xh.set('name',str(hkey))
            xh.text = loader.writeHold(h)
        return ET.ElementTree(root)

    def loadXML(self,tree):
        """Loads a multipath from a multipath xml tree (ElementTree)."""
        from ..io import loader
        self.sections = []
        self.holdSet = dict()
        self.settings = dict()
        root = tree.getroot()
        for k,v in root.attrib.items():
            self.settings[k]=v
        for sec in root.findall('section'):
            s = MultiPath.Section()
            for k,v in sec.attrib.items():
                s.settings[k]=v
            milestones = sec.findall('milestone')
            for m in milestones:
                if 'config' not in m.attrib:
                    raise ValueError("Milestone does not contain config attribute")
                s.configs.append(loader.readVector(m.attrib['config']))
                if 'time' in m.attrib:
                    if s.times==None: s.times = []
                    s.times.append(float(m.attrib['time']))
                if 'velocity' in m.attrib:
                    if s.velocities==None: s.velocities = []
                    s.velocities.append(loader.readVector(m.attrib['velocity']))
            for obj in sec.findall('ikgoal'):
                s.ikObjectives.append(loader.readIKObjective(obj.text))
                s.ikObjectives[-1].text = obj.text
            for h in sec.findall('hold'):
                if 'index' in h.attrib:
                    s.holdIndices.append(int(h.attrib['index']))
                elif 'name' in h.attrib:
                    s.holdIndices.append(h.attrib['name'])
                else:
                    s.holds.append(loader.readHold(h.text))
            self.sections.append(s)
        #read global hold set
        for h in root.findall('hold'):
            hold = loader.readHold(h.text)
            if 'name' in h.attrib:
                self.holdSet[h.attrib['name']] = hold
            else:
                self.holdSet[len(self.holdSet)] = hold
        return

    def timeToSection(self,t):
        """Returns the section corresponding to the time parameter t"""
        if not self.hasTiming():
            return int(math.floor(t*len(self.sections)))
        else:
            if t < self.startTime(): return -1
            for i,s in enumerate(self.sections):
                if t < s.times[-1]:
                    return i
            return len(self.sections)

    def timeToSegment(self,t):
        """ Returns a (section index,milestone index,param) tuple such that
        interpolation between the section's milestone and its successor
        corresponds to time t."""
        s = self.timeToSection(t)
        if s < 0: return (-1,0,0)
        elif s >= len(self.sections): return (s,0,0)
        sec = self.sections[s]
        if len(sec.times)==0:
            usec = t*len(self.sections)-s
            tsec = (len(sec.milestones)-1)*usec
            i = int(math.floor(tsec))
            u = tsec - i
            return (s,i,u)
        else:
            i = bisect.bisect_left(sec.times,t)
            p = i-1
            u=(t-sec.times[p])/(sec.times[i]-sec.times[p])
            if i==0:
                return (s,0,0)
            return (s,p,u)

    def eval(self,t):
        """Evaluates the MultiPath at time t."""
        (s,i,u) = self.getSegment(t)
        if s < 0: return self.startConfig()
        elif s >= len(self.sections): return self.endConfig()
        if u==0: return self.sections[s].milestones[i]
        return vectorops.interpolate(self.sections[s].milestones[i],self.sections[s].milestones[i+1],u)

    def getTrajectory(self,robot=None,eps=None):
        """Returns a trajectory representation of this MultiPath.  If robot is provided, then a RobotTrajectory
        is returned.  Otherwise, if velocity information is given, then a HermiteTrajectory is returned.
        Otherwise, a Trajectory is returned.

        If robot and eps is given, then the IK constraints along the trajectory are solved and the path is
        discretized at resolution eps.
        """
        from . import trajectory
        res = trajectory.Trajectory()
        if robot is not None:
            res = trajectory.RobotTrajectory(robot)
            if self.sections[0].velocities is not None:
                warnings.warn("MultiPath.getTrajectory: can't discretize IK constraints with velocities specified")
        elif self.sections[0].velocities is not None:
            res = trajectory.HermiteTrajectory()

        if robot is not None and eps is not None:
            from ..plan.robotcspace import ClosedLoopRobotCSpace
            hastiming = self.hasTiming()
            for i,s in enumerate(self.sections):
                space = ClosedLoopRobotCSpace(robot,self.getIKProblem(i))
                for j in range(len(s.configs)-1):
                    ikpath = space.interpolationPath(s.configs[j],s.configs[j+1],eps)
                    if hastiming:
                        t0 = s.times[j]
                        t1 = s.times[j+1]
                    else:
                        t0 = len(res.milestones)
                        t1 = t0 + 1
                    iktimes = [t0 + float(k)/float(len(ikpath)-1)*(t1-t0) for k in range(len(ikpath))]
                    res.milestones += ikpath[:-1]
                    res.times += iktimes[:-1]
            res.milestones.append(self.sections[-1].configs[-1])
        else:
            for s in self.sections:
                res.milestones += s.configs[:-1]
            res.milestones.append(self.sections[-1].configs[-1])
            if self.sections[0].velocities is not None:
                vels = []
                for s in self.sections:
                    assert s.velocities is not None,"Some sections have velocities, some don't?"
                    vels += s.velocities[:-1]
                vels.append(self.sections[-1].velocities[-1])
                for i,q in enumerate(res.milestones):
                    assert len(vels[i]) == len(q),"Velocities don't have the right size?"
                    res.milestones[i] = q + vels[i]
            if not self.hasTiming():
                res.times = list(range(len(res.milestones)))
            else:
                for s in self.sections:
                    res.times += s.times[:-1]
                res.times.append(self.sections[-1].times[-1])
        return res

def _escape_nl(text):
    return escape(text).replace('\n','&#x0A;')

def _prettify(elem,indent_level=0):
    """Return a pretty-printed XML string for the Element.
    """
    indent = "  "
    res = indent_level*indent + '<'+elem.tag.encode('utf-8')
    for k in elem.keys():
        res += " "+k.encode('utf-8')+'="'+_escape_nl(elem.get(k)).encode('utf-8')+'"'
    children  = elem.getchildren()
    if len(children)==0 and not elem.text:
        res += ' />'
        return res
    res += '>'
    
    if elem.text:
        res += _escape_nl(elem.text).encode('utf-8')
    for c in children:
        res += '\n'+_prettify(c,indent_level+1)
    if len(children)>0:
        res += '\n'+indent_level*indent
    res += '</'+elem.tag.encode('utf-8')+'>'
    return res


