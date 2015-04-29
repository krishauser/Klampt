"""This module defines the MultiPath class, and methods for loading and
saving multipaths from xml files.

It can also be run in script mode to print stats about the path, to time-scale
or reverse a path, and concatenate multiple MultiPaths together.
"""

from hold import *
from loader import *
import vectorops
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
from xml.dom import minidom

class MultiPath:
    """Contains a list of Sections which are fixed-stance paths or
    trajectories.
    """
    
    class Section:
        """Contains a path or time-parameterized trajectory, as well as a
        list of holds and ik constraints
        
        If the times member is set, this is time parameterized.  Otherwise,
        it is just a path.
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
        if self.sections[0].times==None: return 0
        return self.sections[0].times[0]

    def endTime(self):
        """Returns the final time parameter"""
        if self.sections[-1].times==None: return sum(len(s.configs)-1 for s in self.sections)
        return self.sections[-1].times[-1]

    def hasTiming(self):
        """Returns true if the multipath is timed"""
        return self.sections[0].times!=None

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

    def concat(self,path):
        """Appends the path, making sure times and holds are appropriate set"""
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
        for (name,h) in path.holdSet.iteritems():
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

    def load(self,fn):
        """Loads this  multipath from a multipath xml file."""
        tree = ET.parse(fn)
        return self.loadXML(tree)

    def saveXML(self):
        """Saves this multipath to a multipath xml tree (ElementTree)"""
        root = ET.Element("multipath")
        root.attrib = self.settings
        for sec in self.sections:
            xs = ET.Element("section")
            root.append(xs)
            xs.attrib = sec.settings
            for ikgoal in sec.ikObjectives:
                xik = ET.SubElement(xs,"ikgoal")
                #xik.text = writeIKObjective(ikgoal)
                xik.text = ikgoal.text
            for h in sec.holds:
                xh = ET.SubElement(xs,"hold")
                xh.text = writeHold(h)
            for h in sec.holdIndices:
                xh = ET.SubElement(xs,"hold")
                xh.set("name",str(h))
            for i in xrange(len(sec.configs)):
                xm = ET.Element("milestone")
                xs.append(xm)
                xm.set("config",writeVector(sec.configs[i]))
                if sec.times != None:
                    xm.set("time",str(sec.times[i]))
                if sec.velocities != None:
                    xm.set("velocity",writeVector(sec.velocities[i]))
        for h in self.holdSet:
            xh = ET.Element("hold")
            root.append(xh)
            xh.text = writeHold(h)
        return ET.ElementTree(root)

    def loadXML(self,tree):
        """Loads a multipath from a multipath xml tree (ElementTree)."""
        self.sections = []
        self.holdSet = dict()
        self.settings = dict()
        root = tree.getroot()
        for k,v in root.attrib.iteritems():
            self.settings[k]=v
        for sec in root.findall('section'):
            s = MultiPath.Section()
            for k,v in sec.attrib.iteritems():
                s.settings[k]=v
            milestones = sec.findall('milestone')
            for m in milestones:
                if 'config' not in m.attrib:
                    raise ValueError("Milestone does not contain config attribute")
                s.configs.append(readVector(m.attrib['config']))
                if 'time' in m.attrib:
                    if s.times==None: s.times = []
                    s.times.append(float(m.attrib['time']))
                if 'velocity' in m.attrib:
                    if s.velocities==None: s.velocities = []
                    s.velocities.append(readVector(m.attrib['velocity']))
            for obj in sec.findall('ikgoal'):
                s.ikObjectives.append(readIKObjective(obj.text))
                s.ikObjectives[-1].text = obj.text
            for h in sec.findall('hold'):
                if 'index' in h.attrib:
                    s.holdIndices.append(int(h.attrib['index']))
                elif 'name' in h.attrib:
                    s.holdIndices.append(h.attrib['name'])
                else:
                    s.holds.append(readHold(h.text))
            self.sections.append(s)
        #read global hold set
        for h in root.findall('hold'):
            hold = readHold(h.text)
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
            i = int(math.floor(tseg))
            u = tseg - i
            return (s,i,u)
        else:
            i = bisect.bisect_left(sec.times,t)
            p = i-1
            u=(t-sec.times[p])/(sec.times[i]-sec.times[p])
            if i==0:
                return (s,0,0)
            return (s,p,u)

    def eval(self,t):
        (s,i,u) = self.getSegment(t)
        if s < 0: return self.startConfig()
        elif s >= len(self.sections): return self.endConfig()
        if u==0: return self.sections[s].milestones[i]
        return vectorops.interpolate(self.sections[s].milestones[i],self.sections[s].milestones[i+1],u)

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


def _main():
    import sys
    import optparse
    usage = "Usage: %prog [options] filename(s)"
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-s", "--start", action="store_true",dest="start",default=False, help="Print start config")
    parser.add_option("-g", "--goal", action="store_true",dest="goal",default=False, help="Print goal config")
    parser.add_option("--duration", action="store_true",dest="duration",default=False, help="Print duration")
    parser.add_option("--concat", dest="concat",metavar="FILE", help="Concatenate multiple files, output to FILE")
    parser.add_option("--timescale", dest="timescale",default=None, help="Scale timing")
    parser.add_option("--timeshift", dest="timeshift",default=None, help="Shift timing")
    (options,args) = parser.parse_args()
    
    if len(args) < 1:
        print "Not enough arguments"
        parser.print_help()
        exit(1)

    paths = [MultiPath() for arg in args]
    for p,arg in zip(paths,args):
        p.load(arg)
    scale = float(options.timescale) if options.timescale else 1
    ofs = float(options.timeshift) if options.timeshift else 0
    if scale != 1 or ofs != 0:
        rev = (scale < 0)
        if rev:
            scale = -scale;
            for p in paths:
                if not p.hasTiming():
                    print "Path does not have timing"
                    continue
                et = p.endTime()
                for s in p.sections:
                    s.times = [ofs+scale*(et-t) for t in s.times]
                    s.times.reverse()
                    s.configs.reverse()
                    if s.velocities:
                        s.velocities.reverse()
                p.sections.reverse()
        else:
            for p in paths:
                if not p.hasTiming():
                    print "Path does not have timing"
                    continue
                for s in p.sections:
                    s.times = [ofs+scale*t for t in s.times]
        if not options.concat:
            print "Warning: time scaling not saved, --concat needs to be specified"
    if options.concat:
        for i in range(1,len(paths)):
            if paths[i-1].endConfig() != paths[i].startConfig():
                print "Warning, path "+str(i)+" has incorrect start config"
                print paths[i-1].endConfig()
                print paths[i].startConfig()
            paths[0].concat(paths[i])
        paths[0].settings['program']=' '.join(sys.argv)
        print "Saving to",options.concat
        paths[0].save(options.concat)
        exit(0)
    for p in paths:
        if options.start:
            print writeVector(p.sections[0].configs[0])
        if options.goal:
            print writeVector(p.sections[-1].configs[-1])
        if options.duration:
            print p.endTime()
        if not options.start and not options.goal and not options.duration:
            print "Loaded MultiPath with",len(p.sections),"sections"
            for i,s in enumerate(p.sections):
                print "Section",i,":",
                print len(s.configs),"milestones",
                if s.times!=None:
                    print "/ times",
                if s.velocities!=None:
                    print "/ velocities",
                print len(p.getStance(i)),"holds",
                print


if __name__ == '__main__':
    _main()
    
