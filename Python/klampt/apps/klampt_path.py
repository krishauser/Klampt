import sys
import optparse
from klampt.model.multipath import MultiPath
from klampt.model.trajectory import Trajectory
from klampt.io import loader

class GeneralizedPath:
    def __init__(self):
        self.type = None
        self.data = None
    def load(self,fn):
        if fn.endswith('xml'):
            self.type = 'MultiPath'
            self.data = MultiPath()
            self.data.load(fn)
        elif fn.endswith('path') or fn.endswith('traj'):
            self.type = 'Trajectory'
            self.data = Trajectory()
            self.data.load(fn)
        elif fn.endswith('configs') or fn.endswith('milestones'):
            self.type = 'Configs'
            self.data = Trajectory()
            self.data.configs = loader.load('Configs',fn)
            self.data.times = list(range(len(self.data.configs)))
        else:
            print("Unknown extension on file",fn,", loading as Trajectory")
            self.type = 'Trajectory'
            self.data = Trajectory()
            self.data.load(fn)
    def save(self,fn):
        if fn.endswith('xml') and self.type != 'MultiPath':
            self.toMultipath()
        self.data.save(fn)
    def hasTiming(self):
        if self.type == 'Configs':
            return False
        if self.type == 'Trajectory':
            return True
        return self.data.hasTiming()
    def duration(self):
        if self.type == 'MultiPath':
            return self.data.endTime()
        else:
            return self.data.duration()
    def start(self):
        if self.type == 'MultiPath':
            return self.data.sections[0].configs[0]
        else:
            return self.data.milestones[0]
    def end(self):
        if self.type == 'MultiPath':
            return self.data.sections[-1].configs[-1]
        else:
            return self.data.milestones[-1]
    def timescale(self,scale,ofs):
        rev = (scale < 0)
        if rev:
            scale = -scale
            if self.type == 'MultiPath':
                if not self.data.hasTiming():
                    print("MultiPath timescale(): Path does not have timing")
                    return
                et = self.data.endTime()
                for s in self.data.sections:
                    s.times = [ofs+scale*(et-t) for t in s.times]
                    s.times.reverse()
                    s.configs.reverse()
                    if s.velocities:
                        s.velocities.reverse()
                self.data.sections.reverse()
            else:
                et = self.data.times[-1]
                self.data.times = [ofs+scale*(et-t) for t in self.data.times]
                self.data.times.reverse()
                self.data.milestones.reverse()
                if self.type == 'Configs':
                    if scale != 1 or ofs != 0:
                        self.type = 'Trajectory'
        else:
            if self.type == 'MultiPath':
                if not self.hasTiming():
                    print("MultiPath timescale(): Path does not have timing")
                    return
                for s in self.data.sections:
                    s.times = [ofs+scale*t for t in s.times]
            else:
                self.data.times = [ofs+scale*t for t in self.data.times]
                if self.type == 'Configs':
                    if scale != 1 or ofs != 0:
                        self.type = 'Trajectory'
    def toMultipath(self):
        if self.type == 'MultiPath':
            return
        else:
            traj = self.data
            mpath = MultiPath()
            mpath.sections.append(MultiPath.Section())
            mpath.sections[0].times = traj.times
            mpath.sections[0].configs = traj.milestones
            self.data = mpath
            self.type = 'MultiPath'
    def toTrajectory(self):
        if self.type == 'Trajectory':
            return
        elif self.type == 'Configs':
            self.type = 'Trajectory'
        else:
            times = []
            milestones = []
            for i,s in enumerate(self.data.sections):
                if i==0:
                    if s.times is not None:
                        times += s.times
                    milestones += s.configs
                else:
                    if s.times is not None:
                        times += s.times[1:]
                    milestones += s.configs[1:]
            if len(times)==0:
                times = list(range(len(milestones)))
            else:
                assert len(times)==len(milestones)
            self.data = Trajectory(times,milestones)
            self.type = 'Trajectory'
    def toConfigs(self):
        if self.type == 'Trajectory':
            self.type = 'Configs'
        elif self.type == 'Configs':
            return
        else:
            self.toTrajectory()
            self.data.times = list(range(len(self.data.milestones)))
            self.type = 'Configs'
    def concat(self,other):
        """Warning: destructive"""
        if self.type == 'MultiPath':
            other.toMultipath()
        elif other.type == 'MultiPath':
            self.toMultipath()
        if self.type == 'MultiPath':
            self.data.concat(other.data)
        else:
            self.data.concat(other.data,relative=True)
    def discretize(self,dt):
        if self.type == 'MultiPath':
            print("Warning, MultiPath discretize is not supported yet")
        else:
            self.data = self.data.discretize(dt)

    def __str__(self):
        if self.type == 'MultiPath':
            ndofs = 0
            if len(self.data.sections) > 0:
                ndofs = len(self.data.sections[0].configs[0])
            res = "MultiPath with %d DOFs, %d sections\n"%(ndofs,len(self.data.sections))
            sstrs = []
            for i,s in enumerate(self.data.sections):
                sres = "  Section %d: %d milestones"%(i,len(s.configs))
                if s.times!=None:
                    sres += " / times"
                if s.velocities!=None:
                    sres += " / velocities"
                sres += ", " + str(len(self.data.getStance(i))) + " holds"
                sstrs.append(sres)
            return res + '\n'.join(sstrs)
        else:
            ndofs = 0
            if len(self.data.milestones) > 0:
                ndofs = len(self.data.milestones[0])
            return '%s with %d DOFs, %d milestones'%(self.type,ndofs,len(self.data.milestones))

def main():
    usage = "Usage: %prog [options] filename(s)\nPerforms basic operations on MultiPath, Trajectory, or Configs files."
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-s", "--start", action="store_true",dest="start",default=False, help="Print start config")
    parser.add_option("-g", "--goal", action="store_true",dest="goal",default=False, help="Print goal config")
    parser.add_option("--duration", action="store_true",dest="duration",default=False, help="Print duration")
    parser.add_option("--concat", action="store_true",dest="concat", default=False, help="Concatenate multiple files")
    parser.add_option("--timescale", dest="timescale",default=None, help="Scale timing")
    parser.add_option("--timeshift", dest="timeshift",default=None, help="Shift timing")
    parser.add_option("--reverse", action="store_true",dest="reverse",default=False, help="Reverse path")
    parser.add_option("-d","--discretize", dest="discretize",metavar="DT", help="Discretize the time domain by spacing DT")
    parser.add_option("-c","--convert", dest="convert",metavar="TYPE", help="Converts to TYPE=Configs, Trajectory, or MultiPath")
    parser.add_option("-o","--output", dest="output",metavar="FILE", help="Output to FILE")
    (options,args) = parser.parse_args()
    
    if len(args) < 1:
        parser.error("Not enough arguments")
        exit(1)

    paths = [GeneralizedPath() for arg in args]
    for p,arg in zip(paths,args):
        p.load(arg)
        if not options.start and not options.goal and not options.duration:
            print("Loaded",p)

    scale = (float(options.timescale) if options.timescale else 1)
    ofs = (float(options.timeshift) if options.timeshift else 0)
    if options.reverse:
        if scale != 1 or ofs != 0:
            print("Cannot specify both --reverse and --timescale or --timeshift options")
            exit(1)
        scale = -1
    if scale != 1 or ofs != 0:
        for p in paths:
            p.timescale(scale,ofs)
        if not options.output:
            print("Warning: time scaled path not saved, --output needs to be specified")
    if options.concat:
        for i in range(1,len(paths)):
            if paths[i-1].end() != paths[i].start():
                print("Warning, path "+str(i)+" has incorrect start config")
                print(paths[i-1].end())
                print(paths[i].start())
            paths[0].concat(paths[i])
        if not options.output:
            print("Warning: concatenated path not saved, --output needs to be specified")
    if options.discretize:
        paths[0].discretize(float(options.discretize))
        if not options.output:
            print("Warning: discretized path not saved, --output needs to be specified")
    if options.convert:
        if options.convert.lower() == 'trajectory':
            paths[0].toTrajectory()
        elif options.convert.lower() == 'multipath':
            paths[0].toMultipath()
        elif options.convert.lower() == 'configs':
            paths[0].toConfigs()
        else:
            parser.error("Invalid convert argument, must be Configs, Trajectory, or Multipath")
            exit(1)
    if options.output:
        print("Saving to",options.output)
        if paths[0].type == 'MultiPath':
            paths[0].data.settings['program']=' '.join(sys.argv)
        paths[0].save(options.output)
    else:
        for p in paths:
            if options.start:
                print(loader.writeVector(p.start()))
            if options.goal:
                print(loader.writeVector(p.end()))
            if options.duration:
                print(p.duration())


if __name__ == '__main__':
    main()
    
