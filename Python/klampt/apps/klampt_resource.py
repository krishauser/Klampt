from klampt import *
from klampt import vis
from klampt.model import types,trajectory,multipath
from klampt.io import resource,loader
from klampt.math import vectorops
import optparse
import os,glob
import sys
import json

def mkdir_p(path):
    import os, errno
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

temp_world = WorldModel()
def read_resource(fn):
    global temp_world
    base,ext = os.path.splitext(fn)
    if ext in ['.rob','.urdf','.obj','.env']:
        id = temp_world.loadElement(fn)
        if id < 0:
            raise IOError("Unable to load item",fn)
        if ext in ['.rob','.urdf']:
            return temp_world.robot(temp_world.numRobots()-1)
        elif ext == '.obj':
            return temp_world.rigidObject(temp_world.numRigidObjects()-1)
        elif ext == '.env':
            return temp_world.terrains(temp_world.numTerrains()-1)
        
    path,filename = os.path.split(fn)
    return resource.get(filename,doedit=False,directory=path)

def make_thumbnail(fn,world):
    """Returns an image for a resource."""
    path,filename = os.path.split(fn)
    ext = os.path.splitext(filename)[1]
    if ext in ['.xml','.rob','.urdf','.obj','.env']:
        #world file
        print(fn)
        world = WorldModel()
        if not world.readFile(fn):
            raise IOError("Unable to read file "+fn)
        im = resource.thumbnail(world,(128,96))
        if im == None:
            print("Could not save thumbnail.")
            exit(0)
        vis.lock()
        del world
        vis.unlock()
        return im
    elif ext in resource.known_types():
        res = resource.get(filename,doedit=False,directory=path)
        im = resource.thumbnail(res,(128,96),world=world)
        if im is None:
            print("Could not save thumbnail.")
            exit(0)
        return im


def extract(object,index):
    """path must be a path-like object"""
    type = types.object_to_type(object)
    if not type.endswith("Trajectory") and type not in ["Configs","MultiPath"]:
        raise ValueError("extract only works with path-like objects.")
    if type == "Configs":
        return object[index]
    if type == "MultiPath":
        return object.getTrajectory().milestones[index]
    return object.milestones[index]

def timescale(path,scale,ofs):
    """path must be a path-like object"""
    type = types.object_to_type(path)
    if not type.endswith("Trajectory") and type not in ["Configs","MultiPath"]:
        raise ValueError("path must be a path-like object.")
    timed_object = True
    if type == "Configs":
        timed_object = False
    if type == "MultiPath":
        timed_object = path.hasTiming()
    if not timed_object:
        if abs(scale) != 1 or int(ofs) != ofs:
            #adding timing
            if type == 'Configs':
                res = trajectory.Trajectory()
                res.times = list(range(len(path)))
                res.milestones = path
                return timescale(res,scale,ofs)
            else:
                #MultiPath
                t0 = 0
                for s in path.sections:
                    s.times = list(range(t0,t0+len(path.configs)))
                    t0 += len(path.configs)
                assert path.hasTiming()
                return timescale(path,scale,ofs)
        if scale < 0:
            if type == "Configs":
                path.reverse()
            else:
                for s in path.sections:
                    s.configs.reverse()
                    if s.velocities:
                        s.velocities.reverse()
                        for i in range(s.velocities):
                            s.velocities[i] = vectorops.mul(s.velocities[i],-1)
                path.sections.reverse()
        if ofs > 0: #pad
            if type == 'Configs':
                return [path[0]]*ofs + path
            else:
                path.sections[0].configs = [path.sections[0].configs[0]]*ofs + path.sections[0].configs
                if path.sections[0].velocities:
                    path.sections[0].velocities = [[0.0]*path.sections[0].configs[0]]*ofs + path.sections[0].velocities
        elif ofs < 0: #gobble
            if type == 'Configs':
                if -ofs >= len(path):
                    ofs = -(len(path)-1)
                return path[-ofs:]
            else:
                raise RuntimeError("TODO: gobbling Multipath configs")
                while ofs > 0 and len(path.sections) > 0:
                    if -ofs >= len(path.sections[0]):
                        ofs = -(len(path.sections[0])-1)
                    nconfigs = len(path.sections[0].configs)
                    path.sections[0].configs = path.sections[0].configs[-ofs:]
                    ofs -= nconfigs
                    if ofs <= 0:
                        path.sections.pop(0)
        return path

    #standard trajectory scaling
    if type == 'MultiPath':
        if scale < 0:
            et = path.endTime()
            for s in path.sections:
                s.times = [ofs-scale*(et-t) for t in s.times]
                s.times.reverse()
                s.configs.reverse()
                if s.velocities:
                    s.velocities.reverse()
                    for i in range(s.velocities):
                        s.velocities[i] = vectorops.mul(s.velocities[i],scale)
            path.sections.reverse()
        else:
            for s in path.sections:
                s.times = [ofs+scale*t for t in s.times]
    else:
        if scale < 0:
            et = path.times[-1]
            path.times = [ofs-scale*(et-t) for t in path.times]
            path.times.reverse()
            path.milestones.reverse()
        else:
            path.times = [ofs+scale*t for t in path.times]
    return path


def combine(resources,world):
    if len(resources) == 0:
        raise ValueError("No resources specified")
    rtypes = [types.object_to_type(r,world) for r in resources]
    for r in rtypes:
        if r != rtypes[0]:
            raise ValueError("Can't combine objects of multiple types")
    if rtypes[0] == 'Geometry3D': #make a group geometry
        return Geometry3D(resources)
    if rtypes[0] in ['Config','Vector']: #make a Configs
        return rtypes
    if rtypes[0] == "Configs": #concatenate milestones
        res = [resources[0][0]]
        for r in resources:
            if r[0] != res[-1]:
                res.append(r[0])
            res += r[1:]
        return res
    if rtypes[0] == 'Trajectory': #concatenate trajectories
        res = resources[0]
        for r in resources[1:]:
            res = res.concat(r,True,'jump')
        return res
    if rtypes[0] == 'MultiPath':
        res = resources[0]
        for r in resources[1:]:
            res = res.concat(r)
        return res
    raise ValueError("Can't combine resources of type {}".format(rtypes[0]))


def pprint(info,indent=0):
    indent_str = ' '*indent
    for k,v in info.items():
        if k == 'type' and isinstance(v,list):
            continue
        if k == 'possible types':
            if 'type' in info and isinstance(info['type'],str):
                pass
            else:
                print("{}type: {}".format(indent_str,v[0]))
        elif isinstance(v,dict):
            print("{}{}:".format(indent_str,k))
            pprint(v,indent+2)
        elif isinstance(v,list) and len(v) > 0 and isinstance(v[0],dict):
            print('{}{}: ['.format(indent_str,k))
            for i in range(len(v)):
                print('{} {}:'.format(indent_str,i))
                pprint(v[i],indent+2)
            print('{}]'.format(indent_str))
        else:
            print("{}{}: {}".format(indent_str,k,v))


def operate(input_file,options,world,source_robot,target_robot):
    if options.thumbnails:
        return make_thumbnail(input_file,world)

    if options.combine:
        assert isinstance(input_file,list)
        inputs = []
        for f in input_file:
            path,filename = os.path.split(f)
            res = resource.get(filename,doedit=False,directory=path)
            inputs.append(res)
        return combine(inputs,world)

    res = read_resource(input_file)
    if options.print_info:
        print("{}:".format(input_file))
        info = types.info(res, world)
        if "possible_types" in info:  #use filename to disambiguate
            ptypes = loader.filename_to_types(input_file)
            if len(ptypes) == 1:
                info["type"]=ptypes[0]
                del info["possible_types"]
        pprint(info,2)
        print()
        return
    if options.transfer_robot is not None:
        #transfer
        return types.transfer(res,source_robot,target_robot)
    elif options.convert_type is not None:
        if options.convert_type.lower() == 'json':
            return loader.to_json(res)
        else:
            return types.convert(res,options.convert_type)
    elif options.extract_item is not None:
        return extract(res,int(options.extract_item))
    elif options.timescale is not None or options.timeshift is not None:
        timescale = 1 if options.timescale is None else options.timescale
        timeshift = 0 if options.timeshift is None else options.timeshift
        return timescale(res,timescale,timeshift)
    else:
        return None

def preferred_file_extension(obj,world):
    type = types.object_to_type(obj,world)
    return loader.TYPE_TO_EXTENSIONS[type][0]

def main():
    usage = "Usage: %prog [options] INPUT [OUTPUT]\nPerforms operations on one or more Klamp't objects.\nINPUT and OUTPUT can refer to files or folders."
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-w", "--world", dest="world",default=None,metavar="WORLD", help="Use WORLD as a reference world")
    parser.add_option("-r", "--robot", dest="robot",default=None,metavar="ROBOT", help="Use ROBOT as a reference robot")
    parser.add_option("--stdout",  dest="print_output", action="store_true", help="Prints output rather than saving it")
    parser.add_option("--info",  dest="print_info", action="store_true", help="Prints high-level information about the resource")
    parser.add_option("--transfer", dest="transfer_robot", default=None,metavar="DEST_ROBOT",help="Convert a Config, Configs, or Trajectory to a different robot")
    parser.add_option("--thumbnails", action="store_true",dest="thumbnails",default=False, help="Compute thumbnails")
    parser.add_option("--convert", dest="convert_type", default=None,metavar="TYPE", help="Convert from one type to another (or to JSON)")
    parser.add_option("--combine", dest="combine", action="store_true", help="Combines multiple items into a compound item, if valid")
    parser.add_option("--extract",dest="extract_item",default=None,metavar="INDEX", help="Extracts item INDEX from compound objects like Configs. Negative values go backwards from end.")
    parser.add_option("--timescale",dest="timescale",default=None,metavar="SCALE", help="Scales the duration of path-like objects.  The start time is preserved.  SCALE=-1 reverses a path.")
    parser.add_option("--timeshift",dest="timeshift",default=None,metavar="SHIFT", help="Shifts the start time of path-like objects.")
    (options,args) = parser.parse_args()

    if len(args) == 0:
        parser.print_usage()
        parser.print_help()
        exit(1)
    if len(args) > 2:
        parser.print_usage()
        parser.error("Must provide 1 or 2 arguments")
        exit(1)

    world = None
    source_robot = None
    target_robot = None
    if options.world is not None:
        world = WorldModel()
        if not world.readFile(os.path.expanduser(options.world)):
            parser.error("Error reading the specified world file")
            return 1
    if options.robot is not None:
        if world is not None:
            if world.numRobots() > 0:
                parser.error("Can't specify both a world with a robot and a robot")
                return 1
        else:
            world = WorldModel()
        if not world.readFile(os.path.expanduser(options.robot)):
            parser.error("Error reading the specified robot file")
            return 1

    input_files =  [args[0]]
    output_file = None
    output_folder = None

    #default policies for converting arguments
    if options.transfer_robot is not None:
        if world is None or world.numRobots() == 0:
            parser.error("Need a source world or robot")
            return 1
        if not world.readFile(os.path.expanduser(options.transfer_robot)):
            parser.error("Error reading the specified target robot")
        source_robot = world.robot(0)
        target_robot = world.robot(world.numRobots()-1)
        output_file = lambda x,file,ext:file+'_transfer'+ext
    elif options.thumbnails:
        output_file = lambda x,file,ext:file+'_thumb.png'
    elif options.convert_type is not None:
        if options.convert_type == 'json':
            convert_ext = '.json'
        else:
            convert_ext = loader.TYPE_TO_EXTENSIONS[options.convert_type][0]
        output_file = lambda x,file,ext:file+convert_ext
    elif options.combine is not None:
        output_file = lambda x,file,ext:'combined'+preferred_file_extension(x,world)
    elif options.extract_item is not None:
        output_file = lambda x,file,ext:file+'_extracted'+preferred_file_extension(x,world)
    elif options.timescale is not None or options.timeshift is not None:
        output_file = lambda x,file,ext:file+'_timewarp'+preferred_file_extension(x,world)

    if not options.print_info and output_file is None:
        parser.error("No operation is performed, --info, --transfer, --thumbnails, --convert, --timeshift, or --timescale must be speciified")
        return 1

    if os.path.isdir(input_files[0]):
        input_folder = input_files[0]
        if len(args) == 2:
            output_folder = args[1]
        else:
            if options.transfer_robot is not None:
                output_folder = 'transfer'
            elif options.thumbnails:
                output_folder = 'thumbnails'
            elif options.convert_type is not None:
                output_folder = 'convert'
            elif options.extract_item is not None:
                output_folder = 'extract'
            elif options.timescale is not None or options.timeshift is not None:
                output_folder = 'timewarp'

        if options.transfer_robot is not None and input_folder != output_folder:  #just transfer names directly
            output_file = lambda x,ext:x+ext

        if not options.print_output:
            print("Outputting to directory",output_folder)
            if os.path.isdir(output_folder):
                mkdir_p(output_folder)

        input_files = []
        for fn in glob.glob(input_folder+"/*"):
            input_files.append(fn)
    else:
        if options.combine:
            input_files = [args[:-1]]
            output_file = args[-1]
        else:
            if len(args) == 2 or options.print_info or options.print_output:
                output_file = args[-1]
            else:
                print("Only one input file is supported with that option")
                return 1

    if options.thumbnails and options.print_output:
        print("Can't print thumbnails to stdout")
        return 1

    #Run it!
    for input_file in input_files:
        try:
            res = operate(input_file,options,world,source_robot,target_robot)
        except Exception as e:
            from traceback import print_exc
            print("Unable to process",input_file)
            print("   Exception:",e)
            print_exc()
            continue

        if res is not None:
            #now what to do with the output
            if callable(output_file):
                base,ext = os.path.splitext(input_file)
                output = output_file(res,base,ext)
                #print("reconfigured filename",input_file,"to",output)
            else:
                output = output_file
            
            path,filename = os.path.split(output)
            if output_folder is not None:
                output = os.path.join(output_folder,filename)
        
            if options.thumbnails:
                print("Saving thumbnail of",input_file,"to",output)
                res.save(output)
            elif options.print_output:
                if isinstance(res,dict):
                    print(res)
                else:
                    print(loader.write(res,'auto'))
            else:
                print("Saving result of processing",input_file,"to",output)
                if isinstance(res,multipath.MultiPath):
                    res.settings['program']=' '.join(sys.argv)
                resource.set(filename,res,directory=path)


if __name__ == '__main__':
    main()
