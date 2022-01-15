from klampt import *
from klampt import vis
from klampt.model import types
from klampt.io import resource,loader
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

def make_thumbnail(fn,outputfile):
    path,filename = os.path.split(fn)
    ext = os.path.splitext(filename)[1]
    if ext in ['.xml','.rob','.obj','.env']:
        #world file
        print(fn)
        world = WorldModel()
        world.readFile(fn)
        im = resource.thumbnail(world,(128,96))
        if im != None:
            im.save(outputfile)
        else:
            print("Could not save thumbnail.")
            exit(0)
        vis.lock()
        del world
        vis.unlock()
    elif ext in resource.known_types():
        res = resource.get(filename,doedit=False,directory=path)
        im = resource.thumbnail(res,(128,96))
        if im != None:
            im.save(outputfile)
        else:
            print("Could not save thumbnail.")
            exit(0)

def main():
    usage = "Usage: %prog [options] INPUT [OUTPUT]\nPerforms operations on one or more Klamp't objects.\nINPUT and OUTPUT can refer to files or folders."
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-w", "--world", dest="world",default=None,metavar="WORLD", help="Use WORLD as a reference world")
    parser.add_option("-r", "--robot", dest="robot",default=None,metavar="ROBOT", help="Use ROBOT as a reference robot")
    parser.add_option("--transfer", dest="transfer_robot", default=None,metavar="DEST_ROBOT",help="Convert a Config, Configs, or Trajectory to a different robot")
    parser.add_option("--thumbnails", action="store_true",dest="thumbnails",default=False, help="Compute thumbnails")
    parser.add_option("--convert", dest="convert_type", default=None,metavar="TYPE", help="Convert from one type to another (or to JSON")
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
        output_file = lambda x,ext:x+'_transfer'+ext
    elif options.thumbnails:
        output_file = lambda x,ext:x+'_thumb.png'
    elif options.convert_type is not None:
        if options.convert_type == 'json':
            convert_ext = '.json'
        else:
            convert_ext = loader.TYPE_TO_EXTENSIONS[options.convert_type][0]
        output_file = lambda x,ext:x+convert_ext

    if output_file is None:
        parser.error("No operation is performed, --transfer, --thumbnails, or --convert must be speciified")
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
            
        if options.transfer_robot is not None and input_folder != output_folder:  #just transfer names directly
            output_file = lambda x,ext:x+ext

        print("Outputting to",output_folder)
        if os.path.isdir(output_folder):
            mkdir_p(output_folder)

        input_files = []
        for fn in glob.glob(input_folder+"/*"):
            input_files.append(fn)
    else:
        if len(args) == 2:
            output_file = args[1]


    #Run it!
    for input_file in input_files:
        if callable(output_file):
            base,ext = os.path.splitext(input_file)
            output = output_file(base,ext)
            #print("reconfigured filename",input_file,"to",output)
        else:
            output = output_file
        if output_folder is not None:
            path,filename = os.path.split(output)
            output = os.path.join(output_folder,filename)
        
        try:
            
            if options.transfer_robot is not None:
                #transfer
                path,filename = os.path.split(input_file)
                res = resource.get(filename,doedit=False,directory=path)
                res2 = types.transfer(res,source_robot,target_robot)
                path,filename = os.path.split(output)
                resource.set(filename,res2,directory=path)
            elif options.thumbnails:
                make_thumbnail(input_file,output,world=world)
            elif options.convert_type is not None:
                path,filename = os.path.split(input_file)
                res = resource.get(filename,doedit=False,directory=path)
                if options.convert_type == 'json':
                    with open(output,'w') as f:
                        json.dump(loader.to_json(res),f)
                else:
                    print("TODO: implement transfer. Try klampt_path instead for path conversions.")
                    return 1
        except Exception as e:
            print("Unable to process",input_file,"->",output)
            print("   Exception:",e)
            continue
        print("Processed",input_file,"->",output)

if __name__ == '__main__':
    main()
