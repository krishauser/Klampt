from klampt import *
from klampt import vis
from klampt.io import resource
import optparse
import os,glob
import sys

def mkdir_p(path):
    import os, errno
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def make_thumbnails(folder,outputfolder):
    for fn in glob.glob(folder+"/*"):
        filename = os.path.basename(fn)
        mkdir_p(outputfolder)
        print(os.path.splitext(filename)[1])
        if os.path.splitext(filename)[1] in ['.xml','.rob','.obj','.env']:
            #world file
            print(fn)
            world = WorldModel()
            world.readFile(fn)
            im = resource.thumbnail(world,(128,96))
            if im != None:
                im.save(os.path.join(outputfolder,filename+".thumb.png"))
            else:
                print("Could not save thumbnail.")
                exit(0)
            vis.lock()
            del world
            vis.unlock()
        elif os.path.splitext(filename)[1] in resource.knownTypes():
            res = resource.get(filename,doedit=False,directory=folder)
            im = resource.thumbnail(res,(128,96))
            if im != None:
                im.save(os.path.join(outputfolder,filename+".thumb.png"))
            else:
                print("Could not save thumbnail.")
                exit(0)

def main():
    usage = "Usage: %prog [options] INPUT_FOLDER [OUTPUT_FOLDER]\nCreates thumbnails from a set of Klamp't objects."
    parser = optparse.OptionParser(usage=usage)
    parser.add_option("-w", "--world", dest="world",metavar="WORLD", help="Use WORLD as a reference world")
    (options,args) = parser.parse_args()

    if len(args) == 0 or len(args) > 2:
        parser.error("Must provide 1 or 2 arguments")
        exit(1)

    world = None
    if parser.world:
        world = WorldModel()
        if not world.readFile(parser.world):
            parser.error("Error reading the specified world file")

    output_folder = 'thumbnails'
    if len(args) == 2:
        output_folder = args[1]
    print("Outputting thumbnails to",output_folder)
    make_thumbnails(args[0],output_folder,world=world)
    
if __name__ == '__main__':
    main()
