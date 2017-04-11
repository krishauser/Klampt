from klampt import *
from klampt import vis
from klampt.io import resource
import os,glob

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
	    print os.path.splitext(filename)[1]
	    if os.path.splitext(filename)[1] in ['.xml','.rob','.obj','.env']:
	    	#world file
	        print fn
	        world = WorldModel()
	        world.readFile(fn)
	        im = resource.thumbnail(world,(128,96))
	        if im != None:
	            im.save(os.path.join(outputfolder,filename+".thumb.png"))
	        else:
	        	print "Could not save thumbnail."
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
	        	print "Could not save thumbnail."
	        	exit(0)

make_thumbnails("../../data","thumbnails")
#make_thumbnails("../../data/robots","thumbnails/robots")
#make_thumbnails("../../data/objects","thumbnails/objects")
#make_thumbnails("../../data/terrains","thumbnails/terrains")
