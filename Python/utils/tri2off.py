import sys
import os
import fnmatch

def load_tri(fn):
    f = open(fn,'r')
    try:
        items = (' '.join(f.readlines())).split();
        nv = int(items[0])
        vtext = zip(items[1:1+nv*3:3],items[2:2+nv*3:3],items[3:3+nv*3:3])
        points = [(float(i[0]),float(i[1]),float(i[2])) for i in vtext]
        items = items[1+nv*3:]
        nt = int(items[0])
        ttext = zip(items[1:1+nt*3:3],items[2:2+nt*3:3],items[3:3+nt*3:3])
        triangles = [(int(i[0]),int(i[1]),int(i[2])) for i in ttext]
        items = items[1+nt*3:]
        if len(items) != 0:
            print('Warning,',len(items),'words at end of file')
    except:
        raise IOError('Error loading tri mesh from '+fn)
    finally:
        f.close()
        return (points,triangles)

def save_off(points,triangles,fn):
    f = open(fn,'w')
    f.write("OFF\n")
    f.write("%d %d 0\n"%(len(points),len(triangles)))
    for p in points:
        f.write("%f %f %f\n"%(p[0],p[1],p[2]))
    for t in triangles:
        f.write("3 %d %d %d\n"%(t[0],t[1],t[2]))
    f.close()

def recursive_glob(folder,pattern):
    for root, dirnames, filenames in os.walk(folder):
        for filename in fnmatch.filter(filenames, pattern):
            yield os.path.join(root, filename)
    return

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "Converts Klampt's simple .tri triangle mesh file to the more standard"
        print "Object File Format (OFF)."
        print
        print "Usage: python tri2off.py in.tri out.off (or provide a folder to convert all .tri files)"
        exit()
    
    tri_fn = sys.argv[1]
    if os.path.isdir(tri_fn):
        folder = tri_fn
        for tri_fn in recursive_glob(folder,"*.tri"):
            off_fn = os.path.splitext(tri_fn)[0]+'.off'
            print "Converting",tri_fn,"to",off_fn,"..."
            (p,t) = load_tri(tri_fn)
            save_off(p,t,off_fn)
        print "Done."
    else:
        off_fn = sys.argv[2]
        (p,t) = load_tri(tri_fn)
        save_off(p,t,off_fn)
    
