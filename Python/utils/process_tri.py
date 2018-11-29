import sys
import math
from collections import defaultdict
from trimesh import *

def load(fn):
    """(fn) Loads a mesh from file fn"""
    mesh = TriMesh()
    mesh.load(fn)
    print 'Mesh has',len(mesh.points),'vertices and',len(mesh.triangles),'triangles'
    bmin = (min([i[0] for i in mesh.points]),min([i[1] for i in mesh.points]),min([i[2] for i in mesh.points]))
    bmax = (max([i[0] for i in mesh.points]),max([i[1] for i in mesh.points]),max([i[2] for i in mesh.points]))
    print 'Bbox:',bmin[0],bmin[1],bmin[2],',',bmax[0],bmax[1],bmax[2]
    mesh.fn = fn
    return mesh

def loadMultiple(fns):
    """(fn) Loads one or more meshes from the list of files fns"""
    mesh = TriMesh()
    for fn in fns:
        m = TriMesh()
        m.load(fn)
        mesh.addTriangles(m.points,m.triangles)
    print 'Mesh has',len(mesh.points),'vertices and',len(mesh.triangles),'triangles'
    bmin = (min([i[0] for i in mesh.points]),min([i[1] for i in mesh.points]),min([i[2] for i in mesh.points]))
    bmax = (max([i[0] for i in mesh.points]),max([i[1] for i in mesh.points]),max([i[2] for i in mesh.points]))
    print 'Bbox:',bmin[0],bmin[1],bmin[2],',',bmax[0],bmax[1],bmax[2]
    mesh.fn = 'merged.tri'
    return mesh

def translate(mesh,x,y,z):
    """(x,y,z) Translates the mesh"""
    x = float(x)
    y = float(y)
    z = float(z)
    for i in range(len(mesh.points)):
        (a,b,c)=mesh.points[i]
        mesh.points[i] = (a+x,b+y,c+z)

def scale(mesh,x,y,z):
    """(x,[y,z]) Scales the mesh"""
    x = float(x)
    if y == None:
        y = x
        z = x
    else:
        y = float(y)
        z = float(z)
    for i in range(len(mesh.points)):
        (a,b,c)=mesh.points[i]
        mesh.points[i] = (a*x,b*y,c*z)

def rotate(mesh,axis,angle):
    """(axis,angle) Rotates the mesh by the angle (in degrees)"""
    angle = float(angle)
    if axis=='x':
        axis=(1,0,0)
    elif axis=='y':
        axis=(0,1,0)
    elif axis=='z':
        axis=(0,0,1)
    elif axis=='-x':
        axis=(1,0,0)
    elif axis=='-y':
        axis=(0,1,0)
    elif axis=='-z':
        axis=(0,0,1)
    else:
        raise ValueError('TODO: parse the axis')
    if len(axis)!=3:
        raise ValueError('Axis is not a 3 tuple')
    mesh.rotate(axis[0],axis[1],axis[2],angle/180*math.pi)

def flip(mesh):
    """Flips the triangles"""
    mesh.flipFaces()

def save(mesh):
    """Saves the mesh"""
    mesh.save(mesh.fn)

def save_as(mesh,fn):
    """(fn) Saves the mesh to fn"""
    mesh.save(fn)

def merge_vertices(mesh,tolerance=0):
    """Merges equal vertices of the mesh.  If tolerance is > 0, then
    vertices that are within distance tolerance of one another (on all
    axes) are merged"""
    tolerance = float(tolerance)
    newverts = defaultdict(list)
    for i,v in enumerate(mesh.points):
        if tolerance == 0:
            index = tuple(v)
        else:
            index = (math.round(v[0]/(2.0*tolerance)),math.round(v[1]/(2.0*tolerance)),math.round(v[2]/(2.0*tolerance)))
        newverts[index].append(i)
    newmesh = TriMesh()
    vertmap = dict()
    for (index,vlist) in newverts.iteritems():
        vavg = list(mesh.points[vlist[0]])
        for v in vlist[1:]:
            vavg[0] += mesh.points[v][0]
            vavg[1] += mesh.points[v][1]
            vavg[2] += mesh.points[v][2]
        if len(vlist) > 1:
            vavg[0] = vavg[0]/len(vlist)
            vavg[1] = vavg[1]/len(vlist)
            vavg[2] = vavg[2]/len(vlist)
        for v in vlist:
            vertmap[v] = len(newmesh.points)
        newmesh.points.append(vavg)
    for t in mesh.triangles:
        newt = (vertmap[t[0]],vertmap[t[1]],vertmap[t[2]])
        if newt[0] == newt[1] or newt[1] == newt[2] or newt[2] == newt[0]:
            continue
        newmesh.triangles.append(newt)
    print "Collapsed",len(mesh.points)-len(newmesh.points),"vertices"
    print "Collapsed",len(mesh.triangles)-len(newmesh.triangles),"triangles"
    mesh.points = newmesh.points
    mesh.triangles = newmesh.triangles

def clean(mesh):
    """Cleans the mesh"""
    newtris = []
    for t in mesh.triangles:
        if t[0] == t[1] or t[1] == t[2] or t[2] == t[0]:
            pass
        else:
            newtris.append(t)
    mesh.triangles = newtris

commandmap = {}

def print_help():
    """Prints help."""
    global commandmap
    for i in commandmap.items():
        if hasattr(i[1],'__doc__') and i[1].__doc__!=None:
            print(i[0]+' '+i[1].__doc__)
        else:
            print(i[0]+': not documented')

commandmap['help']=print_help
commandmap['h']=print_help
commandmap['?']=print_help
commandmap['save']=save
commandmap['s']=save
commandmap['saveas']=save_as
commandmap['sa']=save_as
commandmap['translate']=translate
commandmap['scale']=scale
commandmap['flip']=flip
commandmap['exit']=exit
commandmap['rotate']=rotate
commandmap['clean']=clean
commandmap['merge_vertices']=merge_vertices

def execute_command(mesh,txt):
    global commandmap
    words = txt.split()
    if len(words)==0:
        pass
    elif words[0][0]=='#':  #comment
        pass
    elif not words[0].lower() in commandmap:
        print('Invalid command')
        print_help()
    else:
        commandmap[words[0]](mesh,*words[1:])

interactive = False
meshes = []
initial_commands = []
i = 1
while i < len(sys.argv):
    if sys.argv[i]=='-s':
        f = open(sys.argv[i+1],'r')
        initial_commands += f.readlines()
        f.close()
        i+=1
    elif sys.argv[i]=='-i':
        interactive=True
    else:
        meshes.append(sys.argv[i])
    i += 1

if interactive:
    print("Loading "+" ".join(meshes)+" into merged.tri")
    if len(meshes) > 0:
        mesh = loadMultiple(meshes)
    else:
        mesh = TriMesh()
        mesh.load(meshes[0])
        print 'Mesh has',len(mesh.points),'vertices and',len(mesh.triangles),'triangles'
        bmin = (min([i[0] for i in mesh.points]),min([i[1] for i in mesh.points]),min([i[2] for i in mesh.points]))
        bmax = (max([i[0] for i in mesh.points]),max([i[1] for i in mesh.points]),max([i[2] for i in mesh.points]))
        print 'Bbox:',bmin[0],bmin[1],bmin[2],',',bmax[0],bmax[1],bmax[2]
    for line in initial_commands:
        execute_command(mesh,line)
    while True:
        txt = raw_input('> ')
        execute_command(mesh,txt)
else:
    for m in meshes:
        print("Loading "+m)
        mesh=load(m)
        for line in initial_commands:
            execute_command(mesh,line)
        print("Saving "+m)
        save(mesh)
