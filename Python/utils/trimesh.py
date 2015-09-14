from klampt.vectorops import *
import math
#import numpy

def rotationMatrix(x,y,z,rads):
    c = math.cos(rads)
    s = math.sin(rads)
    #cross = numpy.matrix([[0.0,-z,y],[z,0.0,-x],[-y,x,0.0]])
    #return numpy.eye(3)+c*cross-s*cross*cross;
    val = [[c,0.0,0.0],[0.0,c,0.0],[0.0,0.0,c]]
    rrt = [[x*x,x*y,x*z],[y*x,y*y,y*z],[z*x,z*y,z*z]]
    cross = [[0.0,-z,y],[z,0.0,-x],[-y,x,0.0]]
    for i in range(3):
        for j in range(3):
            val[i][j] += (1.0-c)*rrt[i][j] + s*cross[i][j]
    return val

def mulMat(mat,x):
    return (dot(mat[0],x),dot(mat[1],x),dot(mat[2],x))

def triangleNormal(a,b,c):
    ba = sub(b,a)
    ca = sub(c,a)
    n = cross(ba,ca)
    return div(n,norm(n))

def triangleArea(a,b,c):
    if len(a)!=len(b) or len(a)!=len(c):
        raise 'point dimensions not equal'
    if len(a)==3:
        ba=(b[0]-a[0],b[1]-a[1],b[2]-a[2])
        ca=(c[0]-a[0],c[1]-a[1],c[2]-a[2])
        n = cross(ba,ca)
        return norm(n)*0.5
    elif len(a)==2:
        ba=(b[0]-a[0],b[1]-a[1],b[2]-a[2])
        ca=(c[0]-a[0],c[1]-a[1],c[2]-a[2])
        return cross(ba,ca)*0.5
    else:
        raise 'Triangle must be in 2 or 3D'

def triangleWeight(a,b,c):
    dA=distanceSquared(b,c)
    dB=distanceSquared(a,c)
    dC=distanceSquared(a,b)
    try:
        return (dA+dB+dC)/triangleArea(a,b,c)
    except ZeroDivisionError:
        return float('infinity')

def outputTriangles(ind,i,k):
    j=ind[i][k]
    if j<0:
        return []
    if j<=i or j>=k:
        raise 'invalid indices %d %d %d' % (i,j,k)
    return [(i,j,k)]+outputTriangles(ind,i,j)+outputTriangles(ind,j,k)

def triangulateConvexOptimal(verts):
    if len(verts)<3:
        raise 'Face must have 3 or more points'
    cost=[]
    ind=[]
    for i in range(len(verts)):
        cost.append([0]*len(verts))
        ind.append([-1]*len(verts))
    for i in range(len(verts)-2):
        cost[i][i+2]=triangleWeight(verts[i],verts[i+2],verts[i+1])
        ind[i][i+2]=i+1
    for diag in range(3,len(verts)):
        for i in range(len(verts)-diag):
            k=i+diag
            alts=[(cost[i][j]+cost[j][k]+triangleWeight(verts[i],verts[j],verts[k]),j) for j in range(i+1,k)]
            (cost[i][k],ind[i][k])=min(alts)
    tris=[]
    res=outputTriangles(ind,0,len(verts)-1)
    return res

def triangulateConvex(verts):
    if len(verts)<3:
        raise 'Face must have 3 or more points'
    n=len(verts)
    corner = 0
    largestArea = 0
    for i in range(n):
        leastArea = 1e100
        for j in range(1,n-1):
            v1 = (i+j)%n
            v2 = (i+j+1)%n
            a = triangleArea(verts[i],verts[v1],verts[v2])
            if a < leastArea: leastArea = a
        if leastArea > largestArea:
            largestArea = leastArea
            corner = i
    res = []
    for i in range(1,n-1):
        v1 = (corner+i)%n
        v2 = (corner+i+1)%n
        res.append((corner,v1,v2))
    return res

class TriMesh:
    points = []
    triangles = []

    def __init__(self):
        self.points = []
        self.triangles = []
    
    def triangulate(self,pts,faces):
        tris=[]
        for f in faces:
            inds=triangulateConvex([pts[i] for i in f])
            tris.extend([(f[i],f[j],f[k]) for (i,j,k) in inds]);
        return tris

    def triangulateOptimal(self,pts,faces):
        tris=[]
        for f in faces:
            inds=triangulateConvexOptimal([pts[i] for i in f])
            tris.extend([(f[i],f[j],f[k]) for (i,j,k) in inds]);
        return tris

    def simpleTriangulate(self,pts,faces):
        tris=[]
        for f in faces:
            for i in range(len(f)-2):
                tris.append((f[0],f[i+1],f[i+2]))
        return tris

    def addTriangle(self,pts):
        if len(pts) != 3: raise 'must give a list of 3 points'
        offset = len(self.points)
        self.points = self.points + (pts)
        self.triangles.append((offset,offset+1,offset+2))

    def addConvexPolygon(self,pts):
        inds=triangulateConvex(pts)
        offset = len(self.points)
        self.points = self.points + (pts)
        offsettris = [(a+offset,b+offset,c+offset) for (a,b,c) in inds]
        self.triangles = self.triangles + offsettris
    
    def addTriangles(self,pts,tris):
        offset = len(self.points)
        self.points = self.points + (pts)
        offsettris = [(a+offset,b+offset,c+offset) for (a,b,c) in tris]
        self.triangles = self.triangles + offsettris
        
    def flipYZ(self):
        for p in range(len(self.points)):
            (a,b,c) = self.points[p]
            self.points[p] = (a,-c,b)
            
    def flipZ(self):
        for p in range(len(self.points)):
            (a,b,c) = self.points[p]
            self.points[p] = (a,b,-c)

    def flipFaces(self):
        for t in range(len(self.triangles)):
            (a,b,c) = self.triangles[t]
            self.triangles[t] = (a,c,b)

    def rotate(self,x,y,z,rads):
        R = rotationMatrix(x,y,z,rads)
        for v in range(len(self.points)):
            self.points[v] = mulMat(R,self.points[v])

    def translate(self,x,y,z):
        for v in range(len(self.points)):
            self.points[v] = add(self.points[v],(x,y,z))

    def scale(self,s):
        for v in range(len(self.points)):
            self.points[v] = mul(self.points[v],s)

    def triangleNormal(self,tri):
        a = self.points[self.triangles[tri][0]]
        b = self.points[self.triangles[tri][1]]
        c = self.points[self.triangles[tri][2]]
        return triangleNormal(a,b,c)

    def load(self,fn):
        f = open(fn,'r')
        try:
            items = (' '.join(f.readlines())).split();
            nv = int(items[0])
            vtext = zip(items[1:1+nv*3:3],items[2:2+nv*3:3],items[3:3+nv*3:3])
            self.points = [(float(i[0]),float(i[1]),float(i[2])) for i in vtext]
            items = items[1+nv*3:]
            nt = int(items[0])
            ttext = zip(items[1:1+nt*3:3],items[2:2+nt*3:3],items[3:3+nt*3:3])
            self.triangles = [(int(i[0]),int(i[1]),int(i[2])) for i in ttext]
            items = items[1+nt*3:]
            if len(items) != 0:
                print('Warning,',len(items),'words at end of file')
        except:
            raise IOError('Error loading tri mesh from '+fn)
        finally:
            f.close()
            
    def save(self,fn):
        f = open(fn,'w')
        f.write(str(len(self.points))+'\n')
        for a,b,c in self.points:
            f.write('%g %g %g\n'%(a,b,c))
        f.write(str(len(self.triangles))+'\n')
        for a,b,c in self.triangles:
            f.write(str(a)+' '+str(b)+' '+str(c)+'\n')
        f.close()
