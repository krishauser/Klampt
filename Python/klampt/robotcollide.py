from __future__ import generators
from robotsim import *
import collide
import se3


def makeGeom(trimesh):
    if len(trimesh.indices)==0:
        return None
    inds = collide.intArray(len(trimesh.indices))
    for index,v in enumerate(trimesh.indices):
        inds[index] = v
    verts = collide.doubleArray(len(trimesh.vertices))
    for index,v in enumerate(trimesh.vertices):
        verts[index] = v
    gindex = collide.newGeom()
    collide.makeTriMeshGeom(gindex,verts,inds,len(trimesh.vertices)/3,len(trimesh.indices)/3)
    return gindex

class WorldCollider:
    """
    Members:
      - geomList: a list of (object,geom) pairs
      - mask: a list of sets, indicating which items are activated for
        collision detection.
      - terrains: indicates the geomList indices of each terrain in the world.
      - rigidObjects: indicates the geomList indices of each object in the world
      - robots: indicates the geomList indices of each robot in the world.

    Methods:
      - destroy(): call this to cleanup the collide library
      - updateFrame(obj,geom): using the current transform of the object
      - updateFrames(): use the model's current status to update the geometry
        of the meshes.  Usually this doesn't need to be called.
      - getGeom(obj): finds the geometry corresponding to an object
      - collisionTests(filter1,filter2): returns an iterator over potential
        colliding pairs
      - collisions(filter1,filter2): yields an iterator over collision pairs
      - robotSelfCollisions(r): yields an iterator over robot self collisions
      - robotObjectCollisions(r,o): yields an iterator over robot-object
        collision pairs
      - robotTerrainCollisions(r,t): yields an iterator over robot-terrain
        collision pairs
      - objectTerrainCollide(o,t): returns whether an object and terrain
        collide
      - objectObjectCollide(o1,o2): returns whether two objects collide
    """
    
    def __init__(self,world):
        """Initializes the collision detection structure given a WorldModel
        as input."""
        
        self.world = world
        #a list of (object,geom) pairs
        self.geomList = []
        #self collision mask (2-D array)
        self.mask = []
        #indexing lists
        self.terrains = []
        self.rigidObjects = []
        self.robots = []
        
        for i in xrange(world.numTerrains()):
            t = world.terrain(i)
            g = makeGeom(t.getMesh())
            if g != None:
                self.terrains.append(len(self.geomList))
                self.geomList.append((t,g))
            else:
                self.terrains.append(-1)
        for i in xrange(world.numRigidObjects()):
            o = world.rigidObject(i)
            g = makeGeom(o.getMesh())
            if g != None:
                self.rigidObjects.append(len(self.geomList))
                self.geomList.append((o,g))
            else:
                self.rigidObjects.append(-1)
        for i in xrange(world.numRobots()):
            r = world.robot(i)
            self.robots.append([])
            for j in xrange(r.numLinks()):
                l = r.getLink(j)
                g = makeGeom(l.getMesh())
                if g != None:
                    self.robots[-1].append(len(self.geomList))
                    self.geomList.append((l,g))
                else:
                    self.robots[-1].append(-1)

        #construct the collision mask
        for i in xrange(len(self.geomList)):
            self.mask.append(set())
        for t in self.terrains:
            if t < 0: continue
            for o in self.rigidObjects:
                if o < 0: continue
                self.mask[t].add(o)
                self.mask[o].add(t)
            for r in self.robots:
                for l in r:
                    if l < 0: continue
                    #test for fixed links
                    if self.geomList[l][0].getParent() >= 0:
                        self.mask[l].add(t)
                        self.mask[t].add(l)
                    else:
                        #print "Ignoring fixed link..."
                        pass
        for o in self.rigidObjects:
            if o < 0: continue
            for o2 in self.rigidObjects[:o]:
                if o2 < 0: continue
                self.mask[o].add(o2)
                self.mask[o2].add(o)
            for r in self.robots:
                for l in r:
                    if l < 0: continue
                    self.mask[l].add(o)
                    self.mask[o].add(l)
        for i,r in enumerate(self.robots):
            #robot - robot collision
            for r2 in self.robots[0:i]:
                for l in r:
                    for l2 in r2:
                        if l < 0 or l2 < 0: continue
                        self.mask[l1].add(l2)
                        self.mask[l2].add(l1)
            #robot self-collision
            rob = self.geomList[r[0]][0].getRobot()
            nl = rob.numLinks()
            for i in xrange(nl):
                for j in xrange(i):
                    if rob.selfCollisionEnabled(i,j):
                        self.mask[r[i]].add(r[j])
                        self.mask[r[j]].add(r[i])
                
    def destroy(self):
        for g in self.geomList:
            destroyGeom(g[1])

    def updateFrame(self,object,geom):
        if hasattr(object,'getTransform'):
            R,t = object.getTransform()
            collide.setTriMeshRotation(geom,R)
            collide.setTriMeshTranslation(geom,t)

    def updateFrames(self):
        for (o,g) in self.geomList:
            self.updateFrame(o,g)

    def getGeom(self,object):
        for (o,g) in self.geomList:
            if o==object:
                return g
        return None

    def collisionTests(self,filter1=None,filter2=None):
        """Iterates over ((object,geom),(object,geom)) pairs indicating
        which objects to collide.  filter1 and filter2 optionally indicate
        subsets of objects to collide.

        If filter1 is provided but filter2 is not, then objects in the set
        filter1 will be collided against each other.

        If filter1 and filter2 are provided, then objects that
        satisfy filter1 will be collided against objects that satisfy
        filter2.  In this case there is no checking of duplicates."""
        res = []
        if filter1 == None:
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                for objIndex in objs:
                    #already checked
                    if objIndex < i: continue
                    yield (g,geomList[objIndex])
        elif filter2 == None:
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                if not filter1(g[0]): continue
                for objIndex in objs:
                    #already checked
                    if objIndex < i: continue
                    if not filter1(geomList[objIndex][0]): continue
                    yield (g,geomList[objIndex])
        else:
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                f1 = filter1(g[0])
                f2 = filter2(g[0])
                for objIndex in objs:
                    #already checked
                    if geomList[objIndex][0]==g[0]:
                        continue
                    if f1 and filter2(geomList[objIndex][0]):
                        yield (g,geomList[objIndex])
                    elif f2 and filter1(geomList[objIndex][0]):
                        yield (geomList[objIndex],g)

    def collisions(self,filter1=None,filter2=None):
        """Returns an iterator over the colliding pairs of
        objects, optionally that satisfies the filter(s)"""
        self.updateFrames()
        for (g0,g1) in self.collisionTests(filter1,filter2):
            if collide.collide(g0[1],g1[1]):
                yield (g0[0],g1[0])

    def robotSelfCollisions(self,robot=None):
        """Given robot, tests all self collisions.  If robot is None, all
        robots are tested.  If robots is an index or a RobotModel object
        only collisions for that robot are tested"""
        if isinstance(robot,RobotModel):
            try:
                robot = [r for r in xrange(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        if robot == None:
            #test all robots
            for r in xrange(len(self.robots)):
                for c in self.robotSelfCollisions(r):
                    yield c
            return
        rindices = self.robots[robot]
        for i in rindices:
            if i < 0: continue
            self.updateFrame(*self.geomList[i])
        for i in rindices:
            if i < 0: continue
            for j in rindices:
                if i < j: break
                if j not in self.mask[i]: continue
                if collide.collide(self.geomList[i][1],self.geomList[j][1]):
                    yield (self.geomList[i][0],self.geomList[j][0])
       
    def robotObjectCollisions(self,robot,object=None):
        """Given robot and object indices, tests all collisions between robot
        links and the object.  If object is not provided, all objects
        are tested"""
        if isinstance(robot,RobotModel):
            try:
                robot = [r for r in xrange(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        if isinstance(object,RigidObjectModel):
            try:
                object = [o for o in xrange(self.world.numRigidObjects()) if self.world.rigidObject(o).getID()==object.getID()][0]
            except IndexError:
                raise RuntimeError("RigidObject "+object.getName()+" is not found in the world!")
        if object == None:
            #test all objects
            for o in xrange(len(self.rigidObjects)):
                for c in self.robotObjectCollisions(robot,o):
                    yield c
            return

        rindices = self.robots[robot]
        oindex = self.rigidObjects[object]
        if oindex < 0: return
        self.updateFrame(*self.geomList[oindex])
        for i in rindices:
            if i < 0: continue
            if oindex not in self.mask[i]: continue
            self.updateFrame(*self.geomList[i])
            if collide.collide(self.geomList[oindex][1],self.geomList[i][1]):
                yield (self.geomList[i][0],self.geomList[oindex][0])

    def robotTerrainCollisions(self,robot,terrain=None):
        """Given robot and terrain indices, tests all collisions between robot
        links and the terrain"""
        if isinstance(robot,RobotModel):
            try:
                robot = [r for r in xrange(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        if isinstance(terrain,TerrainModel):
            try:
                terrain = [t for t in xrange(self.world.numTerrains()) if self.world.terrain(t).getID()==terrain.getID()][0]
            except IndexError:
                raise RuntimeError("Terrain "+robot.getName()+" is not found in the world!")
        if terrain == None:
            #test all terrains
            for t in xrange(len(self.terrains)):
                for c in self.robotTerrainCollisions(robot,t):
                    yield c
            return

        rindices = self.robots[robot]
        tindex = self.terrains[terrain]
        if tindex < 0: return
        self.updateFrame(*self.geomList[tindex])
        for i in rindices:
            if i < 0: continue
            if tindex not in self.mask[i]: continue
            self.updateFrame(*self.geomList[i])
            if collide.collide(self.geomList[tindex][1],self.geomList[i][1]):
                yield (self.geomList[i][0],self.geomList[tindex][0])

    def objectTerrainCollisions(self,object,terrain=None):
        if isinstance(object,RigidObjectModel):
            try:
                object = [o for o in xrange(self.world.numRigidObjects()) if self.world.rigidObject(o).getID()==object.getID()][0]
            except IndexError:
                raise RuntimeError("RigidObject "+object.getName()+" is not found in the world!")
        if isinstance(terrain,TerrainModel):
            try:
                terrain = [t for t in xrange(self.world.numTerrains()) if self.world.terrain(t).getID()==terrain.getID()][0]
            except IndexError:
                raise RuntimeError("Terrain "+robot.getName()+" is not found in the world!")
        if terrain == None:
            #test all terrains
            for t in xrange(len(self.terrains)):
                for c in self.objectTerrainCollisions(object,t):
                    yield c
            return
        oindex = self.rigidObjects[object]
        tindex = self.terrains[terrain]
        if oindex < 0: return
        if tindex < 0: return
        if tindex not in self.mask[oindex]: return
        self.updateFrame(*self.geomList[oindex])
        if collide.collide(self.geomList[oindex][1],self.geomList[tindex][1]):
            q = collide.makeCollQuery(self.geomList[oindex][1],self.geomList[tindex][1])
            coll = collide.queryCollide(q)
            d = collide.queryDistance(q,0,0)
            (cp1,cp2) = collide.queryClosestPoints(q)
            #print "Coll",coll,"dist",d,"Cps:",(se3.apply(self.geomList[oindex][0].getTransform(),cp1),cp2)
            
            collide.destroyCollQuery(q)
            yield (self.geomList[oindex][0],self.geomList[tindex][0])
        return

    def objectObjectCollisions(self,object,object2):
        if isinstance(object,RigidObjectModel):
            try:
                object = [o for o in xrange(self.world.numRigidObjects()) if self.world.rigidObject(o).getID()==object.getID()][0]
            except IndexError:
                raise RuntimeError("RigidObject "+object.getName()+" is not found in the world!")
        if isinstance(object2,RigidObjectModel):
            try:
                object2 = [o for o in xrange(self.world.numRigidObjects()) if self.world.rigidObject(o).getID()==object2.getID()][0]
            except IndexError:
                raise RuntimeError("RigidObject "+object2.getName()+" is not found in the world!")
        if object2 == None:
            #test all terrains
            for o in xrange(len(self.rigidObjects)):
                for c in self.objectObjectCollisions(objectot):
                    yield c
            return
        oindex = self.rigidObjects[object]
        oindex2 = self.rigidObjects[object2]
        if oindex < 0: return
        if oindex2 < 0: return
        if oindex not in self.mask[oindex2]: return
        self.updateFrame(*self.geomList[oindex])
        self.updateFrame(*self.geomList[oindex2])
        if collide.collide(self.geomList[oindex][1],self.geomList[oindex2][1]):
            yield (self.geomList[oindex][0],self.geomList[oindex2][0])
        return

    def rayCast(self,s,d,indices=None):
        """Finds the first collision with the ray at source s and direction
        d.  Returns the (object,point) pair or None if no collision is found.
        """
        res = None
        dmin = 1e300
        geoms = (self.geomList if indices==None else [self.geomList[i] for i in indices])
        for g in geoms:
            (coll,pt) = collide.rayCast(g[1],s,d)
            if coll:
                dist = vectorops.dot(d,vectorops,sub(pt,s))
                if dist < dmin:
                    dmin,res = dist,(g[0],pt)
        return res
                
    def rayCastRobot(self,robot,s,d):
        """Given robot index, do ray casting with the given ray"""
        if isinstance(robot,RobotModel):
            try:
                robot = [r for r in xrange(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        rindices = self.robots[robot]
        return self.rayCast(s,d,rindices)
