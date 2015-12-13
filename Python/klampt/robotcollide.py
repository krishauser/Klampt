"""A class for managing collision tests between all objects in a WorldModel.
"""

from __future__ import generators
from robotsim import *
import se3


def bb_intersect(a,b):
    """Returns true if the bounding boxes (a[0]->a[1]) and (b[0]->b[1]) intersect"""
    amin,amax=a
    bmin,bmax=b
    for i in xrange(len(amin)):
        if amax[i] < bmin[i] or bmax[i] < amin[i]:
            return False
    return True

class WorldCollider:
    """
    Attributes:
      - geomList: a list of (object,geom) pairs for all objects in the world
      - mask: a list of sets, indicating which items are activated for
        collision detection for each object in the world.
      - terrains: contains the geomList indices of each terrain in the world.
      - rigidObjects: contains the geomList indices of each object in
        the world
      - robots: contains the geomList indices of each robot in the world.

    Methods:
      - getGeom(obj): finds the geometry corresponding to an object
      - ignoreCollision(obj or obj pair): ignores collisions corresponding to
        an object or pair of objects
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
      - rayCast(ray_source,ray_direction,obj_indices): finds the first
        object intersected by a ray
      - rayCastRobot(robot_index,ray_source_ray_direction): finds the
        first robot link intersected by a ray
    """
    
    def __init__(self,world,ignore=[]):
        """Initializes the collision detection structure given a WorldModel
        as input."""

        world.enableInitCollisions(True)
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
            g = t.geometry()
            if g != None and g.type()!="":
                self.terrains.append(len(self.geomList))
                self.geomList.append((t,g))
            else:
                self.terrains.append(-1)
        for i in xrange(world.numRigidObjects()):
            o = world.rigidObject(i)
            g = o.geometry()
            if g != None and g.type()!="":
                self.rigidObjects.append(len(self.geomList))
                self.geomList.append((o,g))
            else:
                self.rigidObjects.append(-1)
        for i in xrange(world.numRobots()):
            r = world.robot(i)
            self.robots.append([])
            for j in xrange(r.numLinks()):
                l = r.link(j)
                g = l.geometry()
                if g != None and g.type()!="":
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
                for l1 in r:
                    for l2 in r2:
                        if l < 0 or l2 < 0: continue
                        self.mask[l1].add(l2)
                        self.mask[l2].add(l1)
            #robot self-collision
            rob = self.geomList[r[0]][0].robot()
            nl = rob.numLinks()
            for i in xrange(nl):
                for j in xrange(i):
                    if rob.selfCollisionEnabled(i,j):
                        self.mask[r[i]].add(r[j])
                        self.mask[r[j]].add(r[i])
                        
        for i in ignore:
            self.ignoreCollisions(i)
                
    def getGeom(self,object):
        for (o,g) in self.geomList:
            if o==object:
                return g
        return None


    def ignoreCollision(self,ign):
        """Permanently removes an object or a pair of objects from
        consideration.
        ign can be either a single body in the world or a pair of bodies."""
        if hasattr(ign,'__iter__'):
            (a,b) = ign
            ageom = self.getGeom(a)
            bgeom = self.getGeom(b)
            if ageom == None or bgeom == None:
                raise ValueError("Invalid ignore collision item, must be a body in the world")
            self.mask[ageom].discard(bgeom)
            self.mask[bgeom].discard(ageom)
        else:
            #ignore all collisions with the given geometry
            geom = self.getGeom(ign)
            if geom == None:
                raise ValueError("Invalid ignore collision item, must be a body in the world")
            for i in self.mask[geom]:
                #remove it from the list
                self.mask[i].discard(geom)
            self.mask[geom]=set()

    def collisionTests(self,filter1=None,filter2=None,bb_reject=True):
        """Iterates over ((object,geom),(object,geom)) pairs indicating
        which objects should be tested for collision.  The geom objects
        will be instances of Geometry3D.

        E.g., to test collisions, you will call

            for i,j in worldCollider.collisionTests():
                if i[1].collides(j[1]):
                    print "Object",i[0].getName(),"collides with",j[0].getName()
                    
        (Note that for this purpose is easier to just call collisions();
        however you may want to use collisionTests to perform other queries
        like proximity detection.)

        See collisions for a description of the filter1 and
        filter2 arguments.

        The argument bb_reject should be true if we should quick reject
        objects whose bounding boxes are not overlapping (broad phase
        collision detection).  Otherwise, it should be false.
        """
        res = []
        if filter1 == None: #all pairs
            if bb_reject: bblist = [g[1].getBB() for g in self.geomList]
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                for objIndex in objs:
                    #already checked
                    if objIndex < i: continue
                    if bb_reject and not bb_intersect(bblist[i],bblist[objIndex]): continue
                    yield (g,self.geomList[objIndex])
        elif filter2 == None: #self collision with objects passing filter1
            if bb_reject:
                #TODO: bounding box rejection, if requested
                pass
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                if not filter1(g[0]): continue
                for objIndex in objs:
                    #already checked
                    if objIndex < i: continue
                    if not filter1(self.geomList[objIndex][0]): continue
                    yield (g,self.geomList[objIndex])
        else:  #checks everything
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                f1 = filter1(g[0])
                f2 = filter2(g[0])
                for objIndex in objs:
                    #already checked
                    if self.geomList[objIndex][0]==g[0]:
                        continue
                    if f1 and filter2(self.geomList[objIndex][0]):
                        yield (g,self.geomList[objIndex])
                    elif f2 and filter1(self.geomList[objIndex][0]):
                        yield (self.geomList[objIndex],g)

    def collisions(self,filter1=None,filter2=None):
        """Returns an iterator over the colliding pairs of
        objects, optionally that satisfies the filter(s).

        Arguments filter1 and filter2 optionally indicate  subsets of
        objects to collide. If neither filter1 nor filter2 are provided,
        then all pairs are returned.

        If filter1 is provided but filter2 is not, then objects in the set
        filter1 will be collided against each other.

        If filter1 and filter2 are provided, then objects that
        satisfy filter1 will be collided against objects that satisfy
        filter2.  (Note: in this case there is no checking of duplicates,
        i.e., the sets should be disjoint to avoid duplicating work)."""
        for (g0,g1) in self.collisionTests(filter1,filter2):
            if g0[1].collides(g1[1]):
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
            for j in rindices:
                if i < j: break
                if j not in self.mask[i]: continue
                if self.geomList[i][1].collides(self.geomList[j][1]):
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
        for i in rindices:
            if i < 0: continue
            if oindex not in self.mask[i]: continue
            if self.geomList[oindex][1].collides(self.geomList[i][1]):
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
        for i in rindices:
            if i < 0: continue
            if tindex not in self.mask[i]: continue
            if self.geomList[tindex][1].collides(self.geomList[i][1]):
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
        if self.geomList[oindex][1].collides(self.geomList[tindex][1]):
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
        if self.geomList[oindex][1].collides(self.geomList[oindex2][1]):
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
            (coll,pt) = g[1].rayCast(s,d)
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
