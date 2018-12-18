"""Functions and classes for managing collision tests between multiple objects.
In particular, the WorldCollider class makes it easy to ignore various collision pairs
in a WorldModel.
"""


from ..robotsim import *
from ..math import vectorops,se3
import collections


def bb_intersect(a,b):
    """Returns true if the bounding boxes (a[0]->a[1]) and (b[0]->b[1]) intersect"""
    amin,amax=a
    bmin,bmax=b
    return not any(q < u or v < p for (p,q,u,v) in zip(amin,amax,bmin,bmax))

def bb_union(*bbs):
    """Returns a bounding box containing the given bboxes"""
    return [min(*x) for x in zip(*[b[0] for b in bbs])],[max(*x) for x in zip(*[b[1] for b in bbs])]


def self_collision_iter(geomlist,pairs='all'):
    """For a list of Geometry3D's, performs efficient self collision testing.
    Yields an iterator over pairs (i,j) where i and j are indices
    of the colliding geometries.

    If pairs == 'all', all pairs are tested.  If it's a function, it's
    a 2-argument function taking geometry indices and returning true if 
    they should be tested.  Otherwise it can be a list of collision indices.

    Uses a quick bounding box reject test."""
    #bblist = [g.getBB() for g in geomlist]
    if pairs=='all':
        for i,g in enumerate(geomlist):
            for j in range(i+1,len(geomlist)):
                #if not bb_intersect(bblist[i],bblist[j]): continue
                g2 = geomlist[j]
                if g.collides(g2):
                    yield (i,j)
    elif isinstance(pairs, collections.Callable):
        for i,g in enumerate(geomlist):
            for j in range(i+1,len(geomlist)):
                if not pairs(i,j): continue
                #if not bb_intersect(bblist[i],bblist[j]): continue
                g2 = geomlist[j]
                if g.collides(g2):
                    yield (i,j)
    else:
        for (i,j) in pairs:
            #if not bb_intersect(bblist[i],bblist[j]): continue
            g  =geomlist[i]
            g2 = geomlist[j]
            if g.collides(g2):
                yield (i,j)
    return

def group_collision_iter(geomlist1,geomlist2,pairs='all'):
    """Tests whether two sets of geometries collide.

    If pairs == 'all', all pairs are tested.  If it's a function, it's
    a 2-argument function taking geometry indices and returning true if 
    they should be tested.  Otherwise it can be a list of collision indices.

    Uses a quick bounding box reject test.
    """
    if len(geomlist1) == 0 or len(geomlist2) == 0: return
    bblist1 = [g.getBB() for g in geomlist1]
    bblist2 = [g.getBB() for g in geomlist2]
    bb1 = bb_union(*bblist1)
    bb2 = bb_union(*bblist2)
    geoms1 = [(i,g) for (i,g) in enumerate(geomlist1) if bb_intersect(bblist1[i],bb2)]
    geoms2 = [(i,g) for (i,g) in enumerate(geomlist2) if bb_intersect(bblist2[i],bb1)]
    if pairs=='all':
        for i,g in geoms1:
            for j,g2 in geoms2:
                #if not bb_intersect(bblist1[i],bblist2[j]): continue
                if g.collides(g2):
                    yield (i,j)
    elif isinstance(pairs, collections.Callable):
        for i,g in geoms1:
            for j,g2 in geoms2:
                if not pairs(i,j): continue
                #if not bb_intersect(bblist1[i],bblist2[j]): continue
                if g.collides(g2):
                    yield (i,j)
    else:
        for (i,j) in pairs:
            #if not bb_intersect(bblist1[i],bblist2[j]): continue
            g  = geomlist1[i]
            g2 = geomlist2[j]
            if g.collides(g2):
                yield (i,j)


def group_subset_collision_iter(geomlist,alist,blist,pairs='all'):
    """Tests whether two subsets of geometries collide.  Can be slightly faster
    than group_collision_iter if alist and blist overlap.

    If pairs == 'all', all pairs are tested.  If it's a function, it's
    a 2-argument function taking geometry indices and returning true if 
    they should be tested.  Otherwise it can be a list of collision indices.
    In this last case, alist and blist are ignored and can be set to None.

    Uses a quick bounding box reject test.
    """
    if len(alist) == 0 or len(blist) == 0: return
    bblist = [None]*len(geomlist)
    for id in alist:
        bblist[id] = geomlist[id].getBB()
    for id in blist:
        if bblist[id] is not None:
            bblist[id] = geomlist[id].getBB()
    bb1 = bb_union(*[bblist[i] for i in alist])
    bb2 = bb_union(*[bblist[i] for i in blist])
    geoms1 = [(i,geomlist[i]) for i in alist if bb_intersect(bblist[i],bb2)]
    geoms2 = [(i,geomlist[i]) for i in blist if bb_intersect(bblist[i],bb1)]
    if pairs=='all':
        for i,g in geoms1:
            for j,g2 in geoms2:
                #if not bb_intersect(bblist[i],bblist[j]): continue
                if g.collides(g2):
                    yield (i,j)
    elif isinstance(pairs, collections.Callable):
        for i,g in geoms1:
            for j,g2 in geoms2:
                if not pairs(i,j): continue
                #if not bb_intersect(bblist[i],bblist[j]): continue
                if g.collides(g2):
                    yield (i,j)
    else:
        for (i,j) in pairs:
            #if not bb_intersect(bblist[i],bblist[j]): continue
            g  =geomlist[i]
            g2 = geomlist[j]
            if g.collides(g2):
                yield (i,j)


def ray_cast(geomlist,s,d):
    """Finds the first collision among the geometries in geomlist with the ray at source s
    and direction d.  Returns a pair (index,point) if a collision is found, where index is the
    index of the geometry in geomlist, and point is the collision point in world coordinates.
    Returns None if no collision is found.
    """
    res = None
    dmin = 1e300
    for i,g in enumerate(geomlist):
        (coll,pt) = g.rayCast(s,d)
        if coll:
            dist = vectorops.dot(d,vectorops.sub(pt,s))
            if dist < dmin:
                dmin,res = dist,(i,pt)
    return res


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
      - getGeomIndex(obj): finds the geomList index corresponding to an object
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
        
        for i in range(world.numTerrains()):
            t = world.terrain(i)
            g = t.geometry()
            if g != None and g.type()!="":
                self.terrains.append(len(self.geomList))
                self.geomList.append((t,g))
            else:
                self.terrains.append(-1)
        for i in range(world.numRigidObjects()):
            o = world.rigidObject(i)
            g = o.geometry()
            if g != None and g.type()!="":
                self.rigidObjects.append(len(self.geomList))
                self.geomList.append((o,g))
            else:
                self.rigidObjects.append(-1)
        for i in range(world.numRobots()):
            r = world.robot(i)
            self.robots.append([])
            for j in range(r.numLinks()):
                l = r.link(j)
                g = l.geometry()
                if g != None and g.type()!="":
                    self.robots[-1].append(len(self.geomList))
                    self.geomList.append((l,g))
                else:
                    self.robots[-1].append(-1)

        #construct the collision mask
        for i in range(len(self.geomList)):
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
            for i in range(nl):
                for j in range(i):
                    if rob.selfCollisionEnabled(i,j):
                        self.mask[r[i]].add(r[j])
                        self.mask[r[j]].add(r[i])
                        
        for i in ignore:
            self.ignoreCollision(i)
                
    def getGeomIndex(self,object):
        assert isinstance(object,(RobotModel,RobotModelLink,RigidObjectModel,TerrainModel))
        for i,(o,g) in enumerate(self.geomList):
            if o.world==object.world and o.getID()==object.getID():
                assert o.getName()==object.getName()
                assert type(o) == type(object)
                return i
        return None


    def ignoreCollision(self,ign):
        """Permanently removes an object or a pair of objects from
        consideration.
        ign can be either a single body in the world or a pair of bodies."""
        if hasattr(ign,'__iter__'):
            (a,b) = ign
            ageom = self.getGeomIndex(a)
            bgeom = self.getGeomIndex(b)
            if ageom is None or bgeom is None:
                raise ValueError("Invalid ignore collision item, must be a pair of bodies in the world")
            self.mask[ageom].discard(bgeom)
            self.mask[bgeom].discard(ageom)
        else:
            #ignore all collisions with the given geometry
            geom = self.getGeomIndex(ign)
            if geom is None:
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
        if filter1 is None: #all pairs
            if bb_reject: bblist = [g[1].getBB() for g in self.geomList]
            for (i,(g,objs)) in enumerate(zip(self.geomList,self.mask)):
                for objIndex in objs:
                    #already checked
                    if objIndex < i: continue
                    if bb_reject and not bb_intersect(bblist[i],bblist[objIndex]): continue
                    yield (g,self.geomList[objIndex])
        elif filter2 is None: #self collision with objects passing filter1
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
            robot = robot.index
        if robot is None:
            #test all robots
            for r in range(len(self.robots)):
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
            robot = robot.index
        if isinstance(object,RigidObjectModel):
            object = object.index
        if object is None:
            #test all objects
            for o in range(len(self.rigidObjects)):
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
            robot = robot.index
        if isinstance(terrain,TerrainModel):
            terrain = terrain.index
        if terrain is None:
            #test all terrains
            for t in range(len(self.terrains)):
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
            object = object.index
        if isinstance(terrain,TerrainModel):
            terrain = terrain.index
        if terrain is None:
            #test all terrains
            for t in range(len(self.terrains)):
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
            object = object.index
        if isinstance(object2,RigidObjectModel):
            object2 = object2.index
        if object2 is None:
            #test all terrains
            for o in range(len(self.rigidObjects)):
                for c in self.objectObjectCollisions(object):
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
                robot = [r for r in range(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        rindices = self.robots[robot]
        return self.rayCast(s,d,rindices)
