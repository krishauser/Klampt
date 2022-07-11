"""Functions and classes for managing collision tests between multiple objects.

This module defines the :class:`WorldCollider` class, which makes it easy to
ignore various collision pairs in a WorldModel.

For groups of objects, the :meth:`self_collision_iter` and
:meth:`group_collision_iter` functions perform broad-phase collision detection
to speed up collision testing.

The :meth:`ray_cast` function is a convenient way to return the first point of
intersection for a ray and a group of objects.
"""


from ..robotsim import *
from ..math import vectorops,se3
from typing import Union,Optional,List,Tuple,Sequence,Callable,Iterator
from .typing import Vector,Vector3,Matrix3,RigidTransform

BBType = Tuple[Vector3,Vector3]
CollidableType = Union[RobotModel,RobotModelLink,RigidObjectModel,TerrainModel]
WorldBodyType = Union[RobotModelLink,RigidObjectModel,TerrainModel]

def bb_create(*ptlist: Vector3) -> BBType:
    """Creates a bounding box from an optional set of points. If no points
    are provided, creates an empty bounding box."""
    if len(ptlist) == 0:
        return [float('inf')]*3,[float('-inf')]*3
    else:
        bmin,bmax = list(ptlist[0]),list(ptlist[0])
        for i in range(1,len(ptlist)):
            x = ptlist[i]
            bmin = [min(a,b) for (a,b) in zip(bmin,x)]
            bmax = [max(a,b) for (a,b) in zip(bmax,x)]
        return bmin,bmax

def bb_empty(bb: BBType) -> bool:
    """Returns True if the bounding box is empty"""
    return any((a > b) for (a,b) in zip(bb[0],bb[1]))

def bb_intersect(a: BBType, b: BBType) -> bool:
    """Returns true if the bounding boxes (a[0]->a[1]) and (b[0]->b[1]) intersect"""
    amin,amax=a
    bmin,bmax=b
    return not any(q < u or v < p for (p,q,u,v) in zip(amin,amax,bmin,bmax))

def bb_contains(bb: BBType, x: Vector3) -> bool:
    """Returns true if x is inside the bounding box bb"""
    return not any(v < p or v > q for (p,q,v) in zip(bb[0],bb[1],x))

def bb_intersection(*bbs: BBType):
    """Returns the bounding box representing the intersection the given bboxes.
    The result may be empty."""
    return [max(x) for x in zip(*[b[0] for b in bbs])],[min(x) for x in zip(*[b[1] for b in bbs])]

def bb_union(*bbs: BBType):
    """Returns the smallest bounding box containing the given bboxes"""
    return [min(x) for x in zip(*[b[0] for b in bbs])],[max(x) for x in zip(*[b[1] for b in bbs])]


def self_collision_iter(
        geomlist: Sequence[Geometry3D],
        pairs: Union[str,Callable[[int,int],bool],List[Tuple[int,int]]] = 'all'
    ) -> Iterator[Tuple[int,int]]:
    """Performs efficient self collision testing for a list of geometries.

    Args:
        geomlist (list of Geometry3D): the list of geometries
        pairs: can be:

            * 'all': all pairs are tested.  
            * a function test(i,j) -> bool taking geometry indices and
              returning true if they should be tested
            * list of pairs (i,j) of collision indices.

    Uses a quick bounding box reject test.

    Returns:
        Iterator over colliding pairs (i,j) where i and
        j are indices into geomlist.
    """
    #bblist = [g.getBB() for g in geomlist]
    if pairs=='all':
        for i,g in enumerate(geomlist):
            for j in range(i+1,len(geomlist)):
                #if not bb_intersect(bblist[i],bblist[j]): continue
                g2 = geomlist[j]
                if g.collides(g2):
                    yield (i,j)
    elif callable(pairs):
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

def group_collision_iter(
        geomlist1: Sequence[Geometry3D],
        geomlist2: Sequence[Geometry3D],
        pairs: Union[str,Callable[[int,int],bool],List[Tuple[int,int]]] = 'all'
    ) -> Iterator[Tuple[int,int]]:
    """Tests whether two sets of geometries collide.

    Args:
        geomlist1 (list of Geometry3D): set 1
        geomlist2 (list of Geometry3D): set 2
        pairs: can be:

            * 'all': all pairs are tested.  
            * a function test(i,j) -> bool, taking geomlist1 index i and
              geomlist2 index j, and returning true if they should be tested
            * list of pairs (i,j) of collision indices.

    Uses a quick bounding box reject test.

    Returns:
        Iterator over colliding pairs (i,j) where i is
        an index into geomlist1 and j is an index into geomlist.
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
    elif callable(pairs):
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


def group_subset_collision_iter(
        geomlist: Sequence[Geometry3D],
        alist: Sequence[int],
        blist: Sequence[int],
        pairs='all'
    ) -> Iterator[Tuple[int,int]]:
    """Tests whether two subsets of geometries collide.  Can be slightly faster
    than `group_collision_iter` if `alist` and `blist` overlap.

    Args:
        geomlist (list of Geometry3D): a list of all possible geometries
        alist (list of int): collision set 1, containing indices into geomlist
        blist (list of int): collision set 2, containing indices into geomlist
        pairs: can be:

            * 'all': all pairs are tested.  
            * a function test(i,j) -> bool, taking geomlist1 index i and
              geomlist2 index j, and returning true if they should be tested
            * list of pairs (i,j) of collision indices.  In this case, `alist`
              and `blist` are ignored and can be set to None.

    Uses a quick bounding box reject test.

    Returns:
        Iterator over colliding pairs (i,j) where i is
        an index into alist and j is an index into glist.
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
    elif callable(pairs):
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


def ray_cast(
        geomlist: Sequence[Geometry3D],
        s: Vector3,
        d: Vector3
    ) -> Tuple[int,Vector3]:
    """Finds the first collision among the geometries in geomlist with the
    ray at source s and direction d. 

    Returns:
        A pair (index,point) if a collision is found, where:

        * index is the index of the geometry in geomlist
        * point is the collision point in world coordinates.
    
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
    Used in planning routines to mask out objects in the world to check /
    ignore when doing collision detection.

    You should not need to interact directly with this object's attributes.
    Instead, use the methods provided.

    Attributes:
        geomList (list): a list of (object,geom) pairs for all non-empty objects
            in the world.
        mask (list of sets of ints): indicating which items are activated for 
            collision detection.  Basically, a sparse, symmetric boolean matrix 
            over len(geomList)*len(geomList) possible collision pairs.
        terrains (list of ints): contains the geomList indices of each terrain
            in the world.
        rigidObjects (list of ints): contains the geomList indices of each
            object in the world
        robots (list of list of ints): contains the geomList indices of each
            robot in the world.

    """
    
    def __init__(self, world: WorldModel, ignore=[]):
        """Args:
            world (WorldModel): the world to use
            ignore (list, optional): a list of items to pass to ignoreCollision
        """

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
                        #print("Ignoring fixed link...")
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
                        if l1 < 0 or l2 < 0: continue
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
                
    def _getGeomIndex(self,object) -> int:
        """Finds the geomList index corresponding to an object

        Returns:
            The index into self.geomList describing the object
        """
        assert isinstance(object,(RobotModel,RobotModelLink,RigidObjectModel,TerrainModel))
        for i,(o,g) in enumerate(self.geomList):
            if o.world==object.world and o.getID()==object.getID():
                assert o.getName()==object.getName()
                assert type(o) == type(object)
                return i
        return None


    def ignoreCollision(self, ign: Union[WorldBodyType,Tuple[WorldBodyType,WorldBodyType]] ) -> None:
        """Permanently removes an object or a pair of objects from
        consideration.

        Args:
            ign: either a single body (RobotModelLink, RigidObjectModel,
                TerrainModel) in the world, or a pair of bodies.  In the former
                case all collisions with that body will be ignored.
        """
        if hasattr(ign,'__iter__'):
            (a,b) = ign
            ageom = self._getGeomIndex(a)
            bgeom = self._getGeomIndex(b)
            if ageom is None or bgeom is None:
                raise ValueError("Invalid ignore collision item, must be a pair of bodies in the world")
            self.mask[ageom].discard(bgeom)
            self.mask[bgeom].discard(ageom)
        else:
            #ignore all collisions with the given geometry
            geom = self._getGeomIndex(ign)
            if geom is None:
                raise ValueError("Invalid ignore collision item, must be a body in the world")
            for i in self.mask[geom]:
                #remove it from the list
                self.mask[i].discard(geom)
            self.mask[geom]=set()

    def isCollisionEnabled(self, obj_or_pair: Union[WorldBodyType,Tuple[CollidableType,WorldBodyType]] ) -> bool:
        """Returns true if the object or pair of objects are considered for
        collision.

        Args:
            obj_or_pair: either a single body (RobotModelLink, 
                RigidObjectModel, TerrainModel) in the world, or a pair of
                bodies.  In the former case, True is returned if the body
                is checked with anything.
        """
        if hasattr(obj_or_pair,'__iter__'):
            (a,b) = obj_or_pair
            ageom = self._getGeomIndex(a)
            bgeom = self._getGeomIndex(b)
            if ageom is None or bgeom is None:
                return False
            return ageom in self.mask[bgeom] or bgeom in self.mask[ageom]
        else:
            geom = self._getGeomIndex(obj_or_pair)
            if geom is None:
                return False
            return len(self.mask[geom]) > 0

    def collisionTests(self,
            filter1: Optional[Callable[[WorldBodyType],bool]] = None,
            filter2: Optional[Callable[[WorldBodyType],bool]] = None,
            bb_reject: bool = True
        ) -> Iterator[Tuple[Tuple[WorldBodyType,Geometry3D],Tuple[WorldBodyType,Geometry3D]]]:
        """Returns an iterator over potential colliding pairs, which
        should be tested for collisions. 

        Usage:
            To test collisions, you call::

                for i,j in worldCollider.collisionTests():
                    if i[1].collides(j[1]):
                        print("Object",i[0].getName(),"collides with",j[0].getName())
                    
        (Note that for this purpose is easier to just call :meth:`collisions`;
        however you may want to use `collisionTests` to perform other queries
        like proximity detection.)

        Args:
            filter1 (function, optional): has form f(object) -> bool
            filter2 (function, optional): has form f(object) -> bool
            bb_reject (bool, optional): True if we should quick reject objects
                whose bounding boxes are not overlapping (broad phase collision
                detection).  If false, all non-ignored collision pairs are
                returned.

        See :meth:`collisions` for an explanation of how filter1 and filter2
        are interpreted

        Returns:
            Iterates over ``((object1,geom1),(object2,geom2))`` pairs indicating 
            which objects should be tested for collision. They have type:

            - object1, object2: a RobotModelLink, RigidObjectModel, or
                TerrainModel
            - geom1, geom2: Geometry3D corresponding to those objects.

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

    def collisions(self,
            filter1: Optional[Callable[[WorldBodyType],bool]] = None,
            filter2: Optional[Callable[[WorldBodyType],bool]] = None,
        ) -> Iterator[Tuple[Tuple[WorldBodyType,Geometry3D],Tuple[WorldBodyType,Geometry3D]]]:
        """Returns an iterator over the colliding pairs of objects,
        optionally that satisfies the filter(s).

        Args:
            filter1 (function, optional): has form f(object) -> bool
            filter2 (function, optional): has form f(object) -> bool

        filter1 and filter2 are predicates to allow subsets of objects
        to collide.  The argument can be a RobotModelLink, RigidObjectModel
        or TerrainModel.

        If neither filter1 nor filter2 are provided, then all pairs are
        checked. 

        If filter1 is provided but filter2 is not, then objects in the set
        filter1 will be collided against each other.

        If filter1 and filter2 are provided, then objects that
        satisfy filter1 will be collided against objects that satisfy
        filter2.  (Note: in this case there is no checking of duplicates,
        i.e., the sets should be disjoint to avoid duplicating work).
        """
        for (g0,g1) in self.collisionTests(filter1,filter2):
            if g0[1].collides(g1[1]):
                yield (g0[0],g1[0])

    def robotSelfCollisions(self,
            robot: Union[RobotModel,int,None] = None
        ) -> Iterator[Tuple[RobotModelLink,RobotModelLink]]:
        """Yields an iterator over robot self collisions.

        Args:
            robot (RobotModel or int, optional): If None (default), all
            robots are tested.  If an index or a RobotModel object only
            collisions for that robot are tested

        Returns:
            Iterates over colliding
            ``(RobotModelLink,RobotModelLink)`` pairs.
        """
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
       
    def robotObjectCollisions(self,
            robot: Union[RobotModel,int],
            object: Union[RigidObjectModel,int,None] = None
        ) -> Iterator[Tuple[RobotModelLink,RigidObjectModel]]:
        """Yields an iterator over robot-object collision pairs.

        Args:
            robot (RobotModel or int): the robot to test
            object (RigidObjectModel or int, optional): the object to
                test, or None to all objects.

        Returns:
            Iterates over colliding
            (RobotModelLink,RigidObjectModel) pairs.
        """
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

    def robotTerrainCollisions(self,
            robot: Union[RobotModel,int],
            terrain: Union[TerrainModel,int,None] = None
        ) -> Iterator[Tuple[RobotModelLink,TerrainModel]]:
        """Yields an iterator over robot-terrain collision pairs.

        Args:
            robot (RobotModel or int): the robot to test
            terrain (TerrainModel or int, optional): the terrain to
                test, or None to all terrains.

        Returns:
            Iterates over colliding
            (RobotModelLink,TerrainModel) pairs.
        """
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

    def objectTerrainCollisions(self,
            object: Union[RigidObjectModel,int],
            terrain: Union[TerrainModel,int,None] = None
        ) -> Iterator[Tuple[RigidObjectModel,TerrainModel]]:
        """Yields an iterator over object-terrain collision pairs.

        Args:
            object (RigidObjectModel or int): the object to test
            terrain (TerrainModel or int, optional): the terrain to
                test, or None to all terrains.

        Returns:
            Iterates over colliding
            (RigidObjectModel,TerrainModel) pairs.
        """
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

    def objectObjectCollisions(self,
            object: Union[RigidObjectModel,int],
            object2: Union[RigidObjectModel,int,None]
        ) -> Iterator[Tuple[RigidObjectModel,RigidObjectModel]]:
        """Yields an iterator over object-terrain collision pairs.

        Args:
            object (RigidObjectModel or int): the object to test
            object2 (RigidObjectModel or int, optional): the terrain to
                test, or None to all objects.

        Returns:
            Iterates over colliding
            (RigidObjectModel,TerrainModel) pairs.
        """
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

    def rayCast(self,
            s: Vector3,
            d: Vector3,
            indices: Optional[List[int]]=None
        ) -> Union[None,Tuple[WorldBodyType,Vector3]]:
        """Finds the first collision between a ray and objects in the world.

        Args:
            s (list of 3 floats): the ray source
            d (list of 3 floats): the ray direction
            indices (list of ints, optional): if given, the indices of
                geometries in geomList to test.

        Returns:
            The (object,point) pair or None if no collision is found.
        """
        res = None
        dmin = 1e300
        geoms = (self.geomList if indices==None else [self.geomList[i] for i in indices])
        for g in geoms:
            (coll,pt) = g[1].rayCast(s,d)
            if coll:
                dist = vectorops.dot(d,vectorops.sub(pt,s))
                if dist < dmin:
                    dmin,res = dist,(g[0],pt)
        return res
                
    def rayCastRobot(self,
            robot: Union[RobotModel,int],
            s: Vector3,
            d: Vector3
        ) -> Union[None,Tuple[RobotModelLink,Vector3]]:
        """Finds the first collision between a ray and a robot.

        Args:
            robot (RobotModel or int): the robot
            s (list of 3 floats): the ray source
            d (list of 3 floats): the ray direction

        Returns:
            The (object,point) pair or None if no collision is found.
        """
        if isinstance(robot,RobotModel):
            try:
                robot = [r for r in range(self.world.numRobots()) if self.world.robot(r).getID()==robot.getID()][0]
            except IndexError:
                raise RuntimeError("Robot "+robot.getName()+" is not found in the world!")
        rindices = self.robots[robot]
        return self.rayCast(s,d,rindices)
