"""Utility functions for making arrangments / piles of objects.

Key functions are make_object_arrangement and make_object_pile.
"""

from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.model import collide
import random
import math

def get_bound(objects):
    """Obtains the tight outer bounds of the object(s) at their current transforms, in world coordinates."""
    if hasattr(objects,'__iter__'):
        bbs = [get_bound(o) for o in objects]
        if len(bbs) == 1:
            return bbs[0]
        print(bbs)
        return collide.bb_union(*bbs)
    else:
        return objects.geometry().getBBTight()

def xy_randomize(obj,bmin,bmax):
    """Randomizes the xy position and z orientation of an object inside of a bounding box bmin->bmax.
    Assumes the bounding box is large enough to hold the object in any orientation.
    """
    R,t = obj.getTransform()
    R = so3.mul(so3.rotation([0,0,1],random.uniform(0,math.pi*2)),R)
    obj.setTransform(R,t)
    obmin,obmax = obj.geometry().getBB()
    t[0] = random.uniform(bmin[0]-obmin[0],bmax[0]-obmax[0])
    t[1] = random.uniform(bmin[1]-obmin[1],bmax[1]-obmax[1])
    obj.setTransform(R,t)

def xy_jiggle(world,objects,fixed_objects,bmin,bmax,iters,randomize = True):
    """Jiggles the objects' x-y positions within the range bmin - bmax, and randomizes orientation about the z
    axis until the objects are collision free.  A list of fixed objects (fixed_objects) may be given as well.

    Objects for which collision-free resolutions are not found are returned.
    """
    if randomize:
        for obj in objects:
            xy_randomize(obj,bmin,bmax)
    inner_iters = 10
    while iters > 0:
        numConflicts = [0]*len(objects)
        for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
            numConflicts[i] += 1
            numConflicts[j] += 1
        for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
            numConflicts[i] += 1
        
        amax = max((c,i) for (i,c) in enumerate(numConflicts))[1]
        cmax = numConflicts[amax]
        if cmax == 0:
            #conflict free
            return
        print(cmax,"conflicts with object",objects[amax].getName())
        other_geoms = [o.geometry() for o in objects[:amax]+objects[amax+1:]+fixed_objects]
        for it in range(inner_iters):
            xy_randomize(objects[amax],bmin,bmax)
            nc = sum([1 for p in collide.group_collision_iter([objects[amax].geometry()],other_geoms)])
            if nc < cmax:
                break
            iters-=1
        print("Now",nc,"conflicts with object",objects[amax].getName())

    numConflicts = [0]*len(objects)
    for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
        numConflicts[i] += 1
        numConflicts[j] += 1
    for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
        numConflicts[i] += 1
    removed = []
    while max(numConflicts) > 0:
        amax = max((c,i) for (i,c) in enumerate(numConflicts))[1]
        cmax = numConflicts[amax]
        print("Unable to find conflict-free configuration for object",objects[amax].getName(),"with",cmax,"conflicts")
        removed.append(amax)

        #revise # of conflicts -- this could be faster, but whatever...
        numConflicts = [0]*len(objects)
        for (i,j) in collide.self_collision_iter([o.geometry() for o in objects]):
            if i in removed or j in removed:
                continue
            numConflicts[i] += 1
            numConflicts[j] += 1
        for (i,j) in collide.group_collision_iter([o.geometry() for o in objects],[o.geometry() for o in fixed_objects]):
            if i in removed:
                continue
            numConflicts[i] += 1
    return removed

def make_object_arrangement(world,container,objects,container_wall_thickness=0.01,max_iterations=100,remove_failures=False):
    """For a given container and a list of objects in the world, places the objects inside the container with randomized x-y locations
    and z orientations so that they are initially collision free and on the bottom of the container.

    Arguments:
    - world: a WorldModel
    - container: the container RigidObject / Terrain in world into which objects should be spawned.  Assumed axis-aligned.
    - objects: a list of RigidObjects in the world, at arbitrary locations.  They are placed in order.
    - container_wall_thickness: a margin subtracted from the container's outer dimensions into which the objects are spawned.
    - max_iterations: the maximum number of iterations used for sampling object initial poses
    - remove_failures: if True, then instead of returning None on failure, the objects that fail placement are removed from the world
    
    Return value:
    - the positions of objects in world are modified and world is also returned. On failure, returns None.

    Note: Since world is modified in-place, if you wish to make multiple worlds with piles of the same objects, you should use world.copy()
    to store the configuration of the objects. You may also wish to randomize the object ordering using random.shuffle(objects) between instances.
    """
    container_outer_bb = get_bound(container)
    container_inner_bb = (vectorops.add(container_outer_bb[0],[container_wall_thickness]*3),vectorops.sub(container_outer_bb[1],[container_wall_thickness]*3))
    collision_margin = 0.0025
    for object in objects:
        #make sure the bottom of the object touches the bottom of the container
        obb = get_bound(object)
        zmin = obb[0][2]
        R,t = object.getTransform()
        t[2] = container_inner_bb[2] - zmin + collision_margin
        object.setTransform(R,t)

    failures = xy_jiggle(world,rigid_objects,[container],container_inner_bb[0],container_inner_bb[1],max_iterations)
    if len(failures) > 0:
        if remove_failures:
            removeIDs = [objects[i].index for i in failures]
            for i in sorted(removeIDs)[::-1]:
                world.remove(world.rigidObject(i))
        else:
            return None
    return world

def make_object_pile(world,container,objects,container_wall_thickness=0.01,randomize_orientation=True,visualize=False):
    """For a given container and a list of objects in the world, drops the objects inside the container and simulates until stable.

    Arguments:
    - world: a WorldModel
    - container: the container RigidObject / Terrain in world over which objects should be spawned.  Assumed axis-aligned and with an open top.
    - objects: a list of RigidObjects in the world, at arbitrary locations.  They are placed in order.
    - container_wall_thickness: a margin subtracted from the container's outer dimensions into which the objects are spawned.
    - randomize_orientation: if True, the orientation of the objects are completely randomized.  If 'z', only the z orientation is randomized.
      If False or None, the orientation is unchanged
    
    Side effect: the positions of objects in world are modified
    Return value (world,sim):
    - world: the original world
    - sim: the Simulator instance at the state used to obtain the stable placement of the objects.

    Note: Since world is modified in-place, if you wish to make multiple worlds with piles of the same objects, you should use world.copy()
    to store the configuration of the objects. You may also wish to randomize the object ordering using random.shuffle(objects) between instances.
    """
    container_outer_bb = get_bound(container)
    container_inner_bb = (vectorops.add(container_outer_bb[0],[container_wall_thickness]*3),vectorops.sub(container_outer_bb[1],[container_wall_thickness]*3))
    spawn_area = (container_inner_bb[0][:],container_inner_bb[1][:])
    collision_margin = 0.0025
    if visualize:
        from klampt import vis
        from klampt.model import config
        import time
        oldwindow = vis.getWindow()
        newwindow = vis.createWindow("make_object_pile dynamic visualization")
        vis.setWindow(newwindow)
        vis.show()
        visworld = world.copy()
        vis.add("world",visworld)

    sim = Simulator(world)
    sim.setSetting("maxContacts","20")
    sim.setSetting("adaptiveTimeStepping","0")
    Tfar = (so3.identity(),[0,0,-100000])
    for object in objects:
        R,t = object.getTransform()
        object.setTransform(R,Tfar[1])
        sim.body(object).setTransform(*Tfar)
        sim.body(object).enable(False)
    print("Spawn area",spawn_area)
    if visualize:
        vis.lock()
        config.setConfig(visworld,config.getConfig(world))
        vis.unlock()
    for index in range(len(objects)):
        #always spawn above the current height of the pile 
        if index > 0:
            objects_bound = get_bound(objects[:index])
            print("Existing objects bound:",objects_bound)
            zshift = max(0.0,objects_bound[1][2] - spawn_area[0][2])
            spawn_area[0][2] += zshift
            spawn_area[1][2] += zshift
        object = objects[index]
        obb = get_bound(object)
        zmin = obb[0][2]
        R0,t0 = object.getTransform()
        feasible = False
        for sample in range(1000):
            R,t = R0[:],t0[:]
            if randomize_orientation == True:
                R = so3.sample()
            t[2] = spawn_area[1][2] - zmin + t0[2] + collision_margin
            object.setTransform(R,t)
            xy_randomize(object,spawn_area[0],spawn_area[1])
            print("Sampled position of",object.getName(),object.getTransform()[1])
            if not randomize_orientation:
                _,t = object.getTransform()
                object.setTransform(R,t)

            #object spawned, now settle
            sobject = sim.body(object)
            sobject.enable(True)
            sobject.setTransform(*object.getTransform())
            res = sim.checkObjectOverlap()
            if len(res[0]) == 0:
                feasible = True
                #get it low as possible without overlapping
                R,t = object.getTransform()
                for lower in range(100):
                    sobject.setTransform(R,vectorops.add(t,[0,0,-(lower+1)*0.01]))
                    res = sim.checkObjectOverlap()
                    if len(res[0]) != 0:
                        print("Terminated lowering at",lower,"cm lower")
                        sobject.setTransform(R,vectorops.add(t,[0,0,-lower*0.01]))
                        res = sim.checkObjectOverlap()
                        break
                sim.updateWorld()
                break
        if not feasible:
            print("Failed to place object",object.getName())
            return None
        if visualize:
            vis.lock()
            config.setConfig(visworld,config.getConfig(world))
            vis.unlock()
            time.sleep(0.1)
    
    print("Beginning to simulate")
    #start letting everything  fall
    for firstfall in range(10):
        sim.simulate(0.01)
        if visualize:
            vis.lock()
            config.setConfig(visworld,config.getConfig(world))
            vis.unlock()
            time.sleep(0.01)
    maxT = 5.0
    dt = 0.01
    t = 0.0
    wdamping = -0.01
    vdamping = -0.1
    while t < maxT:
        settled = True
        for object in objects:
            sobject = sim.body(object)
            w,v = sobject.getVelocity()
            sobject.applyWrench(vectorops.mul(v,vdamping),vectorops.mul(w,wdamping))
            if vectorops.norm(w) + vectorops.norm(v) > 1e-4:
                #settled
                settled=False
                break
        if settled:
            break
        if visualize:
            t0 = time.time()
        sim.simulate(dt)
        if visualize:
            vis.lock()
            config.setConfig(visworld,config.getConfig(world))
            vis.unlock()
            time.sleep(max(0.0,dt-(time.time()-t0)))
        t += dt
    if visualize:
        vis.show(False)
        vis.setWindow(oldwindow)
    return (world,sim)
