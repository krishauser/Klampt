from ..robotsim import *
from ..math import vectorops,so3,se3
import random
from .. import vis
import time
import math
from ..model.trajectory import Trajectory
from ..model.contact import ContactPoint


def settle(world,obj,
    forcedir=(0,0,-1),forcept=(0,0,0),
    settletol=1e-4,orientationDamping=0.0,
    perturb=0,margin=None,
    debug=False):
    """Assuming that all other elements in the world besides object are frozen,
    this "settles" the object by applying a force in the direction forcedir
    and simulating until the object stops moving.

    An exception is raised if the object is already colliding with the world.

    Args:
        world (WorldModel): the world containing other static and moving
            objects
        obj: a RigidObjectModel, RobotModelLink, or floating-base Robot that 
            will be settled.
        forcedir (list of 3 floats, optional): a vector parallel to the
            direction of force whose magnitude is the maximum distance this 
            procedure will try to move the object.
        forcept (list of 3 floats, optional): local coordinates of the center
            of force application.
        settletol (float, optional): the simulation will stop when two
            subsequent transforms lie within this tolerance.
        orientationDamping (float, optional): a virtual spring will attempt 
            to keep the initial orientation with this torsional spring constant 
        perturb (float, optional): if nonzero, the application force will be 
            perturbed at random by this amount every step.  If equal to 1, this
            means the force is sampled from a 45 degree cone in the direction
            forcedir.
        margin (float, optional): the collision detection margin used in
            simulation.  If None, uses the Simulator default.  Otherwise,
            overrides the default.  Must be at least settletol.
        debug (bool, optional): if True, uses the visualization to debug the
            settling process

    Returns:
        tuple: A pair (transform,touched) with:

            - transform (se3 transform): The resulting se3 transform of the
              object, or None if the object didn't hit anything by the time it
              translated by ||forcedir|| units.
            - touched (dict): a dictionary whose keys are object IDs touched by
              the object at the final transform, and whose values are lists of
              ContactPoints (see :mod:`klampt.model.contact`) giving the
              contacts between obj and the touched object. 

              To convert the result to a hold, call::

                  h = Hold()
                  h.setFixed(obj,sum(touched.values(),[]))
    """
    assert isinstance(world,WorldModel)
    if isinstance(obj,(str,int)):
        obj = world.rigidObject(obj)
        assert obj.index >= 0,"Object "+str(obj)+" does not exist in world"
    elif isinstance(obj,RobotModel):
        raise NotImplementedError("TODO: settle free-floating robots")
    elif isinstance(obj,RobotModelLink):
        if world.index != obj.world:
            raise ValueError("Object is not present in the given world")
        assert obj.robotIndex >= 0 and obj.robotIndex < world.numRobots()
        robot = world.robot(obj.robotIndex)
        assert obj.index >= 0 and obj.index < robot.numLinks()
        newWorld = WorldModel()
        for i in range(world.numRobots()):
            if i == obj.robotIndex:
                continue
            newWorld.add(world.robot(i).getName(),world.robot(i))
        for i in range(world.numRigidObjects()):
            newWorld.add(world.rigidObject(i).getName(),world.rigidObject(i))
        for i in range(world.numTerrains()):
            newWorld.add(world.terrain(i).getName(),world.terrain(i))
        newObj = newWorld.makeRigidObject("obj")
        newObj.geometry().set(obj.geometry())
        newObj.setMass(obj.getMass())
        #newObj.setContactParameters(obj.getContactParameters())
        newObj.setTransform(*obj.getTransform())
        return settle(newWorld,newObj,
            forcedir,forcept,
            settletol,orientationDamping,
            perturb,margin,debug)
    elif isinstance(obj,RigidObjectModel):
        pass
    else:
        raise ValueError("Invalid object type given, only supports RigidObjectModels, RobotModelLinks, and RobotModels")

    #get a bounding box around the object
    forcept_world = se3.apply(obj.getTransform(),forcept)
    bmin,bmax = obj.geometry().getBB()
    #compute radius about forcept, expand BB due to potential for orientation change
    R = 0
    for i in range(3):
        R += pow(max(forcept_world[i]-bmin[i],bmax[i]-forcept_world[i]),2)
    R = math.sqrt(R)
    bmin = [x - R for x in forcept_world]
    bmax = [x + R for x in forcept_world]
    #expand BB about force direction
    for i in range(3):
        if forcedir[i] < 0:
            bmin[i] += forcedir[i]
        else:
            bmax[i] += forcedir[i]
    
    if world.index != obj.world:
        raise ValueError("Object is not present in the given world")
    assert obj.index >= 0 and obj.index < world.numRigidObjects()
    #exclude objects that have no chance of being in way of object
    newWorld = WorldModel()
    newObj = None
    for i in range(world.numRobots()):
        robot = world.robot(i)
        for j in range(robot.numLinks()):
            link = robot.link(j)
            if _bboverlap((bmin,bmax),link):
                #add robot link as static geometry
                newObj = newWorld.makeRigidObject(robot.getName()+":"+link.getName())
                newObj.geometry().set(link.geometry())
                mass = Mass()
                mass.setMass(float('inf'))
                mass.setCom([0]*3)
                mass.setInertia([float('inf')]*3)
                newObj.setMass(mass)
                newObj.setTransform(*link.getTransform())
                #TODO: what surface properties?
    for i in range(world.numRigidObjects()):
        if _bboverlap((bmin,bmax),world.rigidObject(i)):
            o = newWorld.add(world.rigidObject(i).getName(),world.rigidObject(i))
            if i == obj.index:
                newObj = o
        else:
            assert i != obj.index
    for i in range(world.numTerrains()):
        if _bboverlap((bmin,bmax),world.terrain(i)):
            newWorld.add(world.terrain(i).getName(),world.terrain(i))
    world = newWorld
    obj = newObj

    movedist = vectorops.norm(forcedir)
    if movedist < settletol:
        print("sim.settle(): warning, force movement distance less than settletol.  Was this intended?")
        return (obj.getTransform(),[])
    forcedir = vectorops.div(forcedir,movedist)
    forceamt = obj.getMass().mass 

    sim = Simulator(world)
    body = sim.body(obj)
    otherbodies = []
    otherids = []
    for i in range(world.numRigidObjects()):
        otherids.append(world.rigidObject(i).getID())
        otherbodies.append(sim.body(world.rigidObject(i)))
    for i in range(world.numTerrains()):
        otherids.append(world.terrain(i).getID())
        otherbodies.append(sim.body(world.terrain(i)))
    otherids.remove(obj.getID())
    otherbodies = [b for b in otherbodies if b.body != body.body]
    if len(otherbodies) == 0:
        print("sim.settle(): no objects in direction",vectorops.mul(forcedir,movedist))
        return (None,[])
    if margin != None:
        assert margin >= settletol,"Collision margin must be at least settletol"
        for b in otherbodies:
            b.setCollisionPadding(0)
        body.setCollisionPadding(margin)
    else:
        margin = body.getCollisionPadding()
        margin += min([b.getCollisionPadding() for b in otherbodies])

    #set up simulation
    dt = max(settletol,margin*0.5)
    forceamt /= dt*0.5
    sim.setGravity((0,0,0))
    sim.setSimStep(dt)
    for id in otherids:
        sim.enableContactFeedback(obj.getID(),id)
    #turn off all restitution
    s = body.getSurface()
    s.kRestitution = 0
    body.setSurface(s)
    for b in otherbodies:
        s = b.getSurface()
        s.kRestitution = 0
        b.setSurface(s)
    #disable other bodies
    for b in otherbodies:
        b.setVelocity([0.0]*3,[0.0]*3)
        b.enableDynamics(False)
    body.setVelocity([0.0]*3,vectorops.mul(forcedir,movedist))

    sim.simulate(0)
    s = sim.getStatus()
    if s == Simulator.STATUS_CONTACT_UNRELIABLE:
        print("sim.settle(): warning, object already penetrating other objects.  Trying to pull back...")
        T0 = body.getTransform()
        body.setTransform(T0[0],vectorops.madd(T0[1],forcedir,margin))
        sim.simulate(0)
        s = sim.getStatus()
        if s == Simulator.STATUS_CONTACT_UNRELIABLE:
            print("  pulling back failed.")
            return (None,[])
    if debug:
        vis.createWindow("settle")
        vis.add("world",world)
        vis.show()
        time.sleep(1.0)
    springanchorworld = se3.apply(obj.getTransform(),forcept)
    Rspringanchor = obj.getTransform()[0]
    numSettled = 0
    Told = body.getTransform()
    t = 0
    while t < 1:
        #print("Simulating, t =",t)
        if perturb:
            fpert = (random.gauss(0,perturb),random.gauss(0,perturb),random.gauss(0,perturb))
            fpert = vectorops.sub(fpert,vectorops.mul(forcedir,vectorops.dot(forcedir,fpert)))
            f = vectorops.add(forcedir,fpert)
        else:
            f = forcedir
        springanchorbody = se3.apply(body.getObjectTransform(),forcept)
        Rspringbody = body.getObjectTransform()[0]
        #vis.add("Tobject",body.getObjectTransform())
        kSpring = forceamt
        #kSpring = 0
        f = vectorops.madd(vectorops.mul(f,forceamt),vectorops.sub(springanchorworld,springanchorbody),kSpring)
        body.applyForceAtLocalPoint(f,forcept)
        if orientationDamping > 0:
            #local orientation change: transform from body to anchor frame
            wlocal = so3.moment(so3.mul(Rspringanchor,so3.inv(Rspringbody)))
            #world orientation spring
            w = so3.apply(Rspringbody,wlocal)
            #w = so3.apply(Rspringbody,so3.error(Rspringanchor,Rspringbody))
            #vis.add("orientation",Trajectory([0,1],[Told[1],vectorops.madd(Told[1],w,1)]))
            #vis.add("orientationloc",Trajectory([0,1],[Told[1],vectorops.madd(Told[1],wlocal,1)]))
            #vis.setColor("orientationloc",0,1,0)
            #body.applyWrench([0]*3,vectorops.mul(w,orientationDamping))
            body.applyWrench([0]*3,vectorops.mul(wlocal,orientationDamping))
        if debug:
            vis.lock()
            sim.simulate(dt)
            sim.updateWorld()
            vis.unlock()
            time.sleep(0)
            time.sleep(0.1)
        else:
            sim.simulate(dt)
            sim.updateWorld()

        #test for settling
        w,v = body.getVelocity()
        T = body.getTransform()
        err = se3.error(T,Told)
        #if debug:
        #   print("Status:",sim.getStatus(),"velocity",w,v,"error",vectorops.norm(err))
        if vectorops.norm(err) < settletol:
            numSettled += 1
        else:
            numSettled = 0
        if numSettled >= 2:
            print("sim.settle(): Settled at time",t)
            touched = [id for id in otherids if sim.inContact(obj.getID(),id)]
            cps = [sim.getContacts(obj.getID(),id) for id in touched]
            tdict = dict()
            for id,cplist in zip(touched,cps):
                tdict[id] = [ContactPoint(ci[0:3],ci[3:6],ci[6]) for ci in cplist]
            if debug:
                vis.show(False)
            return (body.getObjectTransform(),tdict)
        #apply drag
        body.setVelocity(vectorops.mul(w,0.8),vectorops.mul(v,0.8))
        Told = T
        springanchorworld = vectorops.madd(springanchorworld,forcedir,dt*movedist)
        t += dt
    if debug:
        vis.show(False)
    print("Failed to settle? Final velocity",body.getVelocity())
    touched = [id for id in otherids if sim.inContact(obj.getID(),id)]
    cps = [sim.getContacts(obj.getID(),id) for id in touched]
    tdict = dict()
    for id,cplist in zip(touched,cps):
        tdict[id] = [ContactPoint(ci[0:3],ci[3:6],ci[6]) for ci in cplist]
    return (body.getObjectTransform(),tdict)

def _bboverlap(bb,element):
    if isinstance(element,RobotModel):
        return any(_bboverlap(bb,element.link(i)) for i in range(element.numLinks()))
    else:
        bb2 = element.geometry().getBB()
        print("BBox",bb)
        print("  Testing overlap with",element.getName(),"bbox",bb2)
        for (a,b,c,d) in zip(bb[0],bb[1],bb2[0],bb2[1]):
            if not (a >= c and a <= d or b >= c and b <= d) and not (c >= a and c <= b or d >= a and d <= b):
                print("  No overlap")
                return False
        print("  Overlap")
        return True