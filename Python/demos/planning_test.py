from klampt.plan import robotcspace
from klampt.plan import cspace
from klampt.plan import robotplanning
from klampt.math import se3
from klampt import vis 
from klampt.io import resource
from klampt.model import ik
from klampt.model import trajectory
from klampt.model.collide import WorldCollider
from klampt import *
import time
import sys

#settings: feel free to edit these to see how the results change
DO_SIMPLIFY = 1
DEBUG_SIMPLIFY = 0
MANUAL_SPACE_CREATION = 0
CLOSED_LOOP_TEST = 1
PLAN_TO_GOAL_TEST = 1
MANUAL_PLAN_CREATION = 1

#load the robot / world file
fn = "../../data/robots/jaco.rob"
if len(sys.argv) > 1:
    fn = sys.argv[1]
world = WorldModel()
res = world.readFile(fn)
if not res:
    print "Unable to read file",fn
    exit(0)

robot = world.robot(0)
resource.setDirectory("resources/"+robot.getName())

def simplify(robot):
    """Utility function: replaces a robot's geometry with simplified bounding
    boxes."""
    for i in range(robot.numLinks()):
        geom = robot.link(i).geometry()
        if geom.empty(): continue
        geom.setCurrentTransform(*se3.identity())
        BB = geom.getBB()
        print BB[0],BB[1]
        BBgeom = GeometricPrimitive()
        BBgeom.setAABB(BB[0],BB[1])
        geom.setGeometricPrimitive(BBgeom)

#this line replaces the robot's normal geometry with bounding boxes.
#it makes planning faster but sacrifices accuracy.  Uncomment the line
#vis.dialog() below to examine whether the simplified robot looks
#ok
if DO_SIMPLIFY:
    simplify(robot)

#add the world elements individually to the visualization
vis.add("robot",robot)
for i in range(1,world.numRobots()):
    vis.add("robot"+str(i),world.robot(i))
for i in range(world.numRigidObjects()):
    vis.add("rigidObject"+str(i),world.rigidObject(i))
for i in range(world.numTerrains()):
    vis.add("terrain"+str(i),world.terrain(i))
#if you want to just see the robot in a pop up window...
if DO_SIMPLIFY and DEBUG_SIMPLIFY:
    vis.dialog()

#Automatic construction of space
if not CLOSED_LOOP_TEST:
    if not MANUAL_SPACE_CREATION:
        space = robotplanning.makeSpace(world=world,robot=robot,
                                        edgeCheckResolution=1e-3,
                                        movingSubset='all')
    else:
        #Manual construction of space
        collider = WorldCollider(world)
        space = robotcspace.RobotCSpace(robot,collider)
        space.eps = 1e-3
        space.setup()
else:
    #TESTING: closed loop robot cspace
    collider = WorldCollider(world)
    obj = ik.objective(robot.link(robot.numLinks()-1),local=[0,0,0],world=[0.5,0,0.5])
    vis.add("IK goal",obj)
    vis.dialog()
    space = robotcspace.ClosedLoopRobotCSpace(robot,obj,collider)
    space.eps = 1e-3
    space.setup()

#Generate some waypoint configurations using the resource editor
configs = resource.get("planningtest.configs","Configs",default=[],world=world,doedit=False)
cindex = 0
while True:
    if cindex < len(configs):
        robot.setConfig(configs[cindex])
    (save,q) = resource.edit("plan config "+str(cindex+1),robot.getConfig(),"Config",world=world,description="Press OK to add waypoint, cancel to stop")
    if save:
        if False and not space.feasible(q):
            print "Configuration is infeasible. Failures:"
            print " ",space.cspace.feasibilityFailures(q)
            print "Please pick another"
        else:
            if cindex >= len(configs):
                configs.append(q)
            else:
                configs[cindex] = q
            cindex += 1
    else:
        break
if cindex==0:
    vis.kill()
    exit(0)

configs = configs[:cindex]
resource.set("planningtest.configs",configs)

if CLOSED_LOOP_TEST:
    #need to project those onto the manifold
    for i,q in enumerate(configs):
        configs[i] = space.solveConstraints(q)
    resource.edit("IK solved configs",configs,"Configs",description="These configurations try to solve the IK constraint",world=world)

#set up a settings dictionary here.  This is a random-restart + shortcutting
#SBL planner.
settings = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }

#This code generates a PRM with no specific endpoints
#plan = cspace.MotionPlan(space, "prm", knn=10)
#print "Planning..."
#plan.planMore(500)
#V,E = plan.getRoadmap()
#print len(V),"feasible milestones sampled,",len(E),"edges connected"


#Generate a path connecting the edited configurations
#You might edit the value 500 to play with how many iterations to give the
#planner.
wholepath = [configs[0]]
for i in range(len(configs)-1):
    t0 = time.time()
    print "Creating plan..."
    if MANUAL_PLAN_CREATION:
        #Manual construction of planner
        plan = cspace.MotionPlan(space, **settings)
        plan.setEndpoints(configs[i],configs[i+1])
    elif PLAN_TO_GOAL_TEST:
        #this code uses the robotplanning module's convenience functions to reach the cartesian goal of
        #the specified configuration
        robot.setConfig(configs[i+1])
        obj = ik.fixed_objective(robot.link(robot.numLinks()-1),local=[0,0,0])
        #start from end of previous path
        robot.setConfig(wholepath[-1])
        plan = robotplanning.planToCartesianObjective(world,robot,obj,movingSubset='all',**settings)
    else:
        #this code uses the robotplanning module's convenience functions
        robot.setConfig(configs[i])
        plan = robotplanning.planToConfig(world,robot,configs[i+1],
                                          movingSubset='all',
                                          **settings)
    if plan is None:
        break
    print "Planner creation time",time.time()-t0
    t0 = time.time()
    plan.space.cspace.enableAdaptiveQueries(True)
    print "Planning..."
    for round in range(10):
        plan.planMore(50)
    print "Planning time, 500 iterations",time.time()-t0
    #this code just gives some debugging information. it may get expensive
    V,E = plan.getRoadmap()
    print len(V),"feasible milestones sampled,",len(E),"edges connected"
    path = plan.getPath()
    if path is None or len(path)==0:
        print "Failed to plan path between configuration",i,"and",i+1
        #debug some sampled configurations
        print V[0:min(10,len(V))]
        break
    if CLOSED_LOOP_TEST:
        #the path is currently a set of milestones: discretize it so that it stays near the contact surface
        #TODO: play with second parameter which governs how closely to space the waypoints
        path = space.discretizePath(path)
    wholepath += path[1:]

    print "Constraint testing order:"
    print plan.space.cspace.feasibilityQueryOrder()
    print "Manually optimizing constraint testing order..."
    plan.space.cspace.optimizeQueryOrder()
    print "Optimized constraint testing order:"
    print plan.space.cspace.feasibilityQueryOrder()

    print "Plan stats:"
    print plan.getStats()

    print "CSpace stats:"
    print plan.space.getStats()

    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()

if len(wholepath)>1:
    #print "Path:"
    #for q in wholepath:
    #    print "  ",q
    #if you want to save the path to disk, uncomment the following line
    #wholepath.save("test.path")

    #draw the path as a RobotTrajectory (you could just animate wholepath, but for robots with non-standard joints
    #the results will often look odd).  Animate with 5-second duration
    times = [i*5.0/(len(wholepath)-1) for i in range(len(wholepath))]
    traj = trajectory.RobotTrajectory(robot,times=times,milestones=wholepath)
    #show the path in the visualizer, repeating for 60 seconds
    vis.animate("robot",traj)
    vis.spin(60)
else:
    print "Failed to generate a plan"

vis.kill()
