from klampt import robotcspace
from klampt import cspace
from klampt.robotcollide import WorldCollider
from klampt import *
from klampt import visualization
from klampt import resource
from klampt import robotplanning
import time
import sys

#load the robot / world file
fn = "../../data/robots/jaco.rob"
if len(sys.argv) > 1:
    fn = sys.argv[1]
world = WorldModel()
res = world.readFile(fn)
if not res:
    print "Unable to read file",fn
    exit(0)

def simplify(robot):
    """Utility function: replaces a robot's geometry with simplified bounding
    boxes."""
    for i in range(robot.numLinks()):
        geom = robot.link(i).geometry()
        if geom.empty(): continue
        BB = geom.getBB()
        print BB[0],BB[1]
        BBgeom = GeometricPrimitive()
        BBgeom.setAABB(BB[0],BB[1])
        geom.setGeometricPrimitive(BBgeom)

robot = world.robot(0)
#this line replaces the robot's normal geometry with bounding boxes.
#it makes planning faster but sacrifices accuracy.  Uncomment the line
#visualization.dialog() below to examine whether the simplified robot looks
#ok
simplify(robot)

#add the world elements individually to the visualization
visualization.add("robot",robot)
for i in range(1,world.numRobots()):
    visualization.add("robot"+str(i),world.robot(i))
for i in range(world.numRigidObjects()):
    visualization.add("rigidObject"+str(i),world.rigidObject(i))
for i in range(world.numTerrains()):
    visualization.add("terrain"+str(i),world.terrain(i))
#if you want to just see the robot in a pop up (blocking call)...
#visualization.dialog()

#Automatic construction of space
space = robotplanning.makeSpace(world=world,robot=robot,
                                edgeCheckResolution=1e-2,
                                movingSubset='all')

#Manual construction of space
#collider = WorldCollider(world)
#space = robotcspace.RobotCSpace(robot,collider)
#space.eps = 1e-2
#space.setup()

#Generate some waypoint configurations using the resource editor
configs = []
while True:
    (save,q) = resource.edit("plan config "+str(len(configs)+1),robot.getConfig(),"Config",world=world,description="Press OK to add waypoint, cancel to stop")
    if save:
        if False and not space.feasible(q):
            print "Configuration is infeasible. Failures:"
            print " ",space.cspace.feasibilityFailures(q)
            print "Please pick another"
        else:
            configs.append(q)
            robot.setConfig(q)
    else:
        break
if len(configs)==0:
    exit(0)

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
    #Manual construction of planner
    #plan = cspace.MotionPlan(space, **settings)
    #plan.setEndpoints(configs[0],configs[1])

    #this code uses the robotplanning module's convenience functions
    robot.setConfig(configs[i])
    plan = robotplanning.planToConfig(world,robot,configs[i+1],
                                      movingSubset='all',
                                      **settings)
    if plan is None:
        break
    print "Planning..."
    plan.planMore(500)
    #this code just gives some debugging information
    V,E = plan.getRoadmap()
    print len(V),"feasible milestones sampled,",len(E),"edges connected"
    path = plan.getPath()
    if path is None or len(path)==0:
        print "Failed to plan path between configuration",i,"and",i+1
        #debug some sampled configurations
        print V[0:max(10,len(V))]
        break
    wholepath += path[1:]

    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()

if len(wholepath)>1:
    print "Path:",wholepath
    #show the path in the visualizer, repeating 5 times
    visualization.animate("robot",wholepath)
    visualization.spin((len(wholepath)-1)*5)
else:
    print "Failed to generate a plan"

visualization.kill()
