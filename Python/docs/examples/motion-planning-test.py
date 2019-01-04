from klampt import *
from klampt.plan import robotplanning,cspace
from klampt.io import resource
import time

world = WorldModel()
world.readFile("/home/motion/Klampt-examples/data/tx90cuptable.xml")
robot = world.robot(0)

#this is the CSpace that will be used.  Standard collision and joint limit constraints 
#will be checked
space = robotplanning.makeSpace(world,robot,edgeCheckResolution=0.05)

#fire up a visual editor to get some start and goal configurations
qstart = robot.getConfig()
qgoal = robot.getConfig()
save,qstart = resource.edit("Start config",qstart,"Config",world=world)
#it's worthwile to make sure that it's feasible
while save and not space.feasible(qstart):
    print "Start configuration isn't feasible, please pick one that is collision-free"
    save,qstart = resource.edit("Start config",qstart,"Config",world=world)

save,qgoal = resource.edit("Goal config",qgoal,"Config",world=world)
while save and not space.feasible(qgoal):
    print "Goal configuration isn't feasible, please pick one that is collision-free"
    save,qgoal = resource.edit("Goal config",qgoal,"Config",world=world)

settings = {'type':'rrt',
    'perturbationRadius':0.25,
    'bidirectional':True,
    'shortcut':True,
    'restart':True,
    'restartTermCond':"{foundSolution:1,maxIters:1000}"
}
t0 = time.time()
print "Creating planner..."
#Manual construction of planner 
planner = cspace.MotionPlan(space, **settings)
planner.setEndpoints(qstart,qgoal)
print "Planner creation time",time.time()-t0
t0 = time.time()
print "Planning..."
for round in range(100):
    planner.planMore(50)
    if planner.getPath() is not None:
        break
print "Planning time, 1000 iterations",time.time()-t0

path = planner.getPath()
if path is not None:
    print "Got a path with",len(path),"milestones"
else:
    print "No feasible path was found"

#provide some debugging information
V,E = planner.getRoadmap()
print len(V),"feasible milestones sampled,",len(E),"edges connected"

print "CSpace stats:"
spacestats = space.getStats()
for k in sorted(spacestats.keys()):
    print " ",k,":",spacestats[k]

print "Planner stats:"
planstats = planner.getStats()
for k in sorted(planstats.keys()):
    print " ",k,":",planstats[k]


if path:
    #save planned milestone path to disk
    print "Saving to my_plan.configs"
    resource.set("my_plan.configs",path,"Configs")

    #visualize path as a Trajectory resource
    from klampt.model.trajectory import RobotTrajectory
    traj = RobotTrajectory(robot,range(len(path)),path)
    #resource.edit("Planned trajectory",traj,world=world)

    #visualize path in the vis module
    from klampt import vis
    vis.add("world",world)
    vis.animate(("world",robot.getName()),path)
    vis.add("trajectory",traj)
    vis.spin(float('inf'))

#play nice with garbage collection
planner.space.close()
planner.close()