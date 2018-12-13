from klampt import vis
from klampt.vis.glinterface import GLPluginInterface
from klampt.io import resource
from klampt.model import trajectory
from klampt import *
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

robot = world.robot(0)
resource.setDirectory("resources/"+robot.getName())

#Generate some waypoint configurations using the resource editor
configs = resource.get("pathtest.configs","Configs",description="Set multiple configurations to interpolate",default=[robot.getConfig()],world=world,doedit=True)
if len(configs) < 2:
    print "Didn't add 2 or more milestones, quitting"
    exit(-1)
for q in configs:
    if len(q) != robot.numLinks():
        print "Some configuration isn't of the right size for the robot: %d != %d."%(len(q),robot.numLinks())
        print "   Is pathtest.configs configured for the correct robot? (%s)"%(robot.getName(),)
        exit(-1)
resource.set("pathtest.configs",configs)

#add the world elements individually to the vis
vis.add("robot",robot)
for i in range(1,world.numRobots()):
    vis.add("robot"+str(i),world.robot(i))
for i in range(world.numRigidObjects()):
    vis.add("rigidObject"+str(i),world.rigidObject(i))
for i in range(world.numTerrains()):
    vis.add("terrain"+str(i),world.terrain(i))

traj0 = trajectory.RobotTrajectory(robot,times=range(len(configs)),milestones=configs)

#show the path in the visualizer, repeating for 60 seconds
traj = trajectory.path_to_trajectory(traj0,speed=1.0)
print "*** Resulting duration",traj.endTime(),"***"
vis.animate("robot",traj)
vis.addText("label","Default values")
vis.addPlot("plot")
vis.addPlotItem("plot","robot")

#action callbacks
def setHalfSpeed():
    traj = trajectory.path_to_trajectory(traj0,speed=0.5)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","0.5x speed")
def setDoubleSpeed():
    traj = trajectory.path_to_trajectory(traj0,speed=2.0)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","2x speed")
def setParabolic():
    traj = trajectory.path_to_trajectory(traj0,velocities='parabolic',timing='Linf',speed=1.0)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","Parabolic velocity profile")
def setMinJerk():
    traj = trajectory.path_to_trajectory(traj0,velocities='minimum-jerk',timing='Linf',speed=1.0,zerotol=0.0)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","Start/stop minimum-jerk velocity profile")
def setStartStop():
    traj = trajectory.path_to_trajectory(traj0,velocities='trapezoidal',timing='Linf',speed=1.0,zerotol=0.0)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","Start/stop trapezoidal velocity profile")
def setCosine():
    traj = trajectory.path_to_trajectory(traj0,velocities='cosine',timing='Linf',speed=1.0,zerotol=0.0)
    print "*** Resulting duration",traj.endTime(),"***"
    vis.animate("robot",traj)
    vis.addText("label","Start/stop cosine velocity profile")
def setHermite():
    smoothInput = trajectory.HermiteTrajectory()
    smoothInput.makeSpline(traj0)
    ltraj = smoothInput.configTrajectory()
    dtraj = ltraj.discretize(0.1)
    dtraj = trajectory.RobotTrajectory(robot,dtraj.times,dtraj.milestones)
    traj = trajectory.path_to_trajectory(dtraj,velocities='constant',timing='limited',smoothing=None,
        zerotol=10.0,vmax=robot.getVelocityLimits(),amax=robot.getAccelerationLimits(),
        speed=1.0,)
    print "*** Resulting duration",traj.endTime(),"***"
    #vis.animate("robot",ltraj)
    #vis.animate("robot",dtraj)
    vis.animate("robot",traj)
    vis.addText("label","Hermite trajectory")

class MyVisPlugin(GLPluginInterface):
    def __init__(self):
        GLPluginInterface.__init__(self)
        self.add_action(setHalfSpeed,"0.5x speed",'-')
        self.add_action(setDoubleSpeed,"2x speed",'+')
        self.add_action(setParabolic,"Unsmoothed, parabolic",'p')
        self.add_action(setMinJerk,"Start/stop minimum jerk",'m')
        self.add_action(setStartStop,"Start/stop trapezoidal",'t')
        self.add_action(setCosine,"Start/stop cosine",'c')
        self.add_action(setHermite,"Hermite spline created manually",'h')


vis.pushPlugin(MyVisPlugin())
vis.spin(float('inf'))
vis.kill()
