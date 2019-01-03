# Klamp't Tutorial: Set up a simulated camera sensor and save frames to disk using Python

In this tutorial we will learn how to use the sensor simulation capabilities in the Klamp't framework for each of the supported sensor types: visual, tactile, and inertial. For information of sensor overview and sensor definitions, please refer to the [simulated sensor tutorial using Apps](Documentation/Tutorials/Simulated-sensor-Apps.md)

Difficulty: moderate

Time: 30 minutes


Throughout, we will assume that your code is placed in a directory that shares the same parent as Klamp't. We also assume you have learned how to build applications that link to Klamp't using CMake, e.g. by completing the [simulation tutorial](http://motion.pratt.duke.edu/klampt/tutorial_simulation_0.7.html).

In this section we will go over a simple python script that makes changes to a sensor's characteristics before running a simulation. We'll be using Klampt/data/tx90scenario0.xml as the base world file. The script begins with import statements to include all the necessary modules to run our simulation. The  [vis module](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1vis_1_1glcommon.html) is used to generate visualizations of the robot geometry and sensor data being simulated. The [so3](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1math_1_1so3.html), [se3](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1math_1_1se3.html), and [vectorops](http://motion.pratt.duke.edu/klampt/pyklampt_docs/namespaceklampt_1_1math_1_1vectorops.html) modules are used for vector math and rigid body transformations.
```
from klampt import vis
from klampt import *
from klampt.math import so3,se3,vectorops
from klampt.vis.glcommon import *
import time
```
The following code defines a functuion which will run once each simulation time step to process the depth data. In this case, we simply perform a min and max operation over every pixel of our depth camera to determine the depth range of each frame and print it to the console. This snippet demonstrates two of the methods provided by the sensor class in the python interface: getMeasurements and getSetting.

As the name indicates, getMeasurements is used to get the state of the sensors for the current time step. The getSetting method allows you to query the sensor model for its parameters. The form of the data returned by getMeasurements and the available settings vary for each sensor. 
```
def processDepthSensor(sensor):
  data = sensor.getMeasurements()
  w = int(sensor.getSetting("xres"))
  h = int(sensor.getSetting("yres"))
  mind,maxd = float('inf'),float('-inf')
  for i in range(h):
    for j in range(w):
      pixelofs = (j+i*w)
      rgb = int(data[pixelofs])
      depth = data[pixelofs+w*h]
      mind = min(depth,mind)
      maxd = max(depth,maxd)
  print "Depth range",mind,maxd
```
The next part of the code is used to initialize a world model and configure it by reading in a world file. The simulator is also created, and a reference to a sensor is created using the sensor method of the  [SimRobotController](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1robotsim_1_1SimRobotController.html)  class. In this instance, the sensor is referred to by its name, but it is also possible to use its integer index (i.e. sim.controller(0).sensor(0))
```
world = WorldModel()
world.readFile("../data/simulation_test_worlds/sensortest.xml")
#world.readFile("../../data/tx90scenario0.xml")
robot = world.robot(0)

vis.add("world",world)

sim = Simulator(world)
sensor = sim.controller(0).sensor("rgbd_camera")
```
In the following lines, the getSetting method is used to query the link index the sensor is attached to, and its relative transformation to that link's origin. The setSetting method is used to modify the sensor's parent link, attaching to the world instead of the robot. The link's relative position and orientation is also changed to a random location/direction.
```
print sensor.getSetting("link")
print sensor.getSetting("Tsensor")
sensor.setSetting("link",str(-1))
T = (so3.sample(),[0,0,1.0])
sensor.setSetting("Tsensor",' '.join(str(v) for v in T[0]+T[1]))
```
The remainder of the code adds the sensor to the visualization, defines the object that interfaces with the visualization system, and sets up the loop that performs the simulation stepping.
```
vis.add("sensor",sensor)

class SensorTestWorld (GLPluginInterface):
  def __init__(self):
    robot.randomizeConfig()
    sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)

  def idle(self):
    sim.simulate(0.01)
    sim.updateWorld()
    processDepthSensor(sensor)
    return True

  def keyboardfunc(self,c,x,y):
    if c == ' ':
      robot.randomizeConfig()
      sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)

vis.pushPlugin(SensorTestWorld())
vis.show()
while vis.shown():
  time.sleep(0.1)
vis.kill()
```
The frames can be saved to the local disk in the same way as the [video animation tutorial](Documentation/Tutorials/Animation-generation-using-Python.md) . The visualization of this simulation is as follows
 <p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/simulated%20camera5.JPG"
width="75%" height="75%">
</p>
where the camera is mounted on the robot and grey rectangle area is the camera view