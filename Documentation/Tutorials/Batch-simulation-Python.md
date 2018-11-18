# Klamp't Tutorial: Monte Carlo Simulation

In this tutorial you will learn how to set up and use the Monte Carlo simulation function in Klamp't. It is useful to test if your algorithm or parameters work for different problems within a range, or how reliable a certain behavior is at managing uncertainty.

Difficulty: intermediate

Time: 15-30 minutes


This tutorial will do a very simple Monte Carlo example on a 1-link robot in the tutorials/control example, just to cover the basics on how to run the batch simulation module. Let us create a new file in tutorials/control called batch_test.py and use text editor to open it. In this example, we want to test if the controller works from different initial joint angles within the range [-pi, pi]. After a preset simulation time, we want to see if the angle is successfully controlled to the desired value.

First we have to import necessary modules, copy and paste the following code:
```
from klampt import *
from klampt.sim import batch
from klampt.model import map
import random, math
```
which will import the necessary modules to make Monte-Carlo simulation work.

In our script, we first load the xml file which defines the world. The world contains a one-link robot with an actuator.
```
world = WorldModel()
fn = "world1.xml"
res = world.readFile(fn)
if not res:
    raise RuntimeError("Unable to load world "+fn)
```
Each simulation is initialized from some initial conditions that will be sampled at random, but we need to specify which parts of the world are actually sampled. We use the map module which can set and get named values in a world or simulation. Please refer to the map module documentation to learn more about how to use it. We will define a zero-argument sampling function that will sample the first DOF position of the robot from -pi to pi as follows:
```
item = 'robots[0].config[0]'
itemsampler = lambda: random.uniform(-math.pi, math.pi)
initialConditionSamplers = {item:itemsampler}
```
For each of the N Monte-Carlo runs, the sample itemsampler will be called and the returned value will be assigned to robots[0].config[0]. Any number of items in the world can be sampled by adding them to the initialConditionSamplers dictionary. For example, to sample the initial velocity, you would define a sampler for the 'robots[0].velocity[0]' item.

From these sampled initial conditions, batch.monteCarloSim will create a new Simulator instance and run a simulation trace. To customize the behavior of the simulation trace we can define three callback functions: simInit which is called when the simulation begins, simStep which is called every step, and simTerm which is called to determine whether the simulation should stop. Here we'll just change the simInit function, which is a one-argument function taking in a Simulator. In it we define some parameters of the robot's controller:
```
def simInitFun(sim):
    controller = sim.controller(0)
    controller.setPIDCommand([0],[0])
    kP = 20
    kI = 8
    kD = 5
    controller.setPIDGains([kP],[kI],[kD])
```
which sets the target (0,0) and PID constants. (See the controller tutorial for more details about what these parameters mean.)

Next, we define an array returnItems that defines what data we want to retreive after each simulation run. In this example it means both the joint configuration and the joint velocity of the robot. Then we define duration of simulation and number of simulations, and call the batch.monteCarloSim function to simulate. See the documentation for other options. The return value is a list of (initial condition, return items) pairs.
```
returnItems = ['robots[0].config','robots[0].velocity']
duration = 5
N = 100
res = batch.monteCarloSim(world,duration,initialConditionSamplers,N,returnItems, simInit=simInitFun)
```
Finally, we print the start and end configuration at each run, and use a file to record the data for post processing.
```
f = open('result.txt', 'w')
for i in range(N):
    initialCond,results = res[i]
    startConfig = initialCond['robots[0].config[0]']
    endConfig = results[returnItems[0]]
    print startConfig,"->",endConfig
    f.write('%lf\n'%(endConfig[0]))
f.close()
```
Then run the example by calling
```
python batch_test.py
```
After plotting the resulting error of the 0 angle, we obtain the following distribution:
<p align="center">
<img src="http://motion.pratt.duke.edu/klampt/ErrorHist.jpg"
width="50%" height="50%">
</p>
which is a histogram of the final joint angle. From this figure we can know how the controller performs in order to control the joint angle from arbitrary value to 0 within 5 seconds. This can provide information on how the controller works based on how the parameters are tuned.

More advanced usage could add random parameters to the controller, which are sent as arguments to the simInit, simStep, and simTerm functions. This is accomplished using the special initial condition named 'args', which is a tuple that gets passed to each of these functions. For example, if we wanted to sample the target angle of the controller, we can do so as follows:
```
item = 'robots[0].config[0]'
itemsampler = lambda: random.uniform(-math.pi, math.pi)
initialConditionSamplers = {item:itemsampler}
initialConditionSamplers['args'] = lambda:(random.uniform(-0.5, 0.5),)

def simInitFun(sim,targetAngle):
    controller = sim.controller(0)
    controller.setPIDCommand([targetAngle],[0])
    kP = 20
    kI = 8
    kD = 5
    controller.setPIDGains([kP],[kI],[kD])

returnItems = ['robots[0].config']
duration = 5
N = 100
res = batch.monteCarloSim(world,duration,initialConditionSamplers,N,returnItems, simInit=simInitFun)

f = open('result.txt', 'w')
for i in range(N):
    #print res[i][1]
    initialCond,results = res[i]
    startConfig = initialCond['robots[0].config[0]']
    endConfig = results[returnItems[0]]
    print "from",startConfig,"to",initialCond["args"][0],"->",endConfig
    f.write('%lf\n'%(endConfig[0]))
f.close()
```

