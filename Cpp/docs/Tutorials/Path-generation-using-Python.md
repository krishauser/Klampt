
# Klamp't Tutorial: Generate a path/trajectory from keyframes using Python

In this tutorial we learn how to generate a time-parameterized trajectory from keyframes. The Python will only describe how to generate fixed-duration trajectories while the [app version](Documentation/Tutorials/Path-generation-using-Apps.md) will describe how to generate time-optimal trajectories.

Difficulty: easy

Time: 5 minutes

### Path/Trajectory generation using Python

The Python API does not currently support trajectory optimization. However, it does allow trajectories to be defined as piecewise-linear, linearly interpolated, or smooth spline functions. For more information, see the documentation on the  [Trajectory](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1model_1_1trajectory_1_1Trajectory.html)  superclass. For now, the following code shows an example of how to create and execute a smooth spline trajectory:
```
import sys
from klampt import*
from klampt import vis
from klampt.io import resource
from klampt.model import coordinates
from klampt.model import trajectory
import random, time, math

if __name__=="__main__":
    point = coordinates.addPoint("point")
    vis.add("point", point)
    traj = trajectory.Trajectory()
    minDis = -1
    maxDis = 1
    for i in range(10):
        traj.times.append(i/2.0)
        minDis =-1
        maxDis = 1
        val = 1
        if i%2==1:
            val = -1
        traj.milestones.append([(maxDis-minDis)*i/10.0+minDis, 0, val*0.2])
    traj2 = trajectory.HermiteTrajectory()
    traj2.makeSpline(traj)
    vis.animate("point", traj2)
    vis.show()

    iteration = 0
    while vis.shown():
        iteration+=1
    vis.kill()
```
Notice that every trajectory has a _times_ and a _milestones_ parameter. In order to create a spline trajectory, you must first create a HermiteTrajectory object and then call makeSpline on a previously created trajectory. This will allow HermiteTrajectory to do the requisite math to convert a series of waypoints into a smooth motion when animated. In this instance, you should see a point moving left to right in a vaguely sinusoidal motion.
After the execution of this Python program, the screenshot is 
<p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/Sinusoidal1.JPG"
width="50%" height="50%">
</p>

Extension: What happen if instead of animating traj2, you animate traj? If you would like to explore this tutorial further, please consult the Klampt/Python/demos/trajectorytest.py file.

[Run a simulation](Documentation/Tutorials/Run-a-simulation-Python.md)  also provides a good tutorial on how to simulate a robot by playing back a saved trajectory. Alternatively, you can send a controller a trajectory that is a collection of either piecewise-linear milestones or just linear milestones. Linear milestones are typically sent time-independent. A simulation just attempts to traverse the trajectory while satisfying its constraints. Piecewise-linear milestone paths, on the other hand, tend to enforce some sort of time constraints for getting from one configuration to another.

Additionally, the script Klampt/Python/utils/multipath_to_timed_multipath.py will convert an untimed MultiPath into a timed one, given a certain number of milestones per second. The speed variable at the top of the script will control the speed of the path.

