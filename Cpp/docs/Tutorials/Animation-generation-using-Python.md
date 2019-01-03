

# Klamp't Tutorial: Animate a video of a path/trajectory using Python

In this tutorial we learn how to generate a video using the Python code. Klampt has a namespace _vis_ which provides many convenient visualization plugins. The GLProgram class in the klampt.glprogram has a method that can save the current OpenGL screenshot to the local disk for further manual video conversion. This tutorial demonstrates how this process works.

Difficulty: Middle

Time: 10 minutes

### Animate a video of a path/trajectory using Python
For the demonstration of video generation using code, the previous Klampt/Python/demos/gltemplate.py is used as an example but with some modifications. 

 - GLProgram class has to be imported for the utilization of its save_screen method. As a result, one more importation will be added
```
from klampt.vis.glprogram import GLProgram
```
 - Then GLProgram is an additional inheritance of the previous MyGLViewer class
 ```
 class MyGLViewer(GLSimulationProgram,GLProgram):
```
 - After the inheritance, the save_screen method is attached to _self_ and _self_.save_screen([file name]) can be used when a screenshot is needed. The sample code is as follows:

```
class MyGLViewer(GLSimulationProgram,GLProgram):
    def __init__(self,files):
        #create a world from the given files
        world = WorldModel()
        for fn in files:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
        #initialize the simulation
        GLSimulationProgram.__init__(self,world)
        self.no = 0            # Iterative string name
        self.frame_no = 5	   # Save a screenshot every 5 iterations
        self.frame_no_ind = self.frame_no
        
    def control_loop(self):
        if self.frame_no_ind>0:
            self.frame_no_ind = self.frame_no_ind - 1
        else:
            self.frame_no_ind = self.frame_no
            self.no = self.no + 1
            record_name = "screenshot" + str(self.no) + ".ppm"
            self.save_screen(record_name)
        sim = self.sim
        if sim.getTime() >= 2.0 and sim.getTime()-self.dt < 2.0:
            q=sim.controller(0).getCommandedConfig()
            q[7]-=1.0
            sim.controller(0).setMilestone(q)
            q[7]+=1.5
            sim.controller(0).addMilestone(q)
```

 - Manual conversion. After the screenshot files are saved into the local disk, ffmpeg command is used to generate a video file
```
ffmpeg -f image2 -r 5 -i ./screenshot%d.ppm ./out.mp4
```
where -r 3 option sets the framerate of the output video to have 5 frames per second. 
<p align="center">
<img src="https://raw.githubusercontent.com/ShihaoWang/Figures/master/Animation-generation-using-Python.JPG"
width="75%" height="75%">
</p>

 - Do not forget to clean these temporary files
 ```
 rm -f *.ppm
 ```
