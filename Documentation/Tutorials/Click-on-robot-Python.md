

# Klamp't Tutorial: Process clicks on the robot or world in Python

In this tutorial we learn how to process clicks on the robot or world in the simulation environment with Python. Interactive feedback of the mouse clicks is useful for the determination of the object being clicked and furthermore the click point's x-y position. 

Difficulty: easy

Time: 10 minutes

Python provides convenient methods to process the click commands. After the importation of a visualization plugin such as klampt.vis.glrobotprogram.[GLSimulationPlugin](http://motion.pratt.duke.edu/klampt/pyklampt_docs/classklampt_1_1vis_1_1glrobotprogram_1_1GLSimulationPlugin.html), the following code shows how to use _mousefunc_ and __motionfunc_ to capture the clicks.
```
def mousefunc(self,button,state,x,y):
	#Put your mouse handler here
	#the current example prints out the list of objects clicked whenever
	#you right click
	print "mouse",button,state,x,y
	if button==2:
		if state==0:
			print [o.getName() for o in self.click_world(x,y)]
	    return
    GLSimulationPlugin.mousefunc(self,button,state,x,y)

def motionfunc(self,x,y,dx,dy):
	return GLSimulationPlugin.motionfunc(self,x,y,dx,dy)
  ```