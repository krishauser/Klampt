from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import GLWorldPlugin

class MyPlugin(GLWorldPlugin):
  def __init__(self,world):
    GLWorldPlugin.__init__(self,world)
  def mousefunc(self,button,state,x,y):
    #Put your mouse handler here
    #the current example prints out the list of objects clicked whenever
    #you right click
    print "mouse",button,state,x,y
    if button==2:
      if state==0:
        print [o.getName() for o in self.click_world(x,y)]
        return
    GLWorldPlugin.mousefunc(self,button,state,x,y)

  def motionfunc(self,x,y,dx,dy):
    return GLWorldPlugin.motionfunc(self,x,y,dx,dy)

world = WorldModel()
if not world.readFile("../../../Klampt-examples/data/athlete_plane.xml"):
  raise RuntimeError("Couldn't load world")
vis.run(MyPlugin(world))
