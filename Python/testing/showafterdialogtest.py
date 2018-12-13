from klampt import *
from klampt import vis
from klampt.io import resource
import time

print ("Tests threading after dialog")

world = WorldModel()
world.readFile('../../data/objects/block.obj')
resource.edit("object transform",world.rigidObject(0).getTransform(),world=world)

def launchdialog():
    resource.edit("object transform launched from window",world.rigidObject(0).getTransform(),world=world)

def launchwindow():
    origwindow = vis.getWindow()
    vis.createWindow("Pop up window")
    vis.add("world2",world)
    vis.show()
    vis.setWindow(origwindow)

print ("Now running show() (only works on multithreaded systems, not mac)")
vis.add("world",world)
vis.visualization._vis.add_action(launchdialog,"Launch a dialog","d")
vis.visualization._vis.add_action(launchwindow,"Launch a window","w")
vis.show()
while vis.shown():
    time.sleep(0.1)
vis.kill()