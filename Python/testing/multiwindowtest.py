from klampt import *
from klampt import vis
import time

print ("Tests multiple simultaneous windows.  This requires multithreaded visualization,")
print ("which doesn't work on Mac")

world1 = WorldModel()
world1.readFile('../../data/objects/block.obj')
id1 = vis.createWindow('First')
def firsthello():
	print ("hello from First")
vis.visualization._vis.add_action(firsthello,"First's action","p")
vis.add('world1', world1)
vis.show()

world2 = WorldModel()
world2.readFile('../../data/robots/athlete.rob')
id2 = vis.createWindow('Second')
def secondhello():
	print ("hello from Second")
vis.visualization._vis.add_action(secondhello,"Second's action","q")
vis.add('world2', world2)

vis.show()
while vis.shown():
	time.sleep(0.1)