from klampt import resource
from klampt import visualization
from klampt import *

print """resourcetest.py: This program gives an example of how to use the
resource module."""

worldfile = "../../data/athlete_plane.xml"
robotname = 'athlete'

world = WorldModel()
world.readFile(worldfile)

print "Showing robot in modal dialog box"
visualization.add("robot",world.robot(0))
visualization.add("ee",world.robot(0).link(11).getTransform())
visualization.dialog()

import threading
import time

print "Showing threaded visualization"
lock = threading.Lock()
visualization.show(lock)
for i in range(3):
    lock.acquire()
    q = world.robot(0).getConfig()
    q[9] = 3.0
    world.robot(0).setConfig(q)
    lock.release()
    time.sleep(1.0)
    if not visualization.shown():
        break
    lock.acquire()
    q = world.robot(0).getConfig()
    q[9] = -1.0
    world.robot(0).setConfig(q)
    lock.release()
    time.sleep(1.0)
    if not visualization.shown():
        break
visualization.show(False)

#look in resources/athlete/
resource.setDirectory('resources/'+robotname)

config = resource.get("resourcetest1.config",description="First config, always edited",doedit=True,editor='visual',world=world)
print "Config 1:",config
config = resource.get("resourcetest1.config",description="Trying this again...",editor='visual',world=world)
print "Config 2:",config

config = resource.get("resourcetest2.config",description="Another configuration",editor='visual',world=world)
print "Config 3:",config

if config != None:
    config[3] += 1.0
    resource.set("resourcetest3_high.config",config)
    world.robot(0).setConfig(config)

#testing transform editor
xform = resource.get(name=None,type='RigidTransform',frame=world.robot(0).link(5).getTransform(),world=world)

#this is needed to avoid a Ctrl+C to kill the visualization thread
visualization.kill()
