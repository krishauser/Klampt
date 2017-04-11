from klampt.io import resource
from klampt.vis import visualization
from klampt.model.trajectory import *
from klampt import *

print """resourcetest.py: This program gives an example of how to use the
resource module."""

worldfile = "../../data/athlete_plane.xml"
robotname = 'athlete'

world = WorldModel()
world.readFile(worldfile)

#tests of visualization module interacting with the resource module
"""
print "Showing robot in modal dialog box"
visualization.add("robot",world.robot(0))
visualization.add("ee",world.robot(0).link(11).getTransform())
visualization.dialog()
"""

#tests of visualization module interacting with the resource module
"""
import time
print "Showing threaded visualization"
visualization.show()
for i in range(3):
    visualization.lock()
    q = world.robot(0).getConfig()
    q[9] = 3.0
    world.robot(0).setConfig(q)
    visualization.unlock()
    time.sleep(1.0)
    if not visualization.shown():
        break
    visualization.lock()
    q = world.robot(0).getConfig()
    q[9] = -1.0
    world.robot(0).setConfig(q)
    visualization.unlock()
    time.sleep(1.0)
    if not visualization.shown():
        break
print "Hiding visualization window"
visualization.show(False)
"""

#look in resources/athlete/
resource.setDirectory('resources/'+robotname)

config1 = resource.get("resourcetest1.config",description="First config, always edited",doedit=True,editor='visual',world=world)
print "Config 1:",config1
config2 = resource.get("resourcetest1.config",description="Trying this again...",editor='visual',world=world)
print "Config 2:",config2

config3 = resource.get("resourcetest2.config",description="Another configuration",editor='visual',world=world)
print "Config 3:",config3

if config3 != None:
    config3hi = config3[:]
    config3hi[3] += 1.0
    resource.set("resourcetest3_high.config",config3hi)
    world.robot(0).setConfig(config3hi)

configs = []
if config1 != None: configs.append(config1)
if config2 != None: configs.append(config2)
if config3 != None: configs.append(config3)
print "Configs resource:",configs
configs = resource.get("resourcetest.configs",default=configs,description="Editing config sequence",doedit=True,editor='visual',world=world)

traj = RobotTrajectory(world.robot(0),range(len(configs)),configs)
traj = resource.edit(name="Timed trajectory",value=traj,description="Editing trajectory",world=world)

#testing transform editor.  The first call doesn't attach any geometry to the frame.  The second call attaches geometry to it.
#Both return the edited transform *relative* to the base link.
xform = resource.edit(name="Base link, floating",value=None,type='RigidTransform',frame=world.robot(0).link(5).getTransform(),world=world)
xform = resource.edit(name="Base link, should be attached to robot",value=None,type='RigidTransform',frame=world.robot(0).link(5),world=world)


#this will prompt you to load a Config resource, then ask you to save the edited config
"""
fn,q = resource.load('Config')
resource.edit('loaded config',q,world=world)
resource.save(q)
"""

#this will save thumbnails of all previously created resources
"""
import os,glob
for fn in glob.glob('resources/'+robotname+'/*'):
    filename = os.path.basename(fn)
    print os.path.splitext(filename)[1]
    if os.path.splitext(filename)[1] in resource.knownTypes():
        print fn
        res = resource.get(filename)
        im = resource.thumbnail(res,(128,96),world=world)
        if im != None:
            im.save('resources/'+robotname+"/"+filename+".thumb.png")
"""


#this is needed to avoid a Ctrl+C to kill the visualization thread
visualization.kill()
