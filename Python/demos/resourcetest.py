from klampt import resource
from klampt import *

print """resourcetest.py: This program gives an example of how to use the
resource module."""

worldfile = "../../data/athlete_plane.xml"
robotname = 'athlete'

world = WorldModel()
world.readFile(worldfile)

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
xform = resource.get(name=None,type='RigidTransform',frame=world.robot(0).getLink(5).getTransform(),world=world)
