from klampt import *
from klampt.vis import editors
world = WorldModel()
if not world.readFile("/home/motion/Klampt-examples/data/robots/baxter.rob"):
    raise RuntimeError("Can't read the Baxter file")
links = editors.run(editors.SelectionEditor("active_ik_links",
                value=[15,16,17,18,19,20],
                description="Robot arm links for IK",
                world=world))
