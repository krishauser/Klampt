from __future__ import division
import klampt, time
from klampt import vis
from klampt import *
from klampt.model import sensing
import matplotlib.pyplot as plt

world = WorldModel()
world.readFile('../../data/simulation_test_worlds/sensortest.xml') # a world with RobotiQ, and a camera
sim = Simulator(world)
camera = sim.controller(0).sensor('rgbd_camera')
T = ([1,0,0, 0,0,-1,  0,1,0],[0,-2.0,0.5])
sensing.set_sensor_xform(camera,T,link=-1)

sim.simulate(0.01)
sim.updateWorld()
rgb,depth = sensing.camera_to_images(camera)
print rgb.shape
print depth.shape
plt.imshow(depth) # <---- THIS LINE PREVENTS VIS.SHOW() FROM SHOWING
plt.show()

vis.add('world', world)
vis.show()
while vis.shown():
        time.sleep(0.1)
vis.kill()

plt.show()
quit()
