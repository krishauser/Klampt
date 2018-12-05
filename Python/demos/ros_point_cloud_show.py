#!/usr/bin/env python
import sys
from klampt import *
from klampt import robotsim
from klampt import vis
import time

MULTITHREADED = True

print """ros_point_cloud_show.py: Shows how to receive point clouds from ROS into Klamp't.

Usage: python ros_point_cloud_show.py topic [save]
- topic: the ROS topic to subscribe to.
- save: If "save" is specified, then point clouds are saved to disk in PCD format.

Simply close the window to quit.
"""

topic = sys.argv[1]

#rospy.init_node("Klampt_point_cloud_visualizer")

world = WorldModel()
world.makeRigidObject("point_cloud")
g = world.rigidObject(0).geometry()
g.loadFile("ros:/"+topic)

if len(sys.argv) > 2 and sys.argv[2] == 'save':
	dosave = True
else:
	dosave = False
point_cloud_count = 0
vis.add("world",world)
vis.edit(("world","point_cloud"))

def updatePointCloud():
	#in klampt / robotio.h -- this needs to be done to update ROS
	processed = robotsim.ProcessStreams()
	if processed:
		#don't strictly need the prior if statement, this is just a slight optimization
		#if the sender is slower than the visualization

		if g.empty():
			print "empty"
		else:
			pc = g.getPointCloud()
			print pc.numPoints(),"points and",pc.numProperties(),"properties"
			point_cloud_count += 1
			if dosave:
				print "Saving pcd file to temp%04d.pcd..."%(point_cloud_count,)
				g.saveFile("temp%04d.pcd"%(point_cloud_count,))

		#this needs to be done to refresh the appearance
		a = world.rigidObject(0).appearance()
		a.refresh()

if MULTITHREADED:
	vis.show()
	while vis.shown():
		vis.lock()
		updatePointCloud()
		vis.unlock()
		#TODO: do anything else?

		#runs at most 10Hz
		time.sleep(0.1)
else:
	data = {'next_update_time':time.time()}
	def callback():
		if time.time() >= data['next_update_time']:
			#run at approximately 10Hz
			data['next_update_time'] += 0.1
			updatePointCloud()
	vis.loop(setup=vis.show,callback=callback)

vis.kill()
