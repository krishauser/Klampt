import sys
from klampt import *
from klampt import robotsim
from klampt import vis
import time

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
vis.show()
while vis.shown():
	vis.lock()
	#in klampt / robotio.h -- this needs to be done to update ROS
	processed = robotsim.ProcessStreams()
	if processed:
		#don't strictly need an if statement here, this is just a slight optimization
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
		#vis.dirty(("world","point_cloud"))

	vis.unlock()
	#TODO: do anything?
	time.sleep(0.3)

vis.kill()