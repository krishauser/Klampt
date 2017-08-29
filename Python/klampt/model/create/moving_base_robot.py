"""Common code for creating and moving free-floating moving bases.

The way to do this is to add a "virtual linkage" of 3 translational DOFs
and 3 revolute DOFs.  Some tuning may need to be done to the motor drivers
in order to make the controller stable.
"""

import os
from klampt.math import so3


def make_moving_base_robot(robotfile,world,tempname="temp.rob",debug=False):
	"""Converts the given fixed-base robot file into a moving base robot
	and loads it into the given world.

	If debug = True then the robot file named by tempname is not removed from disk.
	"""
	_template_ = """### Boilerplate kinematics of a drivable floating (translating and rotating) cube with a robot hand mounted on it
TParent 1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  \
1 0 0   0 1 0   0 0 1   0 0 0  
parents -1 0 1 2 3 4 
axis 1 0 0   0 1 0    0 0 1     0 0 1     0 1 0     1 0 0 
jointtype p p p r r r 
qMin -1 -1 -1  -inf -inf -inf
qMax  1  1  1   inf  inf  inf 
q 0 0 0 0 0 0 
links "tx" "ty" "tz" "rz" "ry" "rx"
geometry   ""   ""   ""   ""    ""    "data/objects/cube.tri"
geomscale 1 1 1 1 1 0.01
mass       0.1 0.1 0.1 0.1 0.1 0.1
com 0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   
inertia 0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 \
   0.001 0 0 0 0.001 0 0 0 0.001 
torqueMax  500 500 500 50 50 50 
accMax     4 4 4 4 4 4 4
velMax     2 2 2 3 3 3

joint normal 0
joint normal 1
joint normal 2
joint spin 3
joint spin 4
joint spin 5

driver normal 0 
driver normal 1
driver normal 2
driver normal 3
driver normal 4
driver normal 5

servoP 5000 5000 5000 500 500 500
servoI 10 10 10 .5 .5 .5
servoD 100 100 100 10 10 10
viscousFriction 50 50 50 50 50 50
dryFriction 1 1 1 1 1 1

property sensors <sensors><ForceTorqueSensor link="5" hasForce="1 1 1" hasTorque="1 1 1" /></sensors>
mount 5 "%s" 1 0 0   0 1 0   0 0 1   0 0 0 as "%s"
"""

	robotname = os.path.splitext(os.path.basename(robotfile))[0]
	f.close()
	f2 = open("temp.rob",'w')
	f2.write(_template_ % (robotfile,robotname))
	f2.close()
	world.loadElement("temp.rob")
	robot = world.robot(world.numRobots()-1)
	#set torques
	mass = sum(robot.link(i).getMass().mass for i in range(robot.numLinks()))
	inertia = 0.0
	for i in range(robot.numLinks()):
		m = robot.link(i).getMass()
		inertia += (vectorops.normSquared(m.com)*m.mass + max(m.inertia))
	tmax = robot.getTorqueMax()
	tmax[0] = tmax[1] = tmax[2] = mass*9.8*5
	tmax[3] = tmax[4] = tmax[5] = inertia*9.8*5
	robot.setTorqueMax(tmax)
	if debug:
		robot.saveFile("temp.rob")
	else:
		os.remove("temp.rob")
	return robot


def get_moving_base_xform(robot):
	"""For a moving base robot model, returns the current base rotation
	matrix R and translation t."""
	return robot.link(5).getTransform()

def set_moving_base_xform(robot,R,t):
	"""For a moving base robot model, set the current base rotation
	matrix R and translation t.  (Note: if you are controlling a robot
	during simulation, use send_moving_base_xform_command)
	"""
	q = robot.getConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	robot.setConfig(q)

def send_moving_base_xform_linear(controller,R,t,dt):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t using linear interpolation
	over the duration dt.

	Note: with the reflex model, can't currently set hand commands
	and linear base commands simultaneously
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	controller.setLinear(q,dt)

def send_moving_base_xform_PID(controller,R,t):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t by setting the PID setpoint

	Note: with the reflex model, can't currently set hand commands
	and linear base commands simultaneously
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	v = controller.getCommandedVelocity()
	controller.setPIDCommand(q,v)
