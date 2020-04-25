"""Common code for creating and moving free-floating moving bases.

The way to do this is to add a "virtual linkage" of 3 translational DOFs
and 3 revolute DOFs.  Some tuning may need to be done to the motor drivers
in order to make the controller stable.
"""

import os
from klampt.math import vectorops,so3


def make(robotfile,world,tempname="temp.rob",debug=False):
	"""Converts the given fixed-base robot file into a moving base robot
	and loads it into the given world.

	Args:
		robotfile (str): the name of a fixed-base robot file to load
		world (WorldModel): a world that will contain the new robot
		tempname (str, optional): a name of a temporary file containing
			the moving-base robot
		debug (bool, optional): if True, the robot file named by
			``tempname`` is not removed from disk.

	Returns:
		(RobotModel): the loaded robot, stored in ``world``.
	"""
	_template_ = """### Boilerplate kinematics of a drivable floating (translating and rotating) cube with a robot hand mounted on it
TParent 1 0 0   0 1 0   0 0 1   0 0 0  \\
1 0 0   0 1 0   0 0 1   0 0 0  \\
1 0 0   0 1 0   0 0 1   0 0 0  \\
1 0 0   0 1 0   0 0 1   0 0 0  \\
1 0 0   0 1 0   0 0 1   0 0 0  \\
1 0 0   0 1 0   0 0 1   0 0 0  
parents -1 0 1 2 3 4 
axis 1 0 0   0 1 0    0 0 1     0 0 1     0 1 0     1 0 0 
jointtype p p p r r r 
qMin -1 -1 -1  -inf -inf -inf
qMax  1  1  1   inf  inf  inf 
q 0 0 0 0 0 0 
links "tx" "ty" "tz" "rz" "ry" "rx"
geometry   ""   ""   ""   ""    ""    "{TriangleMesh\\nOFF\\n8 12 0\\n0 0 0\\n0 0 1\\n0 1 0\\n0 1 1\\n1 0 0\\n1 0 1\\n1 1 0\\n1 1 1\\n3 0 1 3\\n3 0 3 2\\n3 4 6 7\\n3 4 7 5\\n3 0 4 5\\n3 0 5 1\\n3 2 3 7\\n3 2 7 6\\n3 0 2 6\\n3 0 6 4\\n3 1 5 7\\n3 1 7 3\\n}"
geomscale 1 1 1 1 1 0.01
mass       0.1 0.1 0.1 0.1 0.1 0.1
com 0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   0 0 0   
inertia 0.001 0 0 0 0.001 0 0 0 0.001 \\
   0.001 0 0 0 0.001 0 0 0 0.001 \\
   0.001 0 0 0 0.001 0 0 0 0.001 \\
   0.001 0 0 0 0.001 0 0 0 0.001 \\
   0.001 0 0 0 0.001 0 0 0 0.001 \\
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

property sensors <sensors><ForceTorqueSensor name="base_force" link="5" hasForce="1 1 1" hasTorque="1 1 1" /></sensors>
mount 5 "%s" 1 0 0   0 1 0   0 0 1   0 0 0 as "%s"
"""

	robotname = os.path.splitext(os.path.basename(robotfile))[0]
	f = open(tempname,'w')
	f.write(_template_ % (robotfile,robotname))
	f.close()
	world.loadElement(tempname)
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
	robot.setName("moving-base["+robotname+"]")
	robot.setTorqueMax(tmax)
	if debug:
		robot.saveFile(tempname)
	else:
		os.remove(tempname)
	return robot


def get_xform(robot):
	"""For a moving base robot model, returns the current base rotation
	matrix R and translation t."""
	return robot.link(5).getTransform()

def set_xform(robot,R,t):
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

def send_xform_linear(controller,R,t,dt):
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

def send_xform_PID(controller,R,t):
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
