"""This just relays a ff_torque_trajectory_controller to the serial output"""

from serialcontroller import ControllerClient
import ff_torque_trajectory_controller
import trajectory_controller
import asyncore

host = 'localhost'
port = 3456
q_file = "../../../hyq_model/q_cmd.txt"
ff_torque_file = "../../../hyq_model/ff_torque_cmd.txt"

#with feedforward torques
s = ControllerClient((host,port),ff_torque_trajectory_controller.make(None,q_file,ff_torque_file))
#without feedforward torques
#s = ControllerClient((host,port),trajectory_controller.make(None,q_file))

#run
asyncore.loop()
