import math

robot = None
dt = 0.01

#for I terms
pid_integrators = [0,0,0,0]

#higher level controller status... you don't need to use these if you wish
status = "pen up"
current_stroke = 0
current_stroke_progress = 0
stroke_list = []

def init(world):
    global robot,stroke_list
    robot = world.robot(0)
    stroke_list = curves()

def getPIDTorqueAndAdvance(q,dq,
                        qdes,dqdes,
                        kP,kI,kD,
                        pid_integrators,dt):
    """ TODO: implement me
    Returns the torques resulting from a set of PID controllers, given:
    - q: current configuration (list of floats)
    - dq: current joint velocities 
    - qdes: desired configuration
    - dqdes: desired joint velocities
    - kP: list of P constants, one per joint
    - kI: list of I constants, one per joint
    - kD: list of D constants, one per joint
    - pid_integrators: list of error integrators, one per joint
    - dt: time step since the last call of this function

    The pid_integrators list should also be updated according to the time step.
    """
    torques = [0]*len(q)
    for i in range(len(q)):
        #only the P term is computed here...
        torques[i] = -kP[i]*(q[i] - qdes[i])
    return torques
    
def getTorque(t,q,dq):
    """ TODO: implement me
    Returns a 4-element torque vector given the current time t, the configuration q, and the joint velocities dq to drive
    the robot so that it traces out the desired curves.
    
    Recommended order of operations:
    1. Monitor and update the current status and stroke
    2. Update the stroke_progress, making sure to perform smooth interpolating trajectories
    3. Determine a desired configuration given current state using IK
    4. Send qdes, dqdes to PID controller
    """
    global robot,status,current_stroke,current_stroke_progress,stroke_list
    qdes = [0,0,0,0]
    dqdes = [0,0,0,0]
    if t > 0.5:
        #move up
        qdes[1] = 0.3
    if t > 1.5:
        #drop the last link down
        qdes[3] = 0.04
    if t > 2.0:
        #move down
        qdes[1] = -0.3
    kP = [20,20,1,200]
    kI = [5,5,1,50]
    kD = [2,2,0.1,5]
    return getPIDTorqueAndAdvance(q,dq,qdes,dqdes,kP,kI,kD,pid_integrators,dt)
    #return [0,1,0,0]

def curves():
    K = [[(0.2,0.05),(0.2,-0.05)],[(0.25,0.05),(0.2,0.0),(0.25,-0.05)]]
    H = [[(0.28,0.05),(0.28,-0.05)],[(0.33,0.05),(0.33,-0.05)],[(0.28,0),(0.33,0)]]
    return K+H

#####################################################################
# Place your written answers here
#
#
#
#
#
#
#
#
#
