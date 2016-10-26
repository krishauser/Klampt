import math
import random
robot = None
t = 0
dt = 0.02
q,dq = None,None

def planarRP_dynamics(q,dq,g,torques,m=1):
    """Calculates the planar RP dynamics at the given state"""
    q1,q2 = q[0],q[1]
    dq1,dq2 = dq[0],dq[1]
    c1 = math.cos(q1)
    s1 = math.sin(q1)
    t1 = torques[0] - m*g*c1 - 2*m*dq1*dq2*q2
    t2 = torques[1] - m*g*s1 + m*dq1*dq1*q2
    if q2*q2 < 1e-8:
        print "Invalid configuration, extension is at 0"
    ddq1 = t1 / (m*q2*q2)
    ddq2 = t2 / m
    return (ddq1,ddq2)

def robot_dynamics(robot,q,dq,g,torques):
    """Calculates a generic robot's dynamics at the given state"""
    robot.setConfig(q)
    robot.setVelocity(dq)
    G = robot.getGravityForces(g)
    return robot.accelFromTorques(vectorops.sub(torques,G))

def init(world):
    global robot,t
    global q,dq
    world.loadElement("Web/Client/Scenarios/dynamics/planarRP.rob")
    robot = world.robot(0)
    q,dq = robot.getConfig(),robot.getVelocity()
    t = 0
    
def advance(t,world):
    global q,dq
    #basic PID control
    t1 = -2*(dq[0] - 0)
    t2 = -200*(q[1] - 1) - 10*(dq[1] - 0)
    
    #centrifugal force test
    if t >= 4 and t < 8:
        t1 = -10*(dq[0] - 8)
     
    #coriolis force test   
    #if t >= 4 and t < 20:
    #    t1 = -10*(dq[0] - 2)
    #    if t > 8:
    #        t2 = -200*(q[1] - 0.5*math.sin(t*4) - 2) - 10*(dq[1] - 2*math.cos(t*4))
    
    #torque limits
    if abs(t1) > 100:
        t1 = t1/abs(t1)*100
    if abs(t2) > 100:
        t2 = t2/abs(t2)*100
        
    print "Torques",t1,t2
    ddq = planarRP_dynamics(q,dq,-9.8,[t1,t2])
    #ddq = robot_dynamics(robot,q,dq,[0,0,-9.8],[t1,t2])
    #euler integration
    t += dt
    q = vectorops.madd(q,dq,dt)
    dq = vectorops.madd(dq,ddq,dt)
    print "Config:",q
    print "Velocity:",dq
    robot.setConfig(q)
    robot.setVelocity(dq)
