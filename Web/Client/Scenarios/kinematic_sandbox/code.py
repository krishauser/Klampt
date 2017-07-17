import random
next_move_time = 1
robot = None

def init(world):
    global robot
    world.readFile("data/tx90scenario0.xml")
    robot = world.robot(0)
    kviz.add_text("HUD1",1,1)
    
def advance(t,world):
    global next_move_time
    kviz.update_text("HUD1",str(t))
    if t >= next_move_time:
        qmin,qmax = robot.getJointLimits()
        q = [random.uniform(a,b) for (a,b) in zip(qmin,qmax)]
        robot.setConfig(q)
        next_move_time += 1.0
    pass
