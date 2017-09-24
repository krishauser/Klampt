import random
next_move_time = 2.0
move_delay = 5.0
robot = None

def init(world):
    global robot
    world.readFile("data/tx90scenario0.xml")
    robot = world.robot(0)
    kviz.add_text("HUD1",1,1)
    
def control_loop(t,controller):
    global next_move_time
    kviz.update_text("HUD1",str(t))
    if t >= next_move_time:
        qmin,qmax = robot.getJointLimits()
        q = [random.uniform(a,b) for (a,b) in zip(qmin,qmax)]
        controller.setMilestone(q)
        next_move_time += move_delay
    pass

def keypress(key):
    #called on keyboard press
    print "Keyboard event: ",key