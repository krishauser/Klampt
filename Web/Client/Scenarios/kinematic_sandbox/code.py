import random
next_move_time = 1
robot = None

def init(world):
    global robot
    # Note: The values of __DIR__ and __KLAMPT_DIR__ are predefined.
    world.readFile(__KLAMPT_DIR__+"data/tx90scenario0.xml")
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

def keypress(key):
    #called on keyboard press
    print "Keyboard event: ",key