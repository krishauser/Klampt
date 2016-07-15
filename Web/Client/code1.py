nextTime = 1
toggle = 0
def init(robot_model):
    global ghost
    ghost=kviz.add_ghost('myGhost')
    kviz.set_color(ghost,[1,0,0,0.25],True)
    q=kviz.get_robot_config()
    q[8]-=2.0
    kviz.set_ghost_config(q,'myGhost')
    kviz.add_text('HUD1',1,1)
    kviz.update_text('HUD1','hello world')

def control_loop(t,controller):
    global nextTime, toggle
    if t >= nextTime:
        q = controller.getCommandedConfig()
        q[8] -= 0.5
        controller.setMilestone(q)
        nextTime += 1.0