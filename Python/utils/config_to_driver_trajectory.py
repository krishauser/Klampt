import klampt
from klampt.model.trajectory import Trajectory

def configToDrivers(robot,q):
    """Converts a configuration of the robot's drivers to a configuration of the robot"""
    n = robot.numLinks()
    assert n == len(q),"Configuration does not match robot model, %d vs %d"%(nd,len(q))
    nd = robot.numDrivers()
    robot.setConfig(q)
    res = []
    for i in range(nd):
        d = robot.getDriver(i)
        res.append(d.getValue())
    return res

def configToDriverTrajectory(robot,traj):
    qtraj = Trajectory()
    qtraj.times = traj.times[:]
    for q in traj.milestones:
        qtraj.milestones.append(configToDrivers(robot,q))
    return qtraj

if __name__=="__main__":
    import sys
    if len(sys.argv) != 4:
	print "config_to_driver_trajectory.py: Converts a trajectory file specified in a"
	print "robot's configuration space into a trajectory in the robot's driver (actuator)"
	print "space."
	print
        print "Usage: python config_to_driver_trajectory.py robot config_traj driver_traj"
        exit(0)
    robotfn = sys.argv[1]
    trajfn = sys.argv[2]
    outfn = sys.argv[3]
    #load the robot file
    world = klampt.WorldModel()
    #this makes it a little faster to load the robot -- you don't need the geometry
    world.enableGeometryLoading(False)
    robot = world.loadRobot(robotfn)
    if robot.getName()=='':
        print "Unable to load robot file",robotfn
        exit(1)
    traj = Trajectory()
    try:
        traj.load(trajfn)
    except IOError:
        print "Unable to load trajectory file",trajfn
        exit(1)

    outtraj = configToDriverTrajectory(robot,traj)
    print "Saving to",outfn,"..."
    outtraj.save(outfn)
