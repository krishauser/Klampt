import klampt
from klampt.trajectory import Trajectory

def driversToConfig(robot,driverConfig):
    """Converts a configuration of the robot's drivers to a configuration of the robot"""
    nd = robot.numDrivers()
    assert nd == len(driverConfig),"Driver configuration does not match robot model, %d vs %d"%(nd,len(driverConfig))
    for i in range(nd):
        d = robot.getDriver(i)
        d.setValue(driverConfig[i])
    return robot.getConfig()

def driverToConfigTrajectory(robot,traj):
    qtraj = Trajectory()
    qtraj.times = traj.times[:]
    for q in traj.milestones:
        qtraj.milestones.append(driversToConfig(robot,q))
    return qtraj

if __name__=="__main__":
    import sys
    if len(sys.argv) != 4:
        print "Usage: driver_to_config_trajectory robot driver_traj config_traj"
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

    outtraj = driverToConfigTrajectory(robot,traj)
    qmin,qmax = robot.getJointLimits()
    for i in range(len(outtraj.milestones[0])):
        qmini = min(q[i] for q in outtraj.milestones)
        qmaxi = max(q[i] for q in outtraj.milestones)
        if qmini < qmin[i] or qmaxi > qmax[i]:
            print "Warning, item %d out of bounds: [%f,%f] not in [%f,%f]"%(i,qmini,qmaxi,qmin[i],qmax[i])
        else:
            print "Range on item %d: [%f,%f]"%(i,qmini,qmaxi)
    print "Saving to",outfn,"..."
    outtraj.save(outfn)
