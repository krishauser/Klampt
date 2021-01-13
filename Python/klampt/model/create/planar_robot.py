import os

def make(n,world,link_length=1,tempname="temp.rob",debug=False):
    """Creates a simple ND planar robot with unit length links, and loads it
    into the given world.

    Args:
        n (int): the number of joints to use
        world (WorldModel): a world that will contain the new robot
        link_length (float, optional): the link lengths (default 1).
        tempname (str, optional): a name of a temporary file containing
            the moving-base robot
        debug (bool, optional): if True, the robot file named by
            ``tempname`` is not removed from disk.

    Returns:
        (RobotModel): the loaded robot, stored in ``world``.
    """

    intro = """### A %dR planar robot ###
    TParent 1 0 0   0 1 0   0 0 1   0 0 0 """ %(n,)
    trans = """   \\\n1 0 0   0 1 0   0 0 1   %g 0 0"""%(link_length,)
    axis = "0 1 0"
    jointtype = "r"
    qMin = "0"
    qMax = "6.28319"
    q ="0"
    geometry = ' "{TriangleMesh\\nOFF\\n8 12 0\\n0.0 -0.05 -0.05\\n0.0 -0.05 0.05\\n0.0 0.05 -0.05\\n0.0 0.05 0.05\\n1.0 -0.05 -0.05\\n1.0 -0.05 0.05\\n1.0 0.05 -0.05\\n1.0 0.05 0.05\\n3 0 1 3\\n3 0 3 2\\n3 4 6 7\\n3 4 7 5\\n3 0 4 5\\n3 0 5 1\\n3 2 3 7\\n3 2 7 6\\n3 0 2 6\\n3 0 6 4\\n3 1 5 7\\n3 1 7 3\\n}"'
    geomscale = "%g"%(max(link_length,0.05))
    mass =  "1"
    torqueMax = "10"
    accMax = "1"
    velMax = "1"

    joint = "joint spin"
    servoP = "50"
    servoI = "100"
    servoD = "3"

    f = open(tempname,'w')
    f.write(intro)
    for i in range(n-1):
        f.write(trans)
    f.write('\\n')
    f.write("axis\t"+"\t".join([axis]*n))
    f.write('\n')
    f.write("jointtype\t"+" ".join([jointtype]*n))
    f.write('\n')
    f.write("qMin\t"+" ".join([qMin]*n))
    f.write('\n')
    f.write("qMax\t"+" ".join([qMax]*n))
    f.write('\n')
    f.write("q\t"+" ".join([q]*n))
    f.write('\n')
    f.write("geometry\t"+" ".join([geometry]*n))
    f.write('\n')
    f.write("geomscale\t"+" ".join([geomscale]*n))
    f.write('\n')
    f.write("mass\t"+" ".join([mass]*n))
    f.write('\n')
    f.write("automass")
    f.write('\n')
    f.write("torqueMax\t"+" ".join([torqueMax]*n))
    f.write('\n')
    f.write("accMax\t"+" ".join([accMax]*n))
    f.write('\n')
    f.write("velMax\t"+" ".join([velMax]*n))
    f.write('\n')
    for i in range(n):
        f.write(joint+" "+str(i)+'\n')
    f.write("servoP\t"+" ".join([servoP]*n))
    f.write('\n')
    f.write("servoI\t"+" ".join([servoI]*n))
    f.write('\n')
    f.write("servoD\t"+" ".join([servoD]*n))
    f.write('\n')
    f.close()

    world.loadElement(tempname)
    robot = world.robot(world.numRobots()-1)
    robot.setName("Planar %dR"%(n,))
    if debug:
        robot.saveFile(tempname)
    else:
        os.remove(tempname)
    return robot