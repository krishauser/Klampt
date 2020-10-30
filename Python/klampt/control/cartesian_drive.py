from klampt.model import ik
from klampt.math import vectorops,so3,se3
import math

class CartesianDriveSolver:
    """Allows continuous, smooth Cartesian commands to be sent to a robot
    that is only joint-controlled.  This is a highly configurable and
    reliable solver suitable for long-duration, precise cartesian motions.
    Specifically, it handles subtleties with out-of-range commands,
    singularities, IK solve tolerances, and oscillations.

    Most of the solver is stateless except for ``driveTransforms`` and
    ``driveSpeedAdjustment``.

    Tests on an Intel i7 show that this can be run for 1 6DOF IK goal at ~200Hz
    in Python with default settings.  Reducing the max # of iterations or
    increasing the tolerance can help with running time. There is also a C++
    version in KrisLibrary/robotics/CartesianDrive.h if you need to squeak out
    extra performance. It includes some perforamcne enhancements, like avoiding
    forward kinematics updates for non-active joints.

    Attributes:
        robot (RobotModel or SubRobotModel): the robot that will be used.
        positionTolerance (float): IK position tolerance, default 1e-4
        rotationTolerance (float): IK orientation tolerance, default 1e-3
        links (list of ints): set of end effectors in the IK constraints.
        baseLinks (list of ints): if given, IK constraints will be treated
            relative to these links
        endEffectorOffsets (list of 3-vectors): the tool coordinates (local
            positions) for each constraint. Must be of size len(links).
        driveTransforms (list of se3 elements): The current cartesian drive
            goals. Must be of size len(links).
        ikGoals (list of IKObjectives):  If set, must be of size len(links).
            The solver updates the end effector DOFs selected for by these
            goals.  If not set, uses fixed position/orientation goals.
        qmin (list of floats, optional): If set, overrides the robot's
            default joint limits.
        qmax (list of floats, optional): If set, overrides the robot's
            default joint limits.
        vmin (list of floats, optional): If set, overrides the robot's
            default velocity limits.
        vmax (list of floats, optional): If set, overrides the robot's
            default velocity limits.
        solver (IKSolver): the IK solver used.  The user can configure
            parameters, e.g., the IK solve tolerance, active DOFs, joint
            limits, etc.
        driveSpeedAdjustment (float): internal speed adjustment in the range
            [0,1]. This helps keep the # of iterations consistent and low per
            time step.

    Usage::

        qstart = controller.getCommandedConfig()
        solver = CartesianDriveSolver(robot)
        solver.start(qstart,5)  #initializes with 1 link
        qcur = qstart
        while not done:
            wdes,vdes = [desired angular and translational velocities]
            progress,qcmd = solver.drive(qcur,wdes,vdes,dt)
            if progress <= 0:
                print("Stopped")
            controller.setCommandedConfig(qcmd)
            qcur = qcmd
    
    """
    def __init__(self,robot):
        self.robot = robot
        self.positionTolerance = 1e-4
        self.rotationTolerance = 1e-3
        self.ikSolveTolerance = 0
        self.driveSpeedAdjustment = 1.0
        self.links = None
        self.baseLinks = None
        self.endEffectorOffsets = None
        self.driveTransforms = None
        self.ikGoals = None
        self.qmin = None
        self.qmax = None
        self.vmin = None
        self.vmax = None
        self.solver = None

    def start(self,q,links,baseLinks=None,endEffectorPositions=None):
        """Configures the IK solver with some set of links

        After start() is called, you can set up additional settings
        for the IK solver by calling :meth:`setTolerance`, :meth:`setMaxIters`,
        :meth:`setActiveDofs`.

        Args:
            q (list of floats): the robot's current configuration.
            links (int, str, list of int, or list of str): the link or links
                to drive.
            baseLinks (int, str, list of int, or list of str, optional): If
                given, uses relative positioning mode. Each drive command is
                treated as relative to the given base link. A value of -1 or an
                empty str indicates no base link.
            endEffectorPositions (3-vector or list of 3-vectors, optional):
                Local offsets for the end effector positions, relative to the
                specified link. If not given, the offsets will be treated as
                zero.
        """
        if not hasattr(links,'__iter__'):
            links = [links]
        if baseLinks is not None:
            if not hasattr(baseLinks,'__iter__'):
                baseLinks = [baseLinks]
            if len(links) != len(baseLinks):
                raise ValueError("Links and baseLinks must have the same length")
        if endEffectorPositions is None:
            endEffectorPositions = [(0,0,0) for i in range(len(links))]
        else:
            assert len(endEffectorPositions) > 0
            if not hasattr(endEffectorPositions[0],'__iter__'):
                endEffectorPositions = [endEffectorPositions]
            if len(endEffectorPositions) != len(links):
                raise ValueError("Links and endEffectorPositions must have the same length")
        self.robot.setConfig(q)
        self.links = links
        self.baseLinks = baseLinks
        self.endEffectorOffsets = endEffectorPositions

        robotLinks = [self.robot.link(l) for l in links]
        self.driveTransforms = [[l.getTransform()[0],l.getWorldPosition(ofs)] for l,ofs in zip(robotLinks,endEffectorPositions)]
        if baseLinks is None:
            baseRobotLinks = [None]*len(links)
        else:
            baseRobotLinks = [self.robot.link(b) if ((isinstance(b,str) and len(b) > 0) or b >= 0) else None for b in baseLinks]
            #compute relative drive transforms
            self.driveTransforms = [list(se3.mul(se3.inv(b.getTransform()),t)) for (b,t) in zip(baseRobotLinks,self.driveTransforms)]
        self.ikGoals = [ik.fixed_objective(l,b,local=ofs) for (l,b,ofs) in zip(robotLinks,baseRobotLinks,endEffectorPositions)]
        self.solver = ik.IKSolver(self.robot)
        self.solver.setMaxIters(20)
        for g in self.ikGoals:
            self.solver.add(g)
        self.driveSpeedAdjustment = 1.0

    def setTolerance(self,positionTolerance,rotationTolerance=None):
        self.positionTolerance = positionTolerance
        if rotationTolerance is None:
            self.rotationTolerance = positionTolerance
        self.rotationTolerance = rotationTolerance

    def setMaxIters(self,maxIters):
        self.solver.setMaxIters(maxIters)

    def setActiveDofs(self,activeDofs):
        self.solver.setActiveDofs(activeDofs)

    def setJointLimits(self,qmin,qmax=None):
        if qmin is None:
            assert qmax is None,"qmin and qmax are either both given or both None"
            self.qmin,self.qmax = qmin,qmax
        else:
            assert qmax is not None,"qmin and qmax are either both given or both None"
            self.qmin,self.qmax = qmin,qmax

    def setVelocityLimits(self,vmin=None,vmax=None):
        if vmin is None:
            assert vmax is None,"vmin and vmax are either both given or both None"
            self.vmin,self.vmax = vmin,vmax
        else:
            assert vmax is not None,"vmin and vmax are either both given or both None"
            self.vmin,self.vmax = vmin,vmax

    def drive(self,qcur,angVel,vel,dt):
        """Drives the robot by an incremental time step to reach the desired
        Cartesian (angular/linear) velocities of the links previously specified
        in start().

        Args:
            qcur (list of floats): The robot's current configuration.
            angVel (3-vector or list of 3-vectors): the angular velocity of
                each driven link.  Set angVel to None to turn off orientation
                control of every constraint. angVel[i] to None to turn off
                orientation control of constraint i.
            vel (3-vector or list of 3-vectors): the linear velocity of each
                driven link.  Set vel to None to turn off position control of
                every constraint.  Set vel[i] to None to turn off position 
                control of constraint i.
            dt (float): the time step.

        Returns:
            tuple: A pair ``(progress,qnext)``. ``progress`` gives a value
            in the range [0,1] indicating indicating how far along the nominal
            drive amount the solver was able to achieve.  If the result is < 0,
            this indicates that the solver failed to make further progress.

            ``qnext`` is the resulting configuration that should be sent to the
            robot's controller.

        For longer moves, you should pass qnext back to this function as qcur.
        """
        if angVel is None:
            #turn off orientation control
            if vel is None:
                #nothing specified
                return (1.0,qcur)
            angVel = [None]*len(self.links)
        else:
            assert len(angVel) > 0
            if not hasattr(angVel[0],'__iter__'):
                angVel = [angVel]
        if vel is None:
            #turn off position control
            vel = [None]*len(self.links)
        else:
            assert len(vel) > 0
            if not hasattr(vel[0],'__iter__'):
                vel = [vel]
        if len(qcur) != self.robot.numLinks():
            raise ValueError("Invalid size of current configuration, %d != %d"%(len(qcur),self.robot.numLinks()))
        if len(angVel) != len(self.links):
            raise ValueError("Invalid # of angular velocities specified, %d != %d"%(len(angVel),len(self.links)))
        if len(vel) != len(self.links):
            raise ValueError("Invalid # of translational velocities specified, %d != %d"%(len(vel),len(self.links)))
        anyNonzero = False
        zeroVec = [0,0,0]
        for (w,v) in zip(angVel,vel):
            if w is not None and list(w) != zeroVec:
                anyNonzero=True
                break
            if v is not None and list(v) != zeroVec:
                anyNonzero=True
                break
        if not anyNonzero:
            return (1.0,qcur)

        qout = [v for v in qcur]

        #update drive transforms
        self.robot.setConfig(qcur)
        robotLinks = [self.robot.link(l) for l in self.links]
  
        #limit the joint movement by joint limits and velocity
        tempqmin,tempqmax = self.robot.getJointLimits()
        if self.qmin is not None:
            tempqmin = self.qmin
        if self.qmax is not None:
            tempqmax = self.qmax
        vmax = self.robot.getVelocityLimits()
        vmin = vectorops.mul(vmax,-1)
        if self.vmin is not None:
            vmin = self.vmin
        if self.vmax is not None:
            vmax = self.vmax
        for i in range(self.robot.numLinks()):
            tempqmin[i] = max(tempqmin[i],qcur[i]+dt*vmin[i])
            tempqmax[i] = min(tempqmax[i],qcur[i]+dt*vmax[i])

        #Setup the IK solver parameters
        self.solver.setJointLimits(tempqmin,tempqmax)
        tempGoals = [ik.IKObjective(g) for g in self.ikGoals]
        for i in range(len(self.links)):
            if math.isfinite(self.positionTolerance) and math.isfinite(self.rotationTolerance):
                tempGoals[i].rotationScale = self.positionTolerance/(self.positionTolerance+self.rotationTolerance)
                tempGoals[i].positionScale = self.rotationTolerance/(self.positionTolerance+self.rotationTolerance)
            elif not math.isfinite(self.positionTolerance) and not math.isfinite(self.rotationTolerance):
                pass
            else:
                tempGoals[i].rotationScale = min(self.positionTolerance,self.rotationTolerance)/self.rotationTolerance
                tempGoals[i].positionScale = min(self.positionTolerance,self.rotationTolerance)/self.positionTolerance
        tolerance = min(1e-6,min(self.positionTolerance,self.rotationTolerance)/(math.sqrt(3.0)*len(self.links)))
        self.solver.setTolerance(tolerance)

        #want to solve max_t s.t. there exists q satisfying T_i(q) = TPath_i(t)
        #where TPath_i(t) = Advance(Tdrive_i,Screw_i*t)
        if self.driveSpeedAdjustment == 0:
            self.driveSpeedAdjustment = 0.1
        anyFailures = False
        while self.driveSpeedAdjustment > 0:
            #advance the desired cartesian goals
            #set up IK parameters: active dofs, IKGoal
            amount = dt * self.driveSpeedAdjustment
            desiredTransforms = [[None,None] for i in range(len(self.links))]
            for i in range(len(self.links)):
                if vel[i] is not None:
                    desiredTransforms[i][1] = vectorops.madd(self.driveTransforms[i][1],vel[i],amount)
                    tempGoals[i].setFixedPosConstraint(self.endEffectorOffsets[i],desiredTransforms[i][1])
                else:
                    tempGoals[i].setFreePosition()
                if angVel[i] is not None:
                    increment = so3.from_moment(vectorops.mul(angVel[i],amount))
                    desiredTransforms[i][0] = so3.mul(increment,self.driveTransforms[i][0]);
                    tempGoals[i].setFixedRotConstraint(desiredTransforms[i][0]);
                else:
                    tempGoals[i].setFreeRotConstraint()
                self.solver.set(i,tempGoals[i])

            err0 = self.solver.getResidual()
            quality0 = vectorops.norm(err0)

            res = self.solver.solve()
            q = self.robot.getConfig()
            activeDofs = self.solver.getActiveDofs()
            for k in activeDofs:
                if q[k] < tempqmin[k] or q[k] > tempqmax[k]:
                    #the IK solver normalizer doesn't care about absolute
                    #values for joints that wrap around 2pi
                    if tempqmin[k] <= q[k] + 2*math.pi and q[k] + 2*math.pi <= tempqmax[k]:
                        q[k] += 2*math.pi
                    elif tempqmin[k] <= q[k] - 2*math.pi and q[k] - 2*math.pi <= tempqmax[k]:
                        q[k] -= 2*math.pi
                    else:
                        print("CartesianDriveSolver: Warning, result from IK solve is out of bounds: index",k,",",tempqmin[k],"<=",q[k],"<=",tempqmax[k])
                        q[k] = max(min(q[k],tempqmax[k]),tempqmin[k])
            self.robot.setConfig(q)

            #now evaluate quality of the solve
            errAfter = self.solver.getResidual()
            qualityAfter = vectorops.norm(errAfter)
            if qualityAfter > quality0:
                #print("CartesianDriveSolver: Solve failed: original configuration was better:",quality0,"vs",qualityAfter)
                #print("  solver result was",res,"increment",amount)
                res = False
            elif qualityAfter < quality0 - 1e-8:
                res = True
            
            if res:
                #success!
                for k in activeDofs:
                    qout[k] = q[k]
                    assert tempqmin[k]<=q[k] and q[k]<=tempqmax[k]

                #now advance the driven transforms
                #figure out how much to drive along screw
                numerator = 0  # this will get sum of distance * screws
                denominator = 0  # this will get sum of |screw|^2 for all screws
                #result will be numerator / denominator
                achievedTransforms = [(l.getTransform()[0],l.getWorldPosition(ofs)) for l,ofs in zip(robotLinks,self.endEffectorOffsets)]
                #TODO: get transforms relative to baseLink
                for i in range(len(self.links)):
                    if vel[i] is not None:
                        trel = vectorops.sub(achievedTransforms[i][1],self.driveTransforms[i][1])
                        vellen = vectorops.norm(vel[i])
                        axis = vectorops.div(vel[i],max(vellen,1e-8))
                        ut = vellen
                        tdistance = vectorops.dot(trel,axis)
                        tdistance = min(max(tdistance,0.0),dt*vellen)
                        numerator += ut*tdistance;
                        denominator += ut**2;
                    if angVel[i] is not None:
                        Rrel = so3.mul(achievedTransforms[i][0],so3.inv(self.driveTransforms[i][0]))
                        vellen = vectorops.norm(angVel[i])
                        angVelRel = so3.apply(so3.inv(self.driveTransforms[i][0]),angVel[i])
                        rotaxis = vectorops.div(angVelRel,max(vellen,1e-8))
                        Rdistance = axis_rotation_magnitude(Rrel,rotaxis)
                        Rdistance = min(max(Rdistance,0.0),dt*vellen)
                        uR = vellen
                        numerator += uR*Rdistance
                        denominator += uR**2

                distance = numerator / max(denominator,1e-8)
                  
                #computed error-minimizing distance along screw motion
                for i in range(len(self.links)):
                    if vel[i] is not None:
                        self.driveTransforms[i][1] = vectorops.madd(self.driveTransforms[i][1],vel[i],distance);
                    else:
                        self.driveTransforms[i][1] = achievedTransforms[i][1];
                    if angVel[i] is not None:
                        Rincrement = so3.from_moment(vectorops.mul(angVel[i],distance))
                        self.driveTransforms[i][0] = normalize_rotation(so3.mul(Rincrement,self.driveTransforms[i][0]));
                    else:
                        self.driveTransforms[i][0] = achievedTransforms[i][0]

                if math.ceil(distance / dt * 10) < math.floor(self.driveSpeedAdjustment*10):
                    self.driveSpeedAdjustment -= 0.1
                    self.driveSpeedAdjustment = max(self.driveSpeedAdjustment,0.0)
                elif not anyFailures:
                    #increase drive velocity
                    if self.driveSpeedAdjustment < 1.0:
                        self.driveSpeedAdjustment += 0.1
                        self.driveSpeedAdjustment = min(1.0,self.driveSpeedAdjustment)

                return (distance / dt, qout)
            else:
                #IK failed: try again with a slower speed adjustment
                anyFailures = True
                self.driveSpeedAdjustment -= 0.1
                if self.driveSpeedAdjustment <= 1e-8:
                    self.driveSpeedAdjustment = 0.0
                    break
        #no progress
        return (-1,qcur)

    def getTrajectory(self,qcur,angVel,vel,dt,numSteps,reset=True):
        """Retrieves an entire trajectory produced by stepping ``numSteps`` 
        steps at resolution ``dt``. 
  
        If ``reset`` is true (on by default), resets the ``driveTransforms``
        and ``driveSpeedAdjustment`` to the initial state after the trajectory
        is computed.
        """
        startDriveTransforms = None
        startDriveSpeedAdjustment = self.driveSpeedAdjustment
        if reset:
            startDriveTransforms = [T for T in self.driveTransforms]
        
        qout = [qcur]
        for i in range(numSteps):
            frac,qnext = self.drive(qout[-1],angVel,vel,dt)
            if frac <= 0:
                #done, just stop
                qout += [qout[-1]]*(numSteps+1-i)
                break
            qout.append(qnext)

        if reset:
            self.driveTransforms = self.startDriveTransforms
            self.driveSpeedAdjustment = startDriveSpeedAdjustment


def axis_rotation_magnitude(R,a):
    """Given a rotation matrix R and a unit axis a, determines how far R rotates
    about a. Specifically, if R = from_axis_angle((a,v)) this should return v (modulo pi).
    """
    cterm = so3.trace(R) - vectorops.dot(a,so3.apply(R,a))
    mR = so3.matrix(R)
    sterm = -(a[0]*(mR[1][2]-mR[2][1]) + a[1]*(mR[2][0]-mR[0][2]) + a[2]*(mR[0][1]-mR[1][0]))
    return math.atan2(sterm,cterm)


def normalize_rotation(R):
    """Given a matrix close to a proper (orthogonal) rotation matrix,
    returns a true orthogonal matrix."""
    q = so3.quaternion(R)  #normalizes
    return so3.from_quaternion(q)


