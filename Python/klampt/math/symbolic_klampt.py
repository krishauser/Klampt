""" Defines many functions of Klampt as symbolic Functions.

Currently implemented:

- so3, se3
- ik
- some collide functions
- RobotModel kinematics

TODO:

- RobotModel dynamics
- Trajectories
- Geometries
- support polygons
"""
from .symbolic import *
from .symbolic_linalg import *
from .. import *
from . import so3,se3
from ..model import ik,collide
from ..io import loader
import weakref

def _so3_rotation(axis,angle):
    """Symbolic version of so3.rotation"""
    cm = cos(angle)
    sm = sin(angle)

    #m = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
    cp = so3.cross_product(axis)
    R = mul(cp,sm) 
    R2 = [0]*9
    for i in range(3):
        for j in range(3):
            R2[i*3+j] = axis[i]*axis[j]*(1.-cm)
    R2[0] += cm
    R2[4] += cm
    R2[8] += cm
    return R + expr(R2)

class SO3Context(Context):
    """Defines some functions in the so3 module:

    - identity, matrix, inv, mul, apply, rotation, error, distance
    - from_matrix, from_rpy, rpy, from_quaternion, quaternion, from_rotation_vector, rotation_vector
    - eq_constraint: equality constraint necessary for SO3 variables
    - quaternion_constraint: equality constraint necessary for quaternion variables

    Completeness table

    =================  =============  ==============
    Function           Derivative     Simplification 
    =================  =============  ==============
      identity         N/A            N/A             
      matrix           Y              Y               
      inv              Y              Y               
      mul                                             
      apply            Y,Y                            
      rotation         N,Y                            
      from_matrix      Y                              
      from_rpy                                        
      rpy                                             
      from_quaternion  Y                              
      quaternion                                      
      from_rotation_v                                 
      rotation_vector                                 
      axis                                            
      angle            Y                              
      error                                           
      distance                                        
      eq_constraint    Y                              
      quaternion_cons  Y                              
    =================  =============  ==============

    """
    def __init__(self):
        Context.__init__(self)
        self.type = Type('V',9)
        Rvar = Variable("R",self.type)
        Rsymb = VariableExpression(Rvar)
        R1 = Variable("R1",self.type)
        R2 = Variable("R2",self.type)
        V3type = Type('V',3)
        q = Variable('q',Type('V',4))
        pointvar = Variable("point",V3type)
        pointsymb = VariableExpression(pointvar)
        self.identity = self.declare(expr(so3.identity()),"identity",[])
        self.identity.returnType = self.type
        self.identity.description = "The identity rotation"
        M = Variable("M",Type('M',(3,3)))
        self.from_matrix = self.declare(flatten(transpose(M)),"from_matrix",['M'])
        self.from_matrix.returnType = self.type
        self.from_matrix.argTypes = [M.type]
        self.from_matrix.description = "Converts from a 3x3 matrix"
        self.from_matrix.setDeriv(0,lambda M,dM:self.from_matrix(dM),asExpr=True)
        self.matrix = self.declare(expr(so3.matrix(Rsymb)),"matrix",["R"])
        self.matrix.returnType = self.from_matrix.argTypes[0]
        self.matrix.argTypes = [self.from_matrix.returnType]
        self.matrix.addSimplifier(['so3.identity'],(lambda R:eye(3)),pre=True)
        self.matrix.description = "Converts to a 3x3 matrix"
        self.matrix.setDeriv(0,lambda R,dR:self.matrix(dR),asExpr=True)
        self.from_matrix.properties['inverse'] = weakref.proxy(self.matrix)
        self.matrix.properties['inverse'] = weakref.proxy(self.from_matrix)
        self.inv = self.declare(expr(so3.inv(Rsymb)),"inv",["R"])
        self.inv.description = "Inverts a rotation"
        self.inv.setDeriv(0,lambda R,dR:self.inv(dR),asExpr=True)
        self.inv.properties['inverse'] = weakref.proxy(self.inv)
        self.inv.returnType = self.type
        self.inv.argTypes = [self.type]
        self.inv.addSimplifier(['so3.identity'],lambda R:R)
        self.mul = self.declare(so3.mul,"mul")
        self.mul.description = "Inverts a rotation"
        self.mul.returnType = self.type
        self.mul.argTypes = [self.type,self.type]
        self.mul.setDeriv(0,lambda R1,R2,dR1:self.mul(dR1,R2),asExpr=True)
        self.mul.setDeriv(1,lambda R1,R2,dR2:self.mul(R1,dR2),asExpr=True)
        self.mul.addSimplifier(['so3.identity',None],(lambda R1,R2:R2),pre=True)
        self.mul.addSimplifier([None,'so3.identity'],(lambda R1,R2:R1),pre=True)
        self.mul.properties['associative'] = True
        self.apply = self.declare(expr(so3.apply(Rsymb,pointsymb)),"apply",["R","point"])
        self.apply.addSimplifier(['so3.identity',None],(lambda R,point:point),pre=True)
        self.apply.addSimplifier([None,'zero'],(lambda R,point:point),pre=True)
        self.apply.returnType = V3type
        self.apply.argTypes = [self.type,V3type]
        self.apply.autoSetJacobians()
        self.rotation = self.declare(so3.rotation,"rotation")
        self.rotation.returnType = self.type
        self.rotation.argTypes = [V3type,Numeric]
        self.rotation.setDeriv(1,lambda axis,angle:so3.cross_product(axis))
        self.rotation.addSimplifier([None,'zero'],(lambda axis,angle:self.identity),pre=True)
        self.from_rpy = self.declare(so3.from_rpy,"from_rpy")
        self.from_rpy.returnType = self.type
        self.from_rpy.argTypes = [V3type]
        self.rpy = self.declare(so3.rpy,"rpy")
        self.rpy.returnType = self.from_rpy.argTypes[0]
        self.rpy.argTypes = [self.from_rpy.returnType]
        self.from_rpy.properties['inverse'] = weakref.proxy(self.rpy)
        self.rpy.properties['inverse'] = weakref.proxy(self.from_rpy)
        self.from_quaternion = self.declare(expr(so3.from_quaternion([q[0],q[1],q[2],q[3]])),"from_quaternion",["q"])
        self.from_quaternion.returnType = self.type
        self.from_quaternion.argTypes = [Type('V',4)]
        self.quaternion = self.declare(so3.quaternion,"quaternion")
        self.quaternion.returnType = self.from_quaternion.argTypes[0]
        self.quaternion.argTypes = [self.from_quaternion.returnType]
        self.from_quaternion.properties['inverse'] = weakref.proxy(self.quaternion)
        self.quaternion.properties['inverse'] = weakref.proxy(self.from_quaternion)
        self.from_rotation_vector = self.declare(so3.from_rotation_vector,"from_rotation_vector")
        self.from_rotation_vector.returnType = self.type
        self.from_rotation_vector.argTypes = [V3type]
        self.rotation_vector = self.declare(so3.rotation_vector,"rotation_vector")
        self.rotation_vector.returnType = self.from_rotation_vector.argTypes[0]
        self.rotation_vector.argTypes = [self.from_rotation_vector.returnType]
        self.from_rotation_vector.properties['inverse'] = weakref.proxy(self.rotation_vector)
        self.rotation_vector.properties['inverse'] = weakref.proxy(self.from_rotation_vector)
        self.axis = self.declare(unit(self.rotation_vector(Rvar)),"rotation",["R"])
        self.axis.returnType = V3type
        self.axis.argTypes = [self.type]
        self.angle = self.declare(so3.angle,"angle")
        self.angle.returnType = Numeric
        self.angle.argTypes = [self.type]
        def angle_jacobian(R):
            cosangle = (R[0]+R[4]+R[8]-1)*0.5
            angle = arccos(cosangle)
            #dangle / dR[0] = -1.0/sqrt(1-cosangle**2) * dcosangle/dR[0]
            dacos = -1.0/sqrt(1-cosangle**2)
            x = 0.5*dacos
            return expr([[x,0,0,0,x,0,0,0,x]])
        self.angle.setJacobian(0,angle_jacobian,asExpr=True)
        self.error = self.declare(so3.error,"error")
        self.error.returnType = V3type
        self.error.argTypes = [self.type,self.type]
        self.distance = self.declare(self.angle(self.mul(self.inv(R1),R2)),"distance",['R1','R2'])
        self.distance.returnType = Numeric
        self.distance.argTypes = [self.type,self.type]
        self.distance.autoSetJacobians()
        self.distance.properties['nonnegative'] = True
        Rm = self.matrix(Rsymb)
        self.eq_constraint = self.declare(dot(Rm.T,Rm),'eq_constraint',['R'])
        self.quaternion_constraint = self.declare(norm2(q)-1,'quaternion_constraint',['q'])
        def sym_canonical(v):
            x=v[0]
            y=v[1]
            z=v[2]
            one_plus_x = x+1
            return if_(abs_(one_plus_x) <= 1e-5,
                array(x,y,z,-y,x,0,-z,0,0),
                array(x,y,z,-y,x + z*z/one_plus_x,-y*z/one_plus_x,-z,-y*z/one_plus_x,x + y*y/one_plus_x))
        v = Variable("v",V3type)
        self.canonical = self.declare(sym_canonical(v).simplify(),'canonical',['v'])
        self.canonical.argTypes = [V3type]
        self.canonical.returnType = self.type
        self.canonical.autoSetJacobians()



class SE3Context(Context):
    """Defines some functions in the se3 module under the se3 namespace

    - make(R,t): makes an SE3 element from a rotation and a translation (equivalent to list(R,t))
    - identity, homogeneous, matrix (alias for homogeneous), inv, mul, apply
    - from_homogeneous: converts from a 4x4 matrix
    - rotation(T): retrieves the rotation corresponding to T
    - translation(T): retrieves the translation corresponding to T

    Completeness table

    =================  =============  ==============
      Function         Derivative     Simplification  
    =================  =============  ==============
      make             Y              N/A             
      identity         Y                              
      homogeneous      Y                              
      matrix           Y                              
      from_homogeneou  Y                              
      inv              Y                              
      mul                                             
      apply            N,Y                            
      rotation         Y              Y               
      translation      Y              Y               
    =================  =============  ==============

    """
    def __init__(self):
        Context.__init__(self)
        self.Rtype = Type('V',9)
        self.ttype = Type('V',3)
        self.type = Type('L',2,[self.Rtype,self.ttype])
        T = Variable("T",self.type)
        R = Variable("R",self.Rtype)
        t = Variable("t",self.ttype)
        self.make = self.declare(array(R,t),"make",['R','t'])
        self.identity = self.declare(se3.identity,"identity")
        self.homogeneous = self.declare(se3.homogeneous,"homogeneous")
        self.homogeneous.addSimplifier(['se3.identity'],lambda T:eye(4))
        Rinv = so3.inv(T[0])
        self.inv = self.declare(array(Rinv,neg(so3.apply(Rinv,T[1]))),"inv",['T'])
        self.inv.autoSetJacobians()
        self.inv.properties['inverse'] = weakref.proxy(self.inv)
        self.inv.addSimplifier(['se3.identity'],lambda T:T)
        self.mul = self.declare(se3.mul,"mul")
        self.mul.setDeriv(0,lambda T1,T2,dT1:self.mul(dT1,T2),asExpr=True)
        self.mul.setDeriv(1,lambda T1,T2,dT2:self.mul(T1,dT2),asExpr=True)
        self.mul.addSimplifier(['se3.identity',None],(lambda T1,T2:T2),pre=True)
        self.mul.addSimplifier([None,'se3.identity'],(lambda T1,T2:T1),pre=True)
        self.mul.properties['associative'] = True
        pt = Variable('pt',self.ttype)
        self.apply = self.declare(so3.apply(T[0],pt)+T[1],"apply",['T','pt'])
        #self.apply.setDeriv(0,lambda T,pt,dT:array(so3.apply(dT[0],pt),dT[1]))
        #self.apply.setDeriv(1,lambda T,pt,dx:so3.apply(T[0],dx))
        self.apply.addSimplifier([None,'zero'],lambda T,pt:T[1])
        self.apply.addSimplifier(['se3.identity',None],lambda T,pt:pt)
        self.apply.autoSetJacobians()
        self.rotation = self.declare(T[0],"rotation",['T'])
        self.translation = self.declare(T[1],"translation",['T'])
        self.make.returnType = self.type
        self.homogeneous.returnType = Type('M',(4,4))
        self.homogeneous.argTypes = [self.type]
        self.homogeneous.setDeriv(0,lambda T,dT:array([[dT[0][0],dT[0][3],dT[0][6],dT[1][0]],
            [dT[0][1],dT[0][4],dT[0][7],dT[1][1]],
            [dT[0][2],dT[0][5],dT[0][8],dT[1][2]],
            [0.,0.,0.,0.]]),asExpr=True)
        M = Variable("M",Type('M',(4,4)))
        self.from_homogeneous = self.declare(array(array(M[0][0],M[1][0],M[2][0],M[0][1],M[1][1],M[2][1],M[0][2],M[1][2],M[2][2]),array(M[0][3],M[1][3],M[2][3])),'from_homogeneous',['M'])
        self.from_homogeneous.autoSetJacobians()
        self.matrix = self.declare(self.homogeneous(T),"matrix",['T'])
        self.make.autoSetJacobians()
        self.matrix.autoSetJacobians()
        self.rotation.autoSetJacobians()
        self.translation.autoSetJacobians()
        self.identity.returnType = self.type
        self.inv.returnType = self.type
        self.inv.argTypes = [self.type]
        self.mul.returnType = self.type
        self.mul.argTypes = [self.type,self.type]
        self.apply.returnType = self.ttype
        self.apply.argTypes = [self.type,self.ttype]
        self.rotation.returnType = self.Rtype
        self.translation.returnType = self.ttype

class IKContext(Context):
    """Performs operations on IKObjective user data objects.

    Defines the functions

    - link(ikobj): returns the link index of an IKObjective
    - robot(ikobj): returns the RobotModel of an IKObjective
    - targetPos(ikobj): returns the target position of an IKObjective
    - targetRot(ikobj): returns the target rotation of an IKObjective
    - targetXform(ikobj): returns the target transform of an IKObjective
    - localPos(ikobj): returns the local position of an IKObjective
    - worldPos(ikgoal,robot): returns the world position of the IKObjective at the robot's current configuration (same as klampt.worldPos(robot,link(ikobj),localPos(ikobj)))
    - worldRot(ikgoal,robot): returns the world rotation of the IKObjective at the robot's current configuration (same as klampt.worldRot(robot,link(ikobj)))
    - residual(ikgoal,robot): returns the combined position and orientation residual of an IKObjective at the robot's current configuration

    Completeness table
    
    =================  =============  ==============
      Function         Derivative     Simplification  
    =================  =============  ==============
      link             N/A            N/A             
      robot            N/A            N/A             
      targetPos        N/A            N/A             
      targetRot        N/A            N/A             
      targetXform      N/A            N/A             
      localPos         N/A            N/A             
      worldPos         N/A,Y(1)       N/A             
      worldRot         N/A,Y(1)       N/A             
      residual         N/A,Y(1)       N/A             
    =================  =============  ==============

    Y(1): yes, for the first derivative
    """
    def __init__(self):
        Context.__init__(self)
        def _link(ikobj):
            return ikobj.link()
        def _robot(ikobj):
            if not hasattr(ikobj,"robot"): return None
            return ikobj.robot
        def _localPos(ikobj):
            return ikobj.getPosition()[0]
        def _targetPos(ikobj):
            return ikobj.getPosition()[1]
        def _targetRot(ikobj):
            return ikobj.getFixedRotation()
        def _targetXform(ikobj):
            return (ikobj.getFixedRotation(),ikobj.getPosition()[1])
        def _worldPos(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            localPos = ikobj.getPosition()[0]
            return ikobj.robot.link(ikobj.link()).getWorldPosition(localPos)
        def _worldPosJacobian(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            localPos = ikobj.getPosition()[0]
            return robot.link(ikobj.link()).getPositionJacobian(localPos)
        def _worldRot(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            return ikobj.robot.link(ikobj.link()).getTransform()[0]
        def _worldRotJacobian(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            #TODO: convert this to 9 - element form
            return robot.link(ikobj.link()).getOrientationJacobian()
        def _residual(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            s = IKSolver(robot)
            s.add(ikobj)
            return s.getResidual()
        def _residualJacobian(ikobj,robot):
            assert hasattr(ikobj,"robot"),"IKObjective must be initialized with a RobotModel instance"
            assert ikobj.robot.index == robot.index
            s = IKSolver(robot)
            s.add(ikobj)
            jac_active = s.getJacobian()
            active = s.getActiveDofs()
            jac = np.zeros((len(jac_active),robot.numLinks()))
            jac[:,active] = np.array(jac_active)
            return jac
        so3type = SO3Context().type
        se3type = SE3Context().type
        pointType = Type('V',3)
        self.type = Type('IKObjective')
        self.link = Function("link",_link,returnType='I')
        self.declare(self.link)
        self.link.argTypes = [self.type]
        self.link.deriv = 0
        self.robot = Function("robot",_robot,returnType='RobotModel')
        self.declare(self.robot)
        self.robot.argTypes = [self.type]
        self.robot.deriv = 0
        self.localPos = self.declare(_localPos,"localPos")
        self.localPos.returnType = pointType
        self.localPos.argTypes = [self.type]
        self.localPos.deriv = 0
        self.targetPos = self.declare(_targetPos,"targetPos")
        self.targetPos.returnType = pointType
        self.targetPos.argTypes = [self.type]
        self.targetPos.deriv = 0
        self.targetRot = self.declare(_targetRot,"targetRot")
        self.targetRot.returnType = so3type
        self.targetRot.argTypes = [self.type]
        self.targetRot.deriv = 0
        self.targetXform = self.declare(_targetXform,"targetXform")
        self.targetXform.returnType = se3type
        self.targetXform.argTypes = [self.type]
        self.targetXform.deriv = 0
        self.worldPos = self.declare(_worldPos,"worldPos")
        self.worldPos.returnType = pointType
        self.worldPos.argTypes = [self.type,Type('RobotModel')]
        self.worldPos.setJacobian("robot",_worldPosJacobian)
        self.worldRot = self.declare(_worldRot,"worldRot")
        self.worldRot.returnType = so3type
        self.worldRot.argTypes = [self.type,Type('RobotModel')]
        self.worldRot.setJacobian("robot",_worldRotJacobian)
        self.residual = self.declare(_residual,"residual")
        self.residual.returnType = Type('V')
        self.residual.argTypes = [self.type,Type('RobotModel')]
        self.residual.setJacobian("robot",_residualJacobian)

class GeometryContext(Context):
    """Performs operations on Geometry3D user data objects.  Derivatives of geom arguments are taken with
    respect to the geometry transforms.

    Defines the functions:

    - geometry(object): calls the function object.geometry() (e.g., object can be a RobotModelLink)
    - setTransform(geom,T): sets the current transform of the geometry and returns it.
    - setCollisionMargin(geom,margin): sets the current collision of the geometry and returns it.
    - bbox(geom): returns the bounding box of the geometry at its current transform.
    - collision(geom1,geom2): returns True if the geometries are colliding.
    - distance(geom1,geom2): returns the distance between the geometries, and if penetrating and 
      the two geometries support signed distance, returns the negative penetation distance.
    - closestPoints(geom1,geom2): returns the pair of closest points between geom1 and geom2.
    - distancePoint(geom,pt): returns the closest point from geom to pt.
    - closestPoint(geom,pt): returns the closest point to pt on geom.
    - rayCast(geom,src,dir): returns the distance t>=0 along the ray src + t*dir that intersects geom, or
      inf if there is no intersection.
      
    NOT IMPLEMENTED YET

    Completeness table
    
    =================  =============  ==============
    Function           Derivative     Simplification  
    =================  =============  ==============
      setTransform     N/A            N/A             
      setCollisionMar  N/A            N/A             
      bbox                            N/A             
      collision        Y              N/A             
      distance                        N/A             
      closestPoints                   N/A             
      distancePoint                   N/A             
      closestPoint                    N/A             
      rayCast                         N/A             
    =================  =============  ==============

    """
    def __init__(self):
        Context.__init__(self)
        Rtype = Type('V',9)
        ttype = Type('V',3)
        self.Ttype = Type('L',2,[Rtype,ttype])
        self.type = Type('Geometry3D')


class CollideContext(Context):
    """Defines the functions --

    - robotSelfCollision(q,robot): returns True if the robot has a collision at q
    - robotCollision(q,context): returns True if the robot has a collision in the world. Saves a collider into the context
    - robotSelfCollisionFree(q,robot): the opposite of robotSelfCollision
    - robotCollisionFree(q,context): the opposite of robotCollision

    """
    def __init__(self):
        Context.__init__(self)
        def _robotCollision(q,context):
            world = context['world']
            if 'robot' in context:
                robot = context['robot']
            else:
                robot = world.robot(0)
            if 'collider' in context:
                collider = context['collider']
            else:
                collider = collide.WorldCollider(world)
            robot.setConfig(q)
            for c in collider.collisions():
                return True
            return False
        def _robotSelfCollision(q,context):
            world = context['world']
            if 'robot' in context:
                robot = context['robot']
            else:
                robot = world.robot(0)
            if 'collider' in context:
                collider = context['collider']
            else:
                collider = collide.WorldCollider(world)
            robot.setConfig(q)
            for c in collider.robotSelfCollisions(robot):
                return True
            return False
        _q = Variable("q",'V')
        _context = UserDataExpression("context")
        self.robotCollision = Function("robotCollision",_robotCollision,returnType='B')
        self.robotCollision.argTypes = [Type('V'),Type('Context')]
        self.robotSelfCollision = Function("robotSelfCollision",_robotSelfCollision,returnType='B')
        self.robotSelfCollision.argTypes = [Type('V'),Type('Context')]
        self.robotCollisionFree = Function("robotCollisionFree",not_(self.robotCollision(_q,_context)),['q','context'])
        self.robotCollisionFree.argTypes = [Type('V'),Type('Context')]
        self.robotSelfCollisionFree = Function("robotSelfCollisionFree",not_(self.robotSelfCollision(_q,_context)),['q','context'])
        self.robotSelfCollisionFree.argTypes = [Type('V'),Type('Context')]
        self.declare(self.robotCollision)
        self.declare(self.robotSelfCollision)
        self.declare(self.robotCollisionFree)
        self.declare(self.robotSelfCollisionFree)
        assert callable(self.robotCollision.func)
        assert callable(self.robotSelfCollision.func)
        assert isinstance(self.robotCollisionFree.func,Expression)
        assert isinstance(self.robotSelfCollisionFree.func,Expression)
        
class KlamptContext(Context):
    """Includes all Klampt-related functions.

    Namespaces:

    - so3: SO3Context
    - se3: SE3Context
    - ik: IKContext
    - collide: CollideContext
    - [main]: functions to perform kinematics operations on WorldModel, RobotModel, and RobotModelLink user data objects.

    The main namespace defines the functions:

    - robot(world,index): returns the index'th robot in the world
    - rigidObject(world,index): returns the index'th robot in the world
    - terrain(world,index): returns the index'th terrain in the world
    - config(robot): returns the current configuration of the robot or object
    - setConfig(robot,q): sets the current configuration of the robot and returns the robot
    - link(robot,index): returns the index'th link of the robot.  (If you want to use a named string as the index, use const(name))
    - transform(object): gets the current transform of the object
    - setTransform(object,T): sets the current transform of the object and returns the object
    - velocity(robot): returns the current velocity of the robot
    - setVelocity(robot,dq): sets the current velocity of the robot and returns the robot
    - worldPos(robot,link,localPos): returns the world position of the point on the given link in the robot's current configuration.
    - localPos(robot,link,worldPos): returns the local position of the point on the given link in the robot's current configuration.
    - worldRot(robot,link): returns the rotation matrix of the given link in the robot's current configuration.
    - com(robot): returns the center of mass of robot at its current configuration
    - gravityTorque(gravity,robot): returns the generalized gravity vector
    - inJointLimits(q,robot): returns True if q is in the robot's joint limits
    - getJson(object,path): returns the value of a given path under the given object's json representation.  For example,
      getJson(ikobjective,const("endPosition[0]")) retrieves the x coordinate of the target position of an IKObjective.
    - setJson(object,path,val): returns a modified copy of the given object, where value is assigned to the given path under 
      the objects json representation.  For example, setJson(ikobjective,const("endPosition[0]"),p) sets the x coordinate of the target
      position of an IKObjective to p. Note: this operation returns a copy of the object, modified.

    Also includes modules linalg, so3, se3, ik, and collide

    ================   =============  ==============
    Function           Derivative     Simplification 
    ================   =============  ==============
      robot            N/A            N/A           
      rigidObject      N/A            N/A           
      terrain          N/A            N/A           
      config           Y              N/A           
      setConfig        Y              N/A           
      link             N/A            N/A           
      transform                       N/A           
      setTransform                    N/A           
      velocity                        N/A           
      setVelocity                     N/A           
      worldPos         Y(1)           N/A           
      localPos                        N/A           
      worldRot                        N/A           
      com              Y(1)           N/A           
      gravityTorque                   N/A           
      inJointLimits    N/A            N/A           
      getJson          N/A            N/A           
      setJson          N/A            N/A          
    ================   =============  ==============

    Y(1): yes, for the first derivative
    """
    def __init__(self,world=None):
        Context.__init__(self)
        if world:
            self.addUserData("world",world)
            if world.numRobots() > 0:
                self.addUserData("robot",world.robot(0))
        self.include(LinAlgContext(),"linalg",modify=True)
        self.include(SO3Context(),"so3",modify=True)
        self.include(SE3Context(),"se3",modify=True)
        self.include(IKContext(),"ik",modify=True)
        self.include(CollideContext(),"collide",modify=True)
        def robot(world,index):
            return world.robot(index)
        def rigidObject(world,index):
            return world.rigidObject(index)
        def terrain(world,index):
            return world.terrain(index)
        def config(robot):
            return robot.getConfig()
        def setConfig(robot,q):
            robot.setConfig(q)
            return robot
        def link(robot,index):
            return robot.link(index)
        def transform(object):
            return object.getConfig()
        def setTransform(object,T):
            object.setTransform(*T)
            return object
        def velocity(robot):
            return robot.getVelocity()
        def setVelocity(robot,dq):
            robot.setVelocity(dq)
            return robot
        def worldPos(robot,link,localPos):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
                assert link.index >= 0
            return link.getWorldPosition(localPos)
        def worldPosJacobian_robot(robot,link,localPos,dq):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
                assert link.index >= 0
            return link.getPositionJacobian(localPos)
        def worldPosJacobian_localPos(robot,link,localPos):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
                assert link.index >= 0
            return so3.matrix(link.getTransform()[0])
        def localPos(robot,link,worldPos):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
            return link.getLocalPosition(worldPos)
        def worldRot(robot,link):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
            return link.getTransform()[0]
        def worldRotJacobian(robot,link):
            if not isinstance(link,RobotModelLink):
                link = robot.link(link)
            #TODO: convert this to 9 - element form
            return link.getOrientationJacobian()
        def com(robot):
            return robot.getCom()
        def comJacobian(robot):
            sumMass = 0.0
            sumJacobian = np.zeros((3,robot.numLinks()))
            for i in range(robot.numLinks()):
                mass = robot.link(i).getMass()
                sumMass += mass.getMass()
                if mass.getMass() > 0:
                    sumJacobian += np.array(robot.link(i).getPositionJacobian(mass.getCom()))*mass.getMass()
            return sumJacobian*(1.0/sumMass)
        def gravityTorque(gravity,robot):
            return robot.getGravityForces(gravity)
        #def inJointLimits(q,robot):
        #    qmin,qmax = robot.getJointLimits()
        #    for v,a,b in zip(q,qmin,qmax):
        #        if v < a or v > b: return False
        #    return True
        #def inJointLimits_simplifier(q,robot):
        #    if isinstance(robot,RobotModel):
        #        qmin,qmax = robot.getJointLimits()
        #        return bound_contains(qmin,qmax,q)
        #    return None
        def str_to_path(s):
            res = s.split('.[]')
            for i,v in enumerate(res):
                try:
                    res[i] = int(v)
                except Exception:
                    pass
            return res
        def getJson(object,path):
            assert isinstance(path,(int,str,tuple,list))
            if isinstance(object,(list,dict)):
                jsonobj = object
            else:
                jsonobj = loader.toJson(object)
            if isinstance(path,str):
                path = str_to_path(path)
                return getJson(jsonobj,path)
            elif isinstance(path,int):
                return jsonobj[path]
            else:
                for item in path:
                    jsonobj = jsonobj[item]
                return jsonobj
        def setJson(object,path,val):
            assert isinstance(path,(int,str,tuple,list))
            if isinstance(object,(list,dict)):
                jsonobj = object
            else:
                jsonobj = loader.toJson(object)
            if isinstance(path,str):
                path = str_to_path(path)
                return setJson(jsonobj,path,val)
            elif isinstance(path,int):
                jsonobj[path] = val
            else:
                root = jsonobj
                for item in path[:-1]:
                    root = root[item]
                root[path[-1]] = val
            return loader.fromJson(jsonobj)

        self.worldType = Type('WorldModel')
        self.robotType = Type('RobotModel')
        self.rigidObjectType = Type('RigidObjectModel')
        self.terrainType = Type('TerrainModel')
        self.linkType = Type('RobotModelLink')
        self.configType = Type('V')
        self.pointType = Type('V',3)
        self.robot = self.declare(robot)
        self.robot.returnType = self.robotType
        self.robot.argTypes = [self.worldType,Integer]
        self.rigidObject = self.declare(rigidObject)
        self.rigidObject.returnType = self.rigidObjectType
        self.rigidObject.argTypes = [self.worldType,Integer]
        self.terrain = self.declare(terrain)
        self.terrain.returnType = self.terrainType
        self.terrain.argTypes = [self.worldType,Integer]
        self.config = self.declare(config)
        self.config.returnType = self.configType
        self.config.argTypes = [self.robotType]
        self.config.setDeriv('robot',(lambda robot,drobot:drobot),asExpr=True,stackable=True)
        self.setConfig = self.declare(setConfig)
        self.setConfig.returnType = self.robotType
        self.setConfig.argTypes = [self.robotType,self.configType]
        self.setConfig.setDeriv('robot',0)
        self.setConfig.setDeriv('q',(lambda robot,q,dq:dq),asExpr=True,stackable=True)
        self.link = self.declare(link)
        self.link.returnType = self.linkType
        self.link.argTypes = [self.robotType,Type('I')]
        self.transform = self.declare(transform)
        self.transform.returnType = self.se3.type
        self.transform.setDeriv('object',(lambda object,dobject:dobject),asExpr=True)
        self.setTransform = self.declare(setTransform)
        self.setTransform.argTypes = [Type('U'),self.se3.type]
        self.setTransform.returnType = Type('U')
        self.setTransform.setDeriv('object',0)
        self.setTransform.setDeriv('T',(lambda object,T,dT:dT),asExpr=True)
        self.velocity = self.declare(velocity)
        self.velocity.returnType = self.configType
        self.velocity.argTypes = [self.robotType]
        self.setVelocity = self.declare(setVelocity)
        self.setVelocity.returnType = self.robotType
        self.setVelocity.argTypes = [self.robotType,self.configType]
        self.worldPos = self.declare(worldPos)
        self.worldPos.returnType = self.pointType
        self.worldPos.argTypes = [self.robotType,Type('I'),self.pointType]
        self.worldPos.setJacobian('robot',worldPosJacobian_robot)
        self.worldPos.setJacobian('localPos',worldPosJacobian_localPos)
        self.localPos = self.declare(localPos)
        self.localPos.returnType = self.pointType
        self.localPos.argTypes = [self.robotType,Type('I'),self.pointType]
        self.worldRot = self.declare(worldRot)
        self.worldRot.returnType = self.so3.type
        self.worldRot.argTypes = [self.robotType,Type('I')]
        self.gravityTorque = self.declare(gravityTorque)
        self.com = self.declare(com)
        self.com.returnType = self.pointType
        self.com.argTypes = [self.robotType]
        self.com.setJacobian('robot',comJacobian)
        self.gravityTorque = self.declare(gravityTorque)
        self.gravityTorque.argTypes = [self.pointType,self.robotType]
        jl = getattr_(UserDataExpression("robot"),const("getJointLimits"))
        self.inJointLimits = Function("inJointLimits",bound_contains(jl[0],jl[1],Variable("q",'V')),["q","robot"])
        #self.inJointLimits = self.declare(inJointLimits)
        #self.inJointLimits.returnType = Type('B')
        #self.inJointLimits.argTypes = [self.configType,self.robotType]
        #self.inJointLimits.simplifier = inJointLimits_simplifier
        self.getJson = self.declare(getJson)
        self.setJson = self.declare(setJson)
        
if __name__ == '__main__':
    world = WorldModel()
    world.readFile("../../../data/athlete_plane.xml")
    ctx = Context()
    ctx.include(KlamptContext(world),"klampt")
    ctx.listFunctions(builtins=True)
    #TODO: analytical derivatives
    print("Context:",ctx.userData)
    q = ctx.addVar('q','V',world.robot(0).numLinks())
    jointLimitTest = ctx.klampt.inJointLimits(q,'klampt.robot')
    comEval = ctx.klampt.com(q,'klampt.robot')
    print(jointLimitTest)
    print(comEval)
    newcontext = ctx.userData.copy()
    newcontext['q'] = world.robot(0).getConfig()
    print(deriv(comEval,q).eval(newcontext))
    qfunc,qvars = ctx.makeVectorFunction(jointLimitTest)
    cmfunc,cmvars = ctx.makeVectorFunction(comEval)
    print(qfunc(world.robot(0).getConfig()))
    print(cmfunc(world.robot(0).getConfig()))
