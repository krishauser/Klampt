from klampt import vectorops
from controller import BaseController,LambdaController
from MotionModel import AdaptiveMotionModel

class VelocityEstimator(BaseController):
    """An estimator that runs by computing the velocity using finite
    differences."""
    def __init__(self,robot=None):
        self.robot = robot
        self.qlast = None
    def signal(self,type,**inputs):
        if type=='reset':
            self.qlast = None
            pass
        return
    def output_and_advance(self,**inputs):
        try:
            dt = inputs["dt"]
            q = inputs["q"]
        except KeyError:
            raise ValueError("Input needs to have configuration 'q' and timestep 'dt'")
        if len(q)==0: return None
        if self.qlast==None:
            dq = [0]*len(q)
        else:
            if self.robot==None:
                dq = vectorops.div(self.robot.sub(q,self.qlast),dt)
            else:
                assert(len(self.qlast)==len(q))
                dq = vectorops.div(self.robot.interpolate_deriv(self.qlast,q),dt)
        self.qlast = q
        return {'dq':dq}

class DifferenceEstimator(LambdaController):
    def __init__(self,arg1,arg2):
        def diff(x,y):
            return vectorops.sub(x,y)
        LambdaController.__init__(self,diff,[arg1,arg2],arg1+" - "+arg2)

class SystemIDEstimator(BaseController):
    """An estimator that runs by updating the adaptive motion model object.
    It does not output anything."""
    def __init__(self,robot,estimator = 'default'):
        self.estimator = estimator
        if estimator == 'default':
            self.estimator = AdaptiveMotionModel('velocity','velocity')
        self.xnames = ['q','dq']
        self.unames = ['dqcmd']
        self.qlast = None
        self.discount = None
        
    def getMotionModel(self):
        return self.estimator

    def signal(self,type,**inputs):
        if type=='reset':
            self.xlast = None
            #TODO: reset the motion model object?
            pass
        return

    def advance(self,**inputs):
        try:
            qcmd = inputs['qcmd']
            q = inputs['q']
            dt = inputs['dt']
        except KeyError:
            return
        if len(q)==0: return
        print len(qcmd),len(q)
        u = vectorops.sub(qcmd,q)
        u = vectorops.div(u,dt)
        print "u estimate:",u
        print "u estimate rarm:",u[26:33]

        try:
            q = inputs[self.xnames[0]]
            dq = inputs[self.xnames[1]]
        except KeyError as e:
            print "Warning, inputs do not contain x key",e
            return
        """
        try:
            u = sum((inputs[uname] for uname in self.unames),[])
        except KeyError as e:
            print "Warning, inputs do not contain u key",e
            #u = [0]*(len(x)/2)
            try:
                u = vectorops.sub(inputs['qcmd'],inputs['q'])
                u = vectorops.div(u,inputs['dt'])
            except KeyError as e:
                print "Warning, inputs do not contain qcmd key either",e
                u = [0]*(len(x)/2)
            """
            #self.xlast = x
            #return
        #do system ID
        if self.qlast != None:
            self.estimator.add(self.qlast,self.dqlast,u,q,dq)
            pass
        #update last state
        self.qlast = q
        self.dqlast = dq
        return

