from online_leastsq import OnlineLeastSquares
import numpy as np
import math

class LinearSystemID:
    """System identification for a system y = Ax + Bu + C with m state
    variables and n inputs."""
    def __init__(self,m,n):
        self.m,self.n = m,n
        self.coeffPattern = [None,None,None]
        self.estimators = [OnlineLeastSquares(self.m+self.n+1) for i in xrange(m)]

    def setPattern(self,Apattern,Bpattern,Cpattern):
        """The patterns are list-of-lists of size mxm, mxn, and a list of
        size m, which indicate whether the (i,j)'th entry of the A,B, and C
        matrices are fixed (respectively).  A None entry indicates a free
        coefficient, while a numeric entry indicates a fixed coefficient.
        """
        self.coeffPattern = [Apattern,Bpattern,Cpattern]
        for i in xrange(self.m):
            self._updateEstimatorSize(i)

    def fixA(self,i,j,value):
        """Sets the i,j'th entry of the A matrix to a fixed value"""
        if self.coeffPattern[0] == None:
            m,n=self.m,self.n
            self.coeffPattern[0] = [[None]*m for i in xrange(m)]
        self.coeffPattern[0][i][j]=value
        self._updateEstimatorSize(i)

    def fixB(self,i,j,value):
        """Sets the i,j'th entry of the B matrix to a fixed value"""
        if self.coeffPattern[1] == None:
            m,n=self.m,self.n
            self.coeffPattern = [[None]*n for i in xrange(m)]
        self.coeffPattern[1][i][j]=value
        self._updateEstimatorSize(i)

    def fixC(self,i,value):
        """Sets the i'th entry of the C vector to a fixed value"""
        if self.coeffPattern[2] == None:
            m,n=self.m,self.n
            self.coeffPattern[2] = [None]*m
        self.coeffPattern[2][i]=value
        self._updateEstimatorSize(i)

    def add(self,x,u,y,weight=1.0):
        """Adds a new datapoint to the estimator"""
        assert(len(y)==self.m)
        assert(len(x)==self.m)
        assert(len(u)==self.n)
        if isinstance(x,np.ndarray): x = x.tolist()
        if isinstance(u,np.ndarray): u = u.tolist()
        xu1 = x + u + [1.0]
        if self.coeffPattern == [None,None,None]:
            for yi,e in zip(y,self.estimators):
                e.add(xu1,yi,weight)
        else:
            #each row might have some fixed values
            for i,(yi,e) in enumerate(zip(y,self.estimators)):
                if e == None: continue
                (xuc,constOffset) = self._toEstimator(i,x,u)
                rhs = yi - constOffset
                e.add(xuc,rhs,weight)
        return

    def discount(self,discountFactor,type='geometric'):
        """Reduces the effects of prior readings."""
        for e in self.estimators:
            e.discount(discountFactor,type)
        return

    def setModelPrior(self,A,B,C,priorWeight):
        """Adds in a prior belief for the model.
        Must be called AFTER fixing coefficients and BEFORE adding any
        datapoints."""
        Cpattern = self.coeffPattern[2]
        for i in xrange(self.m):
            ai = A[i,:].tolist()
            bi = B[i,:].tolist()
            (xuc,constant) = self._toEstimator(i,ai,bi)
            if Cpattern == None or Cpattern[i] == None:
                xuc[-1] = C[i]
            self.estimators[i].setPrior(np.array(xuc),priorWeight)
        return

    def getModel(self):
        """Returns the estimated triple (A,B,C) as numpy arrays"""
        m,n = self.m,self.n
        A = np.zeros((m,m))
        B = np.zeros((m,n))
        C = np.zeros(m)
        Apattern,Bpattern,Cpattern = self.coeffPattern
        for i,e in enumerate(self.estimators):
            aofs = 0
            bofs = m
            cofs = m+n
            if Apattern==None:
                ai = e.x[aofs:m+aofs]
            else:
                bofs=aofs
                ai = []
                for j,pj in enumerate(Apattern[i]):
                    if pj == None:
                        ai.append(e.x[bofs])
                        bofs += 1
                    else:
                        ai.append(pj)
            if Bpattern==None:
                bi = e.x[bofs:n+bofs]
            else:
                cofs=bofs
                bi = []
                for j,pj in enumerate(Bpattern[i]):
                    if pj == None:
                        bi.append(e.x[cofs])
                        cofs += 1
                    else:
                        bi.append(pj)
            if Cpattern==None:
                ci = e.x[cofs]
                cofs+=1
            else:
                if Cpattern[i] == None:
                    ci = e.x[cofs]
                    cofs+=1
                else:
                    ci = Cpattern[i]
            assert(cofs == e.n)
            assert len(ai)==m
            assert len(bi)==n
            A[i,:] = ai
            B[i,:] = bi
            C[i] = ci
        return (A,B,C)
    
    def getOutput(self,x,u):
        """Returns the estimate A*x+B*u+c"""
        assert(len(x)==self.m)
        assert(len(u)==self.n)
        if isinstance(x,np.ndarray): x = x.tolist()
        if isinstance(u,np.ndarray): u = u.tolist()
        dx = []
        if self.coeffPattern == [None,None,None]:
            xuc = np.array(x + u + [1.0])
            for e in self.estimators:
                dx.append(np.dot(e.x,xuc))
        else:
            for i,e in enumerate(self.estimators):
                (xuc,constOffset) = self._toEstimator(i,x,u)
                dx.append(np.dot(e.x,xuc)+constOffset)
        return dx

    def _updateEstimatorSize(self,index):
        """Helper."""
        Apattern,Bpattern,Cpattern = self.coeffPattern
        m,n = self.m,self.n
        numFixed = 0
        if Apattern!=None:
            numFixed += len(v for v in Apattern[index] if v != None)
        if Bpattern!=None:
            numFixed += len(v for v in Bpattern[index] if v != None)
        if Cpattern!=None:
            if Cpattern[index]!=None:
                numFixed += 1
        if numFixed==m+n+1:
            self.estimators[index]=None
        else:
            self.estimators[index]=OnlineLeastSquares(m+n+1-numFixed)
        return

    def _toEstimator(self,index,x,u):
        """Helper: Projects x,u to the pattern xe taken by the index'th
        estimator. Returns the pair (xe,constant offset) where the index'th
        row of Ax+Bu+C is equal to dot(xe,estimator.coeffs) + constOffset."""
        Apattern,Bpattern,Cpattern = self.coeffPattern
        xuc = []
        constOffset = 0
        if Apattern == None:
            xuc += x
        else:
            xuc += [xj for (xj,pj) in zip(x,Apattern[index]) if pj == None]
            constOffset += sum([xj*pj for (xj,pj) in zip(x,Apattern[index]) if pj != None])
        if Bpattern == None:
            xuc += u
        else:
            xuc += [uj for (uj,pj) in zip(u,Bpattern[index]) if pj == None]
            constOffset += sum([uj*pj for (uj,pj) in zip(u,Bpattern[index]) if pj != None])
        if Cpattern == None:
            xuc += [1.0]
        else:
            constOffset = Cpattern[index]
        return (xuc,constOffset)


def testLinearSystemID(A,B,C,N,xScale=1,uScale=1):
    m,n = B.shape
    sysid = LinearSystemID(m,n)
    print "Actual model:",
    print "A:"
    print A
    print "B:"
    print B
    print "C:"
    print C

    X = [np.random.rand(m)*xScale*2-np.ones(m)*xScale for i in range(N)]
    U = [np.random.rand(n)*uScale-np.ones(n)*uScale for i in range(N)]
    for x,u in zip(X,U):
        y = np.dot(A,x)+np.dot(B,u)+C
        sysid.add(x,u,y)
    print "Estimated model:",
    eA,eB,eC=sysid.getModel()
    print "A:"
    print eA
    print "B:"
    print eB
    print "C:"
    print eC
    resid = 0.0
    for x,u in zip(X,U):
        y = np.dot(A,x)+np.dot(B,u)+C
        ey = np.dot(eA,x)+np.dot(eB,u)+C
        #print "Error",np.dot(ey,ey)
        resid += np.dot(ey-y,ey-y)
    print "RMSE",math.sqrt(resid/len(X))

def testNonlinearSystemID(func,m,n,N,discountParams,x0,uScale=1):
    sysid = LinearSystemID(m,n)
    sysid.add(x0,np.zeros(n),x0)

    if not hasattr(discountParams,'__iter__'):
        discountParams = (discountParams,'geometric')

    #simulate a random trace
    X = [x0]
    U = []
    for i in xrange(N):
        u = np.random.rand(n)*uScale*2.0 - np.ones(n)*uScale
        U.append(u)
        X.append(func(X[-1],u))
    resid = 0.0
    discresid = 0.0
    errors = []
    for x,u,xn in zip(X[:-1],U,X[1:]):
        xpred = sysid.getOutput(x,u)
        e2 = np.dot(xn-xpred,xn-xpred)
        errors.append(e2)
        resid += e2
        discount = discountParams[0] if discountParams[1]=='geometric' else (1.0/discountParams[0]-1)/sysid.estimators[0].sumWeight
        discresid = discresid*discount + e2
        sysid.discount(*discountParams)
        sysid.add(x,u,xn)
    print "Nonlinear estimation with discount",discountParams
    print "Final estimated model:",
    eA,eB,eC=sysid.getModel()
    print "A:"
    print eA
    print "B:"
    print eB
    print "C:"
    print eC
    print "RMSE",math.sqrt(resid/(len(X)-1))
    print "Discounted squared errors",discresid
    #print "Errors",errors
    
        
if __name__=='__main__':
    print "Testing standard double integrator with offset"""
    mass = 5.0
    f = 1.0
    A = np.array([[0,1],[0,0]])
    B = np.array([[0],[1/mass]])
    C = np.array([0,f/mass])
    testLinearSystemID(A,B,C,100)

    def ABCFunc(x,u):
        dt = 0.1
        return x + dt*(np.dot(A,x)+np.dot(B,u)+C)
    x0 = np.array([0,0])
    """
    testNonlinearSystemID(ABCFunc,2,1,100,1.0,x0)
    testNonlinearSystemID(ABCFunc,2,1,100,0.95,x0)
    testNonlinearSystemID(ABCFunc,2,1,100,0.9,x0)
    testNonlinearSystemID(ABCFunc,2,1,100,0.8,x0)
    exit(1)
    """

    print "Testing damped pendulum with gravity"""
    def pendulumFunc(x,u):
        theta,dtheta = x
        torque = u[0]
        g = -9.8
        dt = 0.1
        mass = 1
        kD = 0.5
        ddtheta = torque/mass + math.sin(theta)*g - kD*dtheta
        return np.array([theta+dt*dtheta,dtheta+dt*ddtheta])
    x0 = np.array([math.pi*0.5,0])
    testNonlinearSystemID(pendulumFunc,2,1,1000,1.0,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.99,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.95,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.01,'hyperbolic'),x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.05,'hyperbolic'),x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.1,'hyperbolic'),x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.2,'hyperbolic'),x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.5,'hyperbolic'),x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,(0.75,'hyperbolic'),x0)
    """
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.9,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.8,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.7,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.5,x0)
    testNonlinearSystemID(pendulumFunc,2,1,1000,0.25,x0)
    """
    print "Actual:"
    print "[q' ] = [ 1, 0.1 ]*[q ] + [0  ]*u + [0                    ]"
    print "[dq']   [ 0,   1 ] [dq]   [0.1]     [-.98 sin(q) - 0.05 dq]"
