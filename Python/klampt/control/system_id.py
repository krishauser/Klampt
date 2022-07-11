import numpy as np
import math

def check_symmetry(A,tol=1e-7):
    err = np.amax(np.abs(A - A.T))
    if err >= tol:
        print("Error in matrix symmetry? error %f"%(err,))
        return False
    return True

def sherman_woodbury_inplace(Ainv,v,w,c=1.0):
    """Computes (A+vcw^T)^-1 given A^-1, w, and w.
    Does it inplace without creating a temp matrix"""
    if v is w:
        temp = np.dot(Ainv,v)
        den = 1.0+np.dot(v,temp)*c
        scale = c/den
        for i in range(Ainv.shape[0]):
            Ainv[i,:] -= (scale*temp[i])*temp
    else:
        AinvV = np.dot(Ainv,v)
        AinvW = np.dot(Ainv,w)
        scale = c/(1.0+np.dot(v,AinvW)*c)
        for i in range(Ainv.shape[0]):
            Ainv[i,:] -= (scale*AinvV[i])*AinvW


class OnlineLeastSquares:
    """Maintains solutions x to the L2-regularized least squares problem
    min ||Ax-b||^2 + lambda ||x||^2 with dynamic updates to the rows
    of the A matrix and b vector.

    If the problem is degenerate, the degenerate flag is set to True and
    the minimum norm ||x||^2 solution that satisfies Ax=b is returned.

    For numerical stability, this class internally monitors the norm of
    self.AtA and rescales it periodically.  It maintains a parameter
    and all the parameters are scaled by self.scale
    - self.AtA = At*A / self.scale
    - self.Atb = At*b / self.scale
    - self.btb = bt*b / self.scale
    - self.AtAinv = (At*A)^-1 * self.scale
    By default, when ||self.AtA|| > 1e3, rescaling is performed.
    """
    def __init__(self,n,regularizationLambda=0):
        self.n = n
        self.count = 0
        self.sumWeight = 0
        self.scale = 1
        self.rescaleThreshold = 1e1
        self.AtA = np.zeros((n,n))
        self.Atb = np.zeros(n)
        self.btb = 0
        self.AtAinv = np.zeros((n,n))
        self.degenerate = True
        self.x = np.zeros(n)
        self.regularizationLambda = regularizationLambda
        if regularizationLambda > 0:
            self.AtA = np.eye(n)*regularizationLambda
            self.AtAinv = np.eye(n)*(1.0/regularizationLambda)
            self.degenerate = False
            self.sumWeight = 0

    def setPrior(self,xPrior,priorWeight):
        """Can set a prior belief on x.  Works similarly to regularization
        but MUST be done before adding points. """
        assert self.regularizationLambda == 0
        if not isinstance(xPrior,np.ndarray):
            xPrior = np.array(xPrior)
        self.count = 1
        self.sumWeight = priorWeight
        self.scale = 1
        self.AtA = np.eye(self.n)*priorWeight
        self.AtAinv = np.eye(self.n)/priorWeight
        self.Atb = xPrior*priorWeight
        self.btb = np.dot(xPrior,xPrior)*priorWeight
        self.degenerate = False
        self.x = xPrior
            
    def solution(self):
        return self.x

    def residualNorm2(self):
        """Returns the squared norm of the residual ||Ax-b||^2.  Uses the
        formula ||Ax-b||^2 = x^T At A x - 2 x^T At b + b^T b"""
        r2 = (np.dot(self.x,np.dot(self.AtA,self.x)-2.0*self.Atb) + self.btb)*self.scale
        if self.regularizationLambda > 0:
            r2 -= self.regularizationLambda*np.dot(self.x,self.x)
        return r2

    def residualNorm(self):
        """Returns the norm of the residual ||Ax-b||.  """
        return math.sqrt(self.residualNorm2())

    def standardError2(self):
        """Returns the square of the OLS estimate of the standard error."""
        if self.count<=self.n:
            return float('inf')
        return self.residualNorm2()/self.sumWeight*(self.count / (self.count-self.n))

    def standardError(self):
        """Returns the OLS estimate of the standard error."""
        return math.sqrt(self.standardError2())

    def solutionCovariance(self):
        """Returns the covariance matrix of the solution x"""
        return self.standardError2()*self.AtAinv

    def solutionStandardErrors(self):
        """Returns the standard errors of the solution x"""
        s2 = self.standardError2()
        res = [0]*self.n
        for i in range(self.n):
            try:
                res[i] = math.sqrt(s2*self.AtAinv[i,i])
            except ValueError:
                res[i] = float('nan')
        return res

    def predictionAndVariance(self,a):
        """Returns the mean and variance of the prediction at the input
        point a."""
        b = np.dot(a,self.x)
        vc = self.standardError2()
        #x is distributed according to a gaussian with mean self.x and
        #variance solutionCovariance.  Dot product has variance
        #Var(a^T x) = a^T Var(x) a
        #add on the 
        return (b, vc * (1.0 + np.dot(a,np.dot(self.AtAinv,a))))
        
    def setLambda(self,regularizationLambda):
        """Changes the regularization term (and maintains the solution).
        Cost is O(n^3) """
        self.AtA *= self.scale
        self.AtA += (regularizationLambda-self.regularizationLambda)*np.eye(self.n)
        self.AtA /= self.scale
        try:
            self.AtAinv = np.linalg.inv(self.AtA)
            self.degenerate = False
        except np.linalg.LinAlgError:
            self.AtAinv = np.linalg.pinv(self.AtA)
            self.degenerate = True
        self.x = np.dot(self.AtAinv,self.Atb)
        self.regularizationLambda = regularizationLambda
        
    def add(self,a,b,weight=1.0):
        """Adds a new datapoint a^T x ~= b, weighted by the given weight,
        and updates the solution x accordingly.  The update runs in O(n^2)
        time."""
        assert len(a)==self.n
        assert self.AtA.shape==(self.n,self.n)
        assert self.AtAinv.shape==(self.n,self.n)
        assert self.Atb.shape==(self.n,)
        assert weight >= 0
        if not isinstance(a,np.ndarray):
            a = np.array(a)
        self.count += 1
        w = weight/self.scale
        self.sumWeight += weight
        self.Atb += (w*b)*a
        self.btb += w*b*b
        for i in range(self.n):
            self.AtA[i,:] += (w*a[i])*a
        #assert check_symmetry(self.AtA)
        if not self.degenerate:
            #sherman woodbury update
            sherman_woodbury_inplace(self.AtAinv,a,a,w)
            assert check_symmetry(self.AtAinv)
        else:
            self.calc_AtAinv()
        self.x = np.dot(self.AtAinv,self.Atb)
        self.checkRescale()

    def checkRescale(self):
        n = np.linalg.norm(self.AtA)
        if n > self.rescaleThreshold:
            self.scale *= n
            self.AtA *= 1.0/n
            self.Atb *= 1.0/n
            self.btb *= 1.0/n
            self.calc_AtAinv()

    def discount(self,discountFactor,type='geometric'):
        """Discounts the existing data by the given discount factor (should
        be < 1).  Runs in time O(n^2) if the regularization weight is zero,
        otherwise it is O(n^3). Future implementations might get this down
        to O(n^2) in the latter case.
        
        hyperbolic discounting takes the factor as a sort of drifting
        contribution to the average"""
        assert(discountFactor > 0)
        if type == 'hyperbolic':
            #avg' = avg*(1-alpha(N)) + alpha(N)*item
            #if alpha(N) ~= 1/(N+1) then this converges to the true average
            #sum' = (N+1)/N sum*(1-alpha(N)) + alpha(N)(N+1)*item
            #sum' = sum*(1-alpha(N))/(N alpha(N)) + item
            #if alpha is a constant, we get
            #sum' = sum*(1/alpha-1)/N + item
            self.discount((1.0/discountFactor - 1.0)/self.sumWeight)
            return
        if self.regularizationLambda == 0:
            self.scale *= discountFactor
            self.sumWeight *= discountFactor
        else:
            AtAreg = np.eye(self.n)*self.regularizationLambda/self.scale
            self.AtA = self.AtA*discountFactor + AtAreg
            self.Atb *= discountFactor
            self.btb *= discountFactor
            self.sumWeight *= discountFactor
            self.calc_AtAinv()
            self.x = np.dot(self.AtAinv,self.Atb)
        return

    def calc_AtAinv(self):
        assert check_symmetry(self.AtA)
        if self.count < self.n:
            self.AtAinv = np.linalg.pinv(self.AtA)
            self.degenerate = True    
        else:
            try:
                self.AtAinv = np.linalg.inv(self.AtA)
                if check_symmetry(self.AtAinv):
                    #good solution
                    self.degenerate = False
                else:
                    self.AtAinv = np.linalg.pinv(self.AtA)
                    self.degenerate = True
                    print("OnlineLS: degenerate matrix, inv failed to produce symmetric matrix")
            except np.linalg.LinAlgError:
                self.AtAinv = np.linalg.pinv(self.AtA)
                self.degenerate = True
                print("OnlineLS: degenerate matrix")
        return


class LinearSystemID:
    """System identification for a system y = Ax + Bu + C with m state
    variables and n inputs.

    Usage first sets up the matrix structure (optional), then repeatedly calls
    add(x,u,y).  To retrieve the matrices, call :meth:`getModel`.  To retrieve 
    a prediction, call :meth:`getOutput`

    Supports freezing certain entries of the the matrix structure. This is done
    using 3 pattern matrices, where None indicates a free value and a numeric
    constant indicates a fixed value.  The patterns must be frozen before
    datapoints are consumed.
    """
    def __init__(self,m,n):
        self.m,self.n = m,n
        self.coeffPattern = [None,None,None]
        self.estimators = [OnlineLeastSquares(self.m+self.n+1) for i in range(m)]

    def setPattern(self,Apattern,Bpattern,Cpattern):
        """The patterns are list-of-lists of size mxm, mxn, and a list of
        size m, which indicate whether the (i,j)'th entry of the A,B, and C
        matrices are fixed (respectively). 

        If Xpattern[i][j] = None, this indicates a free coefficient, but if
        Xpattern[i][j] is numeric, it indicates that the coefficient is fixed
        at that value.
        """
        self.coeffPattern = [Apattern,Bpattern,Cpattern]
        for i in range(self.m):
            self._updateEstimatorSize(i)

    def fixA(self,i,j,value):
        """Sets the i,j'th entry of the A matrix to a fixed value"""
        if self.coeffPattern[0] == None:
            m,n=self.m,self.n
            self.coeffPattern[0] = [[None]*m for i in range(m)]
        self.coeffPattern[0][i][j]=value
        self._updateEstimatorSize(i)

    def fixB(self,i,j,value):
        """Sets the i,j'th entry of the B matrix to a fixed value"""
        if self.coeffPattern[1] == None:
            m,n=self.m,self.n
            self.coeffPattern = [[None]*n for i in range(m)]
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
        for i in range(self.m):
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

