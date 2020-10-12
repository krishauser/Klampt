import numpy as np
import math

def check_symmetry(A,tol=1e-7):
    err = np.amax(np.abs(A - A.T))
    if err >= tol:
        print "Error in matrix symmetry? error %f"%(err,)
        return False
    return True

def sherman_woodbury_inplace(Ainv,v,w,c=1.0):
    """Computes (A+vcw^T)^-1 given A^-1, w, and w.
    Does it inplace without creating a temp matrix"""
    if v is w:
        temp = np.dot(Ainv,v)
        den = 1.0+np.dot(v,temp)*c
        scale = c/den
        for i in xrange(Ainv.shape[0]):
            Ainv[i,:] -= (scale*temp[i])*temp
    else:
        AinvV = np.dot(Ainv,v)
        AinvW = np.dot(Ainv,w)
        scale = c/(1.0+np.dot(v,AinvW)*c)
        for i in xrange(Ainv.shape[0]):
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
        for i in xrange(self.n):
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
        self.AtA += (regularizationLambda-self.regularizationLambda)*np.eye(n)
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
        for i in xrange(self.n):
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
                    print "OnlineLS: degenerate matrix, inv failed to produce symmetric matrix"
            except np.linalg.LinAlgError:
                self.AtAinv = np.linalg.pinv(self.AtA)
                self.degenerate = True
                print "OnlineLS: degenerate matrix"
        return

        
if __name__=="__main__":
    ols = OnlineLeastSquares(2)
    indep = [[1.0,0.0],[1.0,1.0],[1.0,0.5]]
    dep = [3,1,2.5]
    ols.add(indep[0],dep[0])
    print "Degenerate coeffs",ols.x,"residual2",ols.residualNorm2()
    ols.add(indep[1],dep[1])
    print "Full rank coeffs",ols.x,"residual2",ols.residualNorm2()
    ols.add(indep[2],dep[2])
    print "Overdetermined coeffs",ols.x,"residual2",ols.residualNorm2()
    print "Standard errors",ols.solutionStandardErrors()
