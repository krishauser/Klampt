import klampt.math.autodiff.ad as ad
import torch,numpy as np

class TorchModuleFunction(ad.ADFunctionInterface):
    """Converts a PyTorch function to a Klamp't autodiff function class."""
    def __init__(self,module):
        self.module=module
        self._eval_params=[]
        torch.set_default_dtype(torch.float64)
    
    def __str__(self):
        return str(self.module)
    
    def n_in(self,arg):
        return -1

    def n_out(self):
        return -1

    def eval(self,*args):
        self._eval_params=[]
        for a in args:
            if not isinstance(a,np.ndarray):
                a=np.array([a])
            p=torch.Tensor(a)
            p.requires_grad_(True)
            self._eval_params.append(p)
        
        try:
            self._eval_result=torch.flatten(self.module(*self._eval_params))
            #self._eval_result.forward()
        except Exception as e:
            print('Torch error: %s'%str(e))
        return self._eval_result.detach().numpy()

    def derivative(self,arg,*args):
        #lazily check if forward has been done before
        if not self._same_param(*args):
            self.eval(*args)
        
        rows=[]
        for i in range(self._eval_result.shape[0]):
            if self._eval_params[arg].grad is not None:
                self._eval_params[arg].grad.zero_()
            #this is a major performance penalty, torch does not support jacobian
            #we have to do it row by row
            self._eval_result[i].backward(retain_graph=True)
            rows.append(self._eval_params[arg].grad.detach().numpy().flatten())
        return np.vstack(rows)

    def jvp(self,arg,darg,*args):
        raise NotImplementedError('')

    def _same_param(self,*args):
        if not hasattr(self,"_eval_params"):
            return False
        if len(self._eval_params)!=len(args):
            return False
        for p,a in zip(self._eval_params,args):
            pn = p.detach().numpy()
            if not isinstance(a,np.ndarray):
                a=np.array([a])
            if pn.shape != a.shape:
                return False
            if (pn!=a).any():
                return False
        return True


class ADModule(torch.autograd.Function):
    """Converts a Klamp't autodiff function call or function instance to a
    PyTorch Function. The class must be created with the terminal symbols
    corresponding to the PyTorch arguments to which this is called.
    """
    @staticmethod
    def forward(ctx,func,terminals,*args):
        torch.set_default_dtype(torch.float64)
        if len(args)!=len(terminals):
            raise ValueError("Function %s expected to have %d arguments, instead got %d"%(str(func),len(terminals),len(args)))
        if isinstance(func,ad.ADFunctionCall):
            context={}
            for t,a in zip(terminals,args):
                context[t.name]=a.detach().numpy()
            ret=func.eval(**context)
        elif isinstance(func,ad.ADFunctionInterface):
            context=[]
            for t,a in zip(terminals,args):
                context.append(a.detach().numpy())
            ret=func.eval(*context)
        else: 
            raise ValueError("f must be a ADFunctionCall or ADFunctionInterface")
        ctx.saved_state=(func,terminals,context)
        return torch.Tensor(ret)
    
    @staticmethod
    def backward(ctx,grad):
        ret = [None,None]
        func,terminals,context = ctx.saved_state
        if isinstance(func,ad.ADFunctionCall):
            for k in range(len(terminals)):
                if isinstance(terminals[k],ad.ADTerminal):
                    name = terminals[k].name
                else:
                    name = terminals[k]
                deriv=torch.Tensor(func.derivative(name,**context))
                ret.append(deriv.T@grad)
        elif isinstance(func,ad.ADFunctionInterface):
            for k in range(len(terminals)):
                deriv=torch.Tensor(func.derivative(k,*context))
                ret.append(deriv.T@grad)
        else: 
            raise ValueError("f must be a ADFunctionCall or ADFunctionInterface")
        return tuple(ret)
    
    @staticmethod
    def check_derivatives_torch(func,terminals,h=1e-6,rtol=1e-2,atol=1e-3):
        #sample some random parameters of the appropriate length
        if isinstance(func,ad.ADFunctionInterface):
            params=[]
            for i in range(len(terminals)):
                try:
                    N = func.n_in(i)
                    if N < 0:
                        N = 10
                except NotImplementedError:
                    N = 10
                params.append(torch.randn(N))
        else:
            N = 10
            params = [torch.randn(N) for i in range(len(terminals))]
        for p in params:
            p.requires_grad_(True)
        torch.autograd.gradcheck(ADModule.apply,tuple([func,terminals]+params),eps=h,atol=atol,rtol=rtol,raise_exception=True)


def torch_to_ad(module,args):
    """Converts a PyTorch function applied to args (list of scalars or numpy
    arrays) to a Klamp't autodiff function call on those arguments."""
    wrapper=TorchModuleFunction(module)
    return wrapper(*args)


def ad_to_torch(func,terminals=None):
    """Converts a Klamp't autodiff function call or function instance to a
    PyTorch Function.  If terminals is provided, this is the list of arguments
    that PyTorch will expect.  Otherwise, the variables in the expression
    will be automatically determined by the forward traversal order."""
    if terminals is None:
        if isinstance(func,ad.ADFunctionCall):
            terminals = func.terminals()
        else:
            n_args = func.n_args()
            terminals = [func.argname(i) for i in range(n_args)]
    else:
        if isinstance(func,ad.ADFunctionCall):
            fterminals = func.terminals()
            if len(terminals) != len(fterminals):
                raise ValueError("The number of terminals provided is incorrect")
            for t in terminals:
                if isinstance(t,ad.ADTerminal):
                    name = t.name
                else:
                    name = t
                if name not in fterminals:
                    raise ValueError("Invalid terminal %s, function call %s only has terminals %s"%(name,str(func),str(terminals)))
        else:
            try:
                if len(terminals) != func.n_args():
                    raise ValueError("Invalid number of terminals, function %s expects %d"%(str(func),func.n_args()))
            except NotImplementedError:
                pass
    return ADModule(func,terminals)