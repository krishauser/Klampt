import klampt.math.autodiff.ad as ad
import torch,numpy as np

"""torch to ad wrapper function"""
class TorchModuleFunction:
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

def torch_to_ad(module,args):
    wrapper=TorchModuleFunction(module)
    return ad.ADFunctionCall(wrapper,args)

"""ad to torch wrapper function"""
class ADModule(torch.autograd.Function):
    
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
                deriv=torch.Tensor(func.derivative(terminals[k].name,**context))
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
        N=10
        params=[torch.randn(N) for i in range(len(terminals))]
        for p in params:
            p.requires_grad_(True)
        torch.autograd.gradcheck(ADModule.apply,tuple([func,terminals]+params),eps=h,atol=atol,rtol=rtol,raise_exception=True)
    
