from .robotinterface import RobotInterfaceBase
from .robotinterfaceutils import ThreadedRobotInterface,UTILITY_METHODS,COMMAND_METHODS,QUERY_METHODS
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.client import ServerProxy, Marshaller
import signal
import traceback
from functools import wraps
import sys
import copy

# #helps when numpy is used
# Marshaller.dispatch[np.float64] = Marshaller.dump_double
# Marshaller.dispatch[np.ndarray] = Marshaller.dump_array

class RobotInterfaceServer:
    """A XMLRPC-based server for a RIL motor controller.
    
    Args:
        interface (RobotInterfaceBase): the interface to serve
        ip (str): the IP address (usually '128.0.0.1' or 'localhost')
        port (int): the port of the listening socket
    
    """
    def __init__(self,interface,ip,port=7881):
        self.ip = ip
        self.port = port
        print("Opening RIL server on",ip,port)
        self.server = SimpleXMLRPCServer((ip,port), logRequests=True, allow_none=True)
        self.started = False
        self.interface = interface
        if not interface.properties.get('asynchronous'):
            self.interface = ThreadedRobotInterface(interface)
            self.interface.log = self.log

        def sigint_handler(signum,frame):
            if self.server_started:
                self.interface.close()  
            sys.exit(0)

        self.server.register_introspection_functions()
        for f in COMMAND_METHODS:
            self._register(getattr(interface,f),f)
        for f in QUERY_METHODS:
            if f == 'klamptModel':
                continue
            if f == 'parts':
                def parts():  #None cannot be a dictionary key in XMLRPC
                    res = copy.deepcopy(interface.parts())
                    a = res[None]
                    res[''] = a
                    del res[None]
                    return res
                self._register(parts,f)
                continue
            self._register(getattr(interface,f),f)
        def getProperty(name):
            return interface.properties.get(name,None)
        self._register(getProperty,'getProperty')

        signal.signal(signal.SIGINT, sigint_handler)
    
    def log(self,err):
        """Logs an error. Can be overloaded."""
        print(err)

    def serve(self):
        if not self.interface.initialize():  #runs the controller thread
            raise RuntimeError("RobotInterface of type {} could not be initialized".format(self.interface))
        self.server_started = True
        self.server.serve_forever()

    def _register(self,func,name=None):
        """
        Registers a function to the xmlrpc server under the given name.
        """
        if name is None:
            name = func.__name__
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                tb = traceback.format_exc()
                self.log(tb)
                raise e
        self.server.register_function(wrapper, name)

class DictProxy:
    """Helper class: uses a function to act as a dict getter/setter"""
    def __init__(self,getter,allow_overrides=True):
        self.getter = getter
        self.allow_overrides = allow_overrides
        self.overrides = dict()
    def __getitem__(self,key):
        if key in self.overrides:
            return self.overrides[key]
        return self.getter(key)
    def __setitem__(self,key,value):
        if not self.allow_overrides:
            raise RuntimeError("Unable to set item in DictProxy")
        self.overrides[key] = value
    def __contains__(self,key):
        if key in self.overrides: return True
        return self.getter(key) is not None
    def get(self,key,default_value):
        if key in self.overrides:
            return self.overrides[key]
        try:
            return self.getter(key)
        except KeyError:
            return default_value

class RobotInterfaceClient(RobotInterfaceBase):
    """An XMLRPC-based client that connects to a RobotInterfaceServer.
    
    Args:
        addr (str): the IP address of the server, including port. 
            Localhost port 7881 is used by default.
    """
    def __init__(self,addr='http://localhost:7881'):
        RobotInterfaceBase.__init__(self)
        print("Opening RIL client on",addr)
        self.s = ServerProxy(addr, allow_none=True)
        for f in COMMAND_METHODS:
            setattr(self,f,getattr(self.s,f))
        for f in QUERY_METHODS:
            if f == 'klamptModel':
                continue
            if f == 'parts':
                continue
            setattr(self,f,getattr(self.s,f))
        self.properties = DictProxy(self.s.getProperty)

    def partInterface(self,part):
        raise NotImplementedError("TODO: networked part interface")
    
    def parts(self):
        res = self.s.parts()
        a = res['']
        res[None] = a
        del res['']
        return res
    
    #def klamptModel(self):
