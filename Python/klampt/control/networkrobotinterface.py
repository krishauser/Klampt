"""Classes for building networked RIL servers and clients.

.. versionadded:: 0.9
"""

from .robotinterface import RobotInterfaceBase
from .robotinterfaceutils import _RobotInterfaceState,_RobotInterfaceStructure,_RobotInterfaceSettings,_RobotInterfaceCommand,_RobotInterfaceStatefulBase,_RobotInterfaceStatefulWrapper
from .robotinterfaceutils import _split_settings,_split_state,_gather_settings,_gather_commands,_update_from_settings
from ..model.subrobot import SubRobotModel
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.client import ServerProxy, Marshaller
import signal
import traceback
from functools import wraps
import sys
import copy
from typing import Dict,Tuple,List,Sequence,Any
import warnings

# #helps when numpy is used
# Marshaller.dispatch[np.float64] = Marshaller.dump_double
# Marshaller.dispatch[np.ndarray] = Marshaller.dump_array



class ClientRobotInterfaceBase(_RobotInterfaceStatefulBase):
    """Base class for a robot interface that communicates with a server.
    begin/endStep() will need to be called repeatedly, and on endStep() all
    the communication is performed.
    """
    def __init__(self):
        _RobotInterfaceStatefulBase.__init__(self)
        self._partInterfaces = dict()         #type: Dict[str,_RobotInterfaceStatefulBase]
        self._initialized = False
        
    def connect(self) -> bool:
        raise NotImplementedError("Subclass must handle this")
    
    def disconnect(self) -> bool:
        raise NotImplementedError("Subclass must handle this")
    
    def getInitialData(self) -> Tuple[Dict,_RobotInterfaceStructure,_RobotInterfaceSettings,_RobotInterfaceState]:
        raise NotImplementedError("Subclass must handle this")
    
    def updateSettingsAndCommands(self, settings : _RobotInterfaceSettings, commands : List[_RobotInterfaceCommand]) -> _RobotInterfaceState:
        raise NotImplementedError("Subclass must handle this")
    
    def partInterface(self,part,joint_idx=None):
        if joint_idx is not None:
            raise NotImplementedError("TODO: part interfaces for individual joint indices")
        return self._partInterfaces[part]

    def initialize(self):
        if self._initialized:
            raise RuntimeError("initialize() can only be called once")
        if not self.connect():
            return False
        properties, structure, settings, state = self.getInitialData()
        self.properties = properties
        self._structure = structure
        self._settings = settings
        self._state = state
        self._structure.klamptModel = self.klamptModel()
        if self._structure.klamptModel is None:
            warnings.warn("KlamptModel could not be loaded: {}".format(self.properties.get('klamptModelFile',None)))
            input()
        for (k,inds) in self._structure.parts.items():
            if k is None: continue
            partInterface = _RobotInterfaceStatefulBase()
            partInterface._structure.controlRate = self._structure.controlRate
            partInterface._structure.numJoints = len(inds)
            if self._structure.jointNames is not None:
                partInterface._structure.jointNames = [self._structure.jointNames[i] for i in inds]
            if self._structure.klamptModel is not None:
                partInterface._structure.klamptModel = SubRobotModel(self._structure.klamptModel,inds)
            self._partInterfaces[k] = partInterface
        settings = _split_settings(self._settings,self._structure.parts)
        for (k,setting) in settings.items():
            self._partInterfaces[k]._settings = setting
        self._initialized = True
        return True

    def close(self):
        if self._initialized:
            if not self.disconnect():
                return False
        return True

    def beginStep(self):
        if self._commands:
            raise RuntimeError("Interface didn't clear command queue?")
        #distribute prior command state changes
        states = _split_state(self._state,self._structure.parts)
        for (k,state) in states.items():
            self._partInterfaces[k]._state = state
        #zero out settings delta
        self._delta_settings = _RobotInterfaceSettings()
        for (k,iface) in self._partInterfaces.items():
            iface._delta_settings = _RobotInterfaceSettings()
    
    def endStep(self):
        #read settings changes, command changes and pass to interface
        assert self._delta_settings is not None,"Didn't call beginStep?"
        _gather_settings(self._structure.parts,dict((k,iface._delta_settings) for k,iface in self._partInterfaces.items()),self._delta_settings)
        cmds = _gather_commands([self._commands]+[iface._commands for iface in self._partInterfaces.values()],
                                [self.indices()]+[self._structure.parts[k] for k in self._partInterfaces])
        try:
            next_state = self.updateSettingsAndCommands(self._delta_settings,cmds)
            if next_state is None:
                self._state.status = 'server error'
            else:
                self._state = next_state
        except Exception:
            #must have disconnected
            self._state.status = 'disconnected'
        
        self._commands = []
        self._delta_settings = None
        #clear part command queues
        for (k,iface) in self._partInterfaces.items():
            iface._commands = []
        
    def reset(self):
        if self._state.status == 'disconnected':
            self._initialized = False
            if self.initialize():
                self._state.status = 'ok'
        else:
            super().reset()
        


class ServerRobotInterfaceBase(_RobotInterfaceStatefulBase):
    """Base class for an asynchronous robot interface that serves
    clients.  Subclass will need to start the controller thread, and
    then start a thread that receives RPC calls and call
    getInitialData_impl and updateSettingsAndCommands_impl in response.

    You'll need to pair your server with a client that implements
    the appropriate protocols, e.g., :class:`XMLRPCRobotInterfaceServer` /
    :class:`XMLRPCRobotInterfaceClient`

    Usage (simplified, doesn't do error handling)::

        #TODO: define MyServer as subclass of ServerRobotInterfaceBase

        server = MyServer(addr,interface)
        if not server.initialize():
            print("Error initializing")
            exit(1)
        server.startController()

        #serve the client
        quit = False
        client = None
        while not quit:
            if client is None:
                client = server.accept()
            else:
                func, args = server.read_rpc()
                if func == 'getInitialData':
                    res = server.getInitialData_impl()
                    server.respond_rpc(res)
                elif func == 'updateSettingsAndCommands':
                    res = server.updateSettingsAndCommands_impl(*args)
                    server.respond_rpc(res)
                else:
                    raise RuntimeError("Invalid RPC call?")
            time.sleep(0.01) #some poll rate
        server.stop()
        server.close()

    """
    def __init__(self, interface : RobotInterfaceBase):
        _RobotInterfaceStatefulBase.__init__(self)
        if not isinstance(interface,_RobotInterfaceStatefulBase):
            self._base = _RobotInterfaceStatefulWrapper(interface)
        else:
            self._base = interface
        self._base_initialized = False
        from threading import RLock,Thread
        self._thread = None          #type: Thread
        self._lock = RLock()
        self._properties= None
        self._structure = None
        self._settings = None
        self._state = None
        self._commands = None
        self._stop = False
        
    def getInitialData_impl(self) -> Tuple[Dict,_RobotInterfaceStructure,_RobotInterfaceSettings,_RobotInterfaceState]:
        assert self._base_initialized
        with self._lock:
            self._settings = copy.deepcopy(self._base._settings)
            self._state = copy.deepcopy(self._base._state)
            return self.properties,self._structure,self._settings,self._state
    
    def updateSettingsAndCommands_impl(self, delta_settings : _RobotInterfaceSettings, commands : List[_RobotInterfaceCommand]) -> _RobotInterfaceState:
        with self._lock:
            _update_from_settings(None,self._base._delta_settings,delta_settings)
            self._base._commands += commands
            self._state = copy.deepcopy(self._base._state)
            return self._state
    
    def initialize(self):
        assert not self._base_initialized
        print("Server initializing base interface",self._base)
        if not self._base.initialize():
            return False
        self._base_initialized = True
        self.properties = copy.deepcopy(self._base.properties)

        tempRobot = self._base._structure.klamptModel
        self._base._structure.klamptModel = None
        self._structure = copy.deepcopy(self._base._structure)
        self._base._structure.klamptModel = tempRobot

        return True

    def _advance(self):
        from klampt.control.utils import TimedLooper
        dt = 1.0 / self._base.controlRate()
        looper = TimedLooper(dt)
        with self._lock:
            self._base.beginStep()
        while looper and not self._stop:
            with self._lock:
                self._base.endStep()
                self._base.beginStep()
        self._base.endStep()

    def startController(self):
        from threading import Thread
        self._thread = Thread(target=self._advance)
        self._thread.daemon = True
        self._stop = False
        self._thread.start()

        def sigint_handler(signum,frame):
            self.close()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)
    
    def stop(self):
        """Asks to stop motion -- perhaps call this when all clients disconnect"""
        with self._lock:
            self._base.softStop()
    
    def close(self):
        """Subclasses might want to override this to close clients as well."""
        self._stop = True
        if self._thread is not None:
            self._thread.join()
        if self._base_initialized:
            self._base.close()


class XMLRPCRobotInterfaceServer(ServerRobotInterfaceBase):
    """A XMLRPC-based server for a RIL robot controller.
    
Usage

    Args:
        interface (RobotInterfaceBase): the interface to serve
        ip (str): the IP address (usually '128.0.0.1' or 'localhost')
        port (int): the port of the listening socket. 7881 is used by default.
    
    """
    def __init__(self, interface: RobotInterfaceBase, ip:str, port=7881):
        ServerRobotInterfaceBase.__init__(self,interface)
        self.ip = ip
        self.port = port
        print("Opening RIL server on",ip,port)
        self.server = SimpleXMLRPCServer((ip,port), logRequests=False, allow_none=True)
        self.started = False

        self.server.register_introspection_functions()
        self._register(self.getInitialData,'getInitialData')
        self._register(self.updateSettingsAndCommands,'updateSettingsAndCommands')
    
    def log(self,err):
        """Logs an error. Can be overloaded."""
        print(err)

    def serve(self):
        if not ServerRobotInterfaceBase.initialize(self): 
            raise RuntimeError("RobotInterface of type {} could not be initialized".format(self.interface))
        self.startController()
        self.server.serve_forever()
    
    def getInitialData(self):
        props,struct,settings,state = self.getInitialData_impl()
        return (props,struct.to_json(),settings.to_json(),state.to_json())
    
    def updateSettingsAndCommands(self, delta_settings_json, commands_json):
        delta_settings = _RobotInterfaceSettings()
        commands = []
        delta_settings.from_json(delta_settings_json)
        for cmd_json in commands_json:
            cmd = _RobotInterfaceCommand()
            cmd.from_json(cmd_json)
            commands.append(cmd)
        state = super().updateSettingsAndCommands_impl(delta_settings, commands)
        return state.to_json()

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


class XMLRPCRobotInterfaceClient(ClientRobotInterfaceBase):
    """An XMLRPC-based client that connects to a RobotInterfaceServer.
    
    Args:
        addr (str): the IP address of the server, including port. 
            Localhost port 7881 is used by default.
    """
    def __init__(self,addr='http://localhost:7881'):
        ClientRobotInterfaceBase.__init__(self)
        self.addr = addr
        self.context_mgr = None     #type: ServerProxy
        self.s = None               #type: ServerProxy
        
    def connect(self) -> bool:
        print("Opening RIL client on",self.addr)
        try:
            self.context_mgr = ServerProxy(self.addr, allow_none=True)
            self.s = self.context_mgr.__enter__()
        except Exception as e:
            print("Error connecting: ",e)
            return False
        return True
    
    def disconnect(self) -> bool:
        if self.context_mgr is None:
            return True
        self.context_mgr.__exit__()
        self.context_mgr = None
        self.s = None
    
    def getInitialData(self) -> Tuple[Dict,_RobotInterfaceStructure,_RobotInterfaceSettings,_RobotInterfaceState]:
        props,struct_json,settings_json,state_json = self.s.getInitialData()
        struct = _RobotInterfaceStructure()
        struct.from_json(struct_json)
        settings = _RobotInterfaceSettings()
        settings.from_json(settings_json)
        state = _RobotInterfaceState()
        state.from_json(state_json)
        return (props,struct,settings,state)
    
    def updateSettingsAndCommands(self, delta_settings : _RobotInterfaceSettings, commands : List[_RobotInterfaceCommand]) -> _RobotInterfaceState:
        delta_settings_json = delta_settings.to_json()
        commands_json = [cmd.to_json() for cmd in commands]
        state_json = self.s.updateSettingsAndCommands(delta_settings_json,commands_json)
        state = _RobotInterfaceState()
        state.from_json(state_json)
        return state

