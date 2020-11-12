"""An exporter that converts scenes / animations to shareable HTML files.
"""

from klampt import *
from klampt.model import trajectory
from klampt import robotsim
import json
import pkg_resources

_title_id = '__TITLE__'
_scene_id = '__SCENE_JSON__'
_path_id = '__PATH_JSON__'
_rpc_id = '__RPC_JSON__'
_compressed_id = '__COMPRESSED__'
_dt_id = '__TIMESTEP__'
_frontend_load_id = '__KLAMPT_FRONTEND_LOAD__'

def make_fixed_precision(obj,digits):
    if isinstance(obj,float):
        return round(obj,digits)
    elif isinstance(obj,list):
        for i in range(len(obj)):
            obj[i] = make_fixed_precision(obj[i],digits)
    elif isinstance(obj,tuple):
        return [make_fixed_precision(val,digits) for val in obj]
    elif isinstance(obj,dict):
        for i,v in obj.items():
            obj[i] = make_fixed_precision(v,digits)
    return obj

class HTMLSharePath:
    """An exporter that converts scenes / animations to shareable HTML files.

    Examples::

        sharer = HTMLSharePath("mypath.html",name="My spiffy path")
        sharer.start(sim)  #can accept a sim or a world
        while [simulation is running]:
            #do whatever control you wish to do here
            sim.simulate(...)
            sharer.animate()
        sharer.end() #this saves to the filename given in the constructor

    """
    
    def __init__(self,filename=None,name="Klamp't Three.js app",boilerplate='auto',libraries='static'):
        """
        Args:
            filename (str, optional): the HTML file to generate. If None, then
                the end() method returns the HTML string.
            name (str): the title of the HTML page
            boilerplate (str): the location of the boilerplate HTML file.  If
                'auto', it's automatically found in the ``klampt/data`` folder.
            libraries (str): either 'static' or 'dynamic'. In the latter case,
                the html file loads the libraries from the Klamp't website
                dynamically. This reduces the size of the HTML file by about
                600kb, but the viewer needs an internet connection
        
        """
        self.name = name
        if boilerplate == 'auto':
            boilerplate = pkg_resources.resource_filename('klampt','data/share_path_boilerplate.html')
        f = open(boilerplate,'r')
        self.boilerplate_file = ''.join(f.readlines())
        f.close()
        if libraries == 'static':
            self.klampt_frontend_load_script = pkg_resources.resource_filename('klampt','data/klampt_frontend_load_static.js')
        else:
            if libraries != 'dynamic':
                raise ValueError("The libraries argument must either be 'static' or 'dynamic'")
            self.klampt_frontend_load_script = pkg_resources.resource_filename('klampt','data/klampt_frontend_load_dynamic.js')
        if any(v not in self.boilerplate_file for v in [_scene_id,_path_id,_rpc_id,_compressed_id,_dt_id,_frontend_load_id]):
            raise RuntimeError("Boilerplate file does not contain the right tags")
        self.fn = filename
        self.scene = 'null'
        self.transforms = {}
        self.rpc = []
        self.dt = 0
        self.last_t = 0
    def start(self,world):
        """Begins the path saving with the given WorldModel or Simulator"""
        if isinstance(world,Simulator):
            self.sim = world
            self.world = world.world
            self.last_t = world.getTime()
        else:
            self.sim = None
            self.world = world
        if self.world is not None:
            self.scene = robotsim.ThreeJSGetScene(self.world)
    def animate(self,time=None,rpc=None):
        """Updates the path from the world.  If the world wasn't a simulator, the time
        argument needs to be provided.

        If you want to include extra things, provide them in the rpc argument (as a list
        of KlamptFrontend rpc calls)
        """
        if self.sim is not None and time is None:
            time = self.sim.getTime()
            self.sim.updateWorld()
        if time is None:
            raise ValueError("Time needs to be provided")
        dt = time - self.last_t
        if self.dt == 0:
            self.dt = dt
        if self.dt == 0:
            return
        if abs(dt - self.dt) <= 1e-6:
            dt = self.dt
        numadd = 0
        while dt >= self.dt:
            numadd += 1
            if self.world is not None:
                transforms = json.loads(robotsim.ThreeJSGetTransforms(self.world))
            else:
                transforms = {'object':[]}
            for update in transforms['object']:
                n = update['name']
                mat = make_fixed_precision(update['matrix'],4)
                matpath = self.transforms.setdefault(n,[])
                assert len(matpath) == len(self.rpc)
                lastmat = None
                for m in matpath[::-1]:
                    if m != None:
                        lastmat = m
                        break
                if lastmat != mat:
                    matpath.append(mat)
                else:
                    matpath.append(None)
            if numadd == 1:
                if rpc is not None:
                    assert isinstance(rpc,(list,tuple)),"rpc argument must be a list or a tuple"
                    self.rpc.append(rpc)
                else:
                    self.rpc.append(None)
            else:
                self.rpc.append(None)
            dt -= self.dt
            self.last_t += self.dt
        if numadd > 1:
            print("HTMLSharePath: Note, uneven time spacing, duplicating frame",numadd,"times")
    def end(self,rpc=None):
        if len(self.rpc)==0:
            self.rpc = [rpc]
        elif rpc is not None:
            self.rpc[-1] += rpc
        data = self.boilerplate_file.replace(_title_id,self.name)
        data = data.replace(_scene_id,self.scene)
        data = data.replace(_path_id,json.dumps(self.transforms))
        data = data.replace(_rpc_id,json.dumps(self.rpc))
        data = data.replace(_compressed_id,'true')
        data = data.replace(_dt_id,str(self.dt))
        f = open(self.klampt_frontend_load_script,'r')
        load_script = ''.join(f.readlines())
        f.close()
        data = data.replace(_frontend_load_id,load_script)
        if self.fn is None:
            return data
        else:
            print("Path with",len(self.rpc),"frames saved to",self.fn)
            f = open(self.fn,'w')
            f.write(data)
            f.close()

if __name__ == '__main__':
    import sys
    import os
    from klampt import trajectory
    world = WorldModel()
    if len(sys.argv) == 1:
        world.readFile("../../data/athlete_plane.xml")
        q = world.robot(0).getConfig()
        q[2] = 2
        world.robot(0).setConfig(q)
        sim = Simulator(world)
        share = HTMLSharePath(name="Klamp't simulation path")
        share.start(sim)
        for i in range(100):
            sim.simulate(0.02)
            share.animate()
        share.end()
    else:
        assert len(sys.argv) == 3,"Usage: sharepath.py world.xml robot_path"
        world.readFile(sys.argv[1])
        traj = trajectory.RobotTrajectory(world.robot(0))
        traj.load(sys.argv[2])
        world.robot(0).setConfig(traj.milestones[0])

        dt = 0.02
        excess = 1.0

        share = HTMLSharePath(name="Klamp't path "+os.path.split(sys.argv[2])[1])
        share.start(world)
        share.dt = dt
        t = traj.times[0]
        while t < traj.times[-1] + excess:
            world.robot(0).setConfig(traj.eval(t))
            share.animate(t)
            t += dt
        share.end()
