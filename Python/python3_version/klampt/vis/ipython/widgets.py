"""A Jupyter Notebook interface to Klampt.

Usage:
world = WorldModel()
... #set up the world...
kvis = KlamptWidget(world,width=800,height=640)
display(kvis)   # This pops up a window in Jupyter

# Immedate changes can be made using the methods in KlamptWidget
kvis.addText(name="text_id",text="hello",position=(10,10))
kvis.addSphere(x=0,y=1.5,z=0,r=0.4)

# Change the configuration of things in the world, and then call update() to see the changes
robot = world.robot(0)
q = robot.getConfig()
q[2] += 1.0
robot.setConfig(q)
kvis.update()   # The previous changes are not made until this is called

# If you completely change the number of objects in the world, or their underlying geometries,
# you will need to call w.setWorld(world) again.  This is relatively expensive, so try not to
# do it too often.
world.readElement(...)
kvis.setWorld(world)

"""

from klampt import ThreeJSGetScene,ThreeJSGetTransforms
from klampt.math import vectorops,so3,se3
from klampt.model import types
from klampt.model.trajectory import Trajectory,RobotTrajectory,SE3Trajectory
from klampt import RobotModel,RobotModelLink
import json
import time
import math
import ipywidgets as widgets
from ipywidgets import interact, interactive, fixed, interact_manual
from traitlets import Unicode, Dict, List, Int, validate, observe
import traitlets
import threading

DEFAULT_POINT_RADIUS = 0.05
DEFAULT_AXIS_LENGTH = 0.2
DEFAULT_AXIS_WIDTH = 1

class KlamptWidget(widgets.DOMWidget):
    """
    A Python interface with the Jupyter notebook frontend.
    The API is similar to the vis module, but has a reduced and slightly modified
    set of hooks.

    Public DOMWidget attributes:
    - width: the width of the view in pixels
    - height: the height of the view in pixels

    Private DOMWidget attributes:
    - scene: the scene JSON message
    - transforms: the transforms JSON message
    - rpc: the rpc JSON message
    - _camera: the incoming camera JSON message from the frontend
    - camera: the outgoing camera JSON message
    - drawn: the incoming drawn message from the frontend
    - events: incoming events from the frontend

    Private members:
    - - world: the WorldModel isinstance
    - _extras: a dict mapping extra item names to (type,threejs_items) pairs
    - _rpc_calls: a list of pending RPC calls between beginRpc() and endRpc()
    - _aggregating_rpc: non-zero if between beginRpc and endRpc
    """
    _model_name = Unicode('KlamptModel').tag(sync=True)
    _view_name = Unicode('KlamptView').tag(sync=True)
    _model_module = Unicode('klampt-jupyter-widget').tag(sync=True)
    _view_module = Unicode('klampt-jupyter-widget').tag(sync=True)
    _model_module_version = Unicode('0.1.0').tag(sync=True)
    _view_module_version = Unicode('0.1.0').tag(sync=True)
    width = Int(800).tag(sync=True)
    height = Int(600).tag(sync=True)
    scene = Dict().tag(sync=True)
    transforms = Dict().tag(sync=True)
    rpc = Dict().tag(sync=True)
    _camera = Dict().tag(sync=True)
    camera = Dict().tag(sync=True)
    events = List().tag(sync=True)
    drawn = Int(0).tag(sync=True)

    def __init__(self,world=None,*args,**kwargs):
        widgets.DOMWidget.__init__(self,*args,**kwargs)
        self.world = world
        self._extras = dict()
        self._aggregating_rpc = 0
        self._rpc_calls = []
        if world is not None:
            self.setWorld(world)
        self.rpc = {}
        return
    
    def setWorld(self,world):
        """Resets the world to a new WorldModel object. """
        self.world = world
        self._extras = dict()
        self._aggregating_rpc = 0
        self._rpc_calls = []
        s = ThreeJSGetScene(self.world)
        self.scene = json.loads(s)

    def update(self):
        """Updates the view with changes to the world.  Unlike setWorld(), this only pushes the geometry
        transforms, so it's much faster."""
        if self.world:
            s = ThreeJSGetTransforms(self.world)
            self.transforms = json.loads(s)

    def clear(self):
        """Clears everything from the visualization, including the world."""
        self._extras = dict()
        self._do_rpc({'type':'reset_scene'})

    def clearExtras(self):
        """Erases all ghosts, lines, points, text, etc from the visualization, but keeps the world."""
        self._extras = dict()
        self._do_rpc({'type':'clear_extras'})

    #TODO: implement this to be more similar to the vis API
    #def clearText(self):

    def add(self,name,item,type='auto'):
        """Adds the item to the world, and returns a list of identifiers associated with it.
        
        Supports:
        - Config, as a ghost (list, same size as robot)
        - Configs, as a set of ghosts (list of lists, same size as robot)
        - Vector3, drawn as a sphere (3-list)
        - RigidTransform, drawn as an xform (pair of 9-list and 3-list)
        - Configs, drawn as a polyline (list of 3-lists)
        - Trajectory, drawn either as:
           - a polyline (3D Trajectory objects),
           - set of milestones (Trajectory or RobotTrajectory objects)
           - a polyline + set of rigid transform milestones (SE3Trajectory objects)
        - WorldModel, but only one world at once is supported (same as setWorld).
        """
        if type == 'auto':
            try:
                candidates = types.objectToTypes(item,self.world)
            except Exception:
                raise ValueError("Invalid item, not a known Klamp't type")
            if isinstance(candidates,(list,tuple)):
                type = candidates[0]
            else:
                type = candidates
        if type == 'Config':
            res = self.addGhost(name)
            self.setGhostConfig(name,item)
            return [res]
        elif type == 'Configs':
            if len(item[0]) == 3:
                #it's a polyline
                return [self.addPolyline(name,item)]
            else:
                #it's a set of configurations
                names = []
                for i,q in enumerate(item):
                    iname = name+'_'+str(i)
                    self.addGhost(iname)
                    self.setGhostConfig(q,iname)
                    names.append(iname)
                self._extras[name] = ('Configs',names)
                return names
        elif type == 'Vector3':
            res = self.addSphere(name,item[0],item[1],item[2],DEFAULT_POINT_RADIUS)
            return [res]
        elif type == 'RigidTransform':
            res = self.addXform(name,length=DEFAULT_AXIS_LENGTH,width=DEFAULT_AXIS_WIDTH)
            self.setTransform(name,R=item[0],t=item[1])
            return [res]
        elif type == 'Trajectory':
            if isinstance(item,SE3Trajectory):
                res = []
                ttraj = []
                for i in item.milestones:
                    T = item.to_se3(item.milestones[i])
                    res += self.add(name+"_milestone_"+str(i),T)
                    ttraj.append(T[1])
                res += self.add(name,ttraj)
                self._extras[name] = ('Trajectory',res)
                return res
            elif isinstance(item,RobotTrajectory):
                #it's a set of configurations
                rindex = item.robot.index
                names = []
                for i,q in enumerate(item):
                    iname = name+'_'+str(i)
                    self.addGhost(iname,rindex)
                    self.setGhostConfig(q,iname,rindex)
                    names.append(iname)
                self._extras[name] = ('Configs',names)
                return names
            else:
                return self.add(name,item.milestones)
        elif type == 'WorldModel':
            if name != 'world' or self.world is not None:
                print("KlamptWidget.add: Warning, only one world is supported, and should be added as world")
            self.setWorld(item)
        else:
            raise ValueError("KlamptWidget can't handle objects of type "+type+" yet")

    def remove(self,name):
        """Removes a certain named target, e.g. a ghost, line, text, etc."""
        self._do_rpc({'type':'remove','object':name})

    def hide(self,name,hidden=True):
        """Hides/shows named target, e.g. a ghost, line, text, etc."""
        self._do_rpc({'type':'set_visible','object':name,'value':(not hidden)})

    def resetCamera(self):
        """Resets the camera to the original view"""
        self._do_rpc({'type':'reset_camera'})

    def getCamera(self):
        """Returns a data structure representing the current camera view"""
        res = dict(self._camera).copy()
        if 'r' in res:
            del res['r']
        return res

    def setCamera(self,cam):
        """Sets the current camera view"""
        self.camera = cam
        marked = dict(cam).copy()
        marked['r'] = 1
        self._camera = marked

    def hide(self,name,value=False):
        """Changes the visibility status of a certain named target"""
        target_name = name
        if name in self._extras:
            type,data = self._extras[target]
            if type == 'Config':
                target_name = data
            elif type == 'Configs' or type == 'Trajectory':
                self.beginRpc(strict=False)
                for subitem in data:
                    self._do_rpc({'type':'set_visible','object':subitem,'value':value})
                self.endRpc(strict=False)
                return
        self._do_rpc({'type':'set_visible','object':target_name,'value':value})

    def setColor(self,target,r,g,b,a=1.0):
        """Sets the given RobotModel, RobotModelLink, named link, indexed link,
        or object name to some RGBA color (each channel in the range [0,1])."""
        recursive=False
        target_name = None
        if isinstance(target, (int, float, complex)):
            robot = self.world.robot(0)
            target_as_link = robot.link(target)
            target_name=target_as_link.getName()

        elif isinstance(target,RobotModelLink): 
            target_name=target.getName()

        elif isinstance(target,RobotModel): 
            target_name=target.getName()
            recursive = True

        elif isinstance(target, str):
            target_name=target
            if target in self._extras:
                type,data = self._extras[target]
                if type == 'Config':
                    target_name = data
                    recursive = True
                elif type == 'Configs' or type == 'Trajectory':
                    #it's a group set everything under the group
                    self.beginRpc(strict=False)
                    for subitem in data:
                        self.setColor(subitem,r,g,b,a)
                    self.endRpc(strict=False)
                    return
            else:
                #see if it's the name of a robot
                try:
                    self.world.robot(target).index
                    recursive = True
                except Exception:
                    found = False
                    for r in range(self.world.numRobots()):
                        if self.world.robot(r).link(target) >= 0:
                            found = True
                            break
                    if not found:
                        raise ValueError("ERROR: setColor requires target of either robot, link, index, or string name of object!")
        else:
            raise ValueError("ERROR: setColor requires target of either robot, link, index, or string name of object!")

        rgba_color = [r,g,b,a]

        if recursive:
            self._do_rpc({'type':'set_color','object':target_name,'rgba':rgba_color,'recursive':True})
        else:
            self._do_rpc({'type':'set_color','object':target_name,'rgba':rgba_color})
        #print "Setting link color!",('object',target_name,'rgba'),rgba_color

    def setTransform(self,name,R=so3.identity(),t=[0]*3,matrix=None):
        """Sets the transform of the target object.  If matrix is given, it's a 16-element 
        array giving the 4x4 homogeneous transform matrix, in row-major format.  Otherwise,
        R and t are the 9-element klampt.so3 rotation and 3-element translation."""
        if matrix != None:
            self._do_rpc({'type':'set_transform','object':name,'matrix':matrix})
        else:
            self._do_rpc({'type':'set_transform','object':name,'matrix':[R[0],R[3],R[6],t[0],R[1],R[4],R[7],t[1],R[2],R[5],R[8],t[2],0,0,0,1]})

    def addGhost(self,name="ghost",robot=0):
        """Adds a ghost configuration of the robot that can be posed independently.
        name can be set to identify multiple ghosts. 

        The identifier of the ghost in the three.js scene is prefixname + robot.getName(),
        and all the links are identified by prefixname + link name."""
        if robot < 0 or robot >= self.world.numRobots():
            raise ValueError("Invalid robot specified")
        target_name=self.world.robot(robot).getName()
        self._do_rpc({'type':'add_ghost','object':target_name,'prefix_name':name})
        self._extras[name] = ('Config',name+target_name)
        return name

    def getRobotConfig(self,robot=0):
        """A convenience function.  Gets the robot's configuration in the visualization
        world."""
        if robot < 0 or robot >= self.world.numRobots():
            raise ValueError("Invalid robot specified")
        robot = self.world.robot(robot) 
        q = robot.getConfig()
        return q

    def setGhostConfig(self,q,name="ghost",robot=0):
        """Sets the configuration of the ghost to q.  If the ghost is named, place its name
        in prefixname."""
        if robot < 0 or robot >= self.world.numRobots():
            raise ValueError("Invalid robot specified")
        robot = self.world.robot(robot) 

        q_original = robot.getConfig()

        if len(q) != robot.numLinks():
            raise ValueError("Config must be correct size: %d != %d"%(len(q),robot.numLinks()))
        robot.setConfig(q)

        self.beginRpc(strict=False)
        rpcs = []
        for i in range(robot.numLinks()):
            T = robot.link(i).getTransform()
            p = robot.link(i).getParent()

            if p>=0:
                Tp = robot.link(p).getTransform()
                T = se3.mul(se3.inv(Tp),T)

            mat = se3.homogeneous(T)
            #mat is now a 4x4 homogeneous matrix 

            linkname = name+robot.link(i).getName()
            #send to the ghost link with name "name"...
            self._do_rpc({'type':'set_transform','object':linkname,'matrix':[mat[0][0],mat[0][1],mat[0][2],mat[0][3],mat[1][0],mat[1][1],mat[1][2],mat[1][3],mat[2][0],mat[2][1],mat[2][2],mat[2][3],mat[3][0],mat[3][1],mat[3][2],mat[3][3]]})
        self.endRpc(strict=False)

        robot.setConfig(q_original) #restore original config

    def addText(self,name="HUD_Text1",text="",position=None):
        """Adds a new piece of text displayed on the screen.  name is a unique identifier of
        the text, and position=(x,y) are the coordinates of upper left corner of the the text,
        in percent. """
        if position is None:
            x,y = None,None
        else:
            x,y = position
        self._extras[name] = ('Text',(x,y,text))
        self._do_rpc({'type':'add_text','name':name,'x':x,'y':y,'text':text})

    def addSphere(self,name="Sphere1",x=0,y=0,z=0,r=1):
        """Adds a new sphere to the world with the given x,y,z position and radius r."""
        self._extras[name] = ('Sphere',(x,y,z,r))
        self._do_rpc({'type':'add_sphere','name':name,'x':x,'y':y,'z':z,'r':r})

    def addLine(self,name="Line1",x1=0,y1=0,z1=0,x2=1,y2=1,z2=1):
        """Adds a new line segment to the world connecting point (x1,y1,z1) to (x2,y2,z2)"""
        verts = [x1,y1,z1,x2,y2,z2]
        self._extras[name] = ('Line',verts)
        self._do_rpc({'type':'add_line','name':name,'verts':verts})

    def addXform(self,name="Xform1",length=DEFAULT_AXIS_LENGTH,width=DEFAULT_AXIS_WIDTH):
        """Adds a new transform widget to the world with the given line length and width"""
        self._extras[name] = ('RigidTransform',(length,width))
        self._do_rpc({'type':'add_xform','name':name,'length':length,'width':width})

    def addPolyline(self,name="Line1",pts=[]):
        """Adds a new polygonal line segment to the world connecting the given list of 3-tuples"""
        verts = sum(pts,[])
        self._extras[name] = ('Line',verts)
        self._do_rpc({'type':'add_line','name':name,'verts':verts})

    def addTriangle(self,name="Tri1",a=(0,0,0),b=(1,0,0),c=(0,1,0)):
        """Adds a new triangle with vertices a,b,c.  a,b, and c are 3-lists or 3-tuples."""
        verts = a+b+c
        self._extras[name] = ('Trilist',verts)
        self._do_rpc({'type':'add_trilist','name':name,'verts':verts})

    def addQuad(self,name="Quad1",a=(0,0,0),b=(1,0,0),c=(1,1,0),d=(0,1,0)):
        """Adds a new quad (in CCW order) with vertices a,b,c,d.  a,b,c and d are 3-lists or 3-tuples."""
        verts = a+b+c+a+c+d
        self._extras[name] = ('Trilist',verts)
        self._do_rpc({'type':'add_trilist','name':name,'verts':verts}) 

    def addBillboard(self,name="Billboard",image=[[]],format='auto',crange=[0,1],colormap='auto',filter='linear',size=(1,1)):
        """Adds a 2D billboard to the world.  The image is a 2D array of values, which is
        texure-mapped to a quad of size (w,h).  The format argument determines the format of the image
        data and the colormap argument specifies how the image affects the billboard appearance.

        By default the billboard is centered at (0,0,0) and faces up.  To modify it, call set_transform.

        - image: a 2D array of single-channel values, (r,g,b) tuples, or (r,g,b,a) tuples.  Rows are listed
          top to bottom, rows from left to right.  Or, can also be a URL.
        - format:
          - 'auto': autodetect the type from the image. If the image contains values, the format is 'value'.
          - 'value': the values are mapped through either 'opacity', 'rainbow', or gradient
            color mapping.
          - 'rgb': if the image contains values, they are interpreted as RGB values packed in 24 bit
            integers. Otherwise, the first 3 channels of the tuple are used.
          - 'rgba': if the image contains values, they are interpreted as RGB values packed in 32 bit
            integers. Otherwise, they are assumed to be (r,g,b,a) tuples
        - crange: the range of the given values / channels. By default [0,1], but if you are using uint8
          encoding this should be set to [0,255].
        - colormap: how the color of the billboard should be set based on the image.  Valid values are:
          - 'auto': if the image contains values, the gradient ((0,0,0),(1,1,1)) is used.  Otherwise
            'replace' is used.
          - (color1,color2): interpolates between the two given (r,g,b) or (r,g,b,a) tuples.
          - 'opacity': sets the alpha channel only.
          - 'modulate': the value / rgb / rgba texture modulates the billboard color as set by setColor
        - filter: how values between pixels are interpolated.  Either 'nearest' or 'linear'.
        - size: the (width,height) pair of the billboard, in world units.
        """
        if not isinstance(image,str):
            import struct
            import base64
            bytes = []
            w,h = None,None
            h = len(image)
            for row in image:
                if w == None:
                    w = len(row)
                else:
                    assert w == len(row),"Image is not a 2D array"
            pixel = image[0][0]
            if format == 'auto':
                if hasattr(pixel,'__iter__'):
                    if len(pixel) == 4:
                        format = 'rgba'
                    else:
                        format = 'rgb'
                else:
                    format = 'value'
            else:
                if not hasattr(pixel,'__iter__'):
                    format = 'p'+format
            gradient = (type(colormap) != str)
            for row in image:
                for pixel in row:
                    if format == 'value':
                        u = min(1,max(0,(pixel - crange[0]) / (crange[1]-crange[0])))
                        if gradient:
                            color = vectorops.interpolate(gradient[0],gradient[1],u)
                            r = 0xff * min(1,max(0,color[0]))
                            g = 0xff * min(1,max(0,color[1]))
                            b = 0xff * min(1,max(0,color[2]))
                            packed = (0xff << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
                            bytes.append(struct.pack('<I',packed))
                        else:
                            val = 0xff * u
                            bytes.append(struct.pack('B',val))
                    elif format == 'prgb' or format == 'prgba':
                        bytes.append(struct.pack('<I', pixel))
                    elif format == 'rgb':
                        r = 0xff * min(1,max(0,(pixel[0] - crange[0]) / (crange[1]-crange[0])))
                        g = 0xff * min(1,max(0,(pixel[1] - crange[0]) / (crange[1]-crange[0])))
                        b = 0xff * min(1,max(0,(pixel[2] - crange[0]) / (crange[1]-crange[0])))
                        packed = (0xff << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
                        bytes.append(struct.pack('<I', packed))
                    elif format == 'rgba':
                        r = 0xff * min(1,max(0,(pixel[0] - crange[0]) / (crange[1]-crange[0])))
                        g = 0xff * min(1,max(0,(pixel[1] - crange[0]) / (crange[1]-crange[0])))
                        b = 0xff * min(1,max(0,(pixel[2] - crange[0]) / (crange[1]-crange[0])))
                        a = 0xff * min(1,max(0,(pixel[3] - crange[0]) / (crange[1]-crange[0])))
                        packed = (int(a) << 24) | (int(b) << 16) | (int(g) << 8) | int(r)
                        bytes.append(struct.pack('<I', packed))
                    else:
                        raise ValueError("Invalid format "+format)
            image = base64.b64encode(''.join(bytes))
            self._do_rpc({'type':'add_billboard','name':name,'imagedata':image,'width':w,'height':h,'size':size,'filter':filter,'colormap':colormap})
        else:
            self._do_rpc({'type':'add_billboard','name':name,'image':image,'size':size,'filter':filter,'colormap':colormap})
        self._extras[name] = ('Billboard',image)

    def beginRpc(self,strict=True):
        """Begins collecting a set of RPC calls to be sent at once, which is a bit faster than doing multiple
        addX or setX calls. 

        Usage:
          widget.beginRpc()
          widget.addX()
          ...
          widget.setX()
          widget.endRpc()  #this sends all the messages at once
        """
        if self._aggregating_rpc == 0:
            assert len(self._rpc_calls)==0
        if self._aggregating_rpc != 0 and strict:
            raise RuntimeError("Each beginRpc() call must be ended with an endRpc() call")
        self._aggregating_rpc += 1
        return

    def _do_rpc(self,msg):
        """Internally used to send or queue an RPC call"""
        if self._aggregating_rpc:
            self._rpc_calls.append(msg)
        else:
            self.rpc = msg

    def endRpc(self,strict=True):
        """Ends collecting a set of RPC calls to be sent at once, and sends the accumulated message"""
        if self._aggregating_rpc <= 0 or (self._aggregating_rpc!=1 and strict):
            raise ValueError("Each beginRpc() call must be ended with an endRpc() call")
        self._aggregating_rpc -= 1
        if self._aggregating_rpc == 0 and len(self._rpc_calls) > 0:
            self.rpc = {'type':'multiple','calls':self._rpc_calls}
            self._rpc_calls = []


    @observe('_camera')
    def _recv_camera(self,cam):
        #trigger an update?
        marked = cam['new'].copy()
        marked['r'] = 1
        self._camera = marked

    @observe('events')
    def _recv_events(self,events):
        elist = events['new']
        if len(elist) > 0:
            for event in elist:
                self.on_event(event)
            self.events = []

    @observe('drawn')
    def _recv_drawn(self,drawn):
        self.drawn = 0
        print("Klampt widget drawn!")

    def on_event(self,e):
        print("KlamptWidget got event",e)



def EditConfig(robot,klampt_widget=None,ghost=None,link_selector='slider',link_subset=None,callback=None):
    """Creates a Jupyter widget for interactive editing of the robot's configuration.

    Arguments:
    - robot: the robot to edit
    - klampt_widget: the KlamptWidget visualization to update, or None if you don't want to visualize the
      editing.
    - ghost: if not None, this is the name of the ghost that should be updated. Widget updates are shown on
      the given ghost rather than the actual robot.  To get the ghost configuration, you'll need to update the callback
    - link_selector: how to select links.  Either:
      - 'slider': uses an IntSlider widget
      - 'dropdown': uses a Dropdown widget
      - 'all': shows sliders for all links
    - link_subset: if given, only a subset of links are shown.  Otherwise, only non-fixed links are shown.
    - callback: a function callback(index,q) called when a DOF's value has changed.

    Returns: a widget to be displayed as you like
    """

    qmin,qmax = robot.getJointLimits()
    qedit = robot.getConfig()[:]
    if link_subset == None:
        link_subset = [i for i in range(robot.numLinks()) if qmin[i] != qmax[i]]
    else:
        for link in link_subset:
            if link < 0 or link >= robot.numLinks():
                raise ValueError("Invalid link specified in link_subset")
        link_subset = link_subset[:]

    def _dochange_link(link):
        if not math.isinf(qmin[link]):
            joint_slider.min = qmin[link]
            joint_slider.max = qmax[link]
        else:
            joint_slider.min = -2
            joint_slider.max = 2
        joint_slider.value = qedit[link]
        if klampt_widget and ghost == None:
            #show selected link in color
            #restore old colors
            klampt_widget.beginRpc()
            for i in link_subset:
                klampt_widget.setColor(i,*robot.link(link).appearance().getColor())
            #change new color
            color = robot.link(link).appearance().getColor()
            r,g,b,a = color
            r = 1.0-(1.0-r)*0.5
            g = 1.0-(1.0-g)*0.5
            klampt_widget.setColor(link,r,g,b,a)
            klampt_widget.endRpc()

    def _dochange(link,value):
        if ghost:
            qold = robot.getConfig()
        qedit[link] = value
        robot.setConfig(qedit)
        if klampt_widget:
            if ghost:
                klampt_widget.setGhostConfig(qedit,ghost,robot.index)
            else:
                klampt_widget.update()
        if ghost:
            robot.setConfig(qold)
        if callback:
            callback(link,qedit)

    if link_selector == 'slider':
        link_slider=widgets.IntSlider(description='Link',min=0,max=len(link_subset)-1,value=0)
        joint_slider=widgets.FloatSlider(description='Value',min=0,max=1,value=0.5,step=0.001)

        @interact(index=link_slider)
        def change_link(index):
            link = link_subset[index]
            _dochange_link(link)
        link_slider.observe(lambda change:change_link(change['new']),'value')
                
        def change_joint_value(value):
            link = link_subset[link_slider.value]
            _dochange(link,value)
        joint_slider.observe(lambda change:change_joint_value(change['new']),'value')
        return widgets.VBox([link_slider,joint_slider])
    elif link_selector == 'dropdown':
        link_dropdown=widgets.Dropdown(description='Link',options=[robot.link(i).getName() for i in link_subset],value=robot.link(link_subset[0]).getName())
        joint_slider=widgets.FloatSlider(description='Value',min=0,max=1,value=0.5,step=0.001)

        def change_link(name):
            link = robot.link(name).index
            _dochange_link(link)
        link_dropdown.observe(lambda change:change_link(change['new']),'value')
                
        def change_joint_value(value):
            link = robot.link(link_dropdown.value).index
            _dochange(link,value)
        joint_slider.observe(lambda change:change_joint_value(change['new']),'value')
        return widgets.VBox([link_dropdown,joint_slider])
    elif link_selector == 'all':
        sliders = []
        for link in link_subset:
            sliders.append(widgets.FloatSlider(description=robot.link(link).getName(),min=qmin[link],max=qmax[link],value=qedit[link],step=0.001))
            sliders[-1].observe(lambda value,link=link:_dochange(link,value['new']),'value')
        return widgets.VBox(sliders)
    else:
        raise ValueError("Invalid link_selector, must be slider, dropdown, or all")

def EditPoint(value=None,min=None,max=None,labels=None,
    klampt_widget=None,point_name='edited_point',point_radius=DEFAULT_POINT_RADIUS,
    callback=None):
    """Creates a Jupyter widget for interactive editing of an xyz point

    Arguments:
    - value: the initial value of the point. If given, this must be a list and will hold the edited values.
    - min/max: the minimum and maximum of the point
    - labels: if given, the labels of each channel
    - klampt_widget: the KlamptWidget visualization to update, or None if you don't want to visualize the point.
    - point_name: the name of the point in the visualization world to edit.
    - point_radius: the radius of the visualized point.
    - callback: a function callback(xyz) called when a DOF's value has changed.

    Returns: a VBox widget that can be displayed as you like
    """
    if value is None:
        value = [0,0,0]
    else:
        if not isinstance(value,list):
            raise ValueError("value must be a 3-element list")
        if len(value) != 3:
            raise ValueError("value must be a 3-element list")
    if labels is None:
        labels = 'xyz'
    if min is None:
        min = [-5,-5,-5]
    elif isinstance(min,(int,float)):
        min = [min,min,min]
    if max is None:
        max = [5,5,5]
    elif isinstance(max,(int,float)):
        max = [max,max,max]
    if len(min) != 3:
        raise ValueError("min must be a 3-element list")
    if len(max) != 3:
        raise ValueError("max must be a 3-element list")
    if klampt_widget:
        klampt_widget.addSphere(name=point_name,x=value[0],y=value[1],z=value[2],r=point_radius)
    def _dochange(index,element):
        value[index] = element
        if klampt_widget:
            klampt_widget.addSphere(name=point_name,x=value[0],y=value[1],z=value[2],r=point_radius)
        if callback:
            callback(value)
    elems = []
    for i in range(3):
        elems.append(widgets.FloatSlider(description=labels[i],value=value[i],min=min[i],max=max[i],step=0.001))
        elems[-1].observe(lambda v,i=i:_dochange(i,v['new']),'value')
    return widgets.VBox(elems)

def EditTransform(value=None,xmin=None,xmax=None,labels=None,
    klampt_widget=None,xform_name='edited_xform',axis_length=DEFAULT_AXIS_LENGTH,axis_width=DEFAULT_AXIS_WIDTH,
    callback=None):
    """Creates a Jupyter widget for interactive editing of a rigid transform point

    Arguments:
    - value: the initial value of the transform (klampt.se3 element). If given as (R,t), the R and t members must be lists and will hold the edited values.
    - xmin/xmax: the minimum and maximum of the translation
    - labels: if given, the labels of roll,pitch,yaw and x,y,z
    - klampt_widget: the KlamptWidget visualization to update, or None if you don't want to visualize the point.
    - xform_name: the name of the xform in the visualization world to edit.
    - axis_length,axis_width: the radius of the visualized point.
    - callback: a function callback((R,t)) called when a DOF's value has changed.

    Returns: a VBox widget that can be displayed as you like
    """
    if value is None:
        value = se3.identity()
    else:
        if not isinstance(value,(tuple,list)):
            raise ValueError("value must be a 2-element sequence")
        if len(value) != 2:
            raise ValueError("value must be a 2-element sequence")
        if len(value[0]) != 9:
            raise ValueError("value[0] must be a 9-element list")
        if len(value[1]) != 3:
            raise ValueError("value[1] must be a 3-element list")
    if labels is None:
        labels = ['roll','pitch','yaw','x','y','z']
    if xmin is None:
        xmin = [-5,-5,-5]
    elif isinstance(xmin,(int,float)):
        xmin = [xmin,xmin,xmin]
    if xmax is None:
        xmax = [5,5,5]
    elif isinstance(xmax,(int,float)):
        xmax = [xmax,xmax,xmax]
    if len(xmin) != 3:
        raise ValueError("xmin must be a 3-element list")
    if len(xmax) != 3:
        raise ValueError("xmax must be a 3-element list")
    if klampt_widget:
        klampt_widget.addXform(name=xform_name,length=axis_length,width=axis_width)
        klampt_widget.setTransform(name=xform_name,R=value[0],t=value[1])
    rpy = list(so3.rpy(value[0]))
    def _do_rotation_change(index,element):
        rpy[index] = element
        value[0][:] = so3.from_rpy(rpy)
        if klampt_widget:
            klampt_widget.setTransform(name=xform_name,R=value[0],t=value[1])
        if callback:
            callback(value)
    def _do_translation_change(index,element):
        value[1][index] = element
        if klampt_widget:
            klampt_widget.setTransform(name=xform_name,R=value[0],t=value[1])
        if callback:
            callback(value)
    elems = []
    for i in range(3):
        elems.append(widgets.FloatSlider(description=labels[i],value=rpy[i],min=0,max=math.pi*2,step=0.001))
        elems[-1].observe(lambda v,i=i:_do_rotation_change(i,v['new']),'value')
    for i in range(3):
        elems.append(widgets.FloatSlider(description=labels[3+i],value=value[1][i],min=xmin[i],max=xmax[i],step=0.001))
        elems[-1].observe(lambda v,i=i:_do_translation_change(i,v['new']),'value')
    return widgets.VBox(elems)

class Playback(widgets.VBox):
    """A play/pause/reset widget associated with a KlamptWidget.

    Members:
    - klampt_widget: the widget that should be updated after each advance call
    - advance: a function to be called for each new frame.
    - pause: a function to be called when pause is clicked.
    - reset: a function to be called when reset is clicked.
    - maxframes: the maximum number of frames.  If None, this is unlimited.
    - framerate: number of frames per second desired.  If None, frames are run as
      quickly as possible
    - quiet: if True, suppresses output during play
    - playbutton, stepbutton, pausebutton, resetbutton: the Button widgets
    """
    def __init__(self,klampt_widget=None,advance=None,reset=None,pause=None,maxframes=None,framerate=None,quiet=False):
        """Arguments are the same as the members"""

        self.klampt_widget = klampt_widget
        self.advance = advance
        self.reset = reset
        self.pause = pause
        self.maxframes = maxframes
        self.framerate = framerate
        self.quiet = quiet
        self.playbutton = widgets.Button(
            description='Play',
            disabled=False,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Start the animation',
            icon='play')
        self.stepbutton = widgets.Button(
            description='Step',
            disabled=False,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Step the animation',
            icon='step-forward')
        self.pausebutton = widgets.Button(
            description='Pause',
            disabled=True,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Pause the animation',
            icon='pause')
        self.resetbutton = widgets.Button(
            description='Reset',
            disabled=False,
            button_style='', # 'success', 'info', 'warning', 'danger' or ''
            tooltip='Reset the animation',
            icon='undo')
        lock =  threading.Lock()
        playdata = {'thread':None,'stop':0}
        self.playdata = playdata
        self.lock = lock
        self.frame = 0

        #If we don't create this now, exceptions will never be printed
        self.out = widgets.Output()

        def play_thread_func(lock,playdata):
            #print "Starting play thread"
            if self.framerate is None:
                dt = 0
            else:
                dt = 1.0/self.framerate
            playdata['stop'] = 0

            def do_advance(drawn=False):
                if playdata['stop']:
                    return
                lock.acquire()
                try:
                    self._advance()
                except Exception as e:
                    with self.out:
                        print("Exception occurred during Playback.advance, stopping animation")
                        print(e)
                    playdata['stop'] = 1
                    lock.release()
                    return
                self.frame += 1
                lock.release()

            if self.klampt_widget:
                self.klampt_widget.observe(do_advance,'drawn')
            t0 = time.time()
            do_advance()
            while True:
                if playdata['stop']:
                    break
                lock.acquire()
                if self.maxframes is not None and self.frame >= self.maxframes:
                    #print "Stopping play by completion"
                    self.playbutton.disabled = False
                    self.pausebutton.disabled = True
                    self.frame = 0
                    lock.release()
                    break
                lock.release()
                if not self.klampt_widget:
                    do_advance()
                t1 = time.time()
                time.sleep(max(dt-(t1-t0),0))
                t0 = t1
            if self.klampt_widget:
                self.klampt_widget.unobserve(do_advance,'drawn')
            playdata['thread'] = None
            return

        def on_play(b):
            #print "Play clicked"
            self.pausebutton.disabled = False
            self.playbutton.disabled = True
            assert playdata['thread'] == None
            playdata['thread'] = threading.Thread(target=play_thread_func,args=(lock,playdata))
            playdata['thread'].start()
        def on_pause(b):
            #print "Pause clicked"
            self.stop()
            self._pause()
        def on_step(b):
            #print "Step clicked"
            self.stop()
            self.frame += 1
            self._advance()
        def on_reset(b):
            #print "Reset clicked"
            self.stop()
            self.frame = 0
            self.out.clear_output()
            self._reset()

        self.playbutton.on_click(on_play)
        self.stepbutton.on_click(on_step)
        self.pausebutton.on_click(on_pause)
        self.resetbutton.on_click(on_reset)
        widgets.VBox.__init__(self,[widgets.HBox([self.playbutton,self.stepbutton,self.pausebutton,self.resetbutton]),
            self.out])

    def stop(self):
        """Stops any ongoing playback"""
        lock = self.lock
        playdata = self.playdata
        if playdata['thread'] is not None: 
            #playing
            lock.acquire()
            playdata['stop'] = 1
            lock.release()
            playdata['thread'].join()
            playdata['thread'] = None
            playdata['stop'] = 0
            self.pausebutton.disabled = True
            self.playbutton.disabled = False
        

    def _advance(self):
        if self.advance:
            if self.quiet:
                self.advance()
            else:
                with self.out:
                    self.advance()
        if self.klampt_widget:
            self.klampt_widget.update()
    def _reset(self):
        if self.reset:
            with self.out:
                self.reset()
        if self.klampt_widget:
            self.klampt_widget.update()
    def _pause(self):
        if self.pause:
            with self.out:
                self.pause()
        if self.klampt_widget:
            self.klampt_widget.update()
