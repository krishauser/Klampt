"""Easy I/O with resources stored on disk and visual editing.

Use the :meth:`~klampt.io.resource.get`,
:meth:`~klampt.io.resource.set`, and
:meth:`~klampt.io.resource.edit` functions to retrieve / store / edit
resources dynamically.

:meth:`~klampt.io.resource.load` and :meth:`~klampt.io.resource.save` launch
a file browser (available in Qt only).

By default, resources are stored in the ``resources/`` subdirectory of the
current working directory.
Use :meth:`~klampt.io.resource.get_directory` and
:meth:`~klampt.io.resource.set_drectory` to change where resources are
stored  Alternatively, the ``directory=[DIRNAME]`` keyword
argument can be provided to get, set, load, and save.

.. warning::

    Don't use ``from klampt.io.resource import *``, because this will
    override the built-in set class.

Example usage can be seen in Klampt-examples/Python/demos/resourcetest.py.
"""

from . import loader
from ..model import trajectory
from ..model import multipath
from ..model import types
from .. import robotsim
from ..math import vectorops,se3,so3
import os
import time
from ..robotsim import WorldModel,RobotModel,RobotModelLink,RigidObjectModel,IKObjective
from ..model.subrobot import SubRobotModel
from ..model.contact import ContactPoint
from ..model.contact import Hold
from .. import vis
from ..vis import editors
from ..vis.backends import vis_gl
import warnings

global _directory
global _editTemporaryWorlds
global _thumbnail_window
_directory = 'resources'
_thumbnail_window = None

import collections

class _LRUCache:
    def __init__(self, capacity):
        self.capacity = capacity
        self.cache = collections.OrderedDict()

    def __getitem__(self, key):
        try:
            value = self.cache.pop(key)
            self.cache[key] = value
            return value
        except KeyError:
            raise

    def __setitem__(self, key, value):
        try:
            self.cache.pop(key)
        except KeyError:
            if len(self.cache) >= self.capacity:
                self.cache.popitem(last=False)
        self.cache[key] = value
_editTemporaryWorlds = _LRUCache(10)

def get_directory():
    """Returns the current resource directory."""
    return _directory

def set_directory(value):
    """Sets the current resource directory."""
    global _directory
    _directory = value

def _ensure_dir(f):
    d = os.path.dirname(f)
    if len(d)==0: return
    if not os.path.exists(d):
        os.makedirs(d)

def _get_world(world):
    if isinstance(world,str):
        #a single argument, e.g., a robot file
        global _editTemporaryWorlds
        try:
            return _editTemporaryWorlds[world]
        except KeyError:
            w = WorldModel()
            if not w.readFile(world):
                raise RuntimeError("Error loading world file "+world)
            _editTemporaryWorlds[world] = w
            return w
    return world

def known_extensions():
    """Returns all known resource file extensions"""
    return list(loader.EXTENSION_TO_TYPES.keys())

def known_types():
    """Returns all known resource types"""
    return list(loader.TYPE_TO_EXTENSIONS.keys())+['WorldModel','MultiPath','Point','Rotation','Matrix3','ContactPoint']

def visual_edit_types():
    """Returns types that can be visually edited"""
    return ['Config','Configs','Trajectory','Vector3','Point','RigidTransform','Rotation','WorldModel']

def get(name,type='auto',directory=None,default=None,doedit='auto',description=None,editor='visual',world=None,referenceObject=None,frame=None):
    """Retrieve a resource of the given name from the current resources
    directory.  If the resource does not already exist, an edit prompt will be
    shown, and the result from the prompt will be saved to disk under the
    given name.

    Resources may be of type Config, Configs, IKGoal, Hold,
    Stance, MultiPath, Trajectory/LinearPath, and ContactPoint, or any of
    the basic Klampt object types, geometry types, and math types.

    Resources can also be edited using ``klampt_browser`` and RobotPose.

    Args:
        name (str): the resource name.  If type='auto', this is assumed to have
            a suffix of a file of the desired type.  The name can also be
            nested of the form 'group/subgroup/name.type'.
        type (str): the resource type, or 'auto' to determine it automatically.
        directory (str, optional): the search directory.  If None, uses the 
            current resource directory.
        default (optional): the default value if the resource does not exist on
            disk. If None, some arbitrary default value will be inferred.
        doedit: if 'auto', if the resource does not exist on disk, an
            edit prompt will be displayed.  If False, an RuntimeError will be
            raised if the resource does not exist on disk.  If True, the
            user will be given an edit prompt to optionally edit the resource
            before this call returns.
        description (str, optional): an optional text description of the
            resource, for use in the edit prompt.
        editor (str): either 'visual' or 'console', determining whether to use
            the visual OpenGL or console editor.
        world (WorldModel, optional): for a visual editor, this world will be 
            shown along with the item to edit.  If this is a string it points
            to a file that will be loaded for the world (e.g., a world XML
            file, or a robot file).
        referenceObject (optional): to give visual reference points, one or 
            more RobotModels, ObjectModels, Geometry3D's, or RobotModelLink's
            may be designated to follow the edited object.  Currently works
            with Config's / Configs' / Trajectories / rigid transforms /
            rotations / points.
        frame (optional): for rigid transforms / rotations / points, the 
          reference frame in which the quantity is represented.  This is an 
          element of se3, or an ObjectModel, or a RobotModelLink, or a string 
          indicating a named rigid element of the world.

    Returns:
        Klampt object or None: If the named resource exists, loads and returns 
        a Klampt object.  If the object doesn't exist and ``doedit=True`` and
        the user presses OK on the editor, then the object will be saved as the
        resource ``name``.  Otherwise, ``default`` will be returned.
    """
    if name==None:
        if doedit==False:
            raise RuntimeError("Can't get() an anonymous resource without launching editor")
        success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,referenceObject=referenceObject,frame=frame)
        if not success:
            print("Cancel pressed, returning None")
            return None
        else:
            print("Ok pressed, returning anonymous resource")
            return newvalue
    if directory==None:
        directory = get_directory()
    if type == 'auto':
        type = loader.filename_to_type(name)
    value = None
    fn = os.path.join(directory,name)
    try:
        try:
            value = loader.load(type,fn)
        except IOError:
            raise
        except Exception as e:
            import traceback
            print("Unable to read object from file "+fn)
            print("Traceback: ")
            traceback.print_exc()
            raise IOError()
        if value==None:
            raise IOError("Unable to load from file "+fn)
        if doedit==True:
            success,newvalue = edit(name,value=value,type=type,description=description,editor=editor,world=world,referenceObject=referenceObject,frame=frame)
            if success:
                print("Ok pressed, saving resource to",name)
                value = newvalue
                set(name,value,type=type,directory=directory)
            else:
                print("Cancel pressed, not saving resource to disk")
            return value
        return value
    except IOError as e:
        if doedit!=False:
            print("Resource",fn,"does not exist, launching editor...")
            success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,referenceObject=referenceObject,frame=frame)
            if success:
                print("Ok pressed, saving resource to",name)
                value = newvalue
                set(name,value=value,type=type,directory=directory)
            else:
                print("Cancel pressed, not saving resource to disk")
            return value
        elif default is not None:
            return default
        else:
            print("IO error")
            print(e)
            raise RuntimeError("Resource "+name+" does not exist")
    return

def set(name,value,type='auto',directory=None):
    """Saves a resource to disk under the given name.

    Args:
        name (str): the file name.  Please make sure to get the right file
            extension.  .json files are also OK for many types.
        value: a Klamp't resource (see list of compatible types in get())
        type (str, optional): The resource type.  If 'auto', the type is
            determined by the file extension.  If this fails, the type is
            determined by the value.

    Returns:
        bool: True on success, False otherwise
    """
    if type == 'auto':
        try:
            type = loader.filename_to_type(name)
        except Exception:
            type = types.object_to_types(value)
            if isinstance(type,(list,tuple)):
                type = type[0]
    if directory==None:
        directory = get_directory()    
    fn = os.path.join(directory,name)
    _ensure_dir(fn)
    if type == 'xml':
        #loader can now save some xml files
        return loader.save(value,'auto',fn)
        #if hasattr(value,'saveFile'):
        #    return value.saveFile(fn)
        #raise NotImplementedError("TODO: save other xml files from Python API")
    else:
        return loader.save(value,type,fn)

class FileGetter:
    def __init__(self,title="Open file"):
        self.title = title
        self.directory = ''
        self.filetypes = []
        self.result = None
    def getOpen(self):
        from PyQt5.QtWidgets import QFileDialog
        defaultpattern = None
        patternlist = []
        for (desc,exts) in self.filetypes:
            if defaultpattern is None:
                defaultpattern = exts[0]
            pattern = desc + " ("
            pattern = pattern + ' '.join("*"+ext for ext in exts) + ")"
            patternlist.append(pattern)
        patternlist.append("All files (*.*)")
        if defaultpattern is None:
            defaultpattern = '.txt'
        #print("Pattern list:",patternlist)
        patterns = ";;".join(patternlist)
        self.result = QFileDialog.getOpenFileName(None, self.title, self.directory+'/untitled'+defaultpattern, patterns)
        print("Result from open dialog",self.result)
        if isinstance(self.result,tuple):
            self.result = self.result[0]
    def getSave(self):
        from PyQt5.QtWidgets import QFileDialog
        defaultpattern = None
        patternlist = []
        for (desc,exts) in self.filetypes:
            if defaultpattern is None:
                defaultpattern = exts[0]
            pattern = desc + " ("
            pattern = pattern + ' '.join("*"+ext for ext in exts) + ")"
            patternlist.append(pattern)
        patternlist.append("All files (*.*)")
        if defaultpattern is None:
            defaultpattern = '.txt'
        #print("Pattern list:",patternlist)
        patterns = ";;".join(patternlist)
        self.result = QFileDialog.getSaveFileName(None, self.title, self.directory+'/untitled'+defaultpattern, patterns)
        print("Result from save dialog",self.result)
        if isinstance(self.result,tuple):
            self.result = self.result[0]

def load(type=None,directory=None):
    """Pops up a dialog that asks the user to load a resource file of a
    given type. 

    Args:
        type (str, optional): The Klampt type the user should open.
            If not given, all resource file types are shown in the
            dialog as options.
        directory (str, optional): if given, overrides the current resources
            directory.

    Returns:
        tuple or None: a (filename,value) pair if OK is pressed, or
        None if the operation was canceled
    """
    
    fg = FileGetter('Open resource')
    fg.directory = directory
    if directory==None:
        fg.directory = get_directory()    
    if type is not None:
        extensions=[v for v in loader.TYPE_TO_EXTENSIONS[type]]
        extensions.append('.json')
        fg.filetypes.append((type,extensions))

    def make_getfilename(glbackend):
        fg.getOpen()
        return None
    #These gymnastics are necessary because Qt can only be run in a single thread, and to be compatible 
    #with the visualization you need to use the customUI functions
    old_window = vis.getWindow()
    global _thumbnail_window
    if _thumbnail_window is None:
        _thumbnail_window = vis.createWindow("")
    vis.setWindow(_thumbnail_window)
    vis.customUI(make_getfilename)
    vis.dialog()
    vis.customUI(None)
    vis.setWindow(old_window)
    if len(fg.result) == 0:
        return None
    fn = str(fg.result)
    return fn,get(fn,('auto' if type is None else type),'',doedit=False)

def save(value,type='auto',directory=None):
    """Pops up a dialog that asks the user to save a resource to a
    file of the correct type. 

    Args:
        value: a Klamp't object that has a resource type
        type (str, optional): The Klampt type the user should open.
            If 'auto', the type is auto-detected.
        directory (str, optional): if given, overrides the current resources
            directory.

    Returns:
        str or None: the file saved to, if OK is pressed, or
        None if the operation was canceled.
    """
    fg = FileGetter('Save resource')
    fg.directory = directory
    if directory==None:
        fg.directory = get_directory()    
    if type == 'auto':
        typelist = types.object_to_types(value)
        if isinstance(typelist,str):
            typelist = [typelist]
    else:
        typelist = [type] 
    for type in typelist:
        extensions=[v for v in loader.TYPE_TO_EXTENSIONS[type]]
        extensions.append('.json')
        print("Available extensions for objects of type",type,":",extensions)
        fg.filetypes.append((type,extensions))

    def make_getfilename(glbackend):
        fg.getSave()
        return None
    #These gymnastics are necessary because Qt can only be run in a single thread, and to be compatible 
    #with the visualization you need to use the customUI functions
    old_window = vis.getWindow()
    global _thumbnail_window
    if _thumbnail_window is None:
        _thumbnail_window = vis.createWindow("")
    vis.setWindow(_thumbnail_window)
    vis.customUI(make_getfilename)
    vis.dialog()
    vis.customUI(None)
    vis.setWindow(old_window)
    if len(fg.result) == 0:
        return None
    if set(str(fg.result),value,type,''):
        return str(fg.result)
    return None

class _ThumbnailPlugin(vis_gl.GLVisualizationPlugin):
    def __init__(self,world):
        vis_gl.GLVisualizationPlugin.__init__(self)
        self.world = world
        self.done = False
        self.rendered = 0
        self.image = None
    def display(self):
        self.rendered += 1
        vis_gl.GLVisualizationPlugin.display(self)
    def idle(self):
        vis_gl.GLVisualizationPlugin.idle(self)
        if self.rendered >= 2 and not self.done:
            from OpenGL.GL import glReadPixels,GL_RGBA,GL_UNSIGNED_BYTE
            view = self.window.program.view
            screenshot = glReadPixels( view.x, view.y, view.w, view.h, GL_RGBA, GL_UNSIGNED_BYTE)
            try:
                from PIL import Image
                self.image = Image.frombuffer("RGBA", (view.w, view.h), screenshot, "raw", "RGBA", 0, 0)
            except ImportError:
                try:
                    import Image
                    self.image = Image.frombuffer("RGBA", (view.w, view.h), screenshot, "raw", "RGBA", 0, 0)
                except ImportError:
                    self.image = screenshot
            self.done = True
        return True

def thumbnail(value,size,type='auto',world=None,frame=None):
    """Retrieves an image of the given item, resized to the given size.

    Args:
        value: a resource type.
        size (pair of ints): the (width,height) of the thumbnail, in pixels.
        type (str, optional): the type of value
        world (WorldModel, optional): if given, the resource will be drawn with
            this world in the background.  If the resource needs an associated
            object (e.g., Config, Configs, Trajectory), the object will be drawn
            with that given object.
        frame (se3 element, optional): not supported yet.  Will eventually let
            you draw Points or RigidTransforms relative to some reference
            frame.

    Returns:
        Image or bytes: A PIL Image if PIL is available, or just a raw RGBA
        memory buffer otherwise.

    """
    global _thumbnail_window
    world = _get_world(world)
    if isinstance(value,WorldModel):
        world = value
        value = None
    if type == 'auto' and value is not None:
        typelist = types.object_to_types(value)
        if isinstance(typelist,(list,tuple)):
            type = typelist[0]
        else:
            type = typelist
        if type == None:
            raise ValueError("Un-recognized type")
        if type=='Config' and world is None and len(value)==3:
            type = 'Vector3'
    if type in ['Config','Configs','Trajectory','IKGoal']:
        if world is None:
            raise ValueError("Need a world to draw a thumbnail of type "+type)
    if frame != None:
        if type not in ['RigidTransform','Vector3','Matrix3']:
            raise ValueError("Can't accept frame argument for objects of type "+type)
    old_window = vis.getWindow()
    if _thumbnail_window is None:
        _thumbnail_window = vis.createWindow("")
    vis.setWindow(_thumbnail_window)
    assert not vis.shown()
    vp = vis.getViewport()
    vp.w,vp.h = size
    if vp.w < 256 or vp.h < 256:
        vp.w = vp.w *256 / min(vp.w,vp.h)
        vp.h = vp.h *256 / min(vp.w,vp.h)
    vis.setViewport(vp)
    vp = vis.getViewport()
    plugin = _ThumbnailPlugin(world)
    if world:
        plugin.add("world",world)
    if value is not None:
        if type == 'Config':
            world.robot(0).setConfig(value)
        else:
            plugin.add("item",value)
    plugin.autoFitCamera()
    vis.setPlugin(plugin)
    vis.show()
    plugin.rendered = 0
    while not plugin.done:
        time.sleep(0.1)
    vis.setPlugin(None)
    vis.show(False)
    vis.setWindow(old_window)
    if (vp.w,vp.h) != size and plugin.image.__class__.__name__=='Image':
        try:
            from PIL import Image
            plugin.image.thumbnail(size,Image.ANTIALIAS)
        except ImportError:
            try:
                import Image
                plugin.image.thumbnail(size,Image.ANTIALIAS)
            except ImportError:
                # if this happens then
                # plugin.image is just a raw RGBA memory buffer
                pass
    return plugin.image

def console_edit(name,value,type,description=None,world=None,frame=None):
    print("*********************************************************")
    print() 
    print("Editing resource",name,"of type",type)
    print()
    if description!=None:
        print(description)
        print()
    if frame!=None:
        if hasattr(frame,'getName'):
            print("Reference frame:",frame.getName())
        else:
            print("Reference frame:",frame)
        print()
    print("*********************************************************")
    print("Current value:",value)
    print("Do you wish to change it? (y/n/q) >", end=' ')
    choice = ''
    while choice not in ['y','n','q']:
        choice = input()[0].lower()
        if choice not in ['y','n','q']:
            print("Please enter y/n/q indicating yes/no/quit.")
            print(">", end=' ')
    if choice=='y':
        print("Enter the new desired value below.  You may use native text,")
        print("JSON strings, or file(fn) to indicate a file name.")
        print("New value >", end=' ')
        import json
        data = input()
        if data.startswith('{') or data.startswith('['):
            jsonobj = json.loads(data)
            try:
                obj = loader.fromJson(jsonobj,type)
                return True,obj
            except Exception:
                print("Error loading from JSON, press enter to continue...")
                input()
                return False,value
        elif data.startswith('file('):
            try:
                obj = get(data[5:-1],type,doedit=False)
                if obj==None:
                    return False,value
                return True,obj
            except Exception:
                print("Error loading from file, press enter to continue...")
                input()
                return False,value
        else:
            try:
                obj = loader.read(type,data)
                return True,obj
            except Exception:
                print("Error loading from text, press enter to continue...")
                input()
                return False,value
    elif choice=='n':
        print("Using current value.")
        print("*********************************************************")
        return False,value
    elif choice=='q':
        return False,None


def edit(name,value,type='auto',description=None,editor='visual',world=None,referenceObject=None,frame=None):
    """Launches an editor for the given value.

    Args:
        name (str): the displayed name of the edited value. Can be None, in
            which case 'Anonymous' is displayed.
        value: the value to be edited.  Can be None, in which case 'type'
            must be specified and a default value is created.
        type (str): the type string of the value to be edited.  Usually can be
            auto-detected from value.
        description (str, optional): a descriptive string, displayed to
            the person editing.
        editor (str): either 'visual' or 'console'.  If 'visual', will display
            a GUI for visually editing the item.
            If 'console', the user will have to type in the value.
        world (WorldModel or str, optional): either a WorldModel
            instance or a string specifying a world file. This is necessary
            for visual editing.
        referenceObject (optional): a RobotModel or other object to which the
            value "refers to".  For configurations and trajectories, this is
            the object that will be moved by the trajectory.  In the case of a
            RigidTransform value, this can be an object or a list of objects
            that will be transformed by the transform.  
        frame (optional): for Vector3, Matrix3, Point, Rotation, and
            RigidTransform types, the returned value will be given relative to
            this reference frame.  The reference frame can be either an element
            of se3, a RigidObjectModel, a RobotModelLink, or a string
            indicating a named rigid element of the world.

    Returns:
        tuple: A pair (save, result) containing:

            * save (bool): True if the user pressed OK, False if Cancel or the
              close box were chosen.
            * result: the edited value.

    """
    if name == None and type=='auto':
        raise RuntimeError("Cannot do an anonymous edit without the 'type' argument specified")
    if name == None:
        name = 'Anonymous'
    if type == 'auto':
        type = types.object_to_types(value)
        if type is None:
            raise RuntimeError("Could not autodetect type of object "+name)
        if isinstance(type,(list,tuple)):
            type = type[0]

    if editor=='visual' and vis.init('PyQt') is None:
        print("PyQt is not available, defaulting to console editor")
        editor = 'console'
            
    world = _get_world(world)
    if isinstance(frame,str):
        try:
            oframe = world.rigidObject(frame)
            frame = oframe
        except RuntimeError:
            try:
                if isinstance(referenceObject,RobotModel):
                    oframe = referenceObject.link(frame)
                    frame = oframe
                else:
                    oframe = world.robot(0).link(frame)
                    frame = oframe
            except RuntimeError:
                try:
                    oframe = world.terrain(frame)
                    frame = oframe
                except RuntimeError:
                    raise RuntimeError('Named frame "'+frame+'" is not a valid frame')
    if type in ['Config','Configs','Trajectory']:
        if world==None and referenceObject==None:
            raise RuntimeError("Cannot visually edit a "+type+" resource without a world/referenceObject argument")
        if referenceObject==None and world.numRobots() > 0:
            referenceObject = world.robot(0)
    
    if value is None:
        value = types.make(type,referenceObject)
        if value == None:
            raise RuntimeError("Don't know how to edit objects of type "+type)

    def attach(objects,editor):
        #attach visualization items to the transform
        if isinstance(objects,RobotModelLink):
            r = objects.robot()
            assert objects.index >= 0 and objects.index < r.numLinks()
            descendant = [False]*r.numLinks()
            descendant[objects.index] = True
            for i in range(r.numLinks()):
                p = r.link(i).getParent()
                if p >= 0 and descendant[p]: descendant[i]=True
            for i in range(r.numLinks()):
                if descendant[i]:
                    editor.attach(r.link(i),se3.mul(se3.inv(objects.getTransform()),r.link(i).getTransform()))
        elif hasattr(objects,'getTransform') or hasattr(objects,'getCurrentTransform'):
            editor.attach(objects)
        elif hasattr(objects,'__iter__') and len(objects) > 0:
            editor.attach(objects[0])
            if hasattr(objects[0],'getTransform'):
                T0 = objects[0].getTransform()
            elif hasattr(objects[0],'getCurrentTransform'):
                T0 = objects[0].getCurrentTransform()
            else:
                T0 = None
            for o in objects[1:]:
                if hasattr(o,'getTransform'):
                    To = o.getTransform()
                elif hasattr(o,'getCurrentTransform'):
                    To = o.getCurrentTransform()
                else:
                    To = None
                if To is None or T0 is None:
                    raise ValueError("Can't attach referenceObjects that have no transform defined?")
                editor.attach(o,se3.mul(se3.inv(T0),To))

    if editor == 'console':
        return console_edit(name,value,type,description,world,frame)
    elif editor == 'visual':
        if type == 'Config':
            assert isinstance(referenceObject,(RobotModel,SubRobotModel)),"Can currently only edit Config values with a RobotModel reference object"
            return editors.run(editors.ConfigEditor(name,value,description,world,referenceObject))
        elif type == 'Configs':
            assert isinstance(referenceObject,(RobotModel,SubRobotModel)),"Can currently only edit Configs values with a RobotModel reference object"
            return editors.run(editors.ConfigsEditor(name,value,description,world,referenceObject))
        elif type == 'Trajectory':
            if len(value.milestones)==0 or len(value.milestones[0])>3:
                assert hasattr(value,'robot') or isinstance(referenceObject,(RobotModel,SubRobotModel)),"Can currently only edit N>3 Trajectory values with a RobotModel reference object"
                editor = editors.TrajectoryEditor(name,value,description,world,referenceObject)
            else:
                editor = editors.TrajectoryEditor(name,value,description,world)
                attach(referenceObject,editor)
            return editors.run(editor)
        elif type == 'SE3Trajectory':
            editor = editors.TrajectoryEditor(name,value,description,world)
            attach(referenceObject,editor)
            return editors.run(editor)
        elif type == 'Vector3' or type == 'Point':
            if hasattr(frame,'getTransform'):
                frame = frame.getTransform()
            return editors.run(editors.PointEditor(name,value,description,world,frame))
        elif type == 'RigidTransform' or type == 'Rotation':
            if type == 'RigidTransform' and isinstance(frame,RigidObjectModel):
                return editors.run(editors.ObjectTransformEditor(name,value,description,world,frame))
            if type == 'Rotation':
                #convert from so3 to se3
                value = [value,[0,0,0]]
            Tref = frame
            if hasattr(frame,'getTransform'):
                Tref = frame.getTransform()
            editor = editors.RigidTransformEditor(name,value,description,world,Tref)
            if type == 'Rotation':
                editor.disable_translation()
            attach(referenceObject,editor)
            #Run!
            if type == 'Rotation':
                #convert from se3 to so3
                return editors.run(editor)[0]
            else:
                return editors.run(editor)
        elif type == "GeometricPrimitive":
            Tref = frame
            if hasattr(frame,'getTransform'):
                Tref = frame.getTransform()
            editor = editors.GeometricPrimitiveEditor(name,value,description,world,Tref)
            return editors.run(editor)
        elif type == 'WorldModel':
            return editors.run(editors.WorldEditor(name,value,description))
        else:
            raise RuntimeError("Visual editing of objects of type "+type+" not supported yet")
    else:
        raise ValueError("Invalid value for argument 'editor', must be either 'visual' or 'console'")



def _deprecated_func(oldName,newName):
    import sys
    mod = sys.modules[__name__]
    f = getattr(mod,newName)
    def depf(*args,**kwargs):
        warnings.warn("{} will be deprecated in favor of {} in a future version of Klampt".format(oldName,newName),DeprecationWarning)
        return f(*args,**kwargs)
    depf.__doc__ = 'Deprecated in a future version of Klampt. Use {} instead'.format(newName)
    setattr(mod,oldName,depf)

_deprecated_func("getDirectory","get_directory")
_deprecated_func("setDirectory","set_directory")
_deprecated_func("knownExtensions","known_extensions")
_deprecated_func("knownTypes","known_types")
_deprecated_func("visualEditTypes","visual_edit_types")
