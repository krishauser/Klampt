"""Use the get(), set(), and edit() functions to retrieve / store / edit
resources dynamically.

load() and save() launch a file browser (Qt only).

Use getDirectory() and setDirectory() to set the directory under which
resources are stored.  Alternatively, the directory=[DIRNAME] keyword
argument can be provided to get, set, and edit.

Example usage can be seen in demos/resourcetest.py.
"""

import loader
from ..model import trajectory
from ..model import multipath
from ..model import types
from .. import robotsim
from ..math import vectorops,se3,so3
import os
import time
from ..robotsim import WorldModel,RobotModelLink,RigidObjectModel,IKObjective
from ..model.contact import ContactPoint
from ..model.contact import Hold
from .. import vis


global _directory
global _editTemporaryWorlds
_directory = 'resources'
_editTemporaryWorlds = {}

def getDirectory():
    """Returns the current resource directory."""
    return _directory

def setDirectory(value):
    """Sets the current resource directory."""
    global _directory
    _directory = value

def _ensure_dir(f):
    d = os.path.dirname(f)
    if len(d)==0: return
    if not os.path.exists(d):
        os.makedirs(d)

extensionToType = {'.config':'Config',
                   '.configs':'Configs',
                   '.tri':'Geometry3D',
                   '.off':'Geometry3D',
                   '.stl':'Geometry3D',
                   '.poly':'Geometry3D',
                   '.geom':'GeometricPrimitive',
                   '.pcd':'Geometry3D',
                   '.vector3':'Vector3',
                   '.ikgoal':'IKGoal',
                   '.xform':'RigidTransform',
                   '.hold':'Hold',
                   '.stance':'Stance',
                   '.grasp':'Grasp'}

typeToExtension = dict((v,k) for (k,v) in extensionToType.items())


def filenameToType(name):
    fileName, fileExtension = os.path.splitext(name)
    if fileExtension in extensionToType:
        return extensionToType[fileExtension]
    elif fileExtension == '.xml':
        return 'xml'  #dynamic loading
    elif fileExtension == '.json':
        return 'json'  #dynamic loading
    else:
        raise RuntimeError("Cannot determine type of resource from name "+name)


def get(name,type='auto',directory=None,default=None,doedit='auto',description=None,editor='visual',world=None,frame=None):
    """Retrieve a resource of the given name from the current resources
    directory.  Resources may be of type Config, Configs, IKGoal, Hold,
    Stance, MultiPath, LinearPath, etc. (see Klampt/Modeling/Resources.h for
    a complete list; not all are supported in the Python API).  They can
    also be edited using RobotPose.

    If the resource does not already exist, an edit prompt will be
    shown, and the result from the prompt will be saved to disk under the
    given name.

    Arguments:
        - name: the resource name.  If type='auto', this is assumed to have
          a suffix of a file of the desired type.  The name can also be
          nested of the form 'group/subgroup/name.type'.
        - type: the resource type, or 'auto' to determine it automatically.
        - directory: the search directory.  If None, uses the current
          resource directory.
        - default: the default value if the resource does not exist on disk.
          If None, some arbitrary default value will be inferred.
        - doedit: if 'auto', if the resource does not exist on disk, an
          edit prompt will be displayed.  If False, an RuntimeError will be
          raised if the resource does not exist on disk.  If True, the
          user will be given an edit prompt to optionally edit the resource
          before this call returns.
        - description: an optional text description of the resource, for use
          in the edit prompt.
        - editor: either 'visual' or 'console', determining whether to use the
          visual OpenGL or console editor.
        - world: for a visual editor, this world will be shown along with
          the item to edit.  If this is a string it points to a file
          that will be loaded for the world (e.g., a world XML file, or a
          robot file).
        - frame: for rigid transforms / rotations / points, the reference
          frame for the visual editor.  This is an element of se3, or an
          ObjectModel, or a RobotModelLink, or a string indicating a named
          rigid element of the world.
          """
    if name==None:
        if doedit==False:
            raise RuntimeError("Can't get() an anonymous resource without launching editor")
        success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,frame=frame)
        if not success:
            print "Cancel pressed, returning None"
            return None
        else:
            print "Ok pressed, returning anonymous resource"
            return newvalue
    if directory==None:
        directory = getDirectory()
    if type == 'auto':
        type = filenameToType(name)
    value = None
    try:
        fn = os.path.join(directory,name)
        try:
            if type == 'xml':
                raise NotImplementedError("TODO: load xml files from Python API")
            elif type == 'json':
                f = open(fn,'r')
                text = ''.join(f.readlines())
                f.close()
                value = loader.fromJson(text,type=type)
            else:
                value = loader.load(type,fn)
        except IOError:
            raise
        except Exception,e:
            import traceback
            print "Unable to read object from file "+fn
            print "Traceback: "
            traceback.print_exc()
            raise IOError()
        if value==None:
            raise IOError("Unable to load from file "+fn)
        if doedit==True:
            success,newvalue = edit(name,value=value,type=type,description=description,editor=editor,world=world,frame=frame)
            if success:
                print "Ok pressed, saving resource to",name
                value = newvalue
                set(name,value,type=type,directory=directory)
            else:
                print "Cancel pressed, not saving resource to disk"
            return value
        return value
    except IOError:
        if doedit!=False:
            print "Resource",fn,"does not exist, launching editor..."
            success,newvalue = edit(name,value=default,type=type,description=description,editor=editor,world=world,frame=frame)
            if success:
                print "Ok pressed, saving resource to",name
                value = newvalue
                set(name,value=value,type=type,directory=directory)
            else:
                print "Cancel pressed, not saving resource to disk"
            return value
        elif default is not None:
            return default
        else:
            raise RuntimeError("Resource "+name+" does not exist")
    return

def set(name,value,type='auto',directory=None):
    """Saves a resource to disk under the given name."""
    if type == 'auto':
        type = filenameToType(name)
    if directory==None:
        directory = getDirectory()    
    fn = os.path.join(directory,name)
    _ensure_dir(fn)
    if type == 'xml':
        raise NotImplementedError("TODO: save xml files from Python API")
    elif type == 'json':
        f = open(fn,'w')
        f.write(loader.toJson(value,type=type))
        f.write('\n')
        f.close()
        return True
    else:
        return loader.save(value,type,fn)

class FileGetter:
    def __init__(self,title="Open file"):
        self.title = title
        self.directory = ''
        self.filetypes = []
        self.result = None
    def getOpen(self):
        from PyQt4.QtGui import QFileDialog
        patterns = ""
        if len(self.filetypes) == 0:
            patterns = "All files (*.*)"
        else:
            patternlist = []
            for (desc,exts) in self.filetypes:
                pattern = desc + " ("
                pattern = pattern + ' '.join("*"+ext for ext in exts) + ")"
                patternlist.append(pattern)
            #print patternlist
            patterns = ";;".join(patternlist)
        self.result = QFileDialog.getOpenFileName(None, self.title, self.directory, patterns)
    def getSave(self):
        from PyQt4.QtGui import QFileDialog
        patterns = ""
        if len(self.filetypes) == 0:
            patterns = "All files (*.*)"
        else:
            patternlist = []
            for (desc,exts) in self.filetypes:
                pattern = desc + " ("
                pattern = pattern + ' '.join("*"+ext for ext in exts) + ")"
                patternlist.append(pattern)
            #print patternlist
            patterns = ";;".join(patternlist)
        self.result = QFileDialog.getSaveFileName(None, self.title, self.directory, patterns)

def load(type=None,directory=None):
    """Asks the user to open a resource file of a given type.  If type is not given, all resource file types
    are given as options."""
    
    fg = FileGetter('Open resource')
    fg.directory = directory
    if directory==None:
        fg.directory = getDirectory()    
    if type is not None:
        extensions=[]
        for (k,v) in extensionToType.iteritems():
            if v == type:
                extensions.append(k)
        extensions.append('.json')
        fg.filetypes.append((type,extensions))

    def make_getfilename(glbackend):
        fg.getOpen()
        return None
    #These gymnastics are necessary because Qt can only be run in a single thread, and to be compatible 
    #with the visualization you need to use the customUI functions
    old_window = vis.getWindow()
    vis.customUI(make_getfilename)
    vis.dialog()
    vis.setWindow(old_window)
    if len(fg.result) == 0:
        return None
    if type == None:
        return get(str(fg.result),'auto',directory,doedit=False)
    return get(str(fg.result),type,'',doedit=False)

def save(value,type='auto',directory=None):
    """Asks the user to save the given resource to a file of the correct type.  If type='auto', the type
    is determined automatically"""
    fg = FileGetter('Save resource')
    fg.directory = directory
    if directory==None:
        fg.directory = getDirectory()    
    if type == 'auto':
        typelist = types.objectToTypes(value)
    else:
        typelist = [type] 
    for type in typelist:
        extensions=[]
        for (k,v) in extensionToType.iteritems():
            if v == type:
                extensions.append(k)
        extensions.append('.json')
        fg.filetypes.append((type,extensions))

    def make_getfilename(glbackend):
        fg.getSave()
        return None
    #These gymnastics are necessary because Qt can only be run in a single thread, and to be compatible 
    #with the visualization you need to use the customUI functions
    old_window = vis.getWindow()
    vis.customUI(make_getfilename)
    vis.dialog()
    vis.setWindow(old_window)
    if len(fg.result) == 0:
        return False
    return set(str(fg.result),value,type,'')

def console_edit(name,value,type,description=None,world=None,frame=None):
    print "*********************************************************"
    print 
    print "Editing resource",name,"of type",type
    print
    if description!=None:
        print description
        print
    if frame!=None:
        if hasattr(frame,'getName'):
            print "Reference frame:",frame.getName()
        else:
            print "Reference frame:",frame
        print
    print "*********************************************************"
    print "Current value:",value
    print "Do you wish to change it? (y/n/q) >",
    choice = ''
    while choice not in ['y','n','q']:
        choice = raw_input()[0].lower()
        if choice not in ['y','n','q']:
            print "Please enter y/n/q indicating yes/no/quit."
            print ">",
    if choice=='y':
        print "Enter the new desired value below.  You may use native text,"
        print "JSON strings, or file(fn) to indicate a file name."
        print "New value >",
        import json
        data = raw_input()
        if data.startswith('{') or data.startswith('['):
            jsonobj = json.loads(data)
            try:
                obj = loader.fromJson(jsonobj,type)
                return True,obj
            except Exception:
                print "Error loading from JSON, press enter to continue..."
                raw_input()
                return False,value
        elif data.startswith('file('):
            try:
                obj = get(data[5:-1],type,doedit=False)
                if obj==None:
                    return False,value
                return True,obj
            except Exception:
                print "Error loading from file, press enter to continue..."
                raw_input()
                return False,value
        else:
            try:
                obj = loader.read(type,data)
                return True,obj
            except Exception:
                print "Error loading from text, press enter to continue..."
                raw_input()
                return False,value
    elif choice=='n':
        print "Using current value."
        print "*********************************************************"
        return False,value
    elif choice=='q':
        return False,None


def edit(name,value,type='auto',description=None,editor='visual',world=None,robot=None,frame=None):
    """Launches an editor for the given value.  Returns a pair (save,result)
    where save indicates what the user wanted to do with the edited value
    and result is the edited value."""
    if name == None and type=='auto':
        raise RuntimeError("Cannot do an anonymous edit without the 'type' argument specified")
    if name == None:
        name = 'Anonymous'
    if type == 'auto':
        type = types.objectToTypes(value)
        if type is None:
            raise RuntimeError("Could not autodetect type of object "+name)
        if isinstance(type,(list,tuple)):
            type = type[0]
    if not vis.glinit._PyQtAvailable and editor=='visual':
        print "PyQt is not available, defaulting to console editor"
        editor = 'console'
            
    if isinstance(world,str):
        #a single argument, e.g., a robot file
        global _editTemporaryWorlds
        if world not in _editTemporaryWorlds:
            _editTemporaryWorlds[world] = WorldModel()
            if not _editTemporaryWorlds[world].readFile(world):
                raise RuntimeError("Error loading world file "+world)
        world = _editTemporaryWorlds[world]    
    if isinstance(frame,str):
        try:
            oframe = world.rigidObject(frame)
            frame = oframe
        except RuntimeError:
            try:
                if robot != None:
                    oframe = robot.link(frame)
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
    if value==None:
        if type == 'Config':
            if world==None and robot==None:
                raise RuntimeError("Cannot visually edit a Config resource without a world/robot")
            if robot==None:
                robot = world.robot(0)
            value = robot.getConfig()
        elif type == 'Configs':
            if world==None and robot==None:
                raise RuntimeError("Cannot visually edit a Configs resource without a world/robot")
            if robot==None:
                robot = world.robot(0)
            value = [robot.getConfig()]
        else:
            value = types.make(type)
            if value == None:
                raise RuntimeError("Don't know how to edit objects of type "+type)

    if editor == 'console':
        return console_edit(name,value,type,description,world,frame)
    elif editor == 'visual':
        if type == 'Config':
            return vis.editors.run(vis.editors.ConfigEditor(name,value,description,world,robot))
        elif type == 'Configs':
            return vis.editors.run(vis.editors.ConfigsEditor(name,value,description,world,robot))
        elif type == 'Vector3' or type == 'Point':
            if hasattr(frame,'getTransform'):
                frame = frame.getTransform()
            return vis.editors.run(vis.editors.PointEditor(name,value,description,world,frame))
        elif type == 'Rotation':
            if hasattr(frame,'getTransform'):
                frame = frame.getTransform()
            return vis.editors.run(vis.editors.RotationEditor(name,value,description,world,frame))
        elif type == 'RigidTransform':
            if isinstance(frame,RigidObjectModel):
                return vis.editors.run(vis.editors.ObjectTransformEditor(name,value,description,world,frame))
            if hasattr(frame,'getTransform'):
                frame = frame.getTransform()
            return vis.editors.run(vis.editors.RigidTransformEditor(name,value,description,world,frame))
        else:
            raise RuntimeError("Visual editing of objects of type "+type+" not supported yet")
    else:
        raise ValueError("Invalid value for argument 'editor', must be either 'visual' or 'console'")


