from klampt.io import resource
from klampt.vis import visualization
from klampt.model.trajectory import *
from klampt import *
import sys

MULTITHREADED = False

def config_edit_template(world):
    """Shows how to edit Config, Configs, and Trajectory resources"""
    config1 = resource.get("resourcetest1.config",description="First config, always edited",doedit=True,editor='visual',world=world)
    print "Config 1:",config1
    config2 = resource.get("resourcetest1.config",description="Trying this again...",editor='visual',world=world)
    print "Config 2:",config2

    config3 = resource.get("resourcetest2.config",description="Another configuration",editor='visual',world=world)
    print "Config 3:",config3

    if config3 != None:
        config3hi = config3[:]
        config3hi[3] += 1.0
        resource.set("resourcetest3_high.config",config3hi)
        world.robot(0).setConfig(config3hi)

    configs = []
    if config1 != None: configs.append(config1)
    if config2 != None: configs.append(config2)
    if config3 != None: configs.append(config3)
    print "Configs resource:",configs
    configs = resource.get("resourcetest.configs",default=configs,description="Editing config sequence",doedit=True,editor='visual',world=world)
    if configs is None:
        print "To edit the trajectory, press OK next time"
    else:
        traj = RobotTrajectory(world.robot(0),range(len(configs)),configs)
        traj = resource.edit(name="Timed trajectory",value=traj,description="Editing trajectory",world=world)

def xform_edit_template(world):
    """Testing the transform editor."""
    #The first call doesn't attach any geometry to the frame.  The second call attaches geometry to it.
    #Both return the edited transform *relative* to the link.
    link = world.robot(0).link(5)
    xform = resource.edit(name="Base link, floating",value=None,type='RigidTransform',frame=link.getTransform(),world=world)
    xform = resource.edit(name="Base link, should be relative to robot",value=None,type='RigidTransform',frame=link.getTransform(),referenceObject=link,world=world)
    link = world.robot(0).link(8)
    xform = resource.edit(name="Leg link",value=None,type='RigidTransform',frame=link.getTransform(),world=world)
    xform = resource.edit(name="Leg link, should be relative to robot",value=None,type='RigidTransform',frame=link.getTransform(),referenceObject=link,world=world)

def load_save_template(world):
    """Shows the load/save functions. First loads a Config resource, then ask you to save the edited config"""
    fn,q = resource.load('Config')
    resource.edit('loaded config',q,world=world)
    resource.save(q)

def thumbnail_template(world):
    """Will save thumbnails of all previously created resources"""
    import os,glob
    robotname = world.robot(0).getName()
    for fn in glob.glob('resources/'+robotname+'/*'):
        filename = os.path.basename(fn)
        print os.path.splitext(filename)[1]
        if os.path.splitext(filename)[1] in resource.knownTypes():
            print fn
            res = resource.get(filename)
            im = resource.thumbnail(res,(128,96),world=world)
            if im != None:
                im.save('resources/'+robotname+"/"+filename+".thumb.png")

def vis_interaction_test(world):
    """Run some tests of visualization module interacting with the resource module"""
    print "Showing robot in modal dialog box"
    visualization.add("robot",world.robot(0))
    visualization.add("ee",world.robot(0).link(11).getTransform())
    visualization.dialog()
    
    config = resource.get("resourcetest1.config",description="Should show up without a hitch...",doedit=True,editor='visual',world=world)

    import time
    if MULTITHREADED:
        print "Showing threaded visualization (this will fail on GLUT or Mac OS)"
        visualization.show()
        for i in range(3):
            visualization.lock()
            q = world.robot(0).getConfig()
            q[9] = 3.0
            world.robot(0).setConfig(q)
            visualization.unlock()
            time.sleep(1.0)
            if not visualization.shown():
                break
            visualization.lock()
            q = world.robot(0).getConfig()
            q[9] = -1.0
            world.robot(0).setConfig(q)
            visualization.unlock()
            time.sleep(1.0)
            if not visualization.shown():
                break
        print "Hiding visualization window"
        visualization.show(False)
    else:
        print "Showing single-threaded visualization for 5s"
        visualization.spin(5.0)

    config = resource.get("resourcetest1.config",description="Should show up without a hitch...",doedit=True,editor='visual',world=world)
    

if __name__ == '__main__':
    print """
    ============================================================
    resource_demo.py: This program demonstrates how to use the
    klampt.resource module.
    ============================================================
    """

    worldfile = "../../data/athlete_plane.xml"
    if len(sys.argv) > 1:
        worldfile = sys.argv[1]
    world = WorldModel()
    if not world.readFile(worldfile):
        print "Usage: python resource_demo.py [world file]"
        exit(1)
    robotname = world.robot(0).getName()

    #look in resources/athlete/
    resource.setDirectory('resources/'+robotname)
    print "Configured the resource module to read from",resource.getDirectory()

    templates = {'1':config_edit_template,'2':xform_edit_template,'3':load_save_template,
                '4':thumbnail_template,'5':vis_interaction_test}
    print "Available templates"
    import inspect
    for k in sorted(templates.keys()):
        fname = 'untitled'
        for x in inspect.getmembers(templates[k]):
            if x[0] == '__name__':
                fname = x[1]
        print " %s) %s: %s"%(k,fname,inspect.getdoc(templates[k]))
    entry = raw_input("Which template do you want to run? (1-%d) > "%(len(templates),))
    if entry not in templates:
        print "Invalid selection"
        exit(1)
    templates[entry](world)

    #this is needed to avoid a Ctrl+C to kill the visualization thread
    visualization.kill()
