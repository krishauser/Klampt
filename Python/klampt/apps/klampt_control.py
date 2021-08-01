from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt import vis
from klampt.model import robotinfo
from klampt.model.robotinfo import RobotInfo
from klampt.vis.glcommon import GLWidgetPlugin
from klampt.control.networkrobotinterface import RobotInterfaceClient,RobotInterfaceServer
import math
import time
import sys
import os


class MyGLViewer(GLWidgetPlugin):
    def __init__(self,world,controller):
        GLWidgetPlugin.__init__(self)

        self.world = world
        self.controller = controller
        #read commanded configuration
        q = self.controller.configToKlampt(self.controller.commandedPosition())
        world.robot(0).setConfig(q)
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)

        robot = world.robot(0)
        self.qcmd = None
        self.qsns = None

    def display(self):
        robot = self.world.robot(0)
        #draw robot poser in transparent yellow
        oldcolors = [robot.link(i).appearance().getColor() for i in range(robot.numLinks())]
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(1,1,0,0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(*oldcolors[i])
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

        #this line will draw the world
        if self.qsns is not None:
            robot.setConfig(self.qsns)
        self.world.drawGL()

        #draw commanded configuration in transparent green
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(0,1,0,0.5)
        if self.qcmd is not None:
            robot.setConfig(self.qcmd)
        robot.drawGL()
        #restore colors
        for i in range(robot.numLinks()):
            robot.link(i).appearance().setColor(*oldcolors[i])

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print('[space]: send the current posed milestone')
            print('p: print current sensed position')
        elif c == ' ':
            q = self.robotPoser.get()
            self.controller.beginStep()
            #convert klampt configuration to limb command vector
            self.controller.moveToPosition(self.controller.configFromKlampt(q))
            self.controller.endStep()
        elif c == 'p':
            print("Current config: ",self.qsns)
            if self.qsns is not None:
                print("Current Klampt config: ",self.controller.configToKlampt(self.qsns))
        else:
            GLWidgetPlugin.keyboardfunc(self,c,x,y)
            self.refresh()

    def idle(self):
        self.controller.beginStep()
        self.qsns = self.controller.configToKlampt(self.controller.sensedPosition())
        self.qcmd = self.controller.configToKlampt(self.controller.commandedPosition())
        self.controller.endStep()
        return GLWidgetPlugin.idle(self)
        

def main():
    print("klampt_control: a GUI to control a Klamp't Robot Interface Layer (RIL) robot")
    print()

    import textwrap
    USAGE = textwrap.dedent("""
        USAGE: klampt_control [OPTS] FILES
        
        Where FILES is one or more strings referencing:
        1. A Python file (/path/to/file.py)
        2. A Python module (package.module)
        3. A RobotInfo JSON module (info_file.json)
        4. A robot file, world file, or objects for the visualization/planning
        
        In case 1 and 2, the Python file or module must contain a function
        make(robotModel) returning a RobotInterfaceBase instance.
        
        In cases 2 and 3, a robot file must be provided.
        
        OPTS can be any of
        --server IP: launch the controller as a standalone server (recommend
            using "localhost:7881")
        --client IP: use a client as the controller
        --sim: simulate the robot and use a standard controller.  Equivalent to
            including klampt.control.simrobotinterface in FILES
        """)

    if len(sys.argv) < 2:
        print(USAGE)
        exit(0)
    info = RobotInfo('untitled')
    world = WorldModel()
    mode = 'normal'
    ip = None
    idx = 1
    while idx < len(sys.argv):
        fn = sys.argv[idx]
        idx += 1
        if fn.startswith('-'):
            if fn == '--server':
                mode = 'server'
                ip = sys.argv[idx]
                idx += 1
            elif fn == '--client':
                mode = 'client'
                ip = sys.argv[idx]
                idx += 1
            elif fn == '--sim':
                info.controllerFile = 'klampt.control.simrobotinterface'
            else:
                print("Invalid command-line option")
                print()
                print(USAGE)
                exit(1)
            continue
        basename,ext = os.path.splitext(fn)
        if ext.lower() in ['.rob','.urdf','.xml','.off','.stl','.dae','.ply','.obj','.env']:
            world.loadFile(fn)
            if ext.lower() in ['.rob','.urdf']:
                info.modelFile = os.path.abspath(fn)
            elif ext.lower() == '.xml':
                #TODO: parse XML for robot file?
                pass
        elif ext in ['.py','.pyc']:
            info.controllerFile = fn
        elif ext == '.json':
            info = robotinfo.load(fn)
            if info.controllerFile is None:
                print("RobotInfo file",fn,"doesn't specify controllerFile")
        else: #assume module
            info.controllerFile = fn
    if world.numRobots()==0:
        if info.modelFile is not None:
            print("Loading",info.modelFile,"from paths",info.filePaths)
            model = info.klamptModel()
            world.add(info.name,model)
        else:
            print("No robot models loaded, can't run the visualization")
            exit(1)

    if info.name =='untitled':
        info.name = world.robot(0).getName()
    info.robotModel = world.robot(0)
    robotinfo.register(info)

    if mode == 'normal':
        controller = info.controller()
    elif mode == 'client':
        if ':' in ip:
            addr,port = ip.split(':')
        else:
            addr=ip
            port = 7881
        controller = RobotInterfaceClient(addr,int(port))
    elif mode == 'server':
        if ':' in ip:
            addr,port = ip.split(':')
        else:
            addr=ip
            port = 7881
        controller = info.controller()
        server = RobotInterfaceServer(controller,addr,int(port))
        print("Beginning Robot Interface Layer server for controller",controller)
        print("Press Ctrl+C to exit...")
        server.serve()
        exit(0)

    res = controller.initialize()
    if not res:
        print("Error starting up controller")
        exit(1)

    #start the viewer
    viewer = MyGLViewer(world,controller)
    vis.setWindowTitle("klampt_control {}".format(info.name))
    vis.run(viewer)

    #close the controller
    controller.close()


if __name__ == "__main__":
    main()
