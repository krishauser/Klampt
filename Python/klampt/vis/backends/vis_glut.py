from ..visualization import _WindowManager,_ThreadedWindowManager,_globalLock,VisualizationScene
from .vis_gl import GLVisualizationFrontend,GLVisualizationPlugin,WindowInfo
from .. import glinit,gldraw,glcommon
from ...robotsim import WorldModel,RobotModel
import threading

if not glinit.available('GLUT'):
    raise ImportError("Can't import vis_glut without first calling glinit.init()")

from OpenGL.GL import *
from OpenGL.GLUT import *
import time
import weakref


class GLUTWindowManager(_ThreadedWindowManager):
    def __init__(self):
        self._frontend = GLUTVisualizationFrontend(None)
        #a list of WorldModel indices in the current window.  A world cannot be used in multiple simultaneous
        #windows in GLUT.  If a world is reused with a different window, its display lists will be refreshed.
        self.current_worlds = []
        #list of WindowInfo's
        self.windows = []
        #the index of the current window
        self.current_window = None
        #the name of a window, if no windows exist yet
        self.window_title = "Klamp't visualizer (%s)"%(sys.argv[0],)
        #a callback sent to run
        self.callback = None
        #the current temp frontend if len(self.windows)=0, or windows[current_window].frontend
        _ThreadedWindowManager.__init__(self)

    def reset(self):
        _ThreadedWindowManager.reset(self)
        self.cleanup()
        
    def run_app_thread(self,callback=None):
        global _globalLock
        assert not self.vis_thread_running,"Can't run a new GLUT thread, a thread is already running"
        self.vis_thread_running = True
        if len(self.windows)==0:
            self.windows.append(WindowInfo(self.window_title,self._frontend)) 
            self.current_window = 0
            winfo = self.windows[self.current_window]
            winfo.mode = 'shown'
            winfo.worlds = self.current_worlds
            winfo.active_worlds = self.current_worlds[:]
        glinit._GLBackend.initialize("Klamp't visualization")
        winfo = self.windows[self.current_window]
        print("GLUTWindowManager.run_app_thread: creating window with name",winfo.name,"and status",winfo.mode)
        w = glinit._GLBackend.createWindow(winfo.name)
        self._frontend.windowinfo = weakref.proxy(winfo)
        self._frontend.window_manager = weakref.proxy(self)
        self._frontend.name = winfo.name
        w.setProgram(self._frontend)
        winfo.glwindow = w
        self.callback = callback
        print("Windows",[winfo.name for winfo in self.windows])
        glinit._GLBackend.run()
        print("GLUTWindowManager.run_app_thread: Visualization thread closing...")
        self.cleanup()
        self.vis_thread_running = False
        print("GLUTWindowManager.run_app_thread: terminating.")
        return

    def cleanup(self):
        print("GLUTWindowManager.cleanup()")
        for w in self.windows:
            w.frontend.scene.clear()
            w.worlds = []
            w.active_worlds = []
            #for some reason, destroying windows causes everything to terminate early
            if w.glwindow:
                print("GLUTWindowManager: destroying window",w.glwindow.glutWindowID)
                #glutDestroyWindow(w.glwindow.glutWindowID)
                w.glwindow = None
        self._frontend = GLUTVisualizationFrontend(None)
        self.current_worlds = []
        self.windows = []
        self.current_window = None
        self.window_title = "Klamp't visualizer (%s)"%(sys.argv[0],)
        self.callback = None
    
    def frontend(self):
        return self._frontend

    def scene(self):
        return self._frontend.scene

    def getWindowName(self):
        return self.window_title

    def setWindowName(self,title):
        self.window_title = title
        self.onFrontendChange()

    def resizeWindow(self,w,h):
        self._frontend.reshape(w,h)
    
    def createWindow(self,title):    
        if len(self.windows) == 0:
            #save the defaults in window 0
            self.windows.append(WindowInfo(self.window_title,self._frontend))    
            self.windows[-1].worlds = self.current_worlds
            self.windows[-1].active_worlds = self.current_worlds[:]
        if title is None:
            title = "Window "+str(len(self.windows))
        #make a new window
        self._frontend = GLUTVisualizationFrontend(None)
        self._frontend.window_manager = weakref.proxy(self)
        self.windows.append(WindowInfo(title,self._frontend))
        self.window_title = title
        print("GLUTWindowManager.createWindow: window title",self.window_title,", id",len(self.windows)-1)
        self.current_worlds = []
        id = len(self.windows)-1
        self.current_window = id
        return id

    def setWindow(self,id):
        if id == self.current_window:
            return
        assert id >= 0 and id < len(self.windows),"Invalid window id"
        self._frontend = self.windows[id].frontend
        self.current_worlds = self.windows[id].worlds
        self.window_title = self.windows[id].name
        #print "vis.setWindow(",id,") the window has status",_windows[id].mode
        #PyQt interface allows sharing display lists but GLUT does not.
        #refresh all worlds' display lists that were once active.
        for w in self.current_worlds:
            if w in self.windows[self.current_window].active_worlds:
                print("klampt.vis.setWindow(): world",w,"becoming active in the new window",id)
                for item in self.windows[self.current_window].worldDisplayListItems[w]:
                    self._refreshDisplayLists(item)
                self.windows[self.current_window].active_worlds.remove(w)
        self.windows[id].active_worlds = self.current_worlds[:]
        self.current_window = id

    def getWindow(self):
        return self.current_window

    def setPlugin(self,plugin):
        if not isinstance(self._frontend,GLUTVisualizationFrontend):
            #was multi-view -- now setting plugin
            self._frontend = GLUTVisualizationFrontend()
            if self.current_window is not None:
                if self.windows[self.current_window].glwindow is not None:
                    self._frontend.window = self.windows[self.current_window].glwindow
        if plugin is None:
            self._frontend.setPlugin(self._frontend.scene)
        else:
            self._frontend.setPlugin(plugin)
        if hasattr(plugin,'world'):
            self._checkWindowCurrent(plugin.world)
        self.onFrontendChange()

    def pushPlugin(self,plugin):
        assert isinstance(self._frontend,glcommon.GLPluginProgram),"Can't push a plugin after splitView"
        if len(self._frontend.plugins) == 0:
            self._frontend.setPlugin(self._frontend.scene)
        self._frontend.pushPlugin(plugin)
        self.onFrontendChange()

    def popPlugin(self):
        self._frontend.popPlugin()
        self.onFrontendChange()

    def splitView(self,plugin):
        #create a multi-view widget
        if plugin is None:
            plugin = GLVisualizationPlugin()
        if isinstance(self._frontend,glcommon.GLMultiViewportProgram):
            self._frontend.add_view(plugin)
            if hasattr(plugin,'scene') and isinstance(plugin.scene,VisualizationScene):
                self._frontend.scene = plugin.scene
        else:
            if len(self._frontend.plugins) == 0:
                self.setPlugin(None)
            multiProgram = GLUTMultiWindowVisualizationFrontend(None)
            multiProgram.windowinfo = weakref.proxy(self.windows[self.current_window])
            multiProgram.window = None
            if self.current_window is not None:
                if self.windows[self.current_window].glwindow is not None:
                    multiProgram.window = self.windows[self.current_window].glwindow
            multiProgram.add_view(self._frontend)
            multiProgram.add_view(plugin)
            multiProgram.name = self.window_title
            self._frontend = multiProgram
            multiProgram.scene = self._frontend
            if hasattr(plugin,'scene') and isinstance(plugin.scene,VisualizationScene):
                multiProgram.scene = plugin.scene
        if isinstance(plugin,GLVisualizationPlugin):
            plugin.program = weakref.proxy(self._frontend.views[-1])
        self.onFrontendChange()


    def unlock(self):
        _ThreadedWindowManager.unlock(self)
        self.update()

    def update(self):
        for w in self.windows:
            if w.glwindow:
                w.doRefresh = True

    def show(self):
        if len(self.windows)==0:
            self.windows.append(WindowInfo(self.window_title,self._frontend)) 
            self.current_window = 0
            print("First show(), window title",self.window_title)
        winfo = self.windows[self.current_window]
        winfo.mode = 'shown'
        winfo.worlds = self.current_worlds
        winfo.active_worlds = self.current_worlds[:]
        if not self.vis_thread_running:
            print("GLUTWindowManager.show(): first window shown, starting the visualization thread")
            self._start_app_thread()

    def shown(self):
        return (self.vis_thread_running and self.current_window is not None and self.windows[self.current_window].mode in ['shown','dialog'])

    def hide(self):
        if self.current_window is None:
            return
        self.windows[self.current_window].mode = 'hidden'

    def dialog(self):
        global _globalLock
        if len(self.windows)==0:
            self.windows.append(WindowInfo(self.window_title,self._frontend))
            self.current_window = 0
        w = self.windows[self.current_window]
        w.mode = 'dialog'
        w.worlds = self.current_worlds
        w.active_worlds = self.current_worlds[:]
        if self.multithreaded():
            print("#########################################")
            print("klampt.vis: Running multi-threaded dialog, waiting to complete...")
            if not self.vis_thread_running:
                self._start_app_thread()
            while w.mode == 'dialog':
                time.sleep(0.1)
            print("klampt.vis: ... dialog done.")
            print("#########################################")
            return None
        else:
            print("#########################################")
            print("klampt.vis: Running single-threaded dialog")
            self.in_vis_loop = True
            res = self.run_app_thread()
            self._in_vis_loop = False
            print("klampt.vis: ... dialog done.")
            print("#########################################")
            return res

    def set_custom_ui(self,func):
        if len(self.windows)==0:
            print("Making first window for custom ui")
            self.windows.append(WindowInfo(self.window_title,self._frontend))
            self.current_window = 0
        self.windows[self.current_window].custom_ui = func
        print("klampt.vis: setting custom ui on window",self.current_window)
        return

    def onFrontendChange(self):
        if self.current_window is None:
            return
        w = self.windows[self.current_window]
        w.doReload = True
        w.frontend = self._frontend
        if w.glwindow:
            w.glwindow.reshape(self._frontend.view.w,self._frontend.view.h)
            if w.name != self.window_title:
                glutSetWindow(w.glwindow.glutWindowID)
                glutSetWindowTitle(self.window_title)
        w.name = self.window_title

    def _refreshDisplayLists(self,item):
        if isinstance(item,WorldModel):
            for i in range(item.numRobots()):
                self._refreshDisplayLists(item.robot(i))
            for i in range(item.numRigidObjects()):
                self._refreshDisplayLists(item.rigidObject(i))
            for i in range(item.numTerrains()):
                self._refreshDisplayLists(item.terrain(i))
        elif isinstance(item,RobotModel):
            for i in range(item.numLinks()):
                self._refreshDisplayLists(item.link(i))
        elif hasattr(item,'appearance'):
            item.appearance().refresh(False)

    def _checkWindowCurrent(self,item):
        #print("Checking whether item",item,"is current in the context of window",self.current_window)
        #print("Current worlds",self.current_worlds)
        #print("Current window's active worlds",self.windows[self.current_window].active_worlds)
        if isinstance(item,WorldModel):
            if item.index not in self.current_worlds:
                #PyQt interface allows sharing display lists but GLUT does not.
                #refresh all worlds' display lists that will be shifted to the current window.
                for i,win in enumerate(self.windows):
                    #print("Window",i,"active worlds",win.active_worlds)
                    if item.index in win.active_worlds:
                        #GLUT SPECIFIC
                        print("klampt.vis: world",item.index,"was shown in a different window, now refreshing display lists")
                        self._refreshDisplayLists(item)
                        win.active_worlds.remove(item.index)
                self.current_worlds.append(item.index)
                if self.current_window is not None:
                    self.windows[self.current_window].worldDisplayListItems[item.index].append(weakref.proxy(item))
                #print("klampt.vis: world added to the visualization's world (items:",self.current_worlds,")")
            #else:
            #    print("klampt.vis: world",item,"is already in the current window's world")
        elif hasattr(item,'world'):
            if isinstance(item.world,WorldModel):
                return self._checkWindowCurrent(item.world)
            if isinstance(item.world,int):
                if item.world < 0:
                    return
                if item.world not in self.current_worlds:
                    for i,win in enumerate(self.windows):
                        #print("Window",i,"active worlds",win.active_worlds)
                        if item.world in win.active_worlds:
                            #GLUT SPECIFIC
                            print("klampt.vis: world",item.index,"was shown in a different window, now refreshing display lists")
                            self._refreshDisplayLists(item)
                            win.active_worlds.remove(item.world)
                    self.current_worlds.append(item.world)
                    if self.current_window is not None:
                        self.windows[self.current_window].worldDisplayListItems[item.index].append(weakref.proxy(item))
                    #print("klampt.vis: world added to the visualization's world (items:",self.current_worlds,")")
            

    def do_idle_checks(self):
        #print("GLUTWindowManager.idle checks")
        if self.quit:
            if bool(glutLeaveMainLoop):
                glutLeaveMainLoop()
            else:
                for w in self.windows:
                    w.close()
                    w.glwindow = None
            return
        for windex,winfo in enumerate(self.windows):
            #print(winfo.name,winfo.glwindow,winfo.mode)
            if winfo.glwindow is None and winfo.mode in ['shown','dialog']:
                print("GLUTWindowManager: Launching window %d inside vis thread"%(windex,))
                w = glinit._GLBackend.createWindow(winfo.name)
                self._frontend.windowinfo = weakref.proxy(winfo)
                self._frontend.window_manager = weakref.proxy(self)
                self._frontend.name = winfo.name
                w.setProgram(self._frontend)
                winfo.glwindow = w
                w.initialize()
            if not winfo.frontend.hidden:
                if winfo.mode == 'hidden':
                    print("GLUTWindowManager: hiding window %d (%s)"%(windex,winfo.name))
                    winfo.frontend.hidden = True
                    if winfo.glwindow is not None:
                        glutSetWindow(winfo.glwindow.glutWindowID)
                        glutHideWindow()
            else:
                #print("hidden, waiting...",self.windowinfo.mode)
                if winfo.mode == 'shown':
                    print("GLUTWindowManager: showing window %d (%s)"%(windex,winfo.name))
                    print("GLUT ID",winfo.glwindow.glutWindowID)
                    glutSetWindow(winfo.glwindow.glutWindowID)
                    glutShowWindow()
                    winfo.frontend.hidden = False
                elif winfo.mode == 'dialog':
                    print("GLUTWindowManager: showing window %d (%s) in dialog mode"%(windex,winfo.name))
                    print("GLUT ID",winfo.glwindow.glutWindowID)
                    winfo.frontend.inDialog = True
                    glutSetWindow(winfo.glwindow.glutWindowID)
                    glutShowWindow()
                    winfo.frontend.hidden = False

        if self.in_vis_loop and (len(self.windows)==0 or all(w.mode == 'hidden' for w in self.windows)):
            print("klampt.vis: No windows shown, breaking out of vis loop")
            if bool(glutLeaveMainLoop):
                glutLeaveMainLoop()
            else:
                while glutGetWindow():
                    for w in self.windows:
                        w.close()
                for w in self.windows:
                    w.glwindow = None
            return

        self.in_app_thread = True
        calls = self.threadcalls
        self.threadcalls = []
        for c in calls:
            c()
        if self.callback:
            self.callback()
        self.in_app_thread = False
        return

    def screenshot(self,format,want_depth):
        if threading.current_thread().__class__.__name__ != '_MainThread':
            #already in visualization loop -- just get the image
            return self._frontend.get_screen(format,want_depth)
        return_values = []
        def storeScreenshot(img,depth=None,return_values=return_values):
            return_values.append((img,depth))
        self.screenshotCallback(storeScreenshot,format,want_depth)
        #wait for the vis thread to call the function
        while len(return_values)==0:
            time.sleep(0.01)
        res = return_values[0]
        if not want_depth:
            return res[0]
        else:
            return res

    def screenshotCallback(self,fn,format,want_depth):
        if threading.current_thread().__class__.__name__ != '_MainThread':
            #already in visualization loop -- just get the image
            res = self._frontend.get_screen(format,want_depth)
            if want_depth:
                fn(*res)
            else:
                fn(res)
        def do_screenshot_callback(fn=fn,format=format,want_depth=want_depth):
            res = self._frontend.get_screen(format,want_depth)
            if want_depth:
                fn(*res)
            else:
                fn(res)
        self.threadCall(do_screenshot_callback)


class GLUTVisualizationFrontend(GLVisualizationFrontend):
    def __init__(self,windowinfo):
        GLVisualizationFrontend.__init__(self)
        self.scene = GLUTVisualizationPlugin()
        self.setPlugin(self.scene)
        self.scene.program = weakref.proxy(self)
        self.windowinfo = windowinfo
        self.window_manager = None
        self.inDialog = False
        self.hidden = False
        self.inSubwindow = False
    def display(self):
        global _globalLock
        _globalLock.acquire()
        GLVisualizationFrontend.display(self)
        _globalLock.release()
        return True
    def display_screen(self):
        global _globalLock
        _globalLock.acquire()
        GLVisualizationFrontend.display_screen(self)
        _globalLock.release()
        if self.inSubwindow: 
            return
        glDisable(GL_LIGHTING)
        glColor3f(1,1,1)
        y = 30
        glRasterPos(20,y)
        gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"(Do not close this window except to quit)")
        y += 25
        if self.inDialog:
            glColor3f(1,1,0)
            glRasterPos(20,y)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Dialog mode. Press 'Esc' to return to normal mode")
            y += 25
        else:
            glColor3f(1,1,0)
            glRasterPos(20,y)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Window mode. Press 'Esc' to hide window")
            y += 25
        for a in self.actions:
            if a.key is not None:
                glColor3f(0,0,0)
                glRasterPos(20,y)
                desc = a.short_text 
                if a.description is not None and a.description != a.short_text:
                    desc = desc + ". "+a.description
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_12,a.key+": "+desc)
                y += 14
    def keyboardfunc(self,c,x,y):
        if not self.inSubwindow: 
            if len(c)==1 and ord(c)==27:
                if self.inDialog:
                    print("Esc pressed, hiding dialog")
                    self.inDialog = False
                else:
                    print("Esc pressed, hiding window")
                global _globalLock
                _globalLock.acquire()
                self.windowinfo.mode = 'hidden'
                _globalLock.release()
                return True
            if isinstance(c,bytes):
                c = c.decode('utf-8')
            for a in self.actions:
                if a.key is None:
                    continue
                if a.key.startswith('Ctrl'):
                    if 'ctrl' in self.modifiers():
                        if a.key[5:] == c:
                            a.hook()
                elif a.key.startswith('Shift'):
                    if 'shift' in self.modifiers():
                        if a.key[6:] == c:
                            a.hook()
                elif a.key.startswith('Alt'):
                    if 'alt' in self.modifiers():
                        if a.key[4:] == c:
                            a.hook()
                elif a.key == c:
                    a.hook()
        else:
            return GLVisualizationFrontend.keyboardfunc(self,c,x,y)
    
    def idlefunc(self):
        global _globalLock
        _globalLock.acquire()
        self.window_manager.do_idle_checks()
        _globalLock.release()
        return GLVisualizationFrontend.idlefunc(self)


class GLUTMultiWindowVisualizationFrontend(glcommon.GLMultiViewportProgram):
    def __init__(self,windowinfo):
        glcommon.GLMultiViewportProgram.__init__(self)
        self.windowinfo = windowinfo
        self.window_manager = None
        self.inDialog = False
        self.hidden = False
        self.inSubwindow = False
    def addView(self,view):
        warnings.warn("addView will be deprecated in favor of add_view in a future version of Klampt",DeprecationWarning)
        self.add_view(view)
    def add_view(self,view):
        if isinstance(view,(GLUTVisualizationFrontend,GLUTMultiWindowVisualizationFrontend)):
            view.inSubwindow = True
        glcommon.GLMultiViewportProgram.add_view(self,view)
    def display_screen(self):
        glcommon.GLMultiViewportProgram.display_screen(self)
        if not self.inSubwindow:
            glDisable(GL_LIGHTING)
            glColor3f(1,1,1)
            glRasterPos(20,50)
            gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"(Do not close this window except to quit)")
            if self.inDialog:
                glColor3f(1,1,0)
                glRasterPos(20,80)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Dialog mode. Press 'Esc' to return to normal mode")
            else:
                glColor3f(1,1,0)
                glRasterPos(20,80)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,"In Window mode. Press 'Esc' to hide window")

    def keyboardfunc(self,c,x,y):
        if len(c)==1 and ord(c)==27:
            if self.inDialog:
                print("Esc pressed, hiding dialog")
                self.inDialog = False
            else:
                print("Esc pressed, hiding window")
            global _globalLock
            _globalLock.acquire()
            self.windowinfo.mode = 'hidden'
            _globalLock.release()
            return True
        else:
            return glcommon.GLMultiViewportProgram.keyboardfunc(self,c,x,y)
    
    def idlefunc(self):
        global _globalLock
        _globalLock.acquire()
        self.window_manager.do_idle_checks()
        _globalLock.release()
        return glcommon.GLMultiViewportProgram.idlefunc(self)


class GLUTVisualizationPlugin(GLVisualizationPlugin):
    def __init__(self):
        GLVisualizationPlugin.__init__(self)

    def add(self,name,item,keepAppearance=False,**kwargs):
        GLVisualizationPlugin.add(self,name,item,keepAppearance,**kwargs)
        #need to check whether the item is part of the current GLUT world
        if self.program and self.program.window_manager:
            self.program.window_manager._checkWindowCurrent(item)
