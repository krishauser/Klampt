from ..visualization import _WindowManager,_ThreadedWindowManager,VisualizationScene,_globalLock
from .vis_gl import WindowInfo,GLVisualizationFrontend,GLVisualizationPlugin
from .. import glinit,glcommon
import sys
import weakref
import time
import threading

if not glinit.available('PyQt'):
    raise ImportError("Can't import vis_qt without first calling glinit.init() or vis.init()")

if glinit.available('PyQt5'):
    from PyQt5.QtCore import *
    from PyQt5.QtWidgets import *
else:
    from PyQt4.QtGui import *
    from PyQt4.QtCore import *


class MyQThread(QThread):
    def __init__(self,func,*args):
        self.func = func
        self.args = args
        QThread.__init__(self)
    def run(self):
        self.func(*self.args)


class QtWindowManager(_ThreadedWindowManager):
    def __init__(self):
        self._frontend = GLVisualizationFrontend()
        #list of WindowInfo's
        self.windows = []
        #the index of the current window
        self.current_window = None
        #the name of a window, if no windows exist yet
        self.window_title = "Klamp't visualizer (%s)"%(sys.argv[0],)
        #the current temp frontend if len(self.windows)=0, or windows[current_window].frontend
        _ThreadedWindowManager.__init__(self)

    def reset(self):
        _ThreadedWindowManager.reset(self)
        self.cleanup()
        self._frontend = GLVisualizationFrontend()
        self.windows = []
        self.current_window = None
        self.window_title = "Klamp't visualizer (%s)"%(sys.argv[0],)
    
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
        if title is None:
            title = "Window "+str(len(self.windows))
        #make a new window
        self._frontend = GLVisualizationFrontend()
        self.windows.append(WindowInfo(title,self._frontend))
        self.window_title = title
        id = len(self.windows)-1
        self.current_window = id
        return id

    def setWindow(self,id):
        if id == self.current_window:
            return
        assert id >= 0 and id < len(self.windows),"Invalid window id"
        self._frontend = self.windows[id].frontend
        self.window_title = self.windows[id].name
        #print "vis.setWindow(",id,") the window has status",_windows[id].mode
        self.current_window = id

    def getWindow(self):
        return self.current_window

    def setPlugin(self,plugin):
        if not isinstance(self._frontend,GLVisualizationFrontend):
            #was multi-view -- now setting plugin
            self._frontend = GLVisualizationFrontend()
            if self.current_window is not None:
                if self.windows[self.current_window].glwindow is not None:
                    self._frontend.window = self.windows[self.current_window].glwindow
        if plugin is None:
            self._frontend.set_plugin(self._frontend.scene)
        else:
            self._frontend.set_plugin(plugin)
        self.onFrontendChange()

    def pushPlugin(self,plugin):
        assert isinstance(self._frontend,glcommon.GLPluginProgram),"Can't push a plugin after splitView"
        if len(self._frontend.plugins) == 0:
            self._frontend.set_plugin(self._frontend.scene)
        self._frontend.push_plugin(plugin)
        self.onFrontendChange()

    def popPlugin(self):
        self._frontend.pop_plugin()
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
            multiProgram = glcommon.GLMultiViewportProgram()
            multiProgram.window = None
            if self.current_window is not None:
                if self.windows[self.current_window].glwindow is not None:
                    multiProgram.window = self.windows[self.current_window].glwindow
            multiProgram.add_view(self._frontend)
            multiProgram.add_view(plugin)
            multiProgram.name = self.window_title
            multiProgram.scene = self._frontend.scene
            self._frontend = multiProgram
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

    def run_app_thread(self,callback=None):
        global _globalLock
        self.vis_thread_running = True

        if len(self.windows)==0:
            #first call
            self.windows.append(WindowInfo(self.window_title,self._frontend)) 
            self.current_window = 0
            self.windows[self.current_window].mode = 'shown'
        else:
            if self.windows[self.current_window].mode == 'hidden':
                self.windows[self.current_window].mode = 'shown'
        glinit._GLBackend.initialize("Klamp't visualization")
        
        res = None
        while not self.quit:
            _globalLock.acquire()
            try:  #to be nice to the other thread, we put everything in a try block in case something weird happens
                calls = self.threadcalls
                self.threadcalls = []
                for i,w in enumerate(self.windows):
                    if w.glwindow is None and w.mode != 'hidden':
                        print("vis: creating GL window")
                        w.glwindow = glinit._GLBackend.createWindow(w.name)
                        w.glwindow.setProgram(w.frontend)
                        w.glwindow.setParent(None)
                        w.glwindow.refresh()
                    if w.doRefresh:
                        if w.mode != 'hidden':
                            w.glwindow.updateGL()
                        w.doRefresh = False
                    if w.doReload and w.glwindow is not None:
                        w.glwindow.setProgram(w.frontend)
                        if w.guidata:
                            w.guidata.setWindowTitle(w.name)
                            w.guidata.glwidget = w.glwindow
                            if hasattr(w.guidata,'attachGLWindow'):
                                w.guidata.attachGLWindow()
                            #else:
                            #    w.glwindow.setParent(w.guidata)
                        w.doReload = False
                    if w.mode == 'dialog':
                        print("#########################################")
                        print("klampt.vis: Dialog on window",i)
                        print("#########################################")
                        if w.custom_ui is None:
                            dlg = _MyDialog(w)
                        else:
                            dlg = w.custom_ui(w.glwindow)
                        if dlg is not None:
                            w.glwindow.show()
                            self.in_app_thread = True
                            _globalLock.release()
                            res = dlg.exec_()
                            _globalLock.acquire()
                            w.glwindow.hide()
                            w.glwindow.setParent(None)
                                
                            self.in_app_thread = False
                        print("#########################################")
                        print("klampt.vis: Dialog done on window",i)
                        print("#########################################")
                        w.glwindow.hide()
                        w.glwindow.setParent(None)
                        w.mode = 'hidden'
                    if w.mode == 'shown' and w.guidata is None:
                        print("#########################################")
                        print("klampt.vis: Making window",i)
                        print("#########################################")
                        if w.custom_ui is None:
                            w.guidata = _MyWindow(w)
                        else:
                            w.guidata = w.custom_ui(w.glwindow)
                        def closeMonkeyPatch(self,event,windowinfo=w,index=i,oldcloseevent=w.guidata.closeEvent):
                            oldcloseevent(event)
                            if not event.isAccepted():
                                return
                            windowinfo.mode='hidden'
                            print("#########################################")
                            print("klampt.vis: Window",index,"close")
                            print("#########################################")
                            _globalLock.acquire()
                            w.glwindow.hide()
                            w.mode = 'hidden'
                            if w.glwindow.idleTimer is not None:
                                w.glwindow.idlesleep()
                            w.glwindow.setParent(None)
                            _globalLock.release()
                        w.guidata.closeEvent = closeMonkeyPatch.__get__(w.guidata, w.guidata.__class__)
                        w.guidata.setWindowTitle(w.name)
                        w.glwindow.show()
                        w.guidata.show()
                        if w.glwindow.initialized:
                            #boot it back up again
                            w.glwindow.idlesleep(0)
                    if w.mode == 'shown' and not w.guidata.isVisible():
                        print("#########################################")
                        print("klampt.vis: Showing window",i)
                        print("#########################################")
                        if hasattr(w.guidata,'attachGLWindow'):
                            w.guidata.attachGLWindow()
                        else:
                            w.glwindow.setParent(w.guidata)
                        w.glwindow.show()
                        w.guidata.show()
                    if w.mode == 'hidden' and w.guidata is not None:
                        #prevent deleting the GL window
                        if hasattr(w.guidata,'detachGLWindow'):
                            w.guidata.detachGLWindow()
                        else:
                            w.glwindow.setParent(None)
                            w.guidata.setParent(None)
                        if w.guidata.isVisible():
                            print("#########################################")
                            print("klampt.vis: Hiding window",i)
                            print("#########################################")
                            w.glwindow.hide()
                            w.guidata.hide()
                        w.guidata.close()
                        w.guidata = None
            except Exception:
                print("Exception called in visualization thread; closing.")
                _globalLock.release()
                self.cleanup()
                self.vis_thread_running = False
                raise
            except RuntimeError:
                print("RuntimeError called in visualization thread; assuming this is exception in main thread. Thread closing.")
                _globalLock.release()
                self.cleanup()
                self.vis_thread_running = False
                return
            _globalLock.release()
            self.in_app_thread = True
            for c in calls:
                c()
            glinit._GLBackend.app.processEvents()
            self.in_app_thread = False
            if callback:
                callback()
            else:
                if not self.in_vis_loop:
                    #give other threads time to work
                    time.sleep(0.001)
            if self.in_vis_loop and (len(self.windows)==0 or all(w.mode == 'hidden' for w in self.windows)):
                print("klampt.vis: No windows shown, breaking out of vis loop")
                self.vis_thread_running = False
                return
        print("Visualization thread closing and cleaning up Qt...")
        self.cleanup()
        self.vis_thread_running = False
        return res


    def show(self):
        if len(self.windows)==0:
            self.windows.append(WindowInfo(self.window_title,self._frontend)) 
            self.current_window = 0
        self.windows[self.current_window].mode = 'shown'
        if self.in_vis_loop:
            #this will be handled in the loop, no need to start it
            return
        if not self.vis_thread_running:
            self._start_app_thread()

    def shown(self):
        return (self.vis_thread_running and self.current_window is not None and self.windows[self.current_window].mode in ['shown','dialog'] or self.windows[self.current_window].guidata is not None)

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
        if self.vis_thread_running:
            if self.in_vis_loop:
                #single threaded
                w.mode = 'dialog'
                if w.glwindow is None:
                    print("vis: creating GL window")
                    w.glwindow = glinit._GLBackend.createWindow(w.name)
                    w.glwindow.setProgram(w.frontend)
                    w.glwindow.setParent(None)
                    w.glwindow.refresh()
                if w.custom_ui is None:
                    dlg = _MyDialog(w)
                else:
                    dlg = w.custom_ui(w.glwindow)
                print("#########################################")
                print("klampt.vis: Dialog starting on window",self.current_window)
                print("#########################################")
                res = None
                if dlg is not None:
                    w.glwindow.show()
                    res = dlg.exec_()
                print("#########################################")
                print("klampt.vis: Dialog done on window",self.current_window)
                print("#########################################")
                w.glwindow.hide()
                w.glwindow.setParent(None)
                w.mode = 'hidden'
                return res
                raise RuntimeError("Can't call dialog() inside loop().")
            #just show the dialog and let the thread take over
            assert w.mode == 'hidden',"dialog() called inside dialog?"
            print("#########################################")
            print("klampt.vis: Creating dialog on window",self.current_window)
            print("#########################################")
            _globalLock.acquire()
            w.mode = 'dialog'
            _globalLock.release()

            if not self.in_app_thread or threading.current_thread().__class__.__name__ == '_MainThread':
                print("vis.dialog(): Waiting for dialog on window",self.current_window,"to complete....")
                assert w.mode == 'dialog'
                while w.mode == 'dialog':
                    time.sleep(0.1)
                if w.mode == 'shown':
                    print("klampt.vis: warning, dialog changed from 'dialog' to 'shown' mode?")
                print("vis.dialog(): ... dialog done")
            else:
                #called from another dialog or window!
                print("vis: Creating a dialog from within another dialog or window")
                _globalLock.acquire()
                if w.glwindow is None:
                    print("vis: creating GL window")
                    w.glwindow = glinit._GLBackend.createWindow(w.name)
                    w.glwindow.setProgram(w.frontend)
                    w.glwindow.setParent(None)
                    w.glwindow.refresh()
                if w.custom_ui is None:
                    dlg = _MyDialog(w)
                else:
                    dlg = w.custom_ui(w.glwindow)
                print("#########################################")
                print("klampt.vis: Dialog starting on window",self.current_window)
                print("#########################################")
                if dlg is not None:
                    w.glwindow.show()
                    _globalLock.release()
                    res = dlg.exec_()
                    _globalLock.acquire()
                print("#########################################")
                print("klampt.vis: Dialog done on window",self.current_window)
                print("#########################################")
                w.glwindow.hide()
                w.glwindow.setParent(None)
                w.mode = 'hidden'
                for i,w2 in enumerate(self.windows):
                    print("Returning from dialog items",i,w2.mode)
                _globalLock.release()
            return None
        else:
            w.mode = 'dialog'
            if self.multithreaded():
                print("#########################################")
                print("klampt.vis: Running multi-threaded dialog, waiting to complete...")
                self._start_app_thread()
                assert w.mode == 'dialog'
                while w.mode == 'dialog':
                    time.sleep(0.1)
                if w.mode == 'shown':
                    print("klampt.vis: warning, dialog changed from 'dialog' to 'shown' mode?")
                print("klampt.vis: ... dialog done")
                print("#########################################")
                return None
            else:
                print("#########################################")
                print("klampt.vis: Running single-threaded dialog")
                self.in_vis_loop = True
                res = self.run_app_thread()
                self.in_vis_loop = False
                print("klampt.vis: ... dialog done.")
                print("#########################################")
                return res

    def set_custom_ui(self,func):
        if len(self.windows)==0:
            print("Making first window for custom ui")
            self.windows.append(WindowInfo(self.window_title,self._frontend))
            self.current_window = 0
        self.windows[self.current_window].custom_ui = func
        if func is not None:
            print("klampt.vis: setting custom ui on window",self.current_window)
        return

    def onFrontendChange(self):
        if self.current_window is None:
            return
        w = self.windows[self.current_window]
        w.doReload = True
        w.name = self.window_title
        w.frontend = self._frontend
        if w.glwindow:
            w.glwindow.reshape(self._frontend.view.w,self._frontend.view.h)

    def cleanup(self):
        for w in self.windows:
            w.frontend.scene.clear()
            if w.glwindow:
                try: 
                    w.glwindow.setParent(None)
                    w.glwindow.close()
                    #must be explicitly deleted for some reason in PyQt5...
                except RuntimeError:
                    #already deleted?
                    pass
                del w.glwindow
        glinit._GLBackend.app.processEvents()

        #must be explicitly deleted for some reason in PyQt5...
        del glinit._GLBackend.app
        glinit._GLBackend.app = None

    def screenshot(self,format,want_depth):
        if not self.multithreaded() or self.in_vis_loop or (self.in_app_thread and threading.current_thread().__class__.__name__ != '_MainThread'):
            #already in visualization loop -- just get the image
            return self._frontend.get_screen(format,want_depth)
        if not self.vis_thread_running:
            raise RuntimeError("Can't call screenshot until the thread is running")
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
        # if not self.multithreaded() or self.in_vis_loop:
        #     #already in visualization loop -- just get the image
        #     res = self._frontend.get_screen(format,want_depth)
        #     if want_depth:
        #         fn(*res)
        #     else:
        #         fn(res)
        def do_screenshot_callback(fn=fn,format=format,want_depth=want_depth):
            if not self._frontend.rendered:
                self.threadCall(do_screenshot_callback)
                return
            res = self._frontend.get_screen(format,want_depth)
            self._frontend.rendered = False  #don't do this callback until another frame is drawn
            if want_depth:
                fn(*res)
            else:
                fn(res)
        self.threadCall(do_screenshot_callback)

#Qt specific startup
#need to set up a QDialog and an QApplication
class _MyDialog(QDialog):
    def __init__(self,windowinfo):
        QDialog.__init__(self)
        self.windowinfo = windowinfo
        glwidget = windowinfo.glwindow
        glwidget.setMinimumSize(640,480)
        glwidget.setMaximumSize(4000,4000)
        glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))

        self.description = QLabel("Press OK to continue")
        self.description.setSizePolicy(QSizePolicy(QSizePolicy.Preferred,QSizePolicy.Fixed))
        self.layout = QVBoxLayout(self)
        self.layout.addWidget(glwidget)
        self.layout.addWidget(self.description)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok,Qt.Horizontal, self)
        self.buttons.accepted.connect(self.accept)
        self.layout.addWidget(self.buttons)
        self.setWindowTitle(windowinfo.name)
        glwidget.name = windowinfo.name
    
class _MyWindow(QMainWindow):
    def __init__(self,windowinfo):
        QMainWindow.__init__(self)
        self.windowinfo = windowinfo
        self.glwidget = windowinfo.glwindow
        self.glwidget.setMinimumSize(self.glwidget.width,self.glwidget.height)
        self.glwidget.setMaximumSize(4000,4000)
        self.glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
        self.setCentralWidget(self.glwidget)
        self.glwidget.setParent(self)
        self.setWindowTitle(windowinfo.name)
        self.glwidget.name = windowinfo.name
        self.saving_movie = False
        self.movie_timer = QTimer(self)
        self.movie_timer.timeout.connect(self.movie_update)
        self.movie_frame = 0
        self.movie_time_last = 0
        self.saving_html = False
        self.html_saver = None
        self.html_start_time = 0
        self.html_timer = QTimer(self)
        self.html_timer.timeout.connect(self.html_update)
        #TODO: for action-free programs, don't add this... but this has to be detected after initializeGL()?
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('&Actions')
        self.glwidget.actionMenu = fileMenu
        visMenu = mainMenu.addMenu('&Visualization')
        a = QAction('Edit appearances...', self)
        a.setStatusTip("Edit the appearance of items in the visualization")
        a.triggered.connect(self.edit_gui)
        self.edit_gui_window = None
        visMenu.addAction(a)
        a = QAction('Save world...', self)
        a.setStatusTip('Saves world to xml file')
        a.triggered.connect(self.save_world)
        visMenu.addAction(a)
        a = QAction('Add to world...', self)
        a.setStatusTip('Adds an item to the world')
        a.triggered.connect(self.add_to_world)
        visMenu.addAction(a)
        a = QAction('Save camera...', self)
        a.setStatusTip('Saves camera settings')
        a.triggered.connect(self.save_camera)
        visMenu.addAction(a)
        a = QAction('Load camera...', self)
        a.setStatusTip('Loads camera settings')
        a.triggered.connect(self.load_camera)
        visMenu.addAction(a)
        a = QAction('Start/stop movie output', self)
        a.setShortcut('Ctrl+M')
        a.setStatusTip('Starts / stops saving movie frames')
        a.triggered.connect(self.toggle_movie_mode)
        visMenu.addAction(a)
        a = QAction('Start/stop html output', self)
        a.setShortcut('Ctrl+H')
        a.setStatusTip('Starts / stops saving animation to HTML file')
        a.triggered.connect(self.toggle_html_mode)
        visMenu.addAction(a)
    
    def getWorld(self):
        if not hasattr(self.glwidget.program,'plugins'):
            return None
        if isinstance(self.glwidget.program,GLVisualizationFrontend):
            scene = self.glwidget.program.scene
            world = scene.items.get('world',None)
            if world is not None: return world.item
            sim = scene.items.get('sim',None)
            if sim is not None: return sim.world
        for p in self.glwidget.program.plugins:
            if hasattr(p,'world'):
                return p.world
            if hasattr(p,'sim'):
                return p.sim.world
            if hasattr(p,'simulator'):
                return p.simulator.world
        return None
    
    def getTimeSource(self):
        if not hasattr(self.glwidget.program,'plugins'):
            return None
        if isinstance(self.glwidget.program,GLVisualizationFrontend):
            scene = self.glwidget.program.scene
            if scene.timeCallback is not None:
                return scene.timeCallback
        for p in self.glwidget.program.plugins:
            if hasattr(p,'sim'):
                return lambda :p.sim.getTime()
            if hasattr(p,'simulator'):
                return lambda :p.simulator.getTime()
        return None
    
    def save_camera(self):
        if not hasattr(self.glwidget.program.scene,'getViewport'):
            print("Program does not appear to have a camera")
            return
        scene = self.glwidget.program.scene
        #fn = QFileDialog.getSaveFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)",options=QFileDialog.DontUseNativeDialog)
        fn = QFileDialog.getSaveFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)")
        if isinstance(fn,tuple):
            fn = fn[0]
        if fn is None:
            return
        v = scene.getViewport()
        v.save_file(fn)
    
    def load_camera(self):
        scene = self.glwidget.program.scene
        v = scene.getViewport()
        #fn = QFileDialog.getOpenFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)",options=QFileDialog.DontUseNativeDialog)
        fn = QFileDialog.getOpenFileName(caption="Viewport file (*.txt)",filter="Viewport file (*.txt);;All files (*.*)")
        if isinstance(fn,tuple):
            fn = fn[0]
        if fn is None:
            return
        v.load_file(fn)
        scene.setViewport(v)

    
    def save_world(self):
        w = self.getWorld()
        if w is None:
            print("Program does not appear to have a world")
        fn = QFileDialog.getSaveFileName(caption="World file (elements will be saved to folder)",filter="World file (*.xml);;All files (*.*)")
        if isinstance(fn,tuple):
            fn = fn[0]
        if fn is not None:
            w.saveFile(str(fn))
            print("Saved to",fn,"and elements were saved to a directory of the same name.")
    
    def add_to_world(self):
        w = self.getWorld()
        if w is None:
            print("Program does not appear to have a world")
        fn = QFileDialog.getOpenFileName(caption="World element",filter="Robot file (*.rob *.urdf);;Object file (*.obj);;Terrain file (*.env *.off *.obj *.stl *.wrl);;All files (*.*)")
        if isinstance(fn,tuple):
            fn = fn[0]
        if fn is not None:
            elem = w.loadElement(str(fn))
            if elem < 0:
                print("Failed loading element",str(fn))
            else:
                pass
                """
                for p in self.glwidget.program.plugins:
                    if isinstance(p,GLVisualizationPlugin):
                        p.getItem('world').setItem(w)
                """
    
    def toggle_movie_mode(self):
        self.saving_movie = not self.saving_movie
        if self.saving_movie:
            self.movie_timer.start(33)
            time_source = self.getTimeSource()
            if time_source is not None:
                self.movie_time_last = time_source()
        else:
            self.movie_timer.stop()
            dlg =  QInputDialog(self)                 
            dlg.setInputMode( QInputDialog.TextInput) 
            dlg.setLabelText("Command")
            dlg.setTextValue('ffmpeg -y -i image%04d.png -vcodec libx264 -pix_fmt yuv420p klampt_record.mp4')
            dlg.resize(600,100)                             
            ok = dlg.exec_()                                
            cmd = dlg.textValue()
            #(cmd,ok) = QInputDialog.getText(self,"Process with ffmpeg?","Command", text='ffmpeg -y -f image2 -i image%04d.png klampt_record.mp4')
            if ok:
                import os,glob
                os.system(str(cmd))
                print("Removing temporary files")
                for fn in glob.glob('image*.png'):
                    os.remove(fn)
    
    def movie_update(self):
        time_source = self.getTimeSource()
        if time_source is not None:
            tnow = time_source()
            while tnow >= self.movie_time_last + 1.0/30.0:
                self.glwidget.program.save_screen('image%04d.png'%(self.movie_frame))
                self.movie_frame += 1
                self.movie_time_last += 1.0/30.0
        else:
            self.glwidget.program.save_screen('image%04d.png'%(self.movie_frame))
            self.movie_frame += 1
    
    def toggle_html_mode(self):
        self.saving_html = not self.saving_html
        if self.saving_html:
            world = self.getWorld()
            if world is None:
                print("There is no world in the current plugin, can't save")
                self.saving_html = False
                return
            fn = QFileDialog.getSaveFileName(caption="Save path HTML file to...",filter="HTML file (*.html);;All files (*.*)")
            if isinstance(fn,tuple):
                fn = fn[0]
            if fn is None:
                self.saving_html = False
                return
            from ...io import html
            self.html_start_time = time.time()
            self.html_saver = html.HTMLSharePath(fn)
            self.html_saver.dt = 0.033;
            self.html_saver.start(world)
            self.html_timer.start(33)
        else:
            self.html_saver.end()
            self.html_timer.stop()
    
    def html_update(self):
        t = None
        if self.html_saver.sim is None:
            #t = time.time()-self.html_start_time
            t = self.html_saver.last_t + 0.034
        self.html_saver.animate(t)
    
    def edit_gui(self):
        if self.edit_gui_window:
            self.edit_gui_window.close()
            self.edit_gui_window = None
        try:
            import pyqtgraph as pg
            import pyqtgraph.parametertree.parameterTypes as pTypes
            from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
        except ImportError as e:
            print(e)
            print('Unable to edit, PyQtGraph is not installed.  Try "pip install pyqtgraph"')
            return

        def _item_to_params(world,name,visappearance):
            attrs = []
            itemdict = {'name':name, 'type':'group', 'children': attrs}
            if len(visappearance.subAppearances) > 0:
                attrs_plus_defaults = visappearance.attributes.overrides
            else:
                attrs_plus_defaults = visappearance.getAttributes()

            for k in sorted(attrs_plus_defaults.keys()):
                v = attrs_plus_defaults[k]
                vvalue = v
                vtype = v.__class__.__name__
                if k=='color':
                    vtype = 'color'
                    vvalue = (int(v[0]*255),int(v[1]*255),int(v[2]*255),int(v[3]*255))
                    #todo, add opacity
                elif k=='label':
                    vtype = 'str'
                elif k in ['hide_label','hidden']:
                    vtype = 'bool'
                elif k=='robot':
                    assert isinstance(v,int)
                    if world is not None:
                        robotnames = {}
                        for i in range(world.numRobots()):
                            robotnames[world.robot(i).getName()] = i
                        attrs.append({'name':k,'type':'list','values':robotnames,'value':v})
                        continue
                elif v is None:
                    #it's an optional parameter, skip for now
                    #TODO: handle this case
                    continue
                elif isinstance(v,(tuple,list)):
                    #its a tuple, skip for now
                    #TODO: handle this case
                    continue
                attrs.append({'name':k,'type':vtype,'value':vvalue})

            for k in sorted(visappearance.subAppearances.keys()):
                    v = visappearance.subAppearances[k]
                    attrs.append(_item_to_params(world,v.name,v))

            return itemdict

        params = []
        if isinstance(self.glwidget.program,GLVisualizationFrontend):
            items = []
            visdict = {'name':'Program', 'type':'group', 'children':items}
            world = self.getWorld()
            scene = self.glwidget.program.scene
            for (k,v) in scene.items.items():
                vdict = _item_to_params(world,k,v)
                items.append(vdict)
            params.append(visdict)
        if len(params)==1:
            params = params[0]['children']
        #print "Showing parameters",params

        ## Create tree of Parameter objects
        p = Parameter.create(name='params', type='group', children=params)

        ## If anything changes in the tree, print a message
        def change(param, changes):
            for param, change, data in changes:
                path = p.childPath(param)
                """
                if path is not None:
                    childName = '.'.join(path)
                else:
                    childName = param.name()
                print('  parameter: %s'% childName)
                print('  change:    %s'% change)
                print('  data:      %s'% str(data))
                print('  ----------')
                """
                if param.type()=='str':
                    value = data
                elif param.type()=='int':
                    value = int(data)
                elif param.type()=='bool':
                    value = bool(data)
                elif param.type()=='float':
                    value = float(data)
                elif param.type()=='color':
                    #data is a QColor
                    value = (float(data.red())/255.0,float(data.green())/255.0,float(data.blue())/255.0,float(data.alpha())/255.0)
                else:
                    raise ValueError("Can't convert to type "+param.type())
                if path[0].startswith("Plugin "):
                    pluginindex = int(path[0][7:])-1
                    plugin = self.glwidget.program.plugins[pluginindex]
                    path = path[1:]
                else:
                    plugin = self.glwidget.program.plugins[0]
                attr = path[-1]
                item = plugin.getItem(path[:-1])
                global _globalLock
                _globalLock.acquire()
                plugin._setAttribute(item,attr,value)
                _globalLock.release()
            self.glwidget.refresh()

        p.sigTreeStateChanged.connect(change)

        """
        def valueChanging(param, value):
            print("Value changing (not finalized): %s %s" % (param, value))
            
        # Too lazy for recursion:
        for child in p.children():
            child.sigValueChanging.connect(valueChanging)
            for ch2 in child.children():
                ch2.sigValueChanging.connect(valueChanging)
        """


        ## Create two ParameterTree widgets, both accessing the same data
        t = ParameterTree()
        t.setParameters(p, showTop=False)
        t.setWindowTitle('pyqtgraph example: Parameter Tree')

        def onload():
            fn = QFileDialog.getOpenFileName(caption="Load visualization config file",filter="JSON file (*.json);;All files (*.*)")
            if isinstance(fn,tuple):
                fn = fn[0]
            if fn is not None:
                self.loadJsonConfig(fn)
                print("TODO: update the edit window according to the loaded visualization parameters")

        def onsave():
            fn = QFileDialog.getSaveFileName(caption="Save visualization config file",filter="JSON file (*.json);;All files (*.*)")
            if isinstance(fn,tuple):
                fn = fn[0]
            if fn is not None:
                self.saveJsonConfig(fn)

        self.edit_gui_window = QWidget()
        self.edit_gui_window.setWindowTitle("Visualization appearance editor")
        layout = QGridLayout()
        self.edit_gui_window.setLayout(layout)
        loadButton = QPushButton("Load...")
        saveButton = QPushButton("Save...")
        loadButton.clicked.connect(onload)
        saveButton.clicked.connect(onsave)
        layout.addWidget(t,0,0,1,2)
        layout.addWidget(loadButton,1,0,1,1)
        layout.addWidget(saveButton,1,1,1,1)
        self.edit_gui_window.resize(400,800)
        self.edit_gui_window.show()
   
    def loadJsonConfig(self,fn):
        if isinstance(self.glwidget.program,GLVisualizationFrontend):
            scene = self.glwidget.program.scene
            scene.loadJsonConfig(fn)
            self.glwidget.refresh()
            return
        print("loadJsonConfig: no visualization plugins active")
    
    def saveJsonConfig(self,fn):
        if isinstance(self.glwidget.program,GLVisualizationFrontend):
            scene = self.glwidget.program.scene
            scene.saveJsonConfig(fn)
            return
        print("saveJsonConfig: no visualization plugins active")
    
    def closeEvent(self,event):
        if self.edit_gui_window:
            self.edit_gui_window.close()
            self.edit_gui_window = None
        if self.saving_movie:
            self.toggle_movie_mode()
        if self.saving_html:
            self.toggle_html_mode()
        #self.html_timer.deleteLater()
        #self.movie_timer.deleteLater()
    
    def close(self):
        """Called to clean up resources"""
        self.html_timer.stop()
        self.movie_timer.stop()
        self.html_timer.deleteLater()
        self.movie_timer.deleteLater()
        self.movie_timer = None
        self.html_timer = None
    
    def detachGLWindow(self):
        """Used for closing and restoring windows, while saving the OpenGL context"""
        self.glwidget.setParent(None)
    
    def attachGLWindow(self):
        """Used for closing and restoring windows, while saving the OpenGL context"""
        self.glwidget.setParent(self)
        self.setCentralWidget(self.glwidget)


