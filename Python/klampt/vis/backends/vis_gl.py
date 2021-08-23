from ..visualization import _globalLock,VisualizationScene
from .. import glcommon
import weakref
from collections import defaultdict
from OpenGL.GL import *
import warnings

class WindowInfo:
    """Mode can be hidden, shown, or dialog"""
    def __init__(self,name,frontend,glwindow=None):
        self.name = name
        self.frontend = frontend
        self.glwindow = glwindow
        self.mode = 'hidden'
        self.guidata = None
        self.custom_ui = None
        self.doRefresh = False
        self.doReload = False
        #needed for GLUT to work properly with multiple windows
        self.worlds = []
        self.active_worlds = []
        self.worldDisplayListItems = defaultdict(list)


class GLVisualizationFrontend(glcommon.GLPluginProgram):
    def __init__(self):
        glcommon.GLPluginProgram.__init__(self)
        self.scene = GLVisualizationPlugin()
        self.setPlugin(self.scene)
        self.scene.program = weakref.proxy(self)
        self.rendered = False

    def addAction(self,hook,short_text,key,description):
        self.add_action(hook,short_text,key,description)

    def displayfunc(self):
        glcommon.GLPluginProgram.displayfunc(self)
        self.rendered = True


class GLVisualizationPlugin(glcommon.GLWidgetPlugin,VisualizationScene):
    def __init__(self):
        glcommon.GLWidgetPlugin.__init__(self)
        VisualizationScene.__init__(self)
        #need to set this to a weakref of the GLProgram being used with this plugin. Automatically done in GLVisualizationFrontend()
        self.program = None
        self.backgroundImage = None
        self.backgroundImageUploaded = False
        self.backgroundImageTexture = None
        self.backgroundImageDisplayList = None
        self.backgroundImageMode = 'stretch'
        self.click_callback = None
        self.hover_callback = None
        self.click_filter = None
        self.hover_highlight_color = (1,1,0,0.5)
        self.click_tolerance = 0.01

    def initialize(self):
        #keep or refresh display lists?
        #self._clearDisplayLists()
        return glcommon.GLWidgetPlugin.initialize(self)

    def getViewport(self):
        return self.view

    def setViewport(self,viewport):
        self.program.set_view(viewport)

    def setBackgroundColor(self,r,g,b,a=1): 
        if self.program is not None:
            self.program.clearColor = [r,g,b,a]
        else:
            print("setBackgroundColor(): doesn't work yet because scene is not bound to a window")

    def edit(self,name,doedit=True):
        global _globalLock
        with _globalLock:
            obj = self.getItem(name)
            if obj is None:
                raise ValueError("Object "+name+" does not exist in visualization")
            if doedit:
                world = self.items.get('world',None)
                if world is not None:
                    world=world.item
                obj.make_editor(world)
                if obj.editor:
                    self.klamptwidgetmaster.add(obj.editor)
            else:
                if obj.editor:
                    self.klamptwidgetmaster.remove(obj.editor)
                    obj.remove_editor()
            self.doRefresh = True
        return obj.editor if doedit else None
    
    def pick(self,click_callback,hover_callback,highlight_color,filter,tolerance):
        global _globalLock
        with _globalLock:
            self.click_callback = click_callback
            self.hover_callback = hover_callback
            self.click_filter = filter
            self.hover_highlight_color = highlight_color
            self.click_tolerance = tolerance

    def hide(self,name,hidden=True):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            item.attributes['hidden'] = hidden
            if item.editor is not None:
                if hidden:
                    self.klamptwidgetmaster.remove(item.editor)
                else:
                    self.klamptwidgetmaster.add(item.editor)
            self.doRefresh = True
        
    def remove(self,name):
        global _globalLock
        with _globalLock:
            item = self.getItem(name)
            if item.editor is not None:
                self.klamptwidgetmaster.remove(item.editor)
        VisualizationScene.remove(self,name)

    def display(self):
        global _globalLock
        with _globalLock:
            #upload background image to a texture if added
            if self.backgroundImage is not None:
                if not self.backgroundImageUploaded:
                    (img,rows,cols,pixformat)= self.backgroundImage
                    if self.backgroundImageTexture is None:
                        self.backgroundImageTexture = glGenTextures(1)
                        glBindTexture(GL_TEXTURE_2D, self.backgroundImageTexture)
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
                        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
                        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
                    else:
                        glBindTexture(GL_TEXTURE_2D, self.backgroundImageTexture)
                    
                    glPixelStorei(GL_UNPACK_ALIGNMENT,1)
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, cols, rows, 0, pixformat, GL_UNSIGNED_BYTE, img)
                    glBindTexture(GL_TEXTURE_2D, 0)
                    self.backgroundImageUploaded = True
            else:
                #make sure to free stuff inside the visualization loop
                if self.backgroundImageDisplayList is not None:
                    self.backgroundImageDisplayList.destroy()
                    self.backgroundImageDisplayList = None
                if self.backgroundImageTexture is not None:
                    glDeleteTextures([self.backgroundImageTexture])
                    self.backgroundImageTexture = None
            #render background image if it exists
            if self.backgroundImageTexture is not None:
                glDisable(GL_CULL_FACE)
                glMatrixMode(GL_PROJECTION)
                glLoadIdentity()
                rows,cols = self.backgroundImage[1],self.backgroundImage[2]
                if self.backgroundImageMode == 'stretch':
                    glOrtho(-1,1,1,-1,-1,1)
                elif self.backgroundImageMode == 'scale':
                    xscale = cols / self.view.w
                    yscale = rows / self.view.h
                    scale = max(xscale,yscale)
                    xrange = xscale / scale
                    yrange = yscale / scale
                    glOrtho(-1.0/xrange,1.0/xrange,1.0/yrange,-1.0/yrange,-1,1)
                elif self.backgroundImageMode == 'fixed':
                    xrange = cols / self.view.w
                    yrange = rows / self.view.h
                    #xrange = 2/(r-l), r-l = 2/xrange
                    glOrtho(-1.0,2/xrange-1,2.0/yrange-1,-1,-1,1)
                elif self.backgroundImageMode == 'center':
                    xrange = cols / self.view.w
                    yrange = rows / self.view.h
                    glOrtho(-1.0/xrange,1.0/xrange,1.0/yrange,-1.0/yrange,-1,1)
                else:
                    glOrtho(-1,1,1,-1,-1,1)
                    warnings.warn("Invalid background image mode {}".format(self.backgroundImageMode))
                    self.backgroundImageMode = 'stretch'
                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                glDisable(GL_DEPTH_TEST)
                glDepthMask(GL_FALSE)
                glEnable(GL_TEXTURE_2D)
                glDisable(GL_LIGHTING)
                glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
                if self.backgroundImageDisplayList is not None:
                    self.backgroundImageDisplayList.draw(self._drawBackgroundImage)
                glDisable(GL_TEXTURE_2D)
                glEnable(GL_CULL_FACE)
                glEnable(GL_LIGHTING)
                glEnable(GL_DEPTH_TEST)
                glDepthMask(GL_TRUE)

                self.program.view.set_current_GL()

            #for items currently being edited AND having the appearance changed, draw the reference object
            #according to the vis settings
            #glcommon.GLWidgetPlugin.display(self)
            #restore any reference objects
            self.updateCamera()
            self.renderGL(self.view)

    def setBackgroundImage(self,img,format='auto',rows='auto',mode='stretch'):
        """Sets an image to go underneath the OpenGL rendering.

        ``img`` must be a list of bytes or numpy array.  If bytes, then
        ``format`` can be either 'rgb' (same as 'auto'), 'bgr', 'rgba', or 
        'bgra', corresponding to the OpenGL format constant.  In this case,
        ``rows`` must be an integer.

        If img is a numpy array with shape w x h x 3, it is assumed to have rgb
        information as channels.  If dtype=uint8, they are assumed to be
        bytes, and otherwise they are assumed to be floats in the range [0,1]

        If img has shape w x h, it is assumed to have rgb information as 
        integers 0x00rrggbb by default.  If format='bgr', then the rgb
        information is assumed to be integers 0xbbggrr.

        If ``img == None``, then the background image is cleared.

        ``mode`` controls how the image is resized to the screen size. 

        - 'stretch' resizes the image to the screen.
        - 'scale' resizes the image with a uniform scale to take up either the
          full width or height of the screen.
        - 'fixed' keeps the image size constant and matched to the upper left.
        - 'center' keeps the image size constant and fixed to the center.
        - 'tile' uses the image as a repeating texture.

        """
        global _globalLock
        if img is None:
            with _globalLock:
                self.backgroundImage = None
                self.backgroundImageUploaded = False
            return
            
        pixformat = GL_RGBA
        if hasattr(img,'shape'):
            import numpy as np
            if len(img.shape) == 3:
                rows = img.shape[0]
                cols = img.shape[1]
                assert img.shape[2] == 3
                if img.dtype == np.uint8:
                    img = img.tobytes()
                else:
                    img = (img*255.0).convert(dtype=np.uint8).tobytes()
                pixformat = GL_RGB
            else:
                assert len(img.shape)==2
                rows = img.shape[0]
                cols = img.shape[1]
                if img.dtype != np.uint32:
                    img = img.astype(np.uint32)
                img = img.tobytes()
                if format == 'bgr':
                    pixformat = GL_BGRA
        else:
            if not isinstance(rows,int):
                raise ValueError("rows must be an integer, not 'auto'")
            pixformat = GL_RGB
            pixsize = 3
            if format == 'bgr':
                pixformat = GL_BGR
            elif format == 'rgba':
                pixsize = 4
                pixformat = GL_RGBA
            elif format == 'bgra':
                pixsize = 4
                pixformat = GL_BGRA
            cols = len(img) // (rows*pixsize)

        with _globalLock:
            self.backgroundImage = (img,rows,cols,pixformat)
            self.backgroundImageUploaded = False
            if self.backgroundImageDisplayList is None:
                self.backgroundImageDisplayList = glcommon.CachedGLObject()
            else:
                self.backgroundImageDisplayList.markChanged()
            self.backgroundImageMode = mode

    def _drawBackgroundImage(self):
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D,self.backgroundImageTexture)
        glBegin(GL_TRIANGLE_FAN)
        glTexCoord2f(0,0)
        glVertex2f(-1,-1)
        glTexCoord2f(1,0)
        glVertex2f(1,-1)
        glTexCoord2f(1,1)
        glVertex2f(1,1)
        glTexCoord2f(0,1)
        glVertex2f(-1,1)
        glEnd()

    def widgetchangefunc(self,edit):
        """Called by GLWidgetPlugin on any widget change"""
        for name,item in self.items.items():
            item.update_editor()

    def display_screen(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.display_screen(self)
        self.renderScreenGL(self.view,self.window)
        _globalLock.release()

    def reshapefunc(self,w,h):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.reshapefunc(self,w,h)
        _globalLock.release()
    def keyboardfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardfunc(self,c,x,y)
        _globalLock.release()
    def keyboardupfunc(self,c,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.keyboardupfunc(self,c,x,y)
        _globalLock.release()
    def mousefunc(self,button,state,x,y):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.mousefunc(self,button,state,x,y)
        if button == 0 and state == 0:
            if self.click_callback is not None:
                cb = self.click_callback
                self.click_callback = None
                self.hover_callback = None
                (item,pt) = self.rayCast(x,y,self.click_filter,self.click_tolerance)
                cb(item.name,item.item,pt)
                if self.hover_item is not None:
                    self.hover_item.highlight(None)
                self.hover_item = None
        _globalLock.release()
    def motionfunc(self,x,y,dx,dy):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.motionfunc(self,x,y,dx,dy)
        if self.hover_callback is not None:
            (item,pt) = self.rayCast(x,y,self.click_filter,self.click_tolerance)
            self.hover_callback(item.name,item.item,pt)
            if self.hover_item is not None:
                if item is None:
                    self.hover_item.highlight(None)
                else:
                    self.hover_item.highlight(self.hover_highlight_color)
            self.hover_item = item
        _globalLock.release()
    def eventfunc(self,type,args=""):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.eventfunc(self,type,args)
        _globalLock.release()
    def closefunc(self):
        global _globalLock
        _globalLock.acquire()
        glcommon.GLWidgetPlugin.closefunc(self)
        _globalLock.release()

    def idle(self):
        global _globalLock
        _globalLock.acquire()
        VisualizationScene.updateTime(self)
        _globalLock.release()
        return False


