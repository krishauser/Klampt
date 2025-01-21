try:
    from OpenGL import GL,GLU
    _HAS_OPENGL = True
except:
    _HAS_OPENGL = False

from . import camera
from . import gldraw
from ..math import so3,se3,vectorops
from ..robotsim import Viewport
import math
import warnings
from ..model.typing import Vector3,RigidTransform
from typing import Tuple

class GLViewport(Viewport):
    """
    A class describing an OpenGL camera view.  Extends the capabilities of
    klampt.Viewport with a camera controller and some Mac OpenGL information.

    Attributes:
        x,y,w,h,n,f,fx,fy,cx,cy,ori,xform (from Viewport): the image box, projection,
            clipping planes, intrinsics, and orientation.
        screenDeviceScale (float): if not 1, multiply screen pixel coordinates
            by this to get OpenGL pixel coordinates (usually Mac Retina
            displays)
        controller: an orbit camera controlling the target, zoom, and orientation
            of the "trackball" (see :class:`orbit`)

    """
    def __init__(self):
        Viewport.__init__(self)
        self.ori = 'opengl'
        self.setFOV(math.radians(60.0))
        #near and far clipping planes
        self.n = 0.2
        self.f = 100
        self.screenDeviceScale = 1
        self.controller = camera.orbit()
        self.controller.dist = 6.0

    def contains(self, x : float, y : float) -> bool:
        return x >= self.x and y >= self.y and x < self.x + self.w and y < self.y + self.h

    def set_transform(self, T : RigidTransform, convention='standard') -> None:
        """Sets the pose of the camera, with T given in world coordinates.

        If convention = 'openGL', the Z axis of T is the *backward* direction of
        the camera, with Y pointing *up* and X pointing to the *right*.

        If convention = 'standard', the Z axis of T is the *forward* direction of
        the camera, with Y pointing *down* and X pointing to the *right*
        """
        if convention == 'openGL':
            self.controller.set_matrix(T)
        else:
            xzflip = [1,0,0,  0,-1,0,  0,0,-1]
            self.controller.set_matrix((so3.mul(T[0],xzflip),T[1]))

    def get_transform(self,convention='standard') -> RigidTransform:
        """Gets the pose of the camera, with T given in world coordinates.

        If convention = 'openGL', the Z axis of T is the *backward* direction of
        the camera, with Y pointing *up* and X pointing to the *right*.

        If convention = 'standard', the Z axis of T is the *forward* direction of
        the camera, with Y pointing *down* and X pointing to the *right*
        """
        if convention == 'openGL':
            return self.controller.matrix()
        else:
            T = self.controller.matrix()
            xzflip = [1,0,0,  0,-1,0,  0,0,-1]
            return (so3.mul(T[0],xzflip),T[1])

    def fit(self, center : Vector3, radius : float) -> None:
        """Fits the viewport to an object filling a sphere of a certain center
        and radius"""
        self.controller.tgt = center
        self.controller.dist = radius*2
        zmin,zmax = self.n, self.f
        if radius < self.n:
            zmin = radius*0.5
        if radius*3 > self.f:
            zmax =radius*3.5
        self.n = zmin
        self.f = zmax

    def update_viewport(self) -> Viewport:
        """Updates the pose of the `vp` attribute from the controller."""
        self.setPose(*self.controller.matrix())
        return self

    def click_ray(self, x:float, y:float) -> Tuple[Vector3,Vector3]:
        """Returns a pair of 3-tuples indicating the ray source and direction
        in world coordinates for a screen-coordinate point (x,y) with y indicating
        top-down pixel coordinates."""
        self.update_viewport()
        s = self.clickSource(x,self.h-y)
        d = self.clickDirection(x,self.h-y)
        return (s,d)

    def project(self, pt:Vector3, clip=True) -> Vector3:
        """Given a point in world space, returns the (x,y,z) coordinates of the projected
        pixel.  z is given in absolute coordinates, while x,y are given in pixel values.

        If clip=True and the point is out of the viewing volume, then None is returned.
        Otherwise, if the point is exactly at the focal plane then the middle of the viewport
        is returned.
        """
        self.update_viewport()
        pimg = Viewport.project(self,pt)
        if clip:
            if pimg[0] < 0 or pimg[0] >= self.w or pimg[1] < 0 or pimg[1] >= self.h:
                return None
        return pimg

    def set_current_GL(self) -> None:
        """Sets up the view in the current OpenGL context"""
        if not _HAS_OPENGL:
            raise RuntimeError("PyOpenGL is not installed, cannot use set_current_gl")
        n,f = self.n,self.f
        if self.controller.dist*1.05 > f:
            #allow super zoomed-out views to work without adjusting far plane
            f = self.controller.dist*1.05
        
        # Projection
        def getFrustumMatrix(l,b,r,t,n,f):
            m = [[0.0]*4 for i in range(4)]
            m[0][0]=2*n/(r-l)
            m[0][2]=(r+l)/(r-l)
            m[1][1]=2*n/(t-b)
            m[1][2]=(t+b)/(t-b)
            m[2][2]=-(f+n)/(f-n)
            m[2][3]=-2*f*n/(f-n)
            m[3][2]=-1
            return m
            
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        nbb = self.viewRectangle(self.n)
        proj = getFrustumMatrix(nbb[0],nbb[1],nbb[2],nbb[3],self.n,self.f)
        cols = list(zip(*proj))
        pack = sum((list(c) for c in cols),[])
        GL.glMultMatrixf(pack)
        
        # ModelView matrix
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        mat = se3.homogeneous(se3.inv(self.controller.matrix()))
        cols = list(zip(*mat))
        pack = sum((list(c) for c in cols),[])
        GL.glMultMatrixf(pack)

    def save_file(self,fn : str) -> None:
        """Saves to a viewport txt file. The file format is compatible with
        the RobotTest, RobotPose, and SimTest apps.
        """
        self.update_viewport()
        txt = self.toText()
        f = open(str(fn),'w')
        f.write(txt)
        f.write('\n')
        f.write("ORBITDIST %f\n"%(self.controller.dist,))
        f.close()

    def load_file(self,fn : str) -> None:
        """Loads from a viewport txt file. The file format is compatible with
        the RobotTest, RobotPose, and SimTest apps.
        """
        f = open(str(fn),'r')
        lines = []
        for line in f:
            entries = line.split()
            if len(entries) == 0:
                continue
            kw = entries[0]
            args = entries[1:]
            if kw == 'ORBITDIST':
                self.controller.dist = float(args[0])
            else:
                lines.append(line)
        f.close()
        if not self.fromText('\n'.join(lines)):
            raise IOError("Unable to load viewport from text file")
        #now set the controller parameters from the viewport matrix
        T = self.getPose()
        self.controller.set_matrix(T)

    def drawGL(self,draw_frustum=True,draw_coords=True) -> None:
        """Draws an OpenGL widget illustrating the viewport."""
        GL.glPushMatrix()

        mat = se3.homogeneous(self.get_transform())
        cols = list(zip(*mat))
        pack = sum((list(c) for c in cols),[])
        GL.glMultMatrixf(pack)
        n,f = self.n,self.f
        #note that +z is *backward* in the camera view
        if draw_frustum:
            xmin,ymin,xmax,ymax = self.viewRectangle(1.0)
            GL.glDisable(GL.GL_LIGHTING)
            GL.glColor3f(1,1,0)
            GL.glBegin(GL.GL_LINES)
            #near plane
            GL.glVertex3f(n*xmax,n*ymax,n)
            GL.glVertex3f(n*xmax,n*ymin,n)
            GL.glVertex3f(n*xmax,n*ymin,n)
            GL.glVertex3f(n*xmin,n*ymin,n)
            GL.glVertex3f(n*xmin,n*ymin,n)
            GL.glVertex3f(n*xmin,n*ymax,n)
            GL.glVertex3f(n*xmin,n*ymax,n)
            GL.glVertex3f(n*xmax,n*ymax,n)
            #far plane
            GL.glVertex3f(f*xmax,f*ymax,f)
            GL.glVertex3f(f*xmax,f*ymin,f)
            GL.glVertex3f(f*xmax,f*ymin,f)
            GL.glVertex3f(f*xmin,f*ymin,f)
            GL.glVertex3f(f*xmin,f*ymin,f)
            GL.glVertex3f(f*xmin,f*ymax,f)
            GL.glVertex3f(f*xmin,f*ymax,f)
            GL.glVertex3f(f*xmax,f*ymax,f)
            #connections
            GL.glVertex3f(n*xmax,n*ymax,n)
            GL.glVertex3f(f*xmax,f*ymax,f)
            GL.glVertex3f(n*xmax,n*ymin,n)
            GL.glVertex3f(f*xmax,f*ymin,f)
            GL.glVertex3f(n*xmin,n*ymin,n)
            GL.glVertex3f(f*xmin,f*ymin,f)
            GL.glVertex3f(n*xmin,n*ymax,n)
            GL.glVertex3f(f*xmin,f*ymax,f)
            GL.glEnd()

        if draw_coords:
            gldraw.xform_widget(se3.identity(),0.1,0.01)
            
        GL.glPopMatrix()
