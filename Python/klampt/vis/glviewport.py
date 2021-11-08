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

class GLViewport:
    """
    A class describing an OpenGL camera view.

    Attributes:
        x,y (int): upper left hand corner of the view in the OpenGL canvas, in
            screen pixels.
        w,h (int): width and height of the view, in screen pixels.
        screenDeviceScale (float): if not 1, multiply screen pixel coordinates
            by this to get OpenGL pixel coordinates (usually Mac Retina
            displays)
        orthogonal (bool): if true, does an orthogonal projection. (Not
            supported yet.)
        camera: an orbit camera controlling the target, zoom, and orientation
            of the "trackball" (see :class:`orbit`)
        fov (float): the camera field of view in x direction, in degrees
        clippingplanes (pair of floats): a pair containing the near and far
            clipping planes

    """
    def __init__(self):
        self.orthogonal = False
        self.x,self.y = 0,0
        self.w,self.h = 640,480
        self.screenDeviceScale = 1
        self.camera = camera.orbit()
        self.camera.dist = 6.0
        #x field of view in degrees
        self.fov = 30.0
        #near and far clipping planes
        self.clippingplanes = (0.2,100)

    def contains(self,x,y):
        return x >= self.x and y >= self.y and x < self.x + self.w and y < self.y + self.h

    def set_transform(self,T,convention='standard'):
        """Sets the pose of the camera, with T given in world coordinates.

        If convention = 'openGL', the Z axis of T is the *backward* direction of
        the camera, with Y pointing *up* and X pointing to the *right*.

        If convention = 'standard', the Z axis of T is the *forward* direction of
        the camera, with Y pointing *down* and X pointing to the *right*
        """
        if convention == 'openGL':
            self.camera.set_matrix(T)
        else:
            xzflip = [1,0,0,  0,-1,0,  0,0,-1]
            self.camera.set_matrix((so3.mul(T[0],xzflip),T[1]))

    def get_transform(self,convention='standard'):
        """Gets the pose of the camera, with T given in world coordinates.

        If convention = 'openGL', the Z axis of T is the *backward* direction of
        the camera, with Y pointing *up* and X pointing to the *right*.

        If convention = 'standard', the Z axis of T is the *forward* direction of
        the camera, with Y pointing *down* and X pointing to the *right*
        """
        if convention == 'openGL':
            return self.camera.matrix()
        else:
            T = self.camera.matrix()
            xzflip = [1,0,0,  0,-1,0,  0,0,-1]
            return (so3.mul(T[0],xzflip),T[1])

    def fit(self,center,radius):
        """Fits the viewport to an object filling a sphere of a certain center
        and radius"""
        self.camera.tgt = center
        self.camera.dist = radius*2
        zmin,zmax = self.clippingplanes
        if radius < self.clippingplanes[0]:
            zmin = radius*0.5
        if radius*3 > self.clippingplanes[1]:
            zmax =radius*3.5
        self.clippingplanes = (zmin,zmax)

    def to_viewport(self):
        """Returns a Klampt C++ Viewport() instance corresponding to this view.
        This is used to interface with the Widget classes"""
        vp = Viewport()
        vp.x,vp.y,vp.w,vp.h = self.x,self.y,self.w,self.h
        vp.n,vp.f = self.clippingplanes
        vp.perspective = True
        aspect = float(self.w)/float(self.h)
        rfov = math.radians(self.fov)
        vp.scale = 1.0/(2.0*math.tan(rfov*0.5/aspect)*aspect)
        vp.setRigidTransform(*self.camera.matrix())
        return vp

    def click_ray(self,x,y):
        """Returns a pair of 3-tuples indicating the ray source and direction
        in world coordinates for a screen-coordinate point (x,y)"""
        R,t = self.camera.matrix()
        #from x and y compute ray direction
        u = float(x-(self.x + self.w/2))/self.w
        v = float((self.y + self.h/2) -y)/self.w
        aspect = float(self.w)/float(self.h)
        rfov = math.radians(self.fov)
        scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
        d = (u*scale,v*scale,-1.0)
        d = vectorops.div(d,vectorops.norm(d))
        return (t,so3.apply(R,d))

    def project(self,pt,clip=True):
        """Given a point in world space, returns the (x,y,z) coordinates of the projected
        pixel.  z is given in absolute coordinates, while x,y are given in pixel values.

        If clip=True and the point is out of the viewing volume, then None is returned.
        Otherwise, if the point is exactly at the focal plane then the middle of the viewport
        is returned.
        """
        ploc = se3.apply(se3.inv(self.camera.matrix()),pt)
        if clip:
            if -ploc[2] <= self.clippingplanes[0] or -ploc[2] >= self.clippingplanes[1]:
                return None
        if abs(ploc[2]) < 1e-8:
            return (self.x+self.w/2,self.y+self.h/2,-ploc[2])
        #d = (u*scale,v*scale,-1.0)
        #ploc.x = ploc.z*d.x
        #ploc.y = ploc.z*d.y
        aspect = float(self.w)/float(self.h)
        rfov = math.radians(self.fov)
        scale = 2.0*math.tan(rfov*0.5/aspect)*aspect
        u = -ploc[0]/(ploc[2]*scale)
        v = -ploc[1]/(ploc[2]*scale)
        if clip and (abs(u) > 0.5 or abs(v) > 0.5):
            return None
        x = u*self.w + (self.x + self.w/2)
        y = (self.y + self.h/2) - v*self.w
        return (x,y,-ploc[2])

    def set_current_GL(self):
        """Sets up the view in the current OpenGL context"""
        if not _HAS_OPENGL:
            raise RuntimeError("PyOpenGL is not installed, cannot use set_current_gl")
        # Projection
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        aspect = float(self.w)/float(self.h)
        n,f = self.clippingplanes
        if self.camera.dist*1.05 > f:
            #allow super zoomed-out views to work without adjusting far plane
            f = self.camera.dist*1.05
        fovy = math.degrees(math.atan(math.tan(math.radians(self.fov*0.5))/aspect))*2
        GLU.gluPerspective (fovy,aspect,n,f)

        # Initialize ModelView matrix
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()
        
        # View transformation
        mat = se3.homogeneous(se3.inv(self.camera.matrix()))
        cols = list(zip(*mat))
        pack = sum((list(c) for c in cols),[])
        GL.glMultMatrixf(pack)

    def save_file(self,fn):
        """Saves to a viewport txt file. The file format is compatible with
        the RobotTest, RobotPose, and SimTest apps.
        """
        f = open(str(fn),'w')
        f.write("VIEWPORT\n")
        f.write("FRAME %d %d %d %d\n"%(self.x,self.y,self.w,self.h))
        f.write("PERSPECTIVE 1\n")
        aspect = float(self.w)/float(self.h)
        rfov = self.fov*math.pi/180.0
        scale = 1.0/(2.0*math.tan(rfov*0.5/aspect)*aspect)
        f.write("SCALE %f\n"%(scale,))
        f.write("NEARPLANE %f\n"%(self.clippingplanes[0],))
        f.write("FARPLANE %f\n"%(self.clippingplanes[1],))
        f.write("CAMTRANSFORM ")
        mat = se3.homogeneous(self.camera.matrix())
        f.write(' '.join(str(v) for v in sum(mat,[])))
        f.write('\n')
        f.write("ORBITDIST %f\n"%(self.camera.dist,))
        f.close()

    def load_file(self,fn):
        """Loads from a viewport txt file. The file format is compatible with
        the RobotTest, RobotPose, and SimTest apps.
        """
        f = open(str(fn),'r')
        read_viewport = False
        mat = None
        for line in f:
            entries = line.split()
            if len(entries) == 0:
                continue
            kw = entries[0]
            args = entries[1:]
            if kw == 'VIEWPORT':
                read_viewport = True
                continue
            else:
                if not read_viewport:
                    warnings.warn("File does not appear to be a valid viewport file, must start with VIEWPORT")
                    break
            if kw == 'FRAME':
                self.x,self.y,self.w,self.h = [int(x) for x in args]
            elif kw == 'PERSPECTIVE':
                if args[0] != '1':
                    warnings.warn("CANNOT CHANGE TO ORTHO MODE IN PYTHON VISUALIZATION")
            elif kw == 'SCALE':
                scale = float(args[0])
                aspect = float(self.w)/float(self.h)
                #2.0*math.tan(rfov*0.5/aspect)*aspect = 1.0/scale
                #math.tan(rfov*0.5/aspect) = 0.5/(scale*aspect)
                #rfov*0.5/aspect = math.atan(0.5/(scale*aspect))
                #rfov = 2*aspect*math.atan(0.5/(scale*aspect))
                rfov = math.atan(0.5/(scale*aspect))*2*aspect
                self.fov = math.degrees(rfov)
            elif kw == 'NEARPLANE':
                self.clippingplanes = (float(args[0]),self.clippingplanes[1])
            elif kw == 'FARPLANE':
                self.clippingplanes = (self.clippingplanes[0],float(args[0]))
            elif kw == 'CAMTRANSFORM':
                mat = [args[0:4],args[4:8],args[8:12],args[12:16]]
                for i,row in enumerate(mat):
                    mat[i] = [float(x) for x in row]
            elif kw == 'ORBITDIST':
                self.camera.dist = float(args[0])
            else:
                raise RuntimeError("Invalid viewport keyword "+kw)
        if mat is not None:
            self.camera.set_matrix(se3.from_homogeneous(mat))

        f.close()

    def drawGL(self,draw_frustum=True,draw_coords=True):
        """Draws an OpenGL widget illustrating the viewport."""
        GL.glPushMatrix()

        mat = se3.homogeneous(self.get_transform())
        cols = list(zip(*mat))
        pack = sum((list(c) for c in cols),[])
        GL.glMultMatrixf(pack)
        n,f = self.clippingplanes
        aspect = float(self.w)/float(self.h)
        rfov = math.radians(self.fov)
        scale = math.tan(rfov*0.5/aspect)*aspect
        #note that +z is *backward* in the camera view
        if draw_frustum:
            xmin = (self.x - self.w*0.5)/((self.w)*0.5)
            xmax = (self.x + self.w*0.5)/((self.w)*0.5)
            ymax = -(self.y - self.h*0.5)/((self.h)*0.5)
            ymin = -(self.y + self.h*0.5)/((self.h)*0.5)
            xscale = scale
            yscale = xscale/aspect
            xmin *= xscale
            xmax *= xscale
            ymin *= yscale
            ymax *= yscale
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
