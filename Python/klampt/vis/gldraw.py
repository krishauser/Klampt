"""OpenGL drawing functions for geometric primitives."""

import math
from ..math import vectorops
from ..math import se3
from ..math import spline
import ctypes
from . import glinit
from OpenGL import GL
from OpenGL import GLUT

def point(p):
    """Draws a point at position p (either a 2d or 3d list/tuple)"""
    GL.glBegin(GL.GL_POINTS)
    if len(p)==2:
        GL.glVertex2f(*p)
    elif len(p)==3:
        GL.glVertex3f(*p)
    else:
        GL.glVertex3f(p[0],p[1],p[2])
    GL.glEnd()

def circle(center,radius,res=0.01,filled=True):
    """Draws a 2D circle with the given center, radius, and angular
    resolution.  If filled=true, it is drawn filled, otherwise, it is
    drawn as a wireframe."""
    numdivs = int(math.ceil(radius*math.pi*2/res))
    GL.glNormal3f(0,0,1)
    if filled:
        GL.glBegin(GL.GL_TRIANGLE_FAN)
        GL.glVertex2f(*center)
    else:
        GL.glBegin(GL.GL_LINE_STRIP)
    for i in range(numdivs+1):
        u = float(i)/float(numdivs)*math.pi*2
        GL.glVertex2f(center[0]+radius*math.cos(u),center[1]+radius*math.sin(u))
    GL.glEnd()

def triangle(a,b,c,lighting=True,filled=True):
    """Draws a 3D triangle with points a,b,c.  If lighting is true, computes
    a GL normal vector dynamically. If filled=true, it is drawn filled,
    otherwise, it is drawn as a wireframe."""
    if lighting:
        n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
        try:
            n = vectorops.mul(n,1.0/vectorops.norm(n))
            GL.glNormal3f(*n)
        except ZeroDivisionError:
            pass
    if filled:
        GL.glBegin(GL.GL_TRIANGLES)
    else:
        GL.glBegin(GL.GL_LINE_LOOP)
    GL.glVertex3f(*a)
    GL.glVertex3f(*b)
    GL.glVertex3f(*c)
    GL.glEnd();


def quad(a,b,c,d,lighting=True,filled=True):
    """Draws a 3D quad with points a,b,c,d.  If lighting is true, computes
    a GL normal vector dynamically. If filled=true, it is drawn filled,
    otherwise, it is drawn as a wireframe."""
    if lighting:
        n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
        n = vectorops.mul(n,1.0/vectorops.norm(n))
        GL.glNormal3f(*n)
    if filled:
        GL.glBegin(GL.GL_TRIANGLE_FAN)
    else:
        GL.glBegin(GL.GL_LINE_LOOP)
    GL.glVertex3f(*a)
    GL.glVertex3f(*b)
    GL.glVertex3f(*c)
    GL.glVertex3f(*d)
    GL.glEnd();

def box(a=(0,0,0),b=(1,1,1),lighting=True,filled=True):
    """Draws a 3D axis-aligned bounding box with lower corner a and
    upper corner b.  If lighting is true, sends gl normal vectors.
    If filled = True, draws it solidly.  Otherwise, only the wireframe
    is drawn."""
    GL.glNormal3f(0,0,-1)
    quad((a[0], a[1], a[2]),
         (a[0], b[1], a[2]),
         (b[0], b[1], a[2]),
         (b[0], a[1], a[2]),False,filled)
    GL.glNormal3f(0,0,1)
    quad((a[0], a[1], b[2]),
         (b[0], a[1], b[2]),
         (b[0], b[1], b[2]),
         (a[0], b[1], b[2]),False,filled)
    GL.glNormal3f(-1,0,0)
    quad((a[0], a[1], a[2]),
         (a[0], a[1], b[2]),
         (a[0], b[1], b[2]),
         (a[0], b[1], a[2]),False,filled)
    GL.glNormal3f(1,0,0)
    quad((b[0], a[1], a[2]),
         (b[0], b[1], a[2]),
         (b[0], b[1], b[2]),
         (b[0], a[1], b[2]),False,filled)
    GL.glNormal3f(0,-1,0)
    quad((a[0], a[1], a[2]),
         (b[0], a[1], a[2]),
         (b[0], a[1], b[2]),
         (a[0], a[1], b[2]),False,filled)
    GL.glNormal3f(0,1,0)
    quad((a[0], b[1], a[2]),
         (a[0], b[1], b[2]),
         (b[0], b[1], b[2]),
         (b[0], b[1], a[2]),False,filled)

def centered_box(dims=(1,1,1),lighting=True,filled=True):
    """Draws a box centered around the origin with the given width x height x
    depth"""
    box([-d*0.5 for d in dims],[d*0.5 for d in dims],lighting,filled)

def setcolor(r,g,b,a=1.0,lighting=True):
    """Sets the current color/material, depending on the lighting flag"""
    if lighting:
        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_AMBIENT_AND_DIFFUSE,[r,g,b,a])
    else:
        GL.glColor4f(r,g,b,a)

def xform_widget(T,length,width,lighting=True,fancy=False):
    """Draws an axis-aligned transform widget for the se3 transform T.
    Length / width govern the length / width of the axes.  If fancy=True,
    draws the axes with real volume rather than lines"""
    mat = list(zip(*se3.homogeneous(T)))
    mat = sum([list(coli) for coli in mat],[])

    GL.glPushMatrix()
    GL.glMultMatrixf(mat)

    if fancy:
        #center
        setcolor(1,1,1,1,lighting=lighting)
        box((-width*0.75,-width*0.75,-width*0.75),(width*0.75,width*0.75,width*0.75),lighting=lighting)
        
        #x axis
        setcolor(1,0,0,1,lighting=lighting)
        box((-width*0.5,-width*0.5,-width*0.5),(length,width*0.5,width*0.5),lighting=lighting)
    
        #y axis
        setcolor(0,1,0,1,lighting=lighting)
        box((-width*0.5,-width*0.5,-width*0.5),(width*0.5,length,width*0.5),lighting=lighting)
    
        #z axis
        setcolor(0,0,1,1,lighting=lighting)
        box((-width*0.5,-width*0.5,-width*0.5),(width*0.5,width*0.5,length),lighting=lighting)
    else:
        GL.glDisable(GL.GL_LIGHTING)
        GL.glBegin(GL.GL_LINES)
        GL.glColor4f(1,1,1,1)
        GL.glVertex3f(0,0,0)
        GL.glColor4f(1,0,0,1)
        GL.glVertex3f(length,0,0)
        GL.glColor4f(1,1,1,1)
        GL.glVertex3f(0,0,0)
        GL.glColor4f(0,1,0,1)
        GL.glVertex3f(0,length,0)
        GL.glColor4f(1,1,1,1)
        GL.glVertex3f(0,0,0)
        GL.glColor4f(0,0,1,1)
        GL.glVertex3f(0,0,length)
        GL.glEnd()

    GL.glPopMatrix()

def hermite_curve(x1,v1,x2,v2,res=0.01,textured=False):
    """Draws a 3D Hermite curve with control points x1,v1,x2,v2 and resolution
    res.  If textured=True, generate texture coordinates for each point
    (useful for applying patterns)."""
    bezier_curve(*spline.hermite_to_bezier(x1,v1,x2,v2),res=res,textured=textured)

def bezier_curve(x1,x2,x3,x4,res=0.01,textured=False):
    """Draws a 3D Bezier curve with control points x1,x2,x3,x4 and resolution
    res.  If textured=True, generate texture coordinates for each point
    (useful for applying patterns)."""
    if textured:
        path,params = spline.bezier_discretize(x1,x2,x3,x4,res,return_params=True)
        if len(path)<=1: return
        GL.glBegin(GL.GL_LINE_STRIP)
        for p,u in zip(path,params):
            GL.glTexCoord1f(u)
            GL.glVertex3f(*p)
        GL.glEnd()
    else:
        path = spline.bezier_discretize(x1,x2,x3,x4,res)
        if len(path)<=1: return
        GL.glBegin(GL.GL_LINE_STRIP)
        for p in path:
            GL.glVertex3f(*p)
        GL.glEnd()

def glutBitmapString(font,string):
    """Renders a string using GLUT characters"""
    for c in string:
        GLUT.glutBitmapCharacter(font,ctypes.c_int(ord(c)))
