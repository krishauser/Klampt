"""OpenGL drawing functions for geometric primitives."""

import math
import vectorops
from OpenGL.GL import *

def point(p):
    glBegin(GL_POINTS)
    if len(p)==2:
        glVertex2f(*p)
    elif len(p)==3:
        glVertex3f(*p)
    else:
        glVertex3f(p[0],p[1],p[2])
    glEnd()

def circle(center,radius,res=0.01):
    numdivs = int(math.ceil(self.radius*math.pi*2/res))
    glNormal3f(0,0,1)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(*center)
    for i in xrange(numdivs+1):
        u = float(i)/float(numdivs)*math.pi*2
        glVertex2f(center[0]+radius*math.cos(u),center[1]+radius*math.sin(u))
    glEnd()

def triangle(a,b,c,lighting=True):
    if lighting:
        n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
        n = vectorops.mul(n,1.0/vectorops.norm(n))
        glNormal3f(*n)
    glBegin(GL_TRIANGLES)
    glVertex3f(*a)
    glVertex3f(*b)
    glVertex3f(*c)
    glEnd();


def quad(a,b,c,d,lighting=True):
    if lighting:
        n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
        n = vectorops.mul(n,1.0/vectorops.norm(n))
        glNormal3f(*n)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(*a)
    glVertex3f(*b)
    glVertex3f(*c)
    glVertex3f(*d)
    glEnd();

def box(a=(0,0,0),b=(1,1,1),lighting=True):
    quad((a[0], a[1], a[2]),
         (a[0], b[1], a[2]),
         (b[0], b[1], a[2]),
         (b[0], a[1], a[2]),lighting)
    quad((a[0], a[1], b[2]),
         (b[0], a[1], b[2]),
         (b[0], b[1], b[2]),
         (a[0], b[1], b[2]),lighting)
    quad((a[0], a[1], a[2]),
         (a[0], a[1], b[2]),
         (a[0], b[1], b[2]),
         (a[0], b[1], a[2]),lighting)
    quad((b[0], a[1], a[2]),
         (b[0], b[1], a[2]),
         (b[0], b[1], b[2]),
         (b[0], a[1], b[2]),lighting)
    quad((a[0], a[1], a[2]),
         (b[0], a[1], a[2]),
         (b[0], a[1], b[2]),
         (a[0], a[1], b[2]),lighting)
    quad((a[0], b[1], a[2]),
         (a[0], b[1], b[2]),
         (b[0], b[1], b[2]),
         (b[0], b[1], a[2]),lighting)

def centered_box(dims=(1,1,1),lighting=True):
    box([-d*0.5 for d in dims],[d*0.5 for d in dims],lighting)
