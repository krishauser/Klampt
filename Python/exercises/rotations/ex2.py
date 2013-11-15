import math
from klampt import so3
from klampt import se3
from klampt import vectorops
from klampt.glprogram import *

def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    return vectorops.madd(a,vectorops.sub(b,a),u)

def interpolate_rotation(R1,R2,u):
    """Interpolate linearly between the two rotations R1 and R2.
    TODO: the current implementation doesn't work properly.  Why? """
    m1 = so3.moment(R1)
    m2 = so3.moment(R2)
    mu = interpolate_linear(m1,m2,u)
    angle = vectorops.norm(mu)
    axis = vectorops.div(mu,angle)
    return so3.rotation(axis,angle)

def interpolate_transform(T1,T2,u):
    """Interpolate linearly between the two transformations T1 and T2."""
    return (interpolate_rotation(T1[0],T2[0],u),interpolate_linear(T1[1],T2[1],u))

def draw_GL_frame(T,axis_length=0.1):
    """Draws the rigid transform T as a set of axis-oriented lines
    of length axis_length."""
    R,t = T
    if len(R)!=9 or len(t)!=3:
        print "Incorrect sizes",len(R),len(t)
        raise RuntimeError("")
    #draw transform widget
    glBegin(GL_LINES)
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(1,0,0)
    glVertex3fv(vectorops.madd(t,R[0:3],axis_length))
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(0,1,0)
    glVertex3fv(vectorops.madd(t,R[3:6],axis_length))
    glColor3f(1,1,1)
    glVertex3fv(t)
    glColor3f(0,0,1)
    glVertex3fv(vectorops.madd(t,R[6:9],axis_length))
    glEnd()
    glColor3f(0,0,0)

def draw_wire_box(w):
    glBegin(GL_LINES)
    glVertex3f(-w,-w,-w)
    glVertex3f(w,-w,-w)
    glVertex3f(-w,w,-w)
    glVertex3f(w,w,-w)
    glVertex3f(-w,-w,w)
    glVertex3f(w,-w,w)
    glVertex3f(-w,w,w)
    glVertex3f(w,w,w)
    glVertex3f(-w,-w,-w)
    glVertex3f(-w,w,-w)
    glVertex3f(w,-w,-w)
    glVertex3f(w,w,-w)
    glVertex3f(-w,-w,w)
    glVertex3f(-w,w,w)
    glVertex3f(w,-w,w)
    glVertex3f(w,w,w)
    glVertex3f(-w,-w,w)
    glVertex3f(-w,-w,-w)
    glVertex3f(w,-w,w)
    glVertex3f(w,-w,-w)
    glVertex3f(-w,w,w)
    glVertex3f(-w,w,-w)
    glVertex3f(w,w,w)
    glVertex3f(w,w,-w)
    glEnd()

class GLRotationTest(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self,"GLRotationTest")
        Ra = so3.rotation((0,1,0),math.pi*-0.9)
        Rb = so3.rotation((0,1,0),math.pi*0.9)
        print "Angle between"
        print so3.matrix(Ra)
        print "and"
        print so3.matrix(Rb)
        print "is",so3.distance(Ra,Rb)
        self.Ta = (Ra,(-1,0,1))
        self.Tb = (Rb,(1,0,1))
        self.T = self.Ta
        self.clearColor = [1,1,1,0]        

    def display(self):
        glDisable(GL_LIGHTING)
        #draw the world reference frame as a box
        glLineWidth(1.0)
        glColor3f(0,0,0)
        draw_wire_box(0.1)
        
        glLineWidth(1.0)
        draw_GL_frame(self.Ta,1.0)
        draw_GL_frame(self.Tb,1.0)
        glLineWidth(3.0)
        draw_GL_frame(self.T,1.0)

    def idle(self):
        #interpolate with a period of 3 seconds
        period = 3.0
        u = (self.ttotal%period)/period
        self.T = interpolate_transform(self.Ta,self.Tb,u)

        #uncomment for question 3 
        #print "Angle to a:",so3.distance(self.Ta[0],self.T[0]),", b:",so3.distance(self.Tb[0],self.T[0])


if __name__ == "__main__":
    GLRotationTest().run()

