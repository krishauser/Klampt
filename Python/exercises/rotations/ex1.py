import math
from klampt import so3
from klampt import se3
from klampt import vectorops
from klampt.glprogram import *

def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    return vectorops.madd(a,vectorops.sub(b,a),u)

def interpolate_euler_angles(ea,eb,u,convention='zyx'):
    """Interpolates between the two euler angles.
    TODO: The default implementation interpolates linearly.  Can you
    do better?
    """
    return interpolate_linear(ea,eb,u)

def euler_angle_to_rotation(ea,convention='zyx'):
    """Converts an euler angle representation to a rotation matrix.
    Can use arbitrary axes specified by the convention
    arguments (default is 'zyx', or roll-pitch-yaw euler angles).  Any
    3-letter combination of 'x', 'y', and 'z' are accepted.
    """
    axis_names_to_vectors = dict([('x',(1,0,0)),('y',(0,1,0)),('z',(0,0,1))])
    axis0,axis1,axis2=convention
    R0 = so3.rotation(axis_names_to_vectors[axis0],ea[0])
    R1 = so3.rotation(axis_names_to_vectors[axis1],ea[1])
    R2 = so3.rotation(axis_names_to_vectors[axis2],ea[2])
    return so3.mul(R0,so3.mul(R1,R2))

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

class GLEulerRotationTest(GLRealtimeProgram):
    def __init__(self):
        GLRealtimeProgram.__init__(self,"GLRotationTest")
        #TODO: play around with these euler angles
        self.ea = (math.pi/4,0,0)
        self.eb = (math.pi*7/4,0,0)

        self.Ta = (euler_angle_to_rotation(self.ea),(-1,0,1))
        self.Tb = (euler_angle_to_rotation(self.eb),(1,0,1))
        self.T = self.update_interpolation(0)
        self.clearColor = [1,1,1,0]        

    def update_interpolation(self,u):
        #linear interpolation with euler angles
        e = interpolate_euler_angles(self.ea,self.eb,u)
        t = interpolate_linear(self.Ta[1],self.Tb[1],u)
        return (euler_angle_to_rotation(e),t)

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
        self.T = self.update_interpolation(u)

if __name__ == "__main__":
    GLEulerRotationTest().run()

