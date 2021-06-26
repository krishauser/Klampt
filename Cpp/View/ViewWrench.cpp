#include "ViewWrench.h"      
#include <KrisLibrary/GLdraw/drawextra.h>
using namespace GLDraw;
using namespace Math3D;
using namespace Klampt;

ViewWrench::ViewWrench()
:fscale(1.0),mscale(1.0)
{
    forceColor.set(1,0.5,0);
    momentColor.set(0,1,1);
    centerColor.set(1,1,0);
}

void ViewWrench::DrawGL(const Vector3& center,const Vector3& _f,const Vector3& _m) const
{
    Vector3 f = _f*fscale;
    Vector3 m = _m*fscale;

    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslate(center);
      
    Real r=0.01;
    Real arrowLen = 0.1,arrowWidth=1.7;
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,forceColor);
    Real len = 0.5*Exp(-f.length()*2.0);
    drawCylinder(f*(1.0-len),r);
    glPushMatrix();
    glTranslate(f*(1.0-len));
    drawCone(f*len,r*arrowWidth,8);
    glPopMatrix();
      
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,momentColor);
    len = 0.5*Exp(-m.length()*2.0);
    drawCylinder(m*(1.0-len),r);
    glPushMatrix();
    glTranslate(m*(1.0-len));
    drawCone(m*len,r*arrowWidth,8);
    glPopMatrix();
      
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,centerColor);
    drawSphere(0.015,16,8);
      
    glPopMatrix();
}