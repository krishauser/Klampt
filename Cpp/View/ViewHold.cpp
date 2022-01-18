#include "ViewHold.h"
#include "ViewIK.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
using namespace GLDraw;
using namespace Klampt;

ViewContact::ViewContact()
{
  normalScale = 0.1;
  pointColor.set(0,0,1);
  normalColor.set(1,1,0);
  coneColor.set(1,0.5,0,0.5);
}

void ViewContact::Draw()
{
  glDisable(GL_LIGHTING);
  glPointSize(3);
  pointColor.setCurrentGL();
  glBegin(GL_POINTS);
  glVertex3v(p->x);
  glEnd();

  normalColor.setCurrentGL();
  glBegin(GL_LINES);
  glVertex3v(p->x);
  glVertex3v(p->x + p->n*normalScale);
  glEnd();

  if(coneColor.rgba[3] != 0) {
    if(coneColor.rgba[3] != 1) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }
    glEnable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,coneColor);
    GLColor coneColor2=coneColor;
    coneColor2.rgba[2]=1-coneColor.rgba[2];
    glMaterialfv(GL_BACK,GL_AMBIENT_AND_DIFFUSE,coneColor2);
    coneColor.setCurrentGL();
    glPushMatrix();
      glTranslate(p->x);
      //drawWireConeFlipped(p->n*normalScale,p->kFriction*normalScale,8);
      drawConeFlipped(p->n*normalScale,p->kFriction*normalScale,8);
    glPopMatrix();
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,0);
    glEnable(GL_CULL_FACE);
    if(coneColor.rgba[3] != 1) {
      glDisable(GL_BLEND);
    }
  }
}

ViewHold::ViewHold()
  :drawContacts(true)
{
  outlineColor.set(1,1,0);
}

void ViewHold::Draw()
{
  if(drawContacts) DrawContacts();
  if(outlineColor.rgba[3] != 0) DrawOutline();
}

void ViewHold::DrawContacts()
{
  for(size_t i=0;i<h->contacts.size();i++)
    viewContact.Draw(h->contacts[i]);
}

void ViewHold::DrawOutline()
{
  glDisable(GL_LIGHTING);
  outlineColor.setCurrentGL();
  if(h->contacts.size()==1) {}
  else if(h->contacts.size()==2) {
    glBegin(GL_LINES);
    glVertex3v(h->contacts[0].x);
    glVertex3v(h->contacts[1].x);
    glEnd();
  }
  else {
    glBegin(GL_LINE_LOOP);
    for(size_t i=0;i<h->contacts.size();i++) {
      glVertex3v(h->contacts[i].x);
    }
    glEnd();
  }
}

void ViewHold::DrawLabel(const char* label)
{
  //average the hold positions
  Vector3 pos(Zero);
  for(size_t j=0;j<h->contacts.size();j++)
    pos+=h->contacts[j].x;
  pos /= h->contacts.size();
  
#if HAVE_GLUT
  glRasterPos3f(pos.x,pos.y,pos.z);
  glutBitmapString(GLUT_BITMAP_HELVETICA_18,label);
#endif //HAVE_GLUT
}

void ViewHold::DrawConstraint(const Hold& h,ViewRobot& robotviewer)
{
  Assert(robotviewer.robot != NULL);
  DrawConstraint(h,robotviewer,robotviewer.robot->links[h.link].T_World.R);
}

void ViewHold::DrawConstraint(const Hold& h,ViewRobot& robotviewer,const Matrix3& refMatrix)
{
  Assert(robotviewer.robot != NULL);
  ViewIKGoal viewik;
  viewik.Draw(h.ikConstraint,*robotviewer.robot);
  viewik.DrawLink(h.ikConstraint,robotviewer,refMatrix);
}
