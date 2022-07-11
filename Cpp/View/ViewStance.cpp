#include "ViewStance.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
using namespace GLDraw;
using namespace Klampt;

ViewPolytope::ViewPolytope()
  :poly(NULL),wireframe(false),h(0)
{
  color.set(0.5,0.5,0.5);
}

void ViewPolytope::Draw()
{
  glDisable(GL_LIGHTING);
  color.setCurrentGL();
  if(wireframe) glBegin(GL_LINE_LOOP);
  else  glBegin(GL_TRIANGLE_FAN);
  for(size_t i=0;i<poly->vertices.size();i++) {
    const Geometry::PointRay2D& pt=poly->vertices[i];
    if(pt.isRay)
      glVertex3f(pt.x*1000,pt.y*1000,h);
    else
      glVertex3f(pt.x,pt.y,h);
  }
  glEnd();
}





ViewStance::ViewStance()
  :s(NULL)
{
  forceColor.set(1,0.5,0);
}

void ViewStance::DrawHolds()
{
  for(Stance::const_iterator i=s->begin();i!=s->end();i++) 
    viewHold.Draw(i->second);
}

void ViewStance::DrawForces(const Vector& f,Real scale)
{
  glDisable(GL_LIGHTING);
  forceColor.setCurrentGL();
  glBegin(GL_LINES);
  Assert(f.n == NumContactPoints(*s)*3);
  int m=0;
  for(Stance::const_iterator i=s->begin();i!=s->end();i++) {
    const Hold& h=i->second;
    for(size_t j=0;j<h.contacts.size();j++,m+=3) {
      const Vector3& p=h.contacts[j].x;
      Vector3 fi(f(m),f(m+1),f(m+2));
      glVertex3v(p);
      glVertex3v(p+fi*scale);
    }
  }
  glEnd();
}
