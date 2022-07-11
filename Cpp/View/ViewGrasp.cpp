#include "ViewGrasp.h"
#include "ViewIK.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GL.h>
using namespace GLDraw;
using namespace Klampt;

ViewGrasp::ViewGrasp()
  :viewRobot(NULL)
{
  forceColor.set(1,0.5,0);
}


void ViewGrasp::Draw(const Grasp& g)
{
  for(size_t i=0;i<g.contacts.size();i++)
    viewContact.Draw(g.contacts[i]);

  if(viewRobot) {
    Config oldq = viewRobot->robot->q;
    ViewIKGoal viewIK;
    for(size_t i=0;i<g.constraints.size();i++) 
      viewIK.DrawLink(g.constraints[i],*viewRobot);
    //draw fixed dofs as well
    for(size_t i=0;i<g.constraints.size();i++) {
      RigidTransform T;
      g.constraints[i].GetFixedGoalTransform(T);
      viewRobot->robot->links[g.constraints[i].link].T_World = T;
    }
    for(size_t i=0;i<g.fixedDofs.size();i++) {
      viewRobot->robot->q[g.fixedDofs[i]] = g.fixedValues[i];
      viewRobot->robot->UpdateUpstreamFrames(g.fixedDofs[i],g.fixedDofs[i]);
    }
    for(size_t i=0;i<g.fixedDofs.size();i++) 
      viewRobot->DrawLink_World(g.fixedDofs[i]);
    viewRobot->robot->UpdateConfig(oldq);
  }

  if(!g.forces.empty()) {
    glDisable(GL_LIGHTING);
    forceColor.setCurrentGL();
    glBegin(GL_LINES);
    int m=0;
    Real scale = 1.0;
    for(size_t i=0;i<g.forces.size();i++) {
      const Vector3& p=g.contacts[i].x;
      glVertex3v(p);
      glVertex3v(p+g.forces[i]*scale);
    }
    glEnd();
  }
}
