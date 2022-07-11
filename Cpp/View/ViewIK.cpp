#include "ViewIK.h"
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/GLdraw/drawextra.h>
using namespace GLDraw;
using namespace Klampt;

ViewIKGoal::ViewIKGoal()
{
  lineColor.set(1,0,0);
  linkColor.set(0.5,0.5,0.5,0.5);
  widgetSize = 0.1;
}

//draws lines to the desired location
void ViewIKGoal::Draw(const IKGoal& goal,RobotModel& robot)
{
  Vector3 wp1 = robot.links[goal.link].T_World*goal.localPosition;
  Vector3 wp2 = goal.endPosition;
  Matrix3 Rref; Rref.setIdentity();
  if(goal.destLink >= 0) {
    wp2 = robot.links[goal.destLink].T_World*wp2;
    Rref = robot.links[goal.destLink].T_World.R;
  }
  if(goal.posConstraint == IKGoal::PosFixed) {
    glDisable(GL_LIGHTING);
    lineColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex3v(wp1);
    glVertex3v(wp2);
    glEnd();
  }
  else {
    //TODO: other constraints
  }
  if(goal.rotConstraint == IKGoal::RotFixed) {
    MomentRotation m(goal.endRotation);
    Matrix3 R;
    m.getMatrix(R);
    RigidTransform T;
    T.R = Rref*R;
    T.t = wp2;
    glPushMatrix();
    glMultMatrix(Matrix4(T));
    drawCoords(widgetSize);
    glPopMatrix();
  }
  else if(goal.rotConstraint == IKGoal::RotAxis) {
    lineColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex3v(wp2);
    glVertex3v(wp2 + widgetSize*(Rref*goal.endRotation));
    glVertex3v(wp1);
    glVertex3v(wp1 + widgetSize*(robot.links[goal.link].T_World.R*goal.localAxis));
    glEnd();
  }
}

void ViewIKGoal::DrawLink(const IKGoal& goal,ViewRobot& robotviewer)
{
  Matrix3 m; m.setIdentity();
  DrawLink(goal,robotviewer,m);
}

void ViewIKGoal::DrawLink(const IKGoal& goal,ViewRobot& robotviewer,const Matrix3& refMatrix)
{
  Assert(robotviewer.robot != NULL);
  RigidTransform Tref;
  Tref.R = refMatrix;
  Tref.t.setZero();
  RigidTransform T;
  goal.GetClosestGoalTransform(Tref,T);
  if(goal.destLink >= 0) {
    T = robotviewer.robot->links[goal.destLink].T_World*T;
  }

  glEnable(GL_LIGHTING);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,linkColor.rgba);
  glPushMatrix();
  glMultMatrix(Matrix4(T));
  robotviewer.DrawLink_Local(goal.link);
  glPopMatrix();
  glDisable(GL_BLEND);
}
