#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "RobotPoseWidget.h"
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include "Planning/RobotCSpace.h"
#include <map>
using namespace GLDraw;

Real RobustSolveIK(Robot& robot,RobotIKFunction& f,int iters,Real tol,int numRestarts)
{
  RobotIKSolver solver(f);
  solver.UseBiasConfiguration(robot.q);
  solver.UseJointLimits(TwoPi);
  bool res = solver.Solve(tol,iters);
  if(!res && numRestarts) {
    //attempt to do random restarts
    Config qbest = robot.q;
    Vector residual(f.NumDimensions());
    f(solver.solver.x,residual);
    Real residNorm = residual.normSquared();
    for(int i=0;i<numRestarts;i++) {
      //random restarts
      Config qorig = robot.q;
      RobotCSpace space(robot);
      space.Sample(robot.q);
      swap(robot.q,qorig);
      for(size_t i=0;i<f.activeDofs.mapping.size();i++)
        robot.q(f.activeDofs.mapping[i]) = qorig(f.activeDofs.mapping[i]);
      if(solver.Solve(tol,iters)) {
        qbest = robot.q;
        return 0;
      }
      f(solver.solver.x,residual);
      Real newResidNorm = residual.normSquared();
      if(newResidNorm < residNorm) {
        residNorm = newResidNorm;
        qbest = robot.q;
      }
    }
    robot.UpdateConfig(qbest);
    return residNorm;
  }
  return 0;
}

bool IsFloatingBase(Robot& robot) 
{
  if(robot.joints[0].type == RobotJoint::Floating) return true;
  else if(robot.joints[0].type == RobotJoint::FloatingPlanar) return true;
  //detect poorly set up floating base
  if(robot.links.size() < 6) return false;
  if(robot.links[0].type == RobotLink3D::Prismatic && 
    robot.links[1].type == RobotLink3D::Prismatic &&
    robot.links[2].type == RobotLink3D::Prismatic &&
    robot.links[3].type == RobotLink3D::Revolute && 
    robot.links[4].type == RobotLink3D::Revolute &&
    robot.links[5].type == RobotLink3D::Revolute) {
    Vector3 x,y,z,rz,ry,rx;
    x = robot.links[0].w;
    y = robot.links[1].w;
    z = robot.links[2].w;
    rz = robot.links[3].w;
    ry = robot.links[4].w;
    rx = robot.links[5].w;
    if(dot(x,y) == 0 && dot(x,z) == 0 && dot(y,z) == 0 && 
      dot(rx,ry) == 0 && dot(rx,rz) == 0 && dot(ry,rz) == 0) {
      //TODO: other conventions besides roll-pitch-yaw?
      if(x.x == 1 && y.y == 1 && z.z == 1 && rz.z == 1 && ry.y == 1 && rx.x == 1)
        return true;
    }
  }
  return false;
}

void SetFloatingBase(Robot& robot,const RigidTransform& T)
{
  T.t.get(robot.q(0),robot.q(1),robot.q(2));
  EulerAngleRotation e;
  e.setMatrixZYX(T.R);
  e.get(robot.q(3),robot.q(4),robot.q(5));
}

RigidTransform GetFloatingBase(const Robot& robot)
{
  if(robot.joints[0].type == RobotJoint::Floating || robot.joints[0].type == RobotJoint::FloatingPlanar) {
    return robot.links[robot.joints[0].linkIndex].T_World;
  }
  else {
    return robot.links[5].T_World;
    /*
    EulerAngleRotation e;
    e.set(robot.q(3),robot.q(4),robot.q(5));
    RigidTransform T;
    e.getMatrixZYX(T.R);
    T.t.set(robot.q(0),robot.q(1),robot.q(2));
    return T;
    */
  }
}

RobotLinkPoseWidget::RobotLinkPoseWidget()
  :robot(NULL),viewRobot(NULL),highlightColor(1,1,0,1),hoverLink(-1),draw(true)
{}

RobotLinkPoseWidget::RobotLinkPoseWidget(Robot* _robot,ViewRobot* _viewRobot)
  :robot(_robot),viewRobot(_viewRobot),poseConfig(_robot->q),highlightColor(1,1,0,1),hoverLink(-1),affectedLink(-1),affectedDriver(-1),draw(true)
{}

void RobotLinkPoseWidget::SetActiveDofs(const vector<int>& _activeDofs)
{
  activeDofs = _activeDofs;
}

void RobotLinkPoseWidget::Set(Robot* _robot,ViewRobot* _viewRobot)
{
  robot = _robot;
  viewRobot = _viewRobot;
  poseConfig = _robot->q;
}

bool RobotLinkPoseWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance) 
{ 
  Ray3D r;
  viewport.getClickSource(x,y,r.source);
  viewport.getClickVector(x,y,r.direction);
  int oldHoverLink = hoverLink;
  distance = Inf;    
  hoverLink = affectedLink = affectedDriver = -1;
  highlightedLinks.resize(0);
  Vector3 worldpt;
  Config oldConfig = robot->q;
  robot->UpdateConfig(poseConfig);
  robot->UpdateGeometry();
  vector<int> dofs;
  if(activeDofs.empty()) 
    for(size_t i=0;i<robot->links.size();i++) dofs.push_back((int)i);
  else
    dofs = activeDofs;
  for(size_t j=0;j<dofs.size();j++) {
    int i=dofs[j];
    if(robot->IsGeometryEmpty(i)) continue;
    Real dist;
    if(robot->geometry[i]->RayCast(r,&dist)) {
      if(dist < distance) {
	distance = dist;
	Vector3 worldpt = r.source + dist*r.direction;
	robot->links[i].T_World.mulInverse(worldpt,hoverPt);
	hoverLink = i;
      }
    }
  }
  robot->UpdateConfig(oldConfig);
  robot->UpdateGeometry();
  if(hoverLink != -1) {
    //if it's a weld joint, select up the tree to the first movable link
    map<int,int> linkToJoint;
    for(size_t i=0;i<robot->joints.size();i++) {
      vector<int> inds;
      robot->GetJointIndices(i,inds);
      for(size_t j=0;j<inds.size();j++)
	linkToJoint[inds[j]]=(int)i;
    }
    int link = hoverLink;
    while(robot->joints[linkToJoint[link]].type == RobotJoint::Weld) {
      highlightedLinks.push_back(link);
      if(robot->parents[link] < 0) break;
      link = robot->parents[link];
    } 
    highlightedLinks.push_back(link);
    //get the selected driver
    affectedLink = link;
    for(size_t i=0;i<robot->drivers.size();i++)
      if(robot->DoesDriverAffect(i,affectedLink)) {
	affectedDriver = (int)i;
	break;
      }
  }
  if(hoverLink != oldHoverLink) Refresh();
  return (hoverLink != -1);
}

bool RobotLinkPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) 
{ 
  if(!Hover(x,y,viewport,distance)) return false;
  return true;
}

void RobotLinkPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(affectedDriver < 0) return;
  robot->UpdateConfig(poseConfig);
  //this is for rotational joints
  Real shift = dy*0.02;
  //for prismatic joints, use the distance in space
  if(robot->drivers[affectedDriver].linkIndices.size()==1) {
    int link = robot->drivers[affectedDriver].linkIndices[0];
    if(robot->links[link].type == RobotLink3D::Prismatic) {
      Vector3 pt = robot->links[link].T_World.t;
      float x,y,d;
      viewport.project(pt,x,y,d);
      Vector3 v;
      viewport.getMovementVectorAtDistance(0,dy,d,v);
      shift = -Sign(Real(dy))*v.norm();
    }
  }

  Real val = Clamp(robot->GetDriverValue(affectedDriver)+shift,robot->drivers[affectedDriver].qmin,robot->drivers[affectedDriver].qmax);
  robot->SetDriverValue(affectedDriver,val);
  poseConfig = robot->q;
  Refresh();
}

void RobotLinkPoseWidget::DrawGL(Camera::Viewport& viewport) 
{
  if(draw) {
    robot->UpdateConfig(poseConfig);
    viewRobot->PushAppearance();
    for(size_t i=0;i<poserAppearance.size();i++)
      viewRobot->Appearance(i) = poserAppearance[i];
    if(hasHighlight || hasFocus) {
      for(size_t i=0;i<highlightedLinks.size();i++)
	viewRobot->Appearance(highlightedLinks[i]).ModulateColor(highlightColor,0.5);
    }
    if(!activeDofs.empty()) {
      GLColor black(0,0,0,0);
      for(size_t i=0;i<robot->links.size();i++)
        if(find(activeDofs.begin(),activeDofs.end(),(int)i) == activeDofs.end())
          viewRobot->Appearance(i).ModulateColor(black,0.5);
    }
    viewRobot->Draw();
    //copy display lists, if not already initialized
    for(size_t i=0;i<poserAppearance.size();i++) {
      if(!poserAppearance[i].vertexDisplayList)
	poserAppearance[i] = viewRobot->Appearance(i);
    }
    viewRobot->PopAppearance();

    if(affectedLink >= 0 && (hasHighlight || hasFocus)) {
      //draw joint position widget
      //push depth upward so the widget shows through
      glEnable(GL_LIGHTING);
      glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
      glDepthRange (0.0, 0.9);
      glDisable(GL_CULL_FACE);
      int i = affectedDriver;
      if(i >= 0) {
	if(robot->drivers[i].type == RobotJointDriver::Normal) {
	  Vector3 center;
	  if(robot->parents[affectedLink] < 0) center=robot->links[affectedLink].T0_Parent.t;
	  else center=robot->links[robot->parents[affectedLink]].T_World*robot->links[affectedLink].T0_Parent.t;
	  Vector3 worldAxis = robot->links[affectedLink].T_World.R*robot->links[affectedLink].w;
	  Vector3 x,y;
	  GetCanonicalBasis(worldAxis,x,y);
	  Real q1 = robot->qMin(affectedLink);
	  Real q2 = robot->qMax(affectedLink);
	  if(!IsInf(q1) && !IsInf(q2) && q1 != q2) {
	    if(robot->links[affectedLink].type == RobotLink3D::Revolute) {
	      //rotational joint, draw a strip arc
	      Real r1 = 0.1;
	      Real r2 = 0.12;
	      Real zscale = 0.0;
	      if (q2 > q1+Pi*3/2)
		zscale = 0.01;
	      Real dq = 0.1;
	      Real q = q1;
	      glBegin(GL_TRIANGLE_STRIP);
	      while(q < q2) {
		GLColor col(1,1-0.5*(q-q1)/(q2-q1),0);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col);
		Real c = Cos(q);
		Real s = Sin(q);
		Vector3 p1 = center+worldAxis*zscale*q + c*r1*x + s*r1*y;
		Vector3 p2 = center+worldAxis*zscale*q + c*r2*x + s*r2*y;
		glNormal3v(worldAxis);
		glVertex3v(p1);
		glVertex3v(p2);
		q += dq;
	      }
	      q = q2;
	      GLColor col(1,0.5,0);
	      Real c = Cos(q);
	      Real s = Sin(q);
	      glNormal3v(worldAxis);
	      Vector3 p1 = center+worldAxis*zscale*q + c*r1*x + s*r1*y;
	      Vector3 p2 = center+worldAxis*zscale*q + c*r2*x + s*r2*y;
	      glVertex3v(p1);
	      glVertex3v(p2);
	      glEnd();
	      Real rmid = (r1+r2)*0.5;
	      q = robot->q(affectedLink);
	      c = Cos(q);
	      s = Sin(q);
	      glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col);
	      Vector3 pt = center+worldAxis*zscale*q + c*rmid*x + s*rmid*y;
	      glPushMatrix();
	      glTranslate(pt);
	      drawSphere(0.02,16,8);
	      glPopMatrix();
	    }
	    else {
	      //translational joint, draw a strip
	      glBegin(GL_TRIANGLE_STRIP);
	      Vector3 p1 = center+worldAxis*q1 - x*0.01;
	      Vector3 p2 = center+worldAxis*q1 + x*0.01;
	      GLColor col(1,1,0);
	      glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col);
	      glVertex3v(p1);
	      glVertex3v(p2);
	      p1 = center+worldAxis*q2 - x*0.01;
	      p2 = center+worldAxis*q2 + x*0.01;
	      col.rgba[1] = 0.5;
	      glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col);
	      glVertex3v(p1);
	      glVertex3v(p2);
	      glEnd();
	      p1 = center+worldAxis*robot->q(affectedLink);
	      glPushMatrix();
	      glTranslate(p1);
	      drawSphere(0.02,16,8);
	      glPopMatrix();
	    }
	  }
	}
      }
      glDepthRange (0.0, 1.0);
      glPopAttrib();
    }
  }
}


RobotIKPoseWidget::RobotIKPoseWidget(Robot* _robot)
  :robot(_robot) 
{}

void RobotIKPoseWidget::ClearLink(int link) 
{
  for(size_t i=0;i<poseGoals.size();i++) {
    if(poseGoals[i].link == link) {
      poseGoals.erase(poseGoals.begin()+i);
      poseWidgets.erase(poseWidgets.begin()+i);
      i--;
    }
  }
  RefreshWidgets();
}
void RobotIKPoseWidget::FixPoint(int link,const Vector3& localpt) 
{
  poseGoals.resize(poseGoals.size()+1);
  poseGoals.back().link = link;
  poseGoals.back().destLink = -1;
  poseGoals.back().localPosition = localpt;
  poseGoals.back().SetFixedPosition(robot->links[link].T_World*localpt);
  poseWidgets.resize(poseGoals.size());
  poseWidgets.back().T.R = robot->links[link].T_World.R;
  poseWidgets.back().T.t = robot->links[link].T_World*localpt;
  poseWidgets.back().enableRotation = false;
  RefreshWidgets();
}
void RobotIKPoseWidget::FixLink(int link) 
{
  poseGoals.resize(poseGoals.size()+1);
  poseGoals.back().link = link;
  poseGoals.back().destLink = -1;
  poseGoals.back().localPosition = robot->links[link].com;
  poseGoals.back().SetFixedPosition(robot->links[link].T_World*robot->links[link].com);
  poseGoals.back().SetFixedRotation(robot->links[link].T_World.R);
  poseWidgets.resize(poseGoals.size());
  poseWidgets.back().T.R = robot->links[link].T_World.R;
  poseWidgets.back().T.t = robot->links[link].T_World*robot->links[link].com;
  poseWidgets.back().enableRotation = true;
  RefreshWidgets();
}
void RobotIKPoseWidget::Add(const IKGoal& goal) {
  poseGoals.push_back(goal);
  poseWidgets.resize(poseWidgets.size()+1);
  poseWidgets.back().T.R = robot->links[goal.link].T_World.R;
  poseWidgets.back().T.t = poseGoals.back().endPosition;  
  if(goal.rotConstraint == IKGoal::RotNone)
    poseWidgets.back().enableRotation = false;
  else
    poseWidgets.back().enableRotation = true;
  RefreshWidgets();
}
void RobotIKPoseWidget::AttachWidget(int widget,int link) 
{
  LOG4CXX_INFO(KrisLibrary::logger(),"Attaching widget from link "<<poseGoals[widget].destLink<<" to "<<link);
  Assert(widget >= 0 && widget < (int)poseGoals.size());
  int oldDest = poseGoals[widget].destLink;
  poseGoals[widget].destLink = link;
  Matrix3 oldR;
  Vector3 oldp;
  if(oldDest >= 0)
    oldp = robot->links[oldDest].T_World*poseGoals[widget].endPosition;
  else
    oldp = poseGoals[widget].endPosition;
  if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) {
    RigidTransform Tgoal;
    poseGoals[widget].GetFixedGoalTransform(Tgoal);
    if(oldDest >= 0) 
      oldR = robot->links[oldDest].T_World.R*Tgoal.R;
    else
      oldR = Tgoal.R;
  }
  if(link >= 0) {
    robot->links[link].T_World.mulInverse(oldp,poseGoals[widget].endPosition);
    if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Desired world rotation "<<oldR<<"\n");
      //LOG4CXX_INFO(KrisLibrary::logger(),"Reference link rotation "<<robot->links[link].T_World.R<<"\n");
      Matrix3 Rlocal;
      Rlocal.mulTransposeA(robot->links[link].T_World.R,oldR);
      //LOG4CXX_INFO(KrisLibrary::logger(),"Desired local rotation "<<Rlocal<<"\n");
      poseGoals[widget].SetFixedRotation(Rlocal);
    }
  }
  else {
    poseGoals[widget].endPosition = oldp;
    if(poseGoals[widget].rotConstraint == IKGoal::RotFixed) 
      poseGoals[widget].SetFixedRotation(oldR);
  }
}
void RobotIKPoseWidget::RefreshWidgets() 
{
  widgets.resize(poseWidgets.size());
  for(size_t i=0;i<widgets.size();i++) {
    widgets[i] = &poseWidgets[i];
  }
  closestWidget = NULL;
  activeWidget = NULL;
  Refresh();
}
int RobotIKPoseWidget::HoverWidget() const
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(closestWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  return index;
}
int RobotIKPoseWidget::ActiveWidget() const
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(activeWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  return index;
}
bool RobotIKPoseWidget::ClearCurrent()
{
  int index = -1;
  for(size_t i=0;i<poseGoals.size();i++)
    if(activeWidget == &poseWidgets[i]) {
      index = (int)i;
      break;
    }
  if(index != -1) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Deleting IK goal on link "<<robot->LinkName(poseGoals[index].link).c_str());
    poseGoals.erase(poseGoals.begin()+index);
    poseWidgets.erase(poseWidgets.begin()+index);
    RefreshWidgets();
    return true;
  }
  return false;
}

bool RobotIKPoseWidget::Hover(int x,int y,Camera::Viewport& viewport,double& closestDistance)
{
  if(WidgetSet::Hover(x,y,viewport,closestDistance)) {
    closestDistance -= 1.0;
    return true;
  }
  return false;
}

bool RobotIKPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& closestDistance)
{
  if(WidgetSet::BeginDrag(x,y,viewport,closestDistance)) {
    closestDistance -= 1.0;
    return true;
  }
  return false;
}

void RobotIKPoseWidget::DrawGL(Camera::Viewport& viewport) 
{
  WidgetSet::DrawGL(viewport);
  glPolygonOffset(0,-1000);
  glEnable(GL_POLYGON_OFFSET_FILL);
  for(size_t i=0;i<poseGoals.size();i++) {
    Vector3 curpos = robot->links[poseGoals[i].link].T_World*poseGoals[i].localPosition;
    Vector3 despos = poseGoals[i].endPosition;
    if(poseGoals[i].destLink >= 0)
      despos = robot->links[poseGoals[i].destLink].T_World*despos;
    glDisable(GL_LIGHTING);
    glColor3f(1,0,0);
    glLineWidth(5.0);
    glBegin(GL_LINES);
    glVertex3v(curpos);
    glVertex3v(despos);
    glEnd();
    glLineWidth(1.0);
      
    poseWidgets[i].DrawGL(viewport);
      
    float color2[4] = {1,0.5,0,1};
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color2); 
    glPushMatrix();
    if(poseGoals[i].rotConstraint == IKGoal::RotFixed) {
      RigidTransform T;
      poseGoals[i].GetFixedGoalTransform(T);
      if(poseGoals[i].destLink >= 0)
	T = robot->links[poseGoals[i].destLink].T_World*T;
      glMultMatrix(Matrix4(T));
      drawBox(0.04,0.04,0.04);
    }
    else {
      glTranslate(despos);
      drawSphere(0.02,16,8);
    }
    glPopMatrix();
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
}

void RobotIKPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(activeWidget) {
    activeWidget->Drag(dx,dy,viewport);
    Refresh();
    int index = ActiveWidget();
    if(index < 0) return;
    if(poseGoals[index].rotConstraint == IKGoal::RotFixed) {
      poseGoals[index].SetFixedRotation(poseWidgets[index].T.R);
      poseGoals[index].SetFixedPosition(poseWidgets[index].T.t);
    }
    else {
      poseGoals[index].SetFixedPosition(poseWidgets[index].T.t);
    }
  }
}

RobotPoseWidget::RobotPoseWidget()
  :useBase(false),linkPoser(),ikPoser(NULL),mode(ModeNormal)
{
  useBase = 0;
}

RobotPoseWidget::RobotPoseWidget(Robot* robot,ViewRobot* viewRobot)
  :useBase(false),linkPoser(robot,viewRobot),ikPoser(robot),mode(ModeNormal)
{
  if(robot->joints[0].type == RobotJoint::Floating) {
    useBase=true;
    basePoser.T = GetFloatingBase(*robot);
  }
  else if(robot->joints[0].type == RobotJoint::FloatingPlanar) {  //only allow movement in x,y, and yaw axes
    useBase=true;
    basePoser.T = GetFloatingBase(*robot);
    basePoser.enableTranslationAxes[2] = 0;
    basePoser.enableRotationAxes[0] = 0;
    basePoser.enableRotationAxes[1] = 0;
    basePoser.enableOriginTranslation = 0;
    basePoser.enableOuterRingRotation = 0;
  }
  else if (IsFloatingBase(*robot)) {
    useBase = true;
    basePoser.T = GetFloatingBase(*robot);
  }
  if(useBase) {
    widgets.resize(3);
    widgets[0]=&basePoser;
    widgets[1]=&linkPoser;
    widgets[2]=&ikPoser;
  }
  else {
    widgets.resize(2);
    widgets[0]=&linkPoser;
    widgets[1]=&ikPoser;
  }
}

void RobotPoseWidget::Set(Robot* robot,ViewRobot* viewRobot)
{
  linkPoser.Set(robot,viewRobot);
  ikPoser.robot = robot;
  if(robot->joints[0].type == RobotJoint::Floating) {
    useBase=true;
    basePoser.T = GetFloatingBase(*robot);
  }
  else if(robot->joints[0].type == RobotJoint::FloatingPlanar) {  //only allow movement in x,y, and yaw axes
    useBase=true;
    basePoser.T = GetFloatingBase(*robot);
    basePoser.enableTranslationAxes[2] = 0;
    basePoser.enableRotationAxes[0] = 0;
    basePoser.enableRotationAxes[1] = 0;
    basePoser.enableOriginTranslation = 0;
    basePoser.enableOuterRingRotation = 0;
  }
  else if (IsFloatingBase(*robot)) {
    useBase = true;
    basePoser.T = GetFloatingBase(*robot);
  }
  if(useBase) {
    widgets.resize(3);
    widgets[0]=&basePoser;
    widgets[1]=&linkPoser;
    widgets[2]=&ikPoser;
  }
  else {
    widgets.resize(2);
    widgets[0]=&linkPoser;
    widgets[1]=&ikPoser;
  }
}

Config RobotPoseWidget::Pose_Conditioned(const Config& qref) const
{
  Robot* robot = linkPoser.robot;
  Assert(qref.n == linkPoser.poseConfig.n);
  Config res = linkPoser.poseConfig;
  for(int i=0;i<robot->q.n;i++) {
    if(robot->links[i].type == RobotLink3D::Revolute && robot->qMin(i)+TwoPi < robot->qMax(i)) {
      if(Abs(res[i]-qref[i]) > Pi) {
	res[i] = AngleDiff(AngleNormalize(res[i]),AngleNormalize(qref[i])) + qref[i];
      }
    }
  }
  return res;
}
  
bool RobotPoseWidget::FixCurrent() 
{
  if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    ikPoser.FixLink(linkPoser.hoverLink);
    return true;
  }
  return false;
}

bool RobotPoseWidget::FixCurrentPoint() 
{
  if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    ikPoser.FixPoint(linkPoser.hoverLink,linkPoser.hoverPt);
    return true;
  }
  return false;
}

bool RobotPoseWidget::DeleteConstraint()
{
  if(activeWidget == &ikPoser) {
    return ikPoser.ClearCurrent();
  }
  else if(linkPoser.hoverLink >= 0) {
    ikPoser.ClearLink(linkPoser.hoverLink);
    return true;
  }
  return false;
}

void RobotPoseWidget::SetAttachIKMode(bool on)
{
  if(on) mode = ModeIKAttach;
  else mode = ModeNormal;
}

void RobotPoseWidget::SetPoseIKMode(bool on)
{
  if(on) mode = ModeIKPose;
  else mode = ModeNormal;
}

void RobotPoseWidget::SetFixedPoseIKMode(bool on)
{
  if(on) mode = ModeIKPoseFixed;
  else mode = ModeNormal;
}

void RobotPoseWidget::SetDeleteIKMode(bool on)
{
  if(on) mode = ModeIKDelete;
  else mode = ModeNormal;
}

void RobotPoseWidget::SetPose(const Config& q)
{
  Robot* robot=linkPoser.robot;
  linkPoser.poseConfig = q;
  if(q != robot->q)
    robot->UpdateConfig(q);
  if(useBase) {
    basePoser.T = GetFloatingBase(*robot);
  }
}

void RobotPoseWidget::DrawGL(Camera::Viewport& viewport)
{
  WidgetSet::DrawGL(viewport);
  if(mode == ModeIKAttach && hasFocus) {
    //draw a line
    if(ikPoser.ActiveWidget() >= 0) {
      Vector3 x;
      attachRay.closestPoint(ikPoser.poseWidgets[ikPoser.ActiveWidget()].T.t,x);
      glColor3f(1,0.5,0);
      glDisable(GL_LIGHTING);
      glLineWidth(3.0);
      glBegin(GL_LINES);
      glVertex3v(ikPoser.poseWidgets[ikPoser.ActiveWidget()].T.t);
      glVertex3v(x);
      glEnd();
    }
  }
}

bool RobotPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  if(mode == ModeIKAttach) {
    bool res = ikPoser.Hover(x,y,viewport,distance);
    if(!res) {
      return false;    
    }
    attachx=x;
    attachy=y;
    Refresh();
    return true;
  }
  else if(mode == ModeIKPose) {
    bool res=WidgetSet::BeginDrag(x,y,viewport,distance);
    if(!res) return false;
    if(closestWidget == &linkPoser) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Adding new point constraint\n");
      res=FixCurrentPoint();
      ikPoser.poseWidgets.back().Hover(x,y,viewport,distance);
      ikPoser.poseWidgets.back().SetHighlight(true);
      //following lines let it be dragged immediately -- comment out if you want it fixed
      if(ikPoser.poseWidgets.back().BeginDrag(x,y,viewport,distance)) {
	closestWidget = &ikPoser;
	ikPoser.closestWidget = &ikPoser.poseWidgets.back();
	return true;
      }
    }
    return true;
  }
  else if(mode == ModeIKPoseFixed) {
    bool res=WidgetSet::BeginDrag(x,y,viewport,distance);
    if(!res) return false;
    if(closestWidget == &linkPoser) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Adding new fixed transform constraint\n");
      res=FixCurrent();
      ikPoser.poseWidgets.back().Hover(x,y,viewport,distance);
      ikPoser.poseWidgets.back().SetHighlight(true);
      //following lines let it be dragged immediately -- comment out if you want it fixed
      if(ikPoser.poseWidgets.back().BeginDrag(x,y,viewport,distance)) {
	closestWidget = &ikPoser;
	ikPoser.closestWidget = &ikPoser.poseWidgets.back();
	return true;
      }
    }
    return true;
  }
  else if(mode == ModeIKDelete) {
    DeleteConstraint();
    return true;
  }
  else {
    return WidgetSet::BeginDrag(x,y,viewport,distance);
  }
}

void RobotPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  if(mode == ModeIKAttach) {
    //LOG4CXX_INFO(KrisLibrary::logger(),"Attach dragging, hover widget "<<ikPoser.ActiveWidget());
    attachx += dx;
    attachy += dy; 
    viewport.getClickSource(attachx,attachy,attachRay.source);
    viewport.getClickVector(attachx,attachy,attachRay.direction);
    double dist;
    bool res=linkPoser.Hover(attachx,attachy,viewport,dist);
    Refresh();
    return;
  }
  else if(mode == ModeIKDelete) {
    return;
  }
  WidgetSet::Drag(dx,dy,viewport);
  if(activeWidget == &basePoser) {
    Robot* robot=linkPoser.robot;
    SetFloatingBase(*robot,basePoser.T);
    robot->UpdateFrames();
    linkPoser.poseConfig = robot->q;
  }
  if(activeWidget == &ikPoser) {
    SolveIK();
  }
  if(activeWidget == &basePoser) {
    SolveIKFixedBase();
  }
  if(activeWidget == &linkPoser) {
    SolveIKFixedJoint(linkPoser.hoverLink);
  }
}

void RobotPoseWidget::EndDrag()
{
  if(mode == ModeIKAttach) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Attaching constraint to "<<linkPoser.hoverLink<<"\n");
    Refresh();
    int link = linkPoser.hoverLink;
    int widget = ikPoser.ActiveWidget();
    ikPoser.robot->UpdateConfig(linkPoser.poseConfig);
    ikPoser.AttachWidget(widget,link);
  }
  WidgetSet::EndDrag();
}

bool RobotPoseWidget::SolveIK(int iters,Real tol)
{
  if(Constraints().empty()) return true;
  if(iters <= 0) iters=100;
  if(tol <= 0) tol = 1e-3;
  //solve the IK problem    
  Robot* robot=linkPoser.robot;
  robot->UpdateConfig(linkPoser.poseConfig);
  
  RobotIKFunction f(*robot);
  f.UseIK(Constraints());
  GetDefaultIKDofs(*robot,Constraints(),f.activeDofs);
  //take out the fixed DOF
  set<int> dofs(f.activeDofs.mapping.begin(),f.activeDofs.mapping.end());
  if(!linkPoser.activeDofs.empty()) {
    //take out non-active dofs
    set<int> intersect;
    for(size_t i=0;i<linkPoser.activeDofs.size();i++)
      if(dofs.count(linkPoser.activeDofs[i]) != 0)
        intersect.insert(linkPoser.activeDofs[i]);
    dofs = intersect;
  }
  f.activeDofs.mapping = vector<int>(dofs.begin(),dofs.end());

  //define start config
  robot->q = linkPoser.poseConfig;
  bool res = (RobustSolveIK(*robot,f,iters,tol,5) == 0);

  linkPoser.poseConfig = robot->q;
  if(useBase)
    basePoser.T = GetFloatingBase(*robot);
  Refresh();
  return res;
}

bool RobotPoseWidget::SolveIKFixedBase(int iters,Real tol)
{
  if(Constraints().empty()) return true;
  if(iters <= 0) iters=100;
  if(tol <= 0) tol = 1e-3;
  //solve the IK problem    
  Robot* robot=linkPoser.robot;
  robot->UpdateConfig(linkPoser.poseConfig);

  RobotIKFunction f(*robot);
  f.UseIK(Constraints());
  GetDefaultIKDofs(*robot,Constraints(),f.activeDofs);
  //take out the base DOFs
  set<int> dofs(f.activeDofs.mapping.begin(),f.activeDofs.mapping.end());
  for(int i=0;i<6;i++)
    dofs.erase(i);
  if(!linkPoser.activeDofs.empty()) {
    //take out non-active dofs
    set<int> intersect;
    for(size_t i=0;i<linkPoser.activeDofs.size();i++)
      if(dofs.count(linkPoser.activeDofs[i]) != 0)
        intersect.insert(linkPoser.activeDofs[i]);
    dofs = intersect;
  }
  f.activeDofs.mapping = vector<int>(dofs.begin(),dofs.end());

  robot->q = linkPoser.poseConfig;
  bool res = (RobustSolveIK(*robot,f,iters,tol,5) == 0);

  linkPoser.poseConfig = robot->q;
  if(useBase)
    basePoser.T = GetFloatingBase(*robot);
  Refresh();
  return res;
}

bool RobotPoseWidget::SolveIKFixedJoint(int fixedJoint,int iters,Real tol)
{
  if(Constraints().empty()) return true;
  if(iters <= 0) iters=100;
  if(tol <= 0) tol = 1e-3;
  //solve the IK problem    
  Robot* robot=linkPoser.robot;
  robot->UpdateConfig(linkPoser.poseConfig);

  RobotIKFunction f(*robot);
  f.UseIK(Constraints());
  GetDefaultIKDofs(*robot,Constraints(),f.activeDofs);
  //take out the fixed DOF
  set<int> dofs(f.activeDofs.mapping.begin(),f.activeDofs.mapping.end());
  dofs.erase(fixedJoint);
  if(!linkPoser.activeDofs.empty()) {
    //take out non-active dofs
    set<int> intersect;
    for(size_t i=0;i<linkPoser.activeDofs.size();i++)
      if(dofs.count(linkPoser.activeDofs[i]) != 0)
        intersect.insert(linkPoser.activeDofs[i]);
    dofs = intersect;
  }
  f.activeDofs.mapping = vector<int>(dofs.begin(),dofs.end());

  robot->q = linkPoser.poseConfig;
  bool res = (RobustSolveIK(*robot,f,iters,tol,5) == 0);

  linkPoser.poseConfig = robot->q;
  if(useBase)
    basePoser.T = basePoser.T = GetFloatingBase(*robot);
  Refresh();
  return res;
}

void RobotPoseWidget::Keypress(char c)
{
  if(c=='s') {
    SolveIK();
  }
}
