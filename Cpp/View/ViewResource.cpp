#include "ViewResource.h"
#include "Planning/RobotCSpace.h"
#include "Planning/RobotTimeScaling.h"
#include <KrisLibrary/math/interpolate.h>
#include "Modeling/Interpolate.h"
#include <KrisLibrary/GLdraw/drawgeometry.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <sstream>
using namespace GLDraw;
using namespace Klampt;

#if 0
  void drawgeom(const GeometricPrimitive2D& geom)
  {
    switch(geom.type) {
    case GeometricPrimitive2D::Point:
      {
	glBegin(GL_POINTS);
	glVertex2v(*AnyCast<Vector2>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Segment:
      {
	const Segment2D* seg=AnyCast<Segment2D>(&geom.data);
	glBegin(GL_LINES);
	glVertex2v(seg->a);
	glVertex2v(seg->b);
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Circle:
      {
	const Circle2D* circle = AnyCast<Circle2D>(&geom.data);
	drawCircle2D(circle->center,circle->radius);
      }
      break;
    case GeometricPrimitive2D::AABB:
      {
	const AABB2D* aabb=AnyCast<AABB2D>(&geom.data);
	glBegin(GL_QUADS);
	glVertex2f(aabb->bmin.x,aabb->bmin.y);
	glVertex2f(aabb->bmax.x,aabb->bmin.y);
	glVertex2f(aabb->bmax.x,aabb->bmax.y);
	glVertex2f(aabb->bmin.x,aabb->bmax.y);
	glEnd();
      }
      break;
    case GeometricPrimitive2D::Box:
      {
	const Box2D* box=AnyCast<Box2D>(&geom.data);
	glBegin(GL_QUADS);
	glVertex2v(box->origin);
	glVertex2v(box->origin+box->dims.x*box->xbasis);
	glVertex2v(box->origin+box->dims.x*box->xbasis+box->dims.y*box->ybasis);
	glVertex2v(box->origin+box->dims.y*box->ybasis);
	glEnd();
	break;
      }
    case GeometricPrimitive2D::Triangle:
      {
	const Triangle2D* tri=AnyCast<Triangle2D>(&geom.data);
	glBegin(GL_TRIANGLES);
	glVertex2v(tri->a);
	glVertex2v(tri->b);
	glVertex2v(tri->c);
	glEnd();
	break;
      }
    default:
      return;
    }
  }
  void drawgeom(const GeometricPrimitive3D& geom)
  {
    switch(geom.type) {
    case GeometricPrimitive3D::Point:
      {
	glBegin(GL_POINTS);
	glVertex3v(*AnyCast<Vector3>(&geom.data));
	glEnd();
      }
      break;
    case GeometricPrimitive3D::Segment:
      {
	const Segment3D* seg=AnyCast<Segment3D>(&geom.data);
	glBegin(GL_LINES);
	glVertex3v(seg->a);
	glVertex3v(seg->b);
	glEnd();
      }
      break;
      /*
    case GeometricPrimitive3D::Circle:
      {
	const Circle3D* circle = AnyCast<Circle3D>(&geom.data);
	glPushMatrix();
	glTranslate(circle->center);
	drawCircle(circle->axis,circle->radius);
	glPopMatrix();
      }
      break;
      */
      /*
    case GeometricPrimitive3D::AABB:
      {
	const AABB3D* aabb=AnyCast<AABB3D>(&geom.data);
	drawBoundingBox(aabb->bmin,aabb->bmax);
      }
      break;
      */
    case GeometricPrimitive3D::Box:
      {
	const Box3D* box=AnyCast<Box3D>(&geom.data);
	Matrix4 m;
	box->getBasis(m);
	glPushMatrix();
	glMultMatrix(m);
	drawBoxCorner(box->dims.x,box->dims.y,box->dims.z);
	glPopMatrix();
	break;
      }
    case GeometricPrimitive3D::Triangle:
      {
	const Triangle3D* tri=AnyCast<Triangle3D>(&geom.data);
	drawTriangle(tri->a,tri->b,tri->c);
	break;
      }
    case GeometricPrimitive3D::Polygon:
      {
	const Polygon3D* p=AnyCast<Polygon3D>(&geom.data);
	Plane3D plane;
	p->getPlane(0,plane);
	glNormal3v(plane.normal);
	glBegin(GL_TRIANGLE_FAN);
	glVertex3v(p->vertices[0]);
	for(size_t i=1;i+1<p->vertices.size();i++) {
	  glVertex3v(p->vertices[i]);
	  glVertex3v(p->vertices[i+1]);
	}
	glEnd();
	break;
      }
    case GeometricPrimitive3D::Sphere:
      {
	const Sphere3D* s=AnyCast<Sphere3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawSphere(s->radius,32,32);
	glPopMatrix();
	break;
      }
    case GeometricPrimitive3D::Cylinder:
      {
	const Cylinder3D* s=AnyCast<Cylinder3D>(&geom.data);
	glPushMatrix();
	glTranslate(s->center);
	drawCylinder(s->axis*s->height,s->radius,32);
	glPopMatrix();
	break;
      }
      break;
    default:
      fprintf(stderr,"draw: Unsupported geometry type\n");
      return;
    }
  }
#endif

ViewResource::ViewResource(RobotModel* robot)
{
  SetRobot(robot);
  pathTime = 0;

  pathIKResolution = 0.005;
}

void ViewResource::SetRobot(RobotModel* robot)
{
  configViewer.robot = robot;
  configsViewer.robot = robot;
  pathViewer.robot = robot;
  configViewer.RestoreAppearance();
  configsViewer.RestoreAppearance();
  pathViewer.RestoreAppearance();
  configViewer.PushAppearance();
  configsViewer.PushAppearance();
  pathViewer.PushAppearance();
  configViewer.SetColors(GLColor(0.5,0.5,0.5,0.7f));
  configsViewer.SetColors(GLColor(0.5,0.5,0.5,0.7f));
  pathViewer.SetColors(GLColor(0.5,0.5,0.5,0.7f));
}
void ViewResource::SetAnimTime(Real time)
{
  pathTime = time;
}

void ViewResource::DrawGL(const ResourcePtr& r)
{
  if(typeid(*r)==typeid(ConfigResource)) {
    if(configViewer.robot==NULL) return;
    const ConfigResource* rc=dynamic_cast<const ConfigResource*>(r.get());
    Config oldq = configViewer.robot->q;
    if(rc->data.n != configViewer.robot->q.n) {
      //fprintf(stderr,"Incorrect robot configuration size: %d vs %d\n",rc->data.n,configViewer.robot->q.n);
    }
    else {
      configViewer.robot->UpdateConfig(rc->data);
      configViewer.Draw();
    }
    configViewer.robot->UpdateConfig(oldq);
  }
  else if(typeid(*r)==typeid(ConfigsResource)) {
    if(configsViewer.robot==NULL) return;
    const ConfigsResource* rc=dynamic_cast<const ConfigsResource*>(r.get());
    Config oldq = configsViewer.robot->q;
    int skip = 1;
    if(rc->configs.size() > 50)
      skip = rc->configs.size()/50;
    for(size_t i=0;i<rc->configs.size();i+=skip) {
      if(rc->configs[i].n != configViewer.robot->q.n) {
	//fprintf(stderr,"Incorrect robot configuration size: %d vs %d\n",rc->configs[i].n,configViewer.robot->q.n);
      }
      else {
	configsViewer.robot->UpdateConfig(rc->configs[i]);
	configsViewer.Draw();
      }
    }
    configsViewer.robot->UpdateConfig(oldq);
  }
  else if(typeid(*r)==typeid(LinearPathResource)) {
    const LinearPathResource* rc=dynamic_cast<const LinearPathResource*>(r.get());
    if(!rc) return;
    RenderLinearPath(rc,pathTime);
  }
  else if(typeid(*r)==typeid(MultiPathResource)) {
    const MultiPathResource* rc=dynamic_cast<const MultiPathResource*>(r.get());
    if(!rc) return;
    RenderMultiPath(rc,pathTime);
  }
  else if(typeid(*r)==typeid(StanceResource)) {
    const StanceResource* rc=dynamic_cast<const StanceResource*>(r.get());
    stanceViewer.DrawHolds(rc->stance);
  }
  else if(typeid(*r)==typeid(GraspResource)) {
    const GraspResource* rc=dynamic_cast<const GraspResource*>(r.get());
    graspViewer.viewRobot = &configViewer;
    graspViewer.Draw(rc->grasp);
  }
  else if(typeid(*r)==typeid(HoldResource)) {
    const HoldResource* rc=dynamic_cast<const HoldResource*>(r.get());
    holdViewer.Draw(rc->hold);
  }
  else if(typeid(*r)==typeid(PointCloudResource)) {
    const PointCloudResource* rc=dynamic_cast<const PointCloudResource*>(r.get());
    if(items.size() != 1 || items[0] != rc) {
      items.resize(1);
      items[0] = rc;
      geometries.resize(1);
      geometries.back().reset(new Geometry::AnyGeometry3D(rc->pointCloud));
      appearances.resize(1);
      appearances.back().Set(*geometries.back());
    }
    appearances.back().DrawGL();
  }
  else if(typeid(*r)==typeid(TriMeshResource)) {
    const TriMeshResource* rc=dynamic_cast<const TriMeshResource*>(r.get());
    if(items.size() != 1 || items[0] != rc) {
      items.resize(1);
      items[0] = rc;
      geometries.resize(1);
      geometries.back().reset(new Geometry::AnyGeometry3D(rc->data));
      appearances.resize(1);
      appearances.back().Set(*geometries.back());
    }
    appearances.back().DrawGL();
  }
  else if(typeid(*r)==typeid(GeometricPrimitive3DResource)) {
    const GeometricPrimitive3DResource* rc=dynamic_cast<const GeometricPrimitive3DResource*>(r.get());
    draw(rc->data);
  }
}

void ViewResource::GetAnimConfig(const ResourcePtr& r,Config& q)
{
  if(typeid(*r)==typeid(LinearPathResource)) {
    const LinearPathResource* rc=dynamic_cast<const LinearPathResource*>(r.get());
    GetLinearPathConfig(rc,pathTime,q);
  }
  else if(typeid(*r)==typeid(MultiPathResource)) {
    const MultiPathResource* rc=dynamic_cast<const MultiPathResource*>(r.get());
    GetMultiPathConfig(rc,pathTime,q);
  }
  else
    q.clear();
}

void ViewResource::GetLinearPathConfig(const LinearPathResource* rc,Real pathTime,Config& q)
{
  q.clear();
  if(rc->times.empty()) {
    return;
  }
  else if(rc->times.front() == rc->times.back()) {
    q = rc->milestones[0];
    return;
  }
  else {
    //TODO: faster tracking using upper_bound?
    Assert(rc->times.size()==rc->milestones.size());
    Assert(rc->times.back() > rc->times.front());
    Real normalizedPathTime = pathTime;
    //looping behavior
    /*
      while(normalizedPathTime < rc->times.front()) 
      normalizedPathTime += rc->times.back()-rc->times.front();
      while(normalizedPathTime > rc->times.back()) 
      normalizedPathTime -= rc->times.back()-rc->times.front();
      pathTime = normalizedPathTime;
    */
    //bouncing behavior
    double cnt = (pathTime-rc->times.front())/(rc->times.back()-rc->times.front());
    int n = (int)Floor(cnt);
    if(n%2==0)
      normalizedPathTime = (cnt-n)*(rc->times.back()-rc->times.front());
    else
      normalizedPathTime = rc->times.back()-(cnt-n)*(rc->times.back()-rc->times.front());
    bool drawn=false;
    for(size_t i=0;i+1<rc->times.size();i++) {
      if(rc->times[i] <= normalizedPathTime && normalizedPathTime <= rc->times[i+1]) {
	Real u=(normalizedPathTime-rc->times[i])/(rc->times[i+1]-rc->times[i]);
	if(pathViewer.robot==NULL)
	  interpolate(rc->milestones[i],rc->milestones[i+1],u,q);
	else {
	  int d=pathViewer.robot->q.n;
	  Assert(rc->milestones[i].n == d);
	  Assert(rc->milestones[i+1].n == d);
	  Interpolate(*pathViewer.robot,rc->milestones[i],rc->milestones[i+1],u,q);
	}
	return;
      }
    }
  }
}

void ViewResource::GetMultiPathConfig(const MultiPathResource* rc,Real pathTime,Config& q)
{
  Real minTime = 0, maxTime = 1;
  if(rc->path.HasTiming()) {
    minTime = rc->path.sections.front().times.front();
    maxTime = rc->path.sections.back().times.back();
  }
  else
    pathTime /= rc->path.sections.size();
  //do bouncing behavior
  double cnt = (pathTime-minTime)/(maxTime-minTime);
  int n = (int)Floor(cnt);
  Real normalizedPathTime;
  if(n%2==0)
    normalizedPathTime = (cnt-n)*(maxTime-minTime)+minTime;
  else
    normalizedPathTime = maxTime-(cnt-n)*(maxTime-minTime);

  if(pathViewer.robot)
    EvaluateMultiPath(*pathViewer.robot,rc->path,normalizedPathTime,q,pathIKResolution);
  else
    rc->path.Evaluate(normalizedPathTime,q);
}

void ViewResource::RenderLinearPath(const LinearPathResource* rc,Real pathTime)
{
  if(pathViewer.robot==NULL) {
    printf("ViewResource: Robot is NULL\n");
    return;
  }
  Config oldq = pathViewer.robot->q;
  Config q;
  GetLinearPathConfig(rc,pathTime,q);
  if(q.size() == oldq.size()) {
    pathViewer.robot->UpdateConfig(q);
    pathViewer.Draw();
    pathViewer.robot->UpdateConfig(oldq);
  }
}

void ViewResource::RenderMultiPath(const MultiPathResource* rc,Real pathTime)
{
  if(pathViewer.robot==NULL) {
    printf("ViewResource: Robot is NULL\n");
    return;
  }
  Config oldq = pathViewer.robot->q;

  Real minTime = rc->path.StartTime(), maxTime = rc->path.EndTime();
  //do bouncing behavior
  double cnt = (pathTime-minTime)/(maxTime-minTime);
  int n = (int)Floor(cnt);
  Real normalizedPathTime;
  if(n%2==0)
    normalizedPathTime = (cnt-n)*(maxTime-minTime)+minTime;
  else
    normalizedPathTime = maxTime-(cnt-n)*(maxTime-minTime);

  Config q;
  EvaluateMultiPath(*pathViewer.robot,rc->path,normalizedPathTime,q,pathIKResolution);
  pathViewer.robot->UpdateConfig(q);
  pathViewer.Draw();
  pathViewer.robot->UpdateConfig(oldq);

  int seg=rc->path.TimeToSection(normalizedPathTime);
  if(seg  < 0)
    seg = 0;
  if(seg >= (int)rc->path.sections.size())
    seg = (int)rc->path.sections.size()-1;
  Stance s;
  rc->path.GetStance(s,seg);
  stanceViewer.DrawHolds(s);
}
