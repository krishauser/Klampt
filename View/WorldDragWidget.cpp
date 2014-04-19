#include "WorldDragWidget.h"
#include <GLdraw/drawextra.h>

WorldDragWidget::WorldDragWidget(RobotWorld* _world)
  :world(_world),active(true),robotsActive(true),objectsActive(true),terrainsActive(false),
   highlightColor(1,1,1,0.3),lineColor(1,0.5,0),lineWidth(5.0),dragging(false),hoverID(-1),highlightID(-1)
{}

void WorldDragWidget::Set(RobotWorld* _world)
{
  world = _world;
}

void WorldDragWidget::Enable(bool _active)
{
  active = _active;
}

void WorldDragWidget::SetHighlight(bool value)
{
  hasHighlight = value;
  if(hasHighlight && hoverID >= 0) {
    //update the object's color
    originalFaceColor = world->GetAppearance(hoverID).faceColor;
    world->GetAppearance(hoverID).faceColor.blend(originalFaceColor,highlightColor,highlightColor.rgba[3]);
    highlightID = hoverID;
  }
  else if(!hasHighlight && highlightID >= 0) {
    //restore the object's color
    world->GetAppearance(highlightID).faceColor = originalFaceColor;
  }
}

bool WorldDragWidget::Hover(int x,int y,Camera::Viewport& viewport,double& distance)
{
  if(!active) return false;
  Ray3D r;
  viewport.getClickSource(x,y,r.source);
  viewport.getClickVector(x,y,r.direction);
  hoverID = -1;
  Real bestD = Inf;
  if(robotsActive) {
    int body;
    Vector3 localpt;
    RobotInfo* rob = world->ClickRobot(r,body,localpt);
    if(rob) {
      hoverPt = localpt;
      hoverID = world->RobotLinkID(rob-&world->robots[0],body);
      const Geometry::AnyCollisionGeometry3D& geom = world->GetGeometry(hoverID);
      Vector3 worldpt = geom.GetTransform()*localpt;
      bestD = worldpt.distance(r.source);
      dragPt = worldpt;
    }
  }
  if(objectsActive) {
    Vector3 localpt;
    RigidObjectInfo* obj = world->ClickObject(r,localpt);
    if(obj) {
      Vector3 worldpt = obj->object->T*localpt;
      Real d=worldpt.distance(r.source);
      if(d < bestD) {
	bestD = d;
	hoverPt = localpt;
	hoverID = world->RigidObjectID(obj-&world->rigidObjects[0]);
	dragPt = worldpt;
      }
    }
  }
  hoverDistance = bestD;
  if(hoverID >= 0)  return true;
  return false;
}

bool WorldDragWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  if(!Hover(x,y,viewport,distance)) return false;
  dragging = true;
  return true;
}

void WorldDragWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  Vector3 v;
  viewport.getMovementVectorAtDistance(dx,dy,hoverDistance,v);
  dragPt += v;
}

void WorldDragWidget::EndDrag()
{
  dragging = false; 
}

void WorldDragWidget::DrawGL(Camera::Viewport& viewport)
{
  if(hoverID < 0) return;
  if(hasFocus) {
    const Geometry::AnyCollisionGeometry3D& geom = world->GetGeometry(hoverID);
    glDisable(GL_LIGHTING);
    lineColor.setCurrentGL();
    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3v(geom.GetTransform()*hoverPt);
    glVertex3v(dragPt);
    glEnd();
    glLineWidth(1);
  }
}
