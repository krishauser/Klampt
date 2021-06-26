#include "ObjectPoseWidget.h"
using namespace GLDraw;
using namespace Klampt;

RigidObjectPoseWidget::RigidObjectPoseWidget()
  :rigidObject(NULL)
{
}

RigidObjectPoseWidget::RigidObjectPoseWidget(RigidObjectModel* object)
  :rigidObject(object)
{
  poser.T = object->T;
  widgets.resize(1);
  widgets[0] = &poser;
}

void RigidObjectPoseWidget::Set(RigidObjectModel* object)
{
  rigidObject = object;
  poser.T = object->T;
  widgets.resize(1);
  widgets[0] = &poser;
}


void RigidObjectPoseWidget::SetPose(const RigidTransform& T)
{
  if(rigidObject) {
    rigidObject->T = T;
    rigidObject->UpdateGeometry();
  }
  poser.T = T;
}

void RigidObjectPoseWidget::DrawGL(Camera::Viewport& viewport)
{
  WidgetSet::DrawGL(viewport);
}

bool RigidObjectPoseWidget::BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance)
{
  return WidgetSet::BeginDrag(x,y,viewport,distance);
}

void RigidObjectPoseWidget::Drag(int dx,int dy,Camera::Viewport& viewport)
{
  WidgetSet::Drag(dx,dy,viewport);
  if(activeWidget == &poser) {
    rigidObject->T = poser.T;
    rigidObject->UpdateGeometry();
  }
}

void RigidObjectPoseWidget::EndDrag()
{
  WidgetSet::EndDrag();
}

void RigidObjectPoseWidget::Keypress(char c)
{
  WidgetSet::Keypress(c);
}
