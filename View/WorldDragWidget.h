#ifndef WORLD_DRAG_WIDGET
#define WORLD_DRAG_WIDGET

#include <GLdraw/Widget.h>
#include "Modeling/World.h"

class WorldDragWidget : public Widget
{
public:
  WorldDragWidget(RobotWorld* world=NULL);
  virtual ~WorldDragWidget() {}
  void Set(RobotWorld* world);
  void Enable(bool active);
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void EndDrag();
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void DrawGL(Camera::Viewport& viewport);

  RobotWorld* world;
  bool active;
  bool robotsActive,objectsActive,terrainsActive;
  GLColor highlightColor,lineColor;
  float lineWidth;
  bool dragging;
  int hoverID;
  Vector3 hoverPt;
  Vector3 dragPt;
  Real hoverDistance;
};

#endif
