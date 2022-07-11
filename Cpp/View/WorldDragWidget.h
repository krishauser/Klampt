#ifndef WORLD_DRAG_WIDGET
#define WORLD_DRAG_WIDGET

#include <KrisLibrary/GLdraw/Widget.h>
#include <Klampt/Modeling/World.h>

namespace Klampt {

class WorldDragWidget : public GLDraw::Widget
{
public:
  WorldDragWidget(WorldModel* world=NULL);
  virtual ~WorldDragWidget() {}
  void Set(WorldModel* world);
  void Enable(bool active);
  virtual void SetHighlight(bool value) override;
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual void EndDrag() override;
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) override;
  virtual void DrawGL(Camera::Viewport& viewport) override;

  WorldModel* world;
  bool active;
  bool robotsActive,objectsActive,terrainsActive;
  GLDraw::GLColor highlightColor,lineColor;
  float lineWidth;
  bool dragging;
  int hoverID;
  Vector3 hoverPt;
  Vector3 dragPt;
  Real hoverDistance;

  int highlightID;
  GLDraw::GLColor originalFaceColor;
};

} // namespace Klampt;

#endif
