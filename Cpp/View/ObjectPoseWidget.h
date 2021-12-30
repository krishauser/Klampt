#ifndef OBJECT_POSE_WIDGET_H
#define OBJECT_POSE_WIDGET_H

#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/GLdraw/TransformWidget.h>
#include <KrisLibrary/robotics/IK.h>
#include <Klampt/Modeling/RigidObject.h>

namespace Klampt {

/** @addtogroup View */
/** @{ */

/** A widget that allows full posing and editing of the robot config including
 * IK constraints and base motion.
 */
class RigidObjectPoseWidget : public GLDraw::WidgetSet
{
public:
  RigidObjectPoseWidget();
  RigidObjectPoseWidget(RigidObjectModel* object);
  void Set(RigidObjectModel* object);

  const RigidTransform& Pose() const { return poser.T; }
  void SetPose(const RigidTransform& q);

  virtual void DrawGL(Camera::Viewport& viewport) override;
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance) override;
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport) override;
  virtual void EndDrag() override;
  virtual void Keypress(char c) override;

  RigidObjectModel* rigidObject;
  GLDraw::TransformWidget poser;
};

/** @} */

} //namespace Klampt

#endif
