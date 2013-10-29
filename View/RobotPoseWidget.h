#ifndef ROBOT_POSE_WIDGET_H
#define ROBOT_POSE_WIDGET_H

#include <GLdraw/Widget.h>
#include <GLdraw/TransformWidget.h>
#include <robotics/IK.h>
#include "ViewRobot.h"
using namespace GLDraw;

/* @defgroup View
 * @brief Definitions of OpenGL drawing routines and UI widgets.
 */

/** @file View/RobotPoseWidget.h
 * @ingroup View
 * @brief Common robot posing routines
 */

/** @addtogroup View */
/** @{ */

/** @brief A widget that allows the robot's driven links to be posed.
 */
class RobotLinkPoseWidget : public Widget
{
public:
  RobotLinkPoseWidget();
  RobotLinkPoseWidget(Robot* robot,ViewRobot* viewRobot);
  virtual ~RobotLinkPoseWidget() {}
  void Set(Robot* robot,ViewRobot* viewRobot);
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void DrawGL(Camera::Viewport& viewport);

  Robot* robot;
  ViewRobot* viewRobot;
  Config poseConfig;
  GLColor highlightColor;
  int hoverLink,hoverDriver;
  Vector3 hoverPt;
};

/** @brief A widget that allows creating and editing IK constraints
 */
class RobotIKPoseWidget : public WidgetSet
{
public:
  RobotIKPoseWidget(Robot* robot);
  ///Clears all constraints on the given link
  void ClearLink(int link);
  ///Fixes the given point on the link at its current location
  void FixPoint(int link,const Vector3& localpt);
  ///Fixes the given link's position and orientation at its current pose
  void FixLink(int link);
  ///Adds a new constraint
  void Add(const IKGoal& goal);
  ///Sets the destination link of the given widget
  void AttachWidget(int widget,int link);
  ///Call this after changing the poseGoals and poseWidgets structure.
  void RefreshWidgets();
  ///Returns the index of the hovered widget
  int HoverWidget() const;
  ///Returns the index of the active widget
  int ActiveWidget() const;
  ///Clears the constraint on the currently active widget
  bool ClearCurrent();
  virtual void DrawGL(Camera::Viewport& viewport);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);

  //these are overloaded to allow IK widgets to shine through
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& closestDistance);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& closestDistance);

  Robot* robot;
  vector<IKGoal> poseGoals;
  vector<TransformWidget> poseWidgets;
};

/** A widget that allows full posing and editing of the robot config including
 * IK constraints and base motion.
 */
class RobotPoseWidget : public WidgetSet
{
public:
  RobotPoseWidget();
  RobotPoseWidget(Robot* robot,ViewRobot* viewRobot);
  void Set(Robot* robot,ViewRobot* viewRobot);

  const Config& Pose() const { return linkPoser.poseConfig; }
  void SetPose(const Config& q);
  vector<IKGoal>& Constraints() { return ikPoser.poseGoals; }
  ///Adds a pos/rot constraint on the currently hovered link
  bool FixCurrent();
  ///Adds a point constraint on the currently hovered link
  bool FixCurrentPoint();
  ///Deletes the currently hovered constraint or all constraints on the currently hovered link
  bool DeleteConstraint();
  ///Turn attach mode on/off
  bool ToggleAttach();
  ///Turn IK posing on/off
  bool TogglePoseIK();

  virtual void DrawGL(Camera::Viewport& viewport);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void EndDrag();
  virtual void Keypress(char c);

  bool useBase;
  TransformWidget basePoser;
  RobotLinkPoseWidget linkPoser;
  RobotIKPoseWidget ikPoser;
  bool poseIKMode;
  bool attachIKMode;
  int attachx,attachy;
  Ray3D attachRay;
};

/** @} */

#endif
