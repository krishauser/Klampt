#ifndef ROBOT_POSE_WIDGET_H
#define ROBOT_POSE_WIDGET_H

#include <KrisLibrary/GLdraw/Widget.h>
#include <KrisLibrary/GLdraw/TransformWidget.h>
#include <KrisLibrary/robotics/IK.h>
#include "ViewRobot.h"

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
class RobotLinkPoseWidget : public GLDraw::Widget
{
public:
  RobotLinkPoseWidget();
  RobotLinkPoseWidget(Robot* robot,ViewRobot* viewRobot);
  virtual ~RobotLinkPoseWidget() {}
  ///Initializer
  void Set(Robot* robot,ViewRobot* viewRobot);
  ///Sets the active dofs
  void SetActiveDofs(const vector<int>& activeDofs);
  virtual bool Hover(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void SetHighlight(bool active);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void DrawGL(Camera::Viewport& viewport);
  void InitDefaultAppearance();

  Robot* robot;
  ViewRobot* viewRobot;
  Config poseConfig;
  GLDraw::GLColor highlightColor;
  int hoverLink,affectedLink,affectedDriver;
  vector<int> activeDofs;
  vector<int> highlightedLinks;
  Vector3 hoverPt;
  bool draw;
  vector<GLDraw::GeometryAppearance> poserAppearance;
};

/** @brief A widget that allows creating and editing IK constraints
 */
class RobotIKPoseWidget : public GLDraw::WidgetSet
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
  ///Sets the transform of the pose goal and its widget 
  void SetPoseAndWidgetTransform(int widget,const RigidTransform& T);
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
  vector<GLDraw::TransformWidget> poseWidgets;
};

/** A widget that allows full posing and editing of the robot config including
 * IK constraints and base motion.
 */
class RobotPoseWidget : public GLDraw::WidgetSet
{
public:
  RobotPoseWidget();
  RobotPoseWidget(Robot* robot,ViewRobot* viewRobot);
  void Set(Robot* robot,ViewRobot* viewRobot);

  const Config& Pose() const { return linkPoser.poseConfig; }
  void SetPose(const Config& q);
  vector<IKGoal>& Constraints() { return ikPoser.poseGoals; }
  ///if there are multiple solutions for the current pose, picks the one that
  ///most closely matches qref (useful for spin joints)
  Config Pose_Conditioned(const Config& qref) const;
  ///Enables / disables editing certain joints
  void SetActiveDofs(const vector<int>& activeDofs);
  ///Adds a pos/rot constraint on the currently hovered link
  bool FixCurrent();
  ///Adds a point constraint on the currently hovered link
  bool FixCurrentPoint();
  ///Deletes the currently hovered constraint or all constraints on the currently hovered link
  bool DeleteConstraint();
  ///Turn attach IK mode on/off
  void SetAttachIKMode(bool);
  ///Turn point IK posing on/off
  void SetPoseIKMode(bool);
  ///Turn fixed IK posing on/off
  void SetFixedPoseIKMode(bool);
  ///Turn delete IK widgets on/off
  void SetDeleteIKMode(bool);

  ///Solves the current IK problem (by default uses 100 iters, tolerance 0.001)
  bool SolveIK(int iters=0,Real tol=0);
  ///Solves the current IK problem with a fixed base
  bool SolveIKFixedBase(int iters=0,Real tol=0);
  ///Solves the current IK problem with a joint fixed in place
  bool SolveIKFixedJoint(int fixedJoint,int iters=0,Real tol=0);

  virtual void DrawGL(Camera::Viewport& viewport);
  virtual bool BeginDrag(int x,int y,Camera::Viewport& viewport,double& distance);
  virtual void Drag(int dx,int dy,Camera::Viewport& viewport);
  virtual void EndDrag();
  virtual void Keypress(char c);

  void Snapshot();
  void Undo();

  bool useBase;
  GLDraw::TransformWidget basePoser;
  RobotLinkPoseWidget linkPoser;
  RobotIKPoseWidget ikPoser;
  enum { ModeNormal, ModeIKAttach, ModeIKPose, ModeIKPoseFixed, ModeIKDelete };
  int mode;
  int attachx,attachy;
  Ray3D attachRay;
  vector<Config> undoConfigs;
  vector<vector<pair<int,RigidTransform> > > undoTransforms;
};

/** @} */

#endif
