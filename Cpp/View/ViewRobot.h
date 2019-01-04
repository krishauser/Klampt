#ifndef VIEW_ROBOT_H
#define VIEW_ROBOT_H

#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/GLdraw/GLDisplayList.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <Klampt/Modeling/Robot.h>

/** @ingroup View
 * @brief Draws the robot (potentially color-coded)
 */
struct ViewRobot
{
  ViewRobot(Robot* robot=NULL);
  ~ViewRobot();
  ///Draws the whole robot
  void Draw(Robot* robot);
  ///Draws the whole robot
  void Draw();
  ///draws link i's geometry in its local frame
  void DrawLink_Local(int i,bool keepAppearance=true); 
  ///draws link i's geometry in the world frame
  void DrawLink_World(int i,bool keepAppearance=true); 
  void DrawCenterOfMass(Real radius = 0.05);
  void DrawLinkCenterOfMass(int i,Real radius = 0.05);
  void DrawLinkFrames(Real size = 0.1);
  void DrawLinkSkeleton();
  void DrawTorques(const Vector& t);
  void SetColors(const GLDraw::GLColor& c);
  void SetColor(int i,const GLDraw::GLColor& c);
  void SetGrey();
  void SetTorqueColors(const Vector& t);
  ///gets the currently active appearance
  GLDraw::GeometryAppearance& Appearance(int link);
  ///pushes a new active appearance
  void PushAppearance();
  ///pops the last active appearance
  void PopAppearance();
  ///restores the base appearance
  void RestoreAppearance();
  ///easy way to save/restore appearance
  vector<GLDraw::GeometryAppearance> GetAppearance();
  ///easy way to save/restore appearance
  void SetAppearance(const vector<GLDraw::GeometryAppearance>& );

  Robot* robot;
  vector< vector<GLDraw::GeometryAppearance> > appearanceStack;
};

#endif
