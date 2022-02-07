#ifndef VIEW_IK_H
#define VIEW_IK_H

#include <KrisLibrary/robotics/IK.h>
#include "ViewRobot.h"

namespace Klampt {

struct ViewIKGoal
{
  ViewIKGoal();
  //draws lines to the desired location
  void Draw(const IKGoal& goal,RobotModel& robot);  
  //draws the link of the robot at the desired location
  void DrawLink(const IKGoal& goal,ViewRobot& robotviewer);
  void DrawLink(const IKGoal& goal,ViewRobot& robotviewer,const Matrix3& refMatrix);

  GLDraw::GLColor lineColor;
  GLDraw::GLColor linkColor;
  Real widgetSize;
};

} //namespace Klampt

#endif
