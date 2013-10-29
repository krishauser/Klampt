#ifndef VIEW_IK_H
#define VIEW_IK_H

#include <robotics/IK.h>
#include "ViewRobot.h"

struct ViewIKGoal
{
  ViewIKGoal();
  //draws lines to the desired location
  void Draw(const IKGoal& goal,Robot& robot);  
  //draws the link of the robot at the desired location
  void DrawLink(const IKGoal& goal,ViewRobot& robotviewer);
  void DrawLink(const IKGoal& goal,ViewRobot& robotviewer,const Matrix3& refMatrix);

  GLColor lineColor;
  GLColor linkColor;
  Real widgetSize;
};

#endif
