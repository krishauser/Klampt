#ifndef VIEW_GRASP_H
#define VIEW_GRASP_H

#include "Contact/Grasp.h"
#include <robotics/Stability.h>
#include <GLdraw/GLColor.h>
#include "ViewHold.h"


/** @ingroup View 
 * @brief Displays a grasp using OpenGL.
 */
struct ViewGrasp
{
  ViewGrasp();
  void Draw(const Grasp& g);

  ViewRobot* viewRobot;
  ViewContact viewContact;
  GLDraw::GLColor forceColor;
};

#endif
