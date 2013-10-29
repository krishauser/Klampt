#ifndef VIEW_STANCE_H
#define VIEW_STANCE_H

#include "Contact/Stance.h"
#include <robotics/Stability.h>
#include <GLdraw/GLColor.h>
#include "ViewHold.h"

/** @ingroup View 
 * @brief Displays a support polygon using OpenGL.
 *
 * Draws a horizontal polygon at height h.
 */
struct ViewPolytope
{
  ViewPolytope();
  void Draw();
  void Draw(const Geometry::UnboundedPolytope2D& _poly) { poly=&_poly; Draw(); }

  const Geometry::UnboundedPolytope2D* poly;
  GLColor color;
  bool wireframe;
  Real h;
};

/** @ingroup View 
 * @brief Displays a stance using OpenGL.
 */
struct ViewStance
{
  ViewStance();
  void DrawHolds();
  void DrawForces(const Vector& f,Real scale=One);
  void DrawHolds(const Stance& _s)
  { s=&_s; DrawHolds(); }
  void DrawForces(const Stance& _s,const Vector& f,Real scale=One)
  { s=&_s; DrawForces(f,scale); }

  const Stance* s;
  ViewHold viewHold;
  GLColor forceColor;
};



#endif
