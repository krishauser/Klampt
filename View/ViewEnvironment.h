#ifndef VIEW_ENVIRONMENT_H
#define VIEW_ENVIRONMENT_H

#include "Modeling/Environment.h"
#include <GLdraw/GeometryAppearance.h>
using namespace GLDraw;

/** @ingroup View 
 * @brief Displays an Environment structure using OpenGL.
 *
 * Draws the terrain mesh and all the candidate holds.
 */
struct ViewEnvironment
{
  enum { NoTexture, NoiseTexture, CheckerTexture, GradientTexture, ColorGradientTexture };
  enum { XYTexCoords, ZTexCoord, ParameterizedTexCoord };

  ViewEnvironment();
  ~ViewEnvironment();
  void Draw(Environment* _env) { env=_env; Draw(); }
  void Draw();

  Environment* env;
  int texture, texCoords, texDivs;
  GeometryAppearance appearance;
};


#endif
