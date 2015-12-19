#ifndef VIEW_TEXTURIZER_H
#define VIEW_TEXTURIZER_H

#include "Modeling/ManagedGeometry.h"
#include <GLdraw/GeometryAppearance.h>
#include <string>

/** @ingroup View 
 * @brief Applies a texture to some object.
 */
struct Texturizer
{
  enum { XYTexCoords, ZTexCoord, ParameterizedTexCoord };

  Texturizer();
  bool Set(ManagedGeometry& geom);

  std::string texture;
  int texCoords, texDivs;
  bool texCoordAutoScale;
};


#endif
