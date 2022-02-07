#ifndef VIEW_TEXTURIZER_H
#define VIEW_TEXTURIZER_H

#include <Klampt/Modeling/ManagedGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <string>

namespace Klampt {

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

} //namespace Klampt

#endif
