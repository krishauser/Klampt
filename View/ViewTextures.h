#ifndef VIEW_TEXTURES_H
#define VIEW_TEXTURES_H

#include <GLdraw/GLTexture1D.h>
#include <GLdraw/GLTexture2D.h>
using namespace GLDraw;

class ViewTextures
{
 public:
  static void Initialize();
  static float GradientTexcoord(float u,float min,float max);

  static GLTexture2D noise;
  static GLTexture2D checker;
  static GLTexture1D grayscaleGradient;
  static GLTexture1D rainbowGradient;
  static GLTexture1D rainbowGradientWithHashmarks;
};

#endif
