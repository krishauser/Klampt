#ifndef VIEW_TEXTURES_H
#define VIEW_TEXTURES_H

#include <GLdraw/GLTexture1D.h>
#include <GLdraw/GLTexture2D.h>

class ViewTextures
{
 public:
  static void Initialize(bool force=false);
  static float GradientTexcoord(float u,float min,float max);

  static GLDraw::GLTexture2D noise;
  static GLDraw::GLTexture2D checker;
  static GLDraw::GLTexture1D grayscaleGradient;
  static GLDraw::GLTexture1D rainbowGradient;
  static GLDraw::GLTexture1D rainbowGradientWithHashmarks;
};

#endif
