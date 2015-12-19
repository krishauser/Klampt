#ifndef VIEW_TEXTURES_H
#define VIEW_TEXTURES_H

#include <GLdraw/GLTextureObject.h>
#include <utils/SmartPointer.h>
#include <image/image.h>
#include <map>
#include <string>
using namespace std;

class ViewTextures
{
 public:
  static void Initialize(bool force=false);
  static void InitializeGL(bool force=false);
  static SmartPointer<Image> Load(const char* fn);
  static float GradientTexcoord(float u,float min,float max);

  static map<string,SmartPointer<Image> > images;
  static map<string,GLDraw::GLTextureObject> textureObjects;
};

#endif
