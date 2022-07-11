#ifndef VIEW_TEXTURES_H
#define VIEW_TEXTURES_H

#include <KrisLibrary/GLdraw/GLTextureObject.h>
#include <KrisLibrary/image/image.h>
#include <map>
#include <string>

namespace Klampt {
    using namespace std;

class ViewTextures
{
 public:
  static void Initialize(bool force=false);
  static void InitializeGL(bool force=false);
  static shared_ptr<Image> Load(const char* fn);
  static float GradientTexcoord(float u,float min,float max);

  static map<string,shared_ptr<Image> > images;
  static map<string,GLDraw::GLTextureObject> textureObjects;
};

} //namespace Klampt

#endif
