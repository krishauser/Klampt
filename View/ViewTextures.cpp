#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "ViewTextures.h"
#include <KrisLibrary/GLdraw/GLTexture1D.h>
#include <KrisLibrary/GLdraw/GLTexture2D.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/image/import.h>
#include <KrisLibrary/math/infnan.h>
using namespace Math;
using namespace GLDraw;

float startValidColors = 0;
bool texture_set_up=false;
bool gl_texture_set_up=false;

#define FLOAT_TO_UCHAR(x) (unsigned char)(x*255.0)


map<string,SmartPointer<Image> > ViewTextures::images;
map<string,GLTextureObject> ViewTextures::textureObjects;

SmartPointer<Image> ViewTextures::Load(const char* fn)
{
  if(images.count(fn) > 0) {
    return images[fn];
  }
  else {
    SmartPointer<Image> img = new Image;
    if(ImportImage(fn,*img)) {
      images[fn] = img;
      return img;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"  Failed.\n");
      return NULL;
    }
  }
}

void ViewTextures::InitializeGL(bool force) 
{
  Initialize(force);
  if(gl_texture_set_up && !force) return;
  FatalError("Not done yet");
}

void ViewTextures::Initialize(bool force) 
{
  if(!force && texture_set_up) return;
  texture_set_up=true;

  const static int gradientSize=256;
  startValidColors = 3.0 / float(gradientSize);

  Image* noise = new Image;
  Image* checker = new Image;
  Image* grayscaleGradient = new Image;
  Image* rainbowGradient = new Image;
  Image* rainbowGradientWithHashmarks = new Image;

  noise->initialize(64,64,Image::A8);
  //noise
  for(int i=0;i<64*64;i++) {
    Real ofs = RandGaussian(0,0.2);
    Real u = 0.5 + Sign(ofs)*Sqr(ofs)*0.3;
    noise->data[i] = (unsigned char)(255.0*Clamp(u,0,1));
  }

  //checkerboard
  checker->initialize(64,64,Image::A8);
  for(int i=0;i<64;i++)
    for(int j=0;j<64;j++) {
      //black and white
      //if((i<=32) ^ (j<=32))  buf[i*64+j]=0;
      //else buf[i*64+j] = 0xff;
      //grey and dark grey
      if((i<=32) ^ (j<=32))  checker->data[i*64+j]=0x80;
      else checker->data[i*64+j] = 0xa0;
    }
 
  //gradient
  grayscaleGradient->initialize(64,1,Image::A8);
  for(int i=0;i<64;i++)
    grayscaleGradient->data[i] = (i<<2);

  rainbowGradient->initialize(gradientSize,1,Image::R8G8B8);
  rainbowGradient->data[0*3] = FLOAT_TO_UCHAR(1.0);
  rainbowGradient->data[0*3+1] = FLOAT_TO_UCHAR(0.0);
  rainbowGradient->data[0*3+2] = FLOAT_TO_UCHAR(1.0);
  rainbowGradient->data[1*3] = FLOAT_TO_UCHAR(1.0);
  rainbowGradient->data[1*3+1] = FLOAT_TO_UCHAR(0.0);
  rainbowGradient->data[1*3+2] = FLOAT_TO_UCHAR(1.0);
  rainbowGradient->data[2*3] = FLOAT_TO_UCHAR(1.0);
  rainbowGradient->data[2*3+1] = FLOAT_TO_UCHAR(0.0);
  rainbowGradient->data[2*3+2] = FLOAT_TO_UCHAR(0.0);
  for(int i=3;i<gradientSize;i++) {
    GLColor col; col.setHSV(float(i-3)/float(gradientSize-3)*240.0,1,1);
    rainbowGradient->data[i*3] = FLOAT_TO_UCHAR(col.rgba[0]);
    rainbowGradient->data[i*3+1] = FLOAT_TO_UCHAR(col.rgba[1]);
    rainbowGradient->data[i*3+2] = FLOAT_TO_UCHAR(col.rgba[2]);
  }

  *rainbowGradientWithHashmarks = *rainbowGradient;
  for(int i=3;i<gradientSize;i++) {
    GLColor col; col.setHSV(float(i-3)/float(gradientSize-3)*240.0,1,(i%16==0?0.5:1.0));
    if(i == 254 || i==252 || i==250) col.scale(col,0.5);
    rainbowGradientWithHashmarks->data[i*3] = FLOAT_TO_UCHAR(col.rgba[0]);
    rainbowGradientWithHashmarks->data[i*3+1] = FLOAT_TO_UCHAR(col.rgba[1]);
    rainbowGradientWithHashmarks->data[i*3+2] = FLOAT_TO_UCHAR(col.rgba[2]);
  }
  images["noise"] = noise;
  images["checker"] = checker;
  images["gradient"] = grayscaleGradient;
  images["colorgradient"] = rainbowGradient;
  images["colorgradient_hashmarks"] = rainbowGradientWithHashmarks;
}

float ViewTextures::GradientTexcoord(float u,float min,float max)
{
  if(u < min || u > max) return 0;
  if(!IsFinite(u)) return 0;
  if(min == max) return 0;
  return (u-min)/(max-min);
}
