#include "ViewTextures.h"
#include <GLdraw/GLColor.h>
#include <math/random.h>
#include <math/infnan.h>
using namespace Math;

float startValidColors = 0;
bool texture_set_up=false;

#define FLOAT_TO_UCHAR(x) (unsigned char)(x*255.0)

GLTexture2D ViewTextures::noise;
GLTexture2D ViewTextures::checker;
GLTexture1D ViewTextures::grayscaleGradient;
GLTexture1D ViewTextures::rainbowGradient;
GLTexture1D ViewTextures::rainbowGradientWithHashmarks;


void ViewTextures::Initialize() 
{
  if(texture_set_up) return;
  texture_set_up=true;

  const static int gradientSize=256;
  unsigned char buf[64*64];
  startValidColors = 3.0 / float(gradientSize);

  //noise
  for(int i=0;i<64*64;i++) {
    Real ofs = RandGaussian(0,0.2);
    Real u = 0.5 + Sign(ofs)*Sqr(ofs)*0.3;
    buf[i] = (unsigned char)(255.0*Clamp(u,0,1));
  }
  noise.setLuminance(buf,64,64);

  //checkerboard
  for(int i=0;i<64;i++)
    for(int j=0;j<64;j++) {
      //black and white
      //if((i<=32) ^ (j<=32))  buf[i*64+j]=0;
      //else buf[i*64+j] = 0xff;
      //grey and dark grey
      if((i<=32) ^ (j<=32))  buf[i*64+j]=0x80;
      else buf[i*64+j] = 0xa0;
    }
  checker.setLuminance(buf,64,64);

  //gradient
  for(int i=0;i<64;i++)
    buf[i] = (i<<2);
  grayscaleGradient.setLuminance(buf,64);
  grayscaleGradient.setWrapClamp();

  buf[0*3] = FLOAT_TO_UCHAR(1.0);
  buf[0*3+1] = FLOAT_TO_UCHAR(0.0);
  buf[0*3+2] = FLOAT_TO_UCHAR(1.0);
  buf[1*3] = FLOAT_TO_UCHAR(1.0);
  buf[1*3+1] = FLOAT_TO_UCHAR(0.0);
  buf[1*3+2] = FLOAT_TO_UCHAR(1.0);
  buf[2*3] = FLOAT_TO_UCHAR(1.0);
  buf[2*3+1] = FLOAT_TO_UCHAR(0.0);
  buf[2*3+2] = FLOAT_TO_UCHAR(0.0);
  for(int i=3;i<gradientSize;i++) {
    GLColor col; col.setHSV(float(i-3)/float(gradientSize-3)*240.0,1,1);
    buf[i*3] = FLOAT_TO_UCHAR(col.rgba[0]);
    buf[i*3+1] = FLOAT_TO_UCHAR(col.rgba[1]);
    buf[i*3+2] = FLOAT_TO_UCHAR(col.rgba[2]);
  }
  rainbowGradient.setRGB(buf,gradientSize);
  rainbowGradient.setWrapClamp();

  for(int i=3;i<gradientSize;i++) {
    GLColor col; col.setHSV(float(i-3)/float(gradientSize-3)*240.0,1,(i%16==0?0.5:1.0));
    if(i == 254 || i==252 || i==250) col.scale(col,0.5);
    buf[i*3] = FLOAT_TO_UCHAR(col.rgba[0]);
    buf[i*3+1] = FLOAT_TO_UCHAR(col.rgba[1]);
    buf[i*3+2] = FLOAT_TO_UCHAR(col.rgba[2]);
  }
  rainbowGradientWithHashmarks.setRGB(buf,gradientSize);
  rainbowGradientWithHashmarks.setWrapClamp();
}

float ViewTextures::GradientTexcoord(float u,float min,float max)
{
  if(u < min || u > max) return 0;
  if(!IsFinite(u)) return 0;
  if(min == max) return 0;
  return (u-min)/(max-min);
}
