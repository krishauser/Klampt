#ifndef _APPEARANCE_H
#define _APPEARANCE_H

/** @file appearance.h
 * @brief C++ bindings for appearance modeling. */


#include "geometry.h"

/** @brief Geometry appearance information.  Supports vertex/edge/face
 * rendering, per-vertex color, and basic color texture mapping.  Uses
 * OpenGL display lists, so repeated calls are fast.
 *
 * For more complex appearances, you will need to call your own OpenGL calls.
 */
class Appearance
{
 public:
  ///Primitive types
  enum { ALL=0,VERTICES=1,EDGES=2,FACES=3};

  Appearance();
  ~Appearance();
  ///call this to rebuild internal buffers, e.g., when the OpenGL context
  ///changes
  void refresh();
  ///Creates a standalone appearance from this appearance
  Appearance clone();
  ///Copies the appearance of the argument into this appearance
  void set(const Appearance&);
  ///Returns true if this is a standalone appearance
  bool isStandalone();
  ///Frees the data associated with this appearance, if standalone 
  void free();
  ///Turns on/off visibility of the object
  void setDraw(bool draw);
  ///Turns on/off visibility of the given primitive
  void setDraw(int primitive,bool draw);
  ///Returns whether this object is visible
  bool getDraw();
  ///Returns whether this primitive is visible
  bool getDraw(int primitive);
  ///Sets color of the object
  void setColor(float r,float g, float b,float a);
  ///Sets color of the given primitive
  void setColor(int primitive,float r,float g, float b,float a);
  void getColor(float out[4]);
  void getColor(int primitive,float out[4]);
  ///Sets per-element color for elements of the given primitive type.
  ///If alpha=True, colors are assumed to be rgba values
  ///Otherwise they are assumed to be rgb values
  void setColors(int primitive,const std::vector<float>& colors,bool alpha=false);
  ///Sets a 1D texture of the given width.  Valid format strings are
  /// - "": turn off texture mapping
  /// - rgb8: unsigned byte RGB colors with red in the most significant byte
  /// - argb8: unsigned byte RGBA colors with alpha in the most significant
  ///          byte
  /// - l8: unsigned byte grayscale colors
  void setTexture1D(int w,const char* format,const std::vector<unsigned char>& bytes);
  ///Sets a 2D texture of the given width/height.  See setTexture1D for 
  ///valid format strings.
  void setTexture2D(int w,int h,const char* format,const std::vector<unsigned char>& bytes);
  ///Sets per-vertex texture coordinates.  If the texture is 1D, uvs is an
  ///array of length n containing 1D texture coordinates.  If the texture is
  ///2D, uvs is an array of length 2n containing U-V coordinates u1, v1,
  ///u2, v2, ..., un, vn.  If uvs is empty, turns off texture mapping
  ///altogether.
  void setTexcoords(const std::vector<double>& uvs);
  ///For point clouds, sets the point size.
  void setPointSize(float size);
  ///Draws the currently associated geometry with this appearance.  A geometry
  ///is assocated with this appearance if this appearance comes from an
  ///element of the WorldMode, or if drawGL(geom) was previously called.
  void drawGL();
  ///Draws the given geometry with this appearance.  NOTE: for best
  ///performance, an appearance should only be drawn with a single geometry.
  ///Otherwise, the OpenGL display lists will be completely recreated
  void drawGL(Geometry3D& geom);

  int world;
  int id;
  void* appearancePtr;
};

#endif
