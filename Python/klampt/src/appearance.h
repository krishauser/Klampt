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
 *
 * Appearances can be either references to appearances of objects in the world,
 * or they can be standalone. 
 *
 * Performance note: Avoid rebuilding buffers (e.g., via :meth:`refresh`)  as 
 * much as possible.
 */
class Appearance
{
 public:
  ///Geometry feature types + EMISSIVE / SPECULAR
  enum { ALL=0,VERTICES=1,EDGES=2,FACES=3, EMISSIVE=4,SPECULAR=5};

  Appearance();
  Appearance(const Appearance& app);
  ~Appearance();
  const Appearance& operator = (const Appearance& rhs); 
  ///call this to rebuild internal buffers, e.g., when the OpenGL context
  ///changes. If deep=True, the entire data structure will be revised. Use this
  ///for streaming data, for example.
  void refresh(bool deep=true);
  ///Creates a standalone appearance from this appearance
  Appearance clone();
  ///Copies the appearance of the argument into this appearance
  void set(const Appearance&);
  ///Returns true if this is a standalone appearance
  bool isStandalone();
  ///Frees the data associated with this appearance, if standalone 
  void free();
  void setDraw(bool draw);
  ///Turns on/off visibility of the object or a feature
  ///
  ///If one argument is given, turns the object visibility on or off
  ///
  ///If two arguments are given, turns the feature (first int argument)
  ///visibility on or off.  feature can be ALL, VERTICES, EDGES, or FACES.
  void setDraw(int feature,bool draw);
  bool getDraw();
  ///Returns whether this object or feature is visible.
  ///
  ///If no arguments are given, returns whether the object is visible.
  ///
  ///If one int argument is given, returns whether the given feature
  ///is visible.  feature can be ALL, VERTICES, EDGES, or FACES.
  bool getDraw(int feature);
  void setColor(float r,float g, float b,float a=1);
  ///Sets color of the object or a feature
  ///
  ///If 3 or 4 arguments are given, changes the object color.
  ///
  ///If 5 arguments are given, changes the color of the given
  ///feature.  feature can be ALL, VERTICES, EDGES, FACES, EMISSIVE,
  ///or SPECULAR.
  void setColor(int feature,float r,float g, float b,float a);
  void getColor(float out[4]);
  ///Gets color of the object or a feature
  ///
  ///If 0 arguments are given, retrieves the main object color.
  ///
  ///If 1 arguments are given, returns the color of the given feature.
  ///feature.  feature can be ALL, VERTICES, EDGES, FACES, EMISSIVE,
  ///or SPECULAR.
  void getColor(int feature,float out[4]);
  ///Sets per-element color for elements of the given feature type.  Must be an mxn
  ///array.  m is the number of features of that type, and n is either 3 or 4.
  ///
  ///If n == 4, they are assumed to be rgba values, and 
  ///
  ///If n == 3,  each row is an rgb value.
  ///
  ///Only supports feature=VERTICES and feature=FACES
  void setColors(int feature,float* np_array2, int m, int n);
  ///Sets the specular highlight shininess and strength.  To turn off, use
  ///``setShininess(0)``.  The specular strength can be set via the second argument.
  ///``setShininess(20,0.1)``.  Note that this changes the specular color
  void setShininess(float shininess,float strength=-1);
  ///Retrieves the specular highlight shininess.
  float getShininess();
  ///Sets the per-element color for the given feature
  void setElementColor(int feature,int element,float r,float g,float b,float a=1);
  ///Gets the per-element color for the given feature
  void getElementColor(int feature,int element,float out[4]);
  ///Sets a 1D texture of the given width.  Valid format strings are
  ///
  /// - "": turn off texture mapping
  /// - l8: unsigned byte grayscale colors
  ///
  void setTexture1D_b(const char* format,unsigned char* np_array,int m);
  ///Sets a 1D texture of the given width.  Valid format strings are
  ///
  /// - "": turn off texture mapping
  /// - rgba8: unsigned byte RGBA colors with red in the 1st byte and alpha in the 4th
  /// - bgra8: unsigned byte RGBA colors with blue in the 1st byte and alpha in the 4th
  ///
  void setTexture1D_i(const char* format,unsigned int* np_array,int m);
  ///Sets a 1D texture of the given width, given a 2D array of channels.
  ///Valid format strings are
  ///
  /// - "": turn off texture mapping
  /// - rgb8: unsigned byte RGB colors with red in the 1st column, green in the 2nd, blue in the 3rd
  /// - bgr8: unsigned byte RGB colors with blue in the 1st column, green in the 2nd, green in the 3rd
  /// - rgba8: unsigned byte RGBA colors with red in the 1st column and alpha in the 4th
  /// - bgra8: unsigned byte RGBA colors with blue in the 1st column and alpha in the 4th
  /// - l8: unsigned byte grayscale colors, one channel
  ///
  void setTexture1D_channels(const char* format,unsigned char* np_array2,int m,int n);
  //note: only docs for last overload are included
  ///Sets a 2D texture of the given width/height.  See :func:`setTexture1D_b` for 
  ///valid format strings.
  ///
  ///The array is given in top to bottom order if ``topdown==True``.
  ///Otherwise, it is given in order bottom to top.
  void setTexture2D_b(const char* format,unsigned char* np_array2,int m,int n,bool topdown=true);
  ///Sets a 2D texture of the given width/height.  See :func:`setTexture1D_i` for 
  ///valid format strings.
  ///
  ///The array is given in top to bottom order if ``topdown==True``.
  ///Otherwise, it is given in order bottom to top.
  void setTexture2D_i(const char* format,unsigned int* np_array2,int m,int n,bool topdown=true);
  ///Sets a 2D texture of the given width/height from a 3D array of channels.
  ///See :func:`setTexture1D_channels` for valid format strings.
  ///
  ///The array is given in top to bottom order if ``topdown==True``.
  ///Otherwise, it is given in order bottom to top.
  void setTexture2D_channels(const char* format,unsigned char* np_array3,int m,int n,int p,bool topdown=true);
  //note: only docs for last overload are included
  ///Sets per-vertex texture coordinates for a 1D texture.
  ///
  ///You may also set uvs to be empty, which turns off texture mapping altogether.
  void setTexcoords1D(double* np_array,int m);
  ///Sets per-vertex texture coordinates for a 2D texture.  uvs is an array of
  ///shape (nx2) containing U-V coordinates [[u1, v1], [u2, v2], ..., [un, vn]]. 
  ///
  ///You may also set uvs to be empty, which turns off texture mapping altogether.
  void setTexcoords2D(double* np_array2,int m,int n);
  ///Sets the texture generation.  The array must be size m x 4, with m in the 
  ///range 0,...,4.  If worldcoordinates=true, the texture generation is 
  ///performed in world coordinates rather than object coordinates.
  void setTexgen(double* np_array2,int m,int n,bool worldcoordinates=false);
  ///Sets whether textures are to wrap (default true)
  void setTexWrap(bool wrap);
  ///For point clouds, sets the point size.
  void setPointSize(float size);
  ///For meshes, sets the crease angle.  Set to 0 to disable smoothing.
  void setCreaseAngle(float creaseAngleRads);
  ///For meshes sets a silhouette radius and color.  Set the radius to 0 to disable
  ///silhouette drawing.
  void setSilhouette(float radius,float r=0,float g=0,float b=0,float a=1);
  ///Draws the currently associated geometry with this appearance.  A geometry
  ///is assocated with this appearance if this appearance comes from an
  ///element of the WorldMode, or if drawGL(geom) was previously called.
  ///
  ///Note that the geometry's current transform is NOT respected, and this only draws
  ///the geometry in its local transform.
  void drawGL();
  ///Draws the given geometry with this appearance.  NOTE: for best
  ///performance, an appearance should only be drawn with a single geometry.
  ///Otherwise, the OpenGL display lists will be completely recreated
  ///
  ///Note that the geometry's current transform is NOT respected, and this only draws
  ///the geometry in its local transform.
  void drawGL(Geometry3D& geom);
  ///Draws the given geometry with this appearance.  NOTE: for best
  ///performance, an appearance should only be drawn with a single geometry.
  ///Otherwise, the OpenGL display lists will be completely recreated
  ///
  ///Differs from drawGL in that the geometry's current transform is applied
  ///before drawing.
  void drawWorldGL(Geometry3D& geom);

  int world;
  int id;
  void* appearancePtr;
};

#endif
