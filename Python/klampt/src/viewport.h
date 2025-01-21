#ifndef _VIEWPORT
#define _VIEWPORT

#include <string>
#include <vector>

/** @file viewport.h 
 * API to interact with with C++ Viewports
 */

/** @brief A class that represents an idealized pinhole camera.
 * 
 * Duplicates the functioning of
 * KrisLibrary/Camera/Viewport.  
 */
class Viewport
{
 public:
  Viewport();
  bool fromJson(const std::string& str);
  std::string toJson() const;
  bool fromText(const std::string& str);
  std::string toText() const;

  /// Resizes the viewport, keeping the same field of view and relative position of the focal point 
  void resize(int w, int h);
  /// Sets the horizontal and optionally the vertical FOV.  If yfov < 0, square pixels will be assumed 
  void setFOV(double xfov, double yfov=-1);
  /// Returns the horizontal FOV. 
  double getFOV() const;
  /// Returns the vertical FOV. 
  double getVFOV() const;
  /// Sets the pose of the camera
  void setPose(const double R[9],const double t[3]);
  /// Gets the pose of the camera
  ///
  ///Return type: RigidTransform
  void getPose(double out[9],double out2[3]) const;
  ///Gets the viewing rectangle (xmin,ymin,xmax,ymax) at a given depth
  ///
  ///Return type: Tuple[float,float,float,float]
  void viewRectangle(double depth, double out[4]) const;
  ///Projects into image coordinates
  ///
  ///Return type: Vector3
  void project(const double pt[3],double out[3]) const;
  /// Provides the source of a ray for an image coordinate (x,y) 
  ///
  ///Return type: Vector3
  void clickSource(double x, double y, double out[3]) const;
  /// Provides the direction of a ray for an image coordinate (x,y)
  ///
  ///Return type: Vector
  void clickDirection(double x, double y, double out[3]) const;

  bool perspective;
  
  int x,y,w,h;    ///< box containing window
  double n, f;    ///< near and far z clipping plans
  double fx, fy;  ///< Focal length in pixels
  double cx, cy;  ///< Principal point relative to (x,y) in pixels

  std::vector<double> xform;  ///< column major 4x4 camera pose transform
  std::string ori;            ///< the camera convention being used. Values include 'opengl', 'opencv', 'ros' (='opencv')
};

#endif // _VIEWPORT