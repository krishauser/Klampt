#ifndef _GEOMETRY_H
#define _GEOMETRY_H

/** @file geometry.h
 * @brief C++ bindings for geometry modeling. */

/** @brief A 3D indexed triangle mesh class.
 *
 * vertices is a list of vertices, given as a list [x1, y1, z1, x2, y2, ...]
 * indices is a list of triangle vertices given as indices into the
 *    vertices list, i.e., [a1,b1,c2, a2,b2,c2, ...]
 */
struct TriangleMesh
{
  void translate(const double t[3]);
  void transform(const double R[9],const double t[3]);

  std::vector<int> indices;
  std::vector<double> vertices;
};

/** @brief A 3D point cloud class.  
 * vertices is a list of vertices, given as a list [x1, y1, z1, x2, y2, ... zn]
 * properties is a list of vertex properties, given as a list
 * [p11, p21, ..., pk1,  p12, p22, ..., pk2, ... , pn1, pn2, ..., pn2]
 * where each vertex has k properties.  The name of each property is given
 * by the propertyNames member.
 */
struct PointCloud
{
  void translate(const double t[3]);
  void transform(const double R[9],const double t[3]);

  std::vector<double> vertices;
  std::vector<std::string> propertyNames;
  std::vector<double> properties;
};

struct GeometricPrimitive
{
  void setPoint(const double pt[3]);
  void setSphere(const double c[3],double r);
  void setSegment(const double a[3],const double b[3]);
  void setAABB(const double bmin[3],const double bmax[3]);
  bool loadString(const char* str);
  std::string saveString() const;

  std::string type;
  std::vector<double> properties;
};

/** @brief A three-D geometry.  Can either be a reference to a
 * world item's geometry, in which case modifiers change the 
 * world item's geometry, or it can be a standalone geometry.
 *
 * Modifiers include any setX() functions, translate(), and transform().
 *
 * Proximity queries include collides(), withinDistance(), distance(), and
 * rayCast().
 */
class Geometry3D
{
 public:
  Geometry3D();
  ~Geometry3D();
  ///Creates a standalone geometry from this geometry
  Geometry3D clone();
  ///Copies the geometry of the argument into this geometry.
  void set(const Geometry3D&);
  ///Returns true if this is a standalone geometry
  bool isStandalone();
  ///Frees the data associated with this geometry, if standalone 
  void free();
  ///Returns the type of geometry: TriangleMesh, PointCloud, or
  ///GeometricPrimitive
  std::string type();
  ///Returns a TriangleMesh if this geometry is of type TriangleMesh
  TriangleMesh getTriangleMesh();
  ///Returns a PointCloud if this geometry is of type PointCloud
  PointCloud getPointCloud();
  ///Returns a GeometricPrimitive if this geometry is of type GeometricPrimitive
  GeometricPrimitive getGeometricPrimitive();
  void setTriangleMesh(const TriangleMesh&);
  void setPointCloud(const PointCloud&);
  void setGeometricPrimitive(const GeometricPrimitive&);
  bool loadFile(const char* fn);
  bool saveFile(const char* fn);
  ///Sets the current transformation (not modifying the underlying data)
  void setCurrentTransform(const double R[9],const double t[3]);
  ///Translates the geometry data 
  void translate(const double t[3]);
  ///Translates/rotates the geometry data 
  void transform(const double R[9],const double t[3]);
  void setCollisionMargin(double margin);
  double getCollisionMargin();
  void getBB(double out[3],double out2[3]);
  bool collides(const Geometry3D& other);
  bool withinDistance(const Geometry3D& other,double tol);
  double distance(const Geometry3D& other,double relErr=0,double absErr=0);
  bool rayCast(const double s[3],const double d[3],double out[3]);

  int world;
  int id;
  void* geomPtr;
};

#endif
