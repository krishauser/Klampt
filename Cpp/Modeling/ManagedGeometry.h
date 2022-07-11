#ifndef KLAMPT_MODELING_MANAGED_GEOMETRY_H
#define KLAMPT_MODELING_MANAGED_GEOMETRY_H

#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <memory>
#include <map>
#include <string>

namespace Klampt {

class GeometryManager;

/** @ingroup Modeling
 * @brief A "smart" geometry loading class that caches previous geometries and 
 * maintains shared collision detection / appearance information.  This greatly 
 * speeds up loading time if multiple instances of the same geometry are loaded
 * from disk, because multiple ManagedGeometry objects can share the same
 * underlying data.  It can also read from dynamic geometry sources (ROS point
 * cloud topics, for now).
 *
 * Shared geometries: The Load function will load a file normally if it hasn't
 * been loaded before. But if it has, this geometry will simply refer to the
 * existing geometry, collision detection data structures, and Appearance
 * information.  This leads to major speed ups in running time and lower memory
 * consumption when multiple objects of the same geometry are loaded.
 *
 * Caching can have some weird effects. If you change the geometry / appearance
 * of a shared ManagedGeometry, then all of the other ManagedGeometrys' will
 * receive the same changes. To avoid this, you can either
 * 1) Call LoadNoCache or RemoveFromCache before the subsequent geometry is loaded, 
 *    which avoids sharing geometries in the first place.
 * 2) Call SetUnique after loading, which entirely breaks the connection between 
 *    the object and the other shared geometries.
 * 3) Call SetUniqueAppearance, which breaks the connection between the object's
 *    appearance and the appearances of other shared geometries.  Note that this
 *    does not break the connection between geometries.
 *
 * Transforming geometries:
 * - The TransformGeometry method automatically does the intended thing, which is
 *   to break the connection between this geometry and shared geometries. Moreover,
 *   if a prior geometry was transformed by the same amount, this geometry will be
 *   shared with the prior call.
 *   a.Load("file.obj");
 *   b.Load("file.obj");   //b is now shared with a
 *   a.TransformGeometry(T);  //this breaks the connection between a and b
 *   b.TransformGeometry(T);  //now a and b have shared geometries again.  But, their appearances are not shared.
 *
 * Note: the Load / LoadNoCache functions can also accept non-file strings.
 *   - Dynamic geometries can be loaded from ROS topics. The string takes the form
 *     "ros://[ROS_TOPIC]" or "ros:PointCloud2//[ROS_TOPIC]". Only PointCloud2
 *     messages are accepted for now.
 *   - Inline strings can be parsed using a string of the form {DATA} where DATA
 *     is a parseable geometry format, e.g.,
 * 
 *       "{GeometricPrimitive
 *       Sphere
 *       0 0 0  1}"
 * 
 *      would load a sphere with center 0,0,0 and radius 1
 */
class ManagedGeometry
{
 public:
  typedef std::shared_ptr<Geometry::AnyCollisionGeometry3D> GeometryPtr;
  typedef std::shared_ptr<GLDraw::GeometryAppearance> AppearancePtr;

  ManagedGeometry();
  ///Copy constructor is a shallow copy
  ManagedGeometry(const ManagedGeometry& rhs);
  ~ManagedGeometry();
  ///Erases the items in this geometry
  void Clear();
  ///Returns true if the pointer is NULL or the geometry itself is empty
  bool Empty() const;
  ///Creates an empty geometry
  GeometryPtr CreateEmpty();
  ///Loads a geometry, with caching
  bool Load(const std::string& filename);
  ///Loads a geometry, without caching
  bool LoadNoCache(const std::string& filename);
  ///Returns NULL if the file hasn't been cached.  Otherwise, returns
  ///a prior instance of the geometry.
  static ManagedGeometry* IsCached(const std::string& filename);
  ///Returns true if this instance is cached.
  bool IsCached() const;
  ///Returns true if this instance is cached and is identical to a file on disk
  bool IsOriginal() const;
  ///Adds to cache, if not already in it
  void AddToCache(const std::string& filename);
  ///Returns the filename to which this object is cached
  const std::string& CachedFilename() const;
  ///Remove self from cache, if in it
  void RemoveFromCache();
  ///Transforms the geometry (requires removing from cache, and
  ///re-initializing collision data). 
  void TransformGeometry(const Math3D::Matrix4& xform);
  ///Returns the shared appearance data
  AppearancePtr Appearance() const;
  ///Returns true if there are multiple objects sharing the appearance data.
  ///If it is shared, then changing one appearance affects multiple objects.
  bool IsAppearanceShared() const;
  ///Removes this geometry from the cache and separates its appearance data
  ///from all other instances of this object.
  void SetUnique();
  ///Makes this item have its own appearance data separate from all other
  ///instances of this object.
  void SetUniqueAppearance();
  ///If the geometry is changed, call this to update the appearance
  void OnGeometryChange();
  ///Renders the object using OpenGL
  void DrawGL();
  ///Renders the opaque parts of the object using OpenGL
  void DrawGLOpaque(bool opaque);
  ///Returns true if this geometry is connected to a dynamic source
  bool IsDynamicGeometry() const;
  ///Updates dynamic geometry, if an update is available.  If no update,
  ///returns false
  bool DynamicGeometryUpdate();

  ///assignment is a shallow copy
  const ManagedGeometry& operator = (const ManagedGeometry& rhs);
  operator bool() const { return geometry != NULL; }
  bool operator !() const { return geometry == NULL; }
  ///Dereferencing by cast, pointer access ->, or *
  operator GeometryPtr() { return geometry; }
  operator GeometryPtr() const { return geometry; }
  GeometryPtr operator ->() { return geometry; }
  const GeometryPtr operator ->() const { return geometry; }
  inline Geometry::AnyCollisionGeometry3D& operator *() { return *geometry; }
  inline const Geometry::AnyCollisionGeometry3D& operator *() const { return *geometry; }

  friend class GeometryManager;
  static GeometryManager manager;

 private:
  std::string cacheKey,dynamicGeometrySource;
  GeometryPtr geometry;
  AppearancePtr appearance;
};

class GeometryManager
{
public:
  GeometryManager();
  ~GeometryManager();
  void Clear();
  
  friend class ManagedGeometry;
  struct GeometryList
  {
    std::vector<ManagedGeometry*> geoms;
  };
  std::map<std::string,GeometryList> cache;
};

} //namespace Klampt

#endif
