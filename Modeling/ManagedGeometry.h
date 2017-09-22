#ifndef MODELING_MANAGED_GEOMETRY_H
#define MODELING_MANAGED_GEOMETRY_H

#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <map>
#include <string>

class GeometryManager;

/** @brief A "smart" geometry loading class that caches previous geometries,
 * and does not re-load or re-initialize existing collision detection data
 * structures if the item has already been loaded.
 *
 * Note: the standard Load function will load a file if it hasn't been loaded
 * before, or if it has, it will copy the existing data structures from
 * a prior loaded value.  If the collision data structure of the prior
 * geometry haven't been created, they will be created and copied to the new
 * geometry.
 *
 * Note: if you wish to transform a geometry by a non-rigid transformation, 
 * you will need to call RemoveFromCache.  Otherwise, all subsequent
 * Load calls will load the transformed geometry.  Here, the TransformGeometry
 * method of this class does this for you.
 *
 * Note: geometries are not shared, but rather cached-and-copied.  Appearances
 * on the other hand are by default shared. To make an object have its own
 * custom appearance, call SetUniqueAppearance().
 */
class ManagedGeometry
{
 public:
  typedef SmartPointer<Geometry::AnyCollisionGeometry3D> GeometryPtr;
  typedef SmartPointer<GLDraw::GeometryAppearance> AppearancePtr;

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
  ///Adds to cache, if not already in it
  void AddToCache(const std::string& filename);
  ///Returns the filename to which this object is cached
  const std::string& CachedFilename() const;
  ///Remove self from cache, if in it
  void RemoveFromCache();
  ///Makes this item have its own geometry data separate from all other
  ///instances of this object, and if there are other instances removes
  ///this item from the cache.
  void SetUnique();
  ///Transforms the geometry (requires removing from cache, and
  ///re-initializing collision data). 
  void TransformGeometry(const Math3D::Matrix4& xform);
  ///Returns the shared appearance data
  AppearancePtr Appearance() const;
  ///Returns true if there are multiple objects sharing the appearance data.
  ///If it is shared, then changing one appearance affects multiple objects.
  bool IsAppearanceShared() const;
  ///Makes this item have its own appearance data separate from all other
  ///instances of this object.
  void SetUniqueAppearance();
  ///If the geometry is changed, call this to update the appearance
  void OnGeometryChange();
  ///Renders the object using OpenGL
  void DrawGL();
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

#endif
