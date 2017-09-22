#include "ManagedGeometry.h"
#include "IO/ROS.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <string.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/stringutils.h>
using namespace Math3D;

#define CACHE_DEBUG 0

GeometryManager::GeometryManager()
{

}

GeometryManager::~GeometryManager()
{
  if(!cache.empty())  {
    fprintf(stderr,"~GeometryManager: Warning, destruction of global objects is out of order?\n");
    for(std::map<std::string,GeometryList>::iterator i=cache.begin();i!=cache.end();i++) {
      fprintf(stderr,"Destroying GeometryManager, have %d items left on name %s\n",(int)i->second.geoms.size(),i->first.c_str());
    }
  }
  Clear();
}

void GeometryManager::Clear()
{
  for(std::map<std::string,GeometryList>::iterator i=cache.begin();i!=cache.end();i++) {
    for(size_t j=0;j<i->second.geoms.size();j++)
      i->second.geoms[j]->cacheKey.clear();
  }
  cache.clear();
}


ManagedGeometry::ManagedGeometry()
{
  appearance = new GLDraw::GeometryAppearance;
}

ManagedGeometry::ManagedGeometry(const ManagedGeometry& rhs)
{
  operator = (rhs);
  //if you're not careful with the cache you can copy appearance pointers directly without any record
  if(cacheKey.empty()) 
    appearance = new GLDraw::GeometryAppearance(*appearance);
}

ManagedGeometry::~ManagedGeometry()
{
#if CACHE_DEBUG
  if(Empty()) {
    assert(appearance.getRefCount() == 1);
    assert(appearance->geom == NULL || appearance->geom == geometry);
    assert(cacheKey.empty());
  }
  else {
    printf("Destroying ManagedGeometry %s, appearance ref count %d\n",(cacheKey.empty() ? "uncached " : cacheKey.c_str()), appearance.getRefCount());
  }
#endif
  RemoveFromCache();
}

bool ManagedGeometry::Empty() const
{
  return geometry == NULL || geometry->Empty();
}

SmartPointer<Geometry::AnyCollisionGeometry3D> ManagedGeometry::CreateEmpty()
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = new Geometry::AnyCollisionGeometry3D;
  appearance = new GLDraw::GeometryAppearance;
  appearance->geom = geometry;
  return geometry;
}

void ManagedGeometry::Clear()
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = NULL;
  appearance = new GLDraw::GeometryAppearance;
}

bool ManagedGeometry::Load(const std::string& filename)
{
  //these lines are sort of like Clear(), but the appearance is kept
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = NULL;
  if(appearance) appearance->geom = NULL;
  //keep appearance

  if(0==strncmp(filename.c_str(),"ros:",4)) {
    return LoadNoCache(filename);
  }

  ManagedGeometry* prev = ManagedGeometry::IsCached(filename);
  if(prev) {
    cacheKey = filename;
    //printf("ManagedGeometry: Copying data from previously loaded file %s\n",filename.c_str());
    if(!prev->geometry->CollisionDataInitialized()) {
      Timer timer;
      prev->geometry->InitCollisionData();
      double t = timer.ElapsedTime();
      if(t > 0.2) 
	printf("ManagedGeometry: Initialized %s collision data structures in time %gs\n",filename.c_str(),t);
    }
    geometry = new Geometry::AnyCollisionGeometry3D(*prev->geometry);
    appearance = prev->appearance;
    appearance->geom = geometry;
    manager.cache[filename].geoms.push_back(this);
#if CACHE_DEBUG
    printf("ManagedGeometry: adding a duplicate of %s to cache.\n",filename.c_str());
#endif
    return true;
  }

  if(LoadNoCache(filename)) {  
#if CACHE_DEBUG
    printf("ManagedGeometry: adding %s to cache.\n",filename.c_str());
#endif
    cacheKey = filename;
    manager.cache[filename].geoms.push_back(this);
    return true;
  }
  return false;
}

bool ManagedGeometry::LoadNoCache(const std::string& filename)
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = new Geometry::AnyCollisionGeometry3D;
  if(appearance) appearance->geom = NULL;
  //keep appearance

  //load from scratch
  const char* fn = filename.c_str();
  if(0==strncmp(fn,"ros:PointCloud2//",17)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    dynamicGeometrySource = filename;
    geometry = new Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
    appearance->Set(*geometry);
    Meshing::PointCloud3D& pc = geometry->AsPointCloud();
    printf("ManagedGeometry subscribing to point cloud on ROS topic %s\n",fn+16);
    return ROSSubscribePointCloud(pc,fn+16);
  }
  else if(0==strncmp(fn,"ros://",6)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    dynamicGeometrySource = filename;
    *geometry = Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
    appearance->Set(*geometry);
    Meshing::PointCloud3D& pc = geometry->AsPointCloud();
    printf("ManagedGeometry subscribing to point cloud on ROS topic %s\n",fn+5);
    return ROSSubscribePointCloud(pc,fn+5);
  }
  
  //TODO: ROS Mesh messages?

  const char* ext=FileExtension(fn);
  if(ext) {
    if(Geometry::AnyGeometry3D::CanLoadExt(ext)) {
      Timer timer;
      geometry = new Geometry::AnyCollisionGeometry3D();
      if(!geometry->Load(fn)) {
        fprintf(stderr,"ManagedGeometry: Error loading geometry file %s\n",fn);
	geometry = NULL;
        return false;
      }
      double t = timer.ElapsedTime();
      if(t > 0.2) 
	printf("ManagedGeometry: loaded %s in time %gs\n",filename.c_str(),t);
      if(geometry->type == Geometry::AnyGeometry3D::TriangleMesh) {
	if(geometry->TriangleMeshAppearanceData() != NULL) {
	  appearance = new GLDraw::GeometryAppearance(*geometry->TriangleMeshAppearanceData());
	  appearance->Set(*geometry);
	}
	else {
	  appearance->Set(*geometry);
	}
      }
      else {
	appearance->Set(*geometry);
      }
      return true;
    }
    else {
      fprintf(stderr,"ManagedGeometry: Unknown file extension %s on file %s\n",ext,fn);
      return false;
    }
  }
  else {
    fprintf(stderr,"ManagedGeometry: No file extension on file %s\n",fn);
    return false;
  }
}

ManagedGeometry* ManagedGeometry::IsCached(const std::string& filename)
{
  std::map<std::string,GeometryManager::GeometryList>::const_iterator i=manager.cache.find(filename);
  if(i==manager.cache.end()) return NULL;
  if(i->second.geoms.empty()) return NULL;
#if CACHE_DEBUG
  printf("ManagedGeometry: retreiving %s from cache.\n",filename.c_str());
#endif
  return i->second.geoms[0];
}

bool ManagedGeometry::IsCached() const
{
  return !cacheKey.empty();
}

const std::string& ManagedGeometry::CachedFilename() const
{
  return cacheKey;
}

void ManagedGeometry::AddToCache(const std::string& filename)
{
  if(!cacheKey.empty()) {
    if(cacheKey != filename)
      printf("ManagedGeometry::AddToCache(): warning, item was previously cached as %s, now being asked to be cached as %s?\n",cacheKey.c_str(),filename.c_str());
    return;
  }
#if CACHE_DEBUG
  printf("ManagedGeometry: adding %s to cache.\n",filename.c_str());
#endif
  cacheKey = filename;
  manager.cache[cacheKey].geoms.push_back(this);
}

void ManagedGeometry::RemoveFromCache()
{
  if(cacheKey.empty()) {
    return;
  }
  std::map<std::string,GeometryManager::GeometryList>::iterator i=manager.cache.find(cacheKey);
  if(i==manager.cache.end()) {
    printf("ManagedGeometry::RemoveFromCache(): warning, item %s was not previously cached?\n",cacheKey.c_str());
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.empty()) {
    printf("ManagedGeometry::RemoveFromCache(): warning, item %s was previously deleted?\n",cacheKey.c_str());
    cacheKey.clear();
    return;
  }
  for(size_t j=0;j<i->second.geoms.size();j++) {
    if(i->second.geoms[j] == this) {
      i->second.geoms.erase(i->second.geoms.begin()+j);
      if(i->second.geoms.empty()) {
        manager.cache.erase(i);
#if CACHE_DEBUG
        printf("ManagedGeometry: removing %s from cache.\n",cacheKey.c_str());
#endif
      }
      cacheKey.clear();
      return;
    }
  }
  printf("ManagedGeometry::RemoveFromCache(): warning, item %s pointer was not previously cached?\n",cacheKey.c_str());
  cacheKey.clear();
}

void ManagedGeometry::SetUnique()
{
  if(cacheKey.empty()) return;
  SetUniqueAppearance();
  std::map<std::string,GeometryManager::GeometryList>::iterator i=manager.cache.find(cacheKey);
  if(i==manager.cache.end()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey.c_str());
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.empty()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey.c_str());
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.size() > 1) {
    //has duplicates, actually have to copy the geometry and remove this
    geometry = new Geometry::AnyCollisionGeometry3D(*geometry);
    OnGeometryChange();
    RemoveFromCache();
  }
}

void ManagedGeometry::TransformGeometry(const Math3D::Matrix4& xform)
{
  if(geometry) {
    SetUnique();
    geometry->Transform(xform);
    geometry->ClearCollisionData();
    OnGeometryChange();
  }
}

void ManagedGeometry::OnGeometryChange()
{
  //may need to refresh appearance?
  if(geometry && appearance)
     appearance->Set(*geometry);
}

ManagedGeometry::AppearancePtr ManagedGeometry::Appearance() const
{
  return appearance; 
}

bool ManagedGeometry::IsAppearanceShared() const
{ 
  if(cacheKey.empty()) return false;
  std::map<std::string,GeometryManager::GeometryList>::const_iterator i=manager.cache.find(cacheKey);
  if(i==manager.cache.end()) 
    return false;
  if(i->second.geoms.empty()) 
    return false;
  for(size_t j=0;j<i->second.geoms.size();j++) {
    if(i->second.geoms[j] != this && i->second.geoms[j]->appearance == appearance) return true;
  }
  return false;
}

void ManagedGeometry::SetUniqueAppearance()
{
  if(appearance && appearance.getRefCount() > 1) {
    appearance = new GLDraw::GeometryAppearance(*appearance);
  }
}

const ManagedGeometry& ManagedGeometry::operator = (const ManagedGeometry& rhs)
{
  RemoveFromCache();

  geometry = rhs.geometry;
  appearance = rhs.appearance;
  appearance->geom = geometry;
  cacheKey = rhs.cacheKey;
  if(!cacheKey.empty()) {
    manager.cache[cacheKey].geoms.push_back(this);
  }

  return *this;
}

void ManagedGeometry::DrawGL()
{
  if(!geometry) return;
  Assert(appearance->geom != NULL);
  if(appearance->geom == NULL)
    appearance->Set(*geometry);
  appearance->DrawGL();
}

bool ManagedGeometry::IsDynamicGeometry() const
{
  return !dynamicGeometrySource.empty();
}

bool ManagedGeometry::DynamicGeometryUpdate()
{
  if(0==strncmp(dynamicGeometrySource.c_str(),"ros://",6)) {
    if(ROSHadUpdate(dynamicGeometrySource.c_str())) {
      OnGeometryChange();
      return true;
    }
  }
  return false;
}


GeometryManager ManagedGeometry::manager;
