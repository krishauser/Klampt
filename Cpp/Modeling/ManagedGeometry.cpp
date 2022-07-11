#include "ManagedGeometry.h"
#include "IO/ROS.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include <string.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/ioutils.h>
using namespace Math3D;
using namespace std;

//Turn this on to debug geometry caching
#define CACHE_DEBUG 0

namespace Klampt {

//defined in XmlWorld.cpp
string ResolveFileReference(const string& path,const string& fn);
string MakeURLLocal(const string& url,const char* url_resolution_path="klampt_downloads");

void SetupDefaultAppearance(GLDraw::GeometryAppearance& app)
{
  app.creaseAngle = DtoR(30.0f);
  app.silhouetteRadius = 0.0025f;
  app.vertexSize = 3.0;
}

GeometryManager::GeometryManager()
{

}

GeometryManager::~GeometryManager()
{
  if(!cache.empty())  {
    LOG4CXX_WARN(KrisLibrary::logger(),"~GeometryManager: Warning, destruction of global objects is out of order?");
    for(map<string,GeometryList>::iterator i=cache.begin();i!=cache.end();i++) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Destroying GeometryManager, have "<<i->second.geoms.size()<<" items left on name "<<i->first);
    }
  }
  Clear();
}

void GeometryManager::Clear()
{
  for(map<string,GeometryList>::iterator i=cache.begin();i!=cache.end();i++) {
    for(size_t j=0;j<i->second.geoms.size();j++)
      i->second.geoms[j]->cacheKey.clear();
  }
  cache.clear();
}


ManagedGeometry::ManagedGeometry()
{
  appearance.reset(new GLDraw::GeometryAppearance);
  SetupDefaultAppearance(*appearance);
}

ManagedGeometry::ManagedGeometry(const ManagedGeometry& rhs)
{
  operator = (rhs);
  //if you're not careful with the cache you can copy appearance pointers directly without any record
  if(cacheKey.empty()) {
    appearance.reset(new GLDraw::GeometryAppearance(*appearance));
    SetupDefaultAppearance(*appearance);
  }
}

ManagedGeometry::~ManagedGeometry()
{
#if CACHE_DEBUG
  if(Empty()) {
    assert(appearance.use_count() == 1);
    assert(appearance->geom == NULL || appearance->geom == geometry.get());
    assert(cacheKey.empty());
  }
  else {
    printf("Destroying ManagedGeometry %s, geometry ref count %d, appearance ref count %d\n",(cacheKey.empty() ? "uncached" : cacheKey.c_str()),
      (cacheKey.empty() ? 1 : (int)manager.cache[cacheKey].geoms.size()),
      appearance.use_count());
  }
#endif
  RemoveFromCache();
}

bool ManagedGeometry::Empty() const
{
  return geometry == NULL || geometry->Empty();
}

shared_ptr<Geometry::AnyCollisionGeometry3D> ManagedGeometry::CreateEmpty()
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = make_shared<Geometry::AnyCollisionGeometry3D>();
  appearance = make_shared<GLDraw::GeometryAppearance>();
  SetupDefaultAppearance(*appearance);
  appearance->geom = geometry.get();
  return geometry;
}

void ManagedGeometry::Clear()
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = NULL;
  appearance = make_shared<GLDraw::GeometryAppearance>();
  SetupDefaultAppearance(*appearance);
}

bool ManagedGeometry::Load(const string& filename)
{
  if(filename.length() == 0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::Load: empty filename?");
    return false;
  }
  if(0==strncmp(filename.c_str(),"ros:",4)) {
    //ros topic, no caching here
    return LoadNoCache(filename);
  }
  if(filename[0] == '{') {
    //direct load, no caching here
    return LoadNoCache(filename);
  }

  ManagedGeometry* prev = ManagedGeometry::IsCached(filename);
  if(prev) {
    //these lines are sort of like Clear(), but the appearance is kept
    RemoveFromCache();
    dynamicGeometrySource.clear();
    geometry = NULL;
    if(appearance) appearance->geom = NULL;
    //keep appearance

    cacheKey = filename;
    //printf("ManagedGeometry: Copying data from previously loaded file %s\n",filename.c_str());
    if(!prev->geometry->CollisionDataInitialized()) {
      Timer timer;
      prev->geometry->InitCollisionData();
      double t = timer.ElapsedTime();
      if(t > 0.2) 
        LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: Initialized "<<filename<<" collision data structures in time "<<t<<"s");
    }
    geometry = make_shared<Geometry::AnyCollisionGeometry3D>(*prev->geometry);
    //geometry = prev->geometry;
    appearance = prev->appearance;
    appearance->geom = geometry.get();
    manager.cache[filename].geoms.push_back(this);
#if CACHE_DEBUG
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry: adding a duplicate of "<<filename<<" to cache");
#endif
    return true;
  }

  if(LoadNoCache(filename)) {  
#if CACHE_DEBUG
    LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: adding "<<filename<<" to cache");
#endif
    cacheKey = filename;
    manager.cache[filename].geoms.push_back(this);
    return true;
  }
  return false;
}

bool ManagedGeometry::LoadNoCache(const string& filename)
{
  RemoveFromCache();
  dynamicGeometrySource.clear();
  geometry = make_shared<Geometry::AnyCollisionGeometry3D>();
  if(appearance) appearance->geom = NULL;
  //keep appearance

  if(filename.length()==0) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::LoadNoCache: empty filename?");
    return false;
  }

  const char* fn = filename.c_str();
  if(fn[0] == '{') {
    //inline file
    if(fn[filename.length()-1] != '}') {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ManagedGeometry::LoadNoCache: bracketed inline string "<<filename<<" doesn't have end brace '}'");
      return false;
    }
    string str = filename.substr(1,filename.length()-1);
    string rawstr = TranslateEscapes(str);
    stringstream ss(rawstr);
    *geometry = Geometry::AnyCollisionGeometry3D();
    if(!geometry->Load(ss)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::LoadNoCache: unable to parse inline string: "<<rawstr);
      return false;
    }
    appearance->Set(*geometry);
    return true;
  }
  if(0==strncmp(fn,"ros:PointCloud2//",17)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    dynamicGeometrySource = filename;
    geometry = make_shared<Geometry::AnyCollisionGeometry3D>(Meshing::PointCloud3D());
    appearance->Set(*geometry);
    Meshing::PointCloud3D& pc = geometry->AsPointCloud();
    LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry subscribing to point cloud on ROS topic "<<(fn+16));
    return ROSSubscribePointCloud(pc,fn+16);
  }
  else if(0==strncmp(fn,"ros://",6)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    dynamicGeometrySource = filename;
    *geometry = Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
    appearance->Set(*geometry);
    Meshing::PointCloud3D& pc = geometry->AsPointCloud();
    LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry subscribing to point cloud on ROS topic "<<(fn+5));
    return ROSSubscribePointCloud(pc,fn+5);
  }
  
  //TODO: ROS Mesh messages?

  const char* ext=FileExtension(fn);
  if(ext) {
    if(Geometry::AnyGeometry3D::CanLoadExt(ext)) {
      Timer timer;
      geometry = make_shared<Geometry::AnyCollisionGeometry3D>();
      string localfile = MakeURLLocal(fn);
      if(localfile.empty()) {
        LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry: Error downloading geometry file "<<fn);
        geometry = NULL;
        return false;
      }
      if(!geometry->Load(localfile.c_str())) {
        LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry: Error loading geometry file "<<fn);
        geometry = NULL;
        return false;
      }
      double t = timer.ElapsedTime();
      if(t > 0.2) 
        LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: loaded "<<filename<<" in time "<<t<<"s");
      if(geometry->type == Geometry::AnyGeometry3D::TriangleMesh) {
        if(geometry->TriangleMeshAppearanceData() != NULL) {
          appearance = make_shared<GLDraw::GeometryAppearance>(*geometry->TriangleMeshAppearanceData());
          SetupDefaultAppearance(*appearance);
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
      LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry: Unknown file extension "<<ext<<" on file "<<fn);
      return false;
    }
  }
  else {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry: No file extension on file "<<fn);
    return false;
  }
}

ManagedGeometry* ManagedGeometry::IsCached(const string& filename)
{
  map<string,GeometryManager::GeometryList>::const_iterator i=manager.cache.find(filename);
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

bool ManagedGeometry::IsOriginal() const
{
  return !cacheKey.empty() && (cacheKey.rfind(']') == string::npos);
}


const string& ManagedGeometry::CachedFilename() const
{
  return cacheKey;
}

void ManagedGeometry::AddToCache(const string& filename)
{
  if(!cacheKey.empty()) {
    if(cacheKey != filename)
      LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::AddToCache(): warning, item was previously cached as "<<cacheKey<<", now being asked to be cached as "<<filename<<"?");
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
  map<string,GeometryManager::GeometryList>::iterator i=manager.cache.find(cacheKey);
  if(i==manager.cache.end()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey<<" was not previously cached?");
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.empty()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey<<" was previously deleted?");
    cacheKey.clear();
    return;
  }
  for(size_t j=0;j<i->second.geoms.size();j++) {
    if(i->second.geoms[j] == this) {
      i->second.geoms.erase(i->second.geoms.begin()+j);
      if(i->second.geoms.empty()) {
        manager.cache.erase(i);
#if CACHE_DEBUG
        LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: removing "<<cacheKey<<" from cache.");
#endif
      }
      cacheKey.clear();
      return;
    }
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey<<" pointer was not previously cached?");
  cacheKey.clear();

  SetUniqueAppearance();
}

void ManagedGeometry::SetUnique()
{
  if(cacheKey.empty()) return;
  SetUniqueAppearance();
  map<string,GeometryManager::GeometryList>::iterator i=manager.cache.find(cacheKey);
  if(i==manager.cache.end()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey);
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.empty()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry::RemoveFromCache(): warning, item "<<cacheKey);
    cacheKey.clear();
    return;
  }
  if(i->second.geoms.size() > 1) {
    //has duplicates, actually have to copy the geometry and remove this
    geometry = make_shared<Geometry::AnyCollisionGeometry3D>(*geometry);
    OnGeometryChange();
    RemoveFromCache();
  }
}

void ManagedGeometry::TransformGeometry(const Math3D::Matrix4& xform)
{
  if(geometry) {
    string newCacheKey;
#if CACHE_DEBUG
    if(!cacheKey.empty()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: transforming geometry "<<cacheKey);
    }
#endif
    if(!cacheKey.empty()) {
      stringstream ss;
      ss<<cacheKey<<"[ transform "<<xform<<"]";
      newCacheKey = ss.str();
      ManagedGeometry* prev = ManagedGeometry::IsCached(newCacheKey);
      if(prev) {
        RemoveFromCache();
        geometry = make_shared<Geometry::AnyCollisionGeometry3D>(*prev->geometry);
        //geometry = prev->geometry;
        if(appearance.use_count() > 1)   //don't share with prior transformed geometry or the un-transformed geometry
          appearance = make_shared<GLDraw::GeometryAppearance>(*appearance);
        appearance->geom = geometry.get();
        cacheKey = newCacheKey;
#if CACHE_DEBUG
        LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: transformed version of "<<cacheKey<<" was in cache.");
#endif
        manager.cache[newCacheKey].geoms.push_back(this);
        OnGeometryChange();
        return;
      }
      else {
#if CACHE_DEBUG
        LOG4CXX_INFO(KrisLibrary::logger(),"ManagedGeometry: couldn't find transformed version of "<<cacheKey);
#endif        
      }
    }
    SetUnique();
    RemoveFromCache();
    geometry->Transform(xform);
    geometry->ClearCollisionData();
    if(!newCacheKey.empty()) {
      cacheKey = newCacheKey;
      manager.cache[newCacheKey].geoms.push_back(this);
    }
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
  map<string,GeometryManager::GeometryList>::const_iterator i=manager.cache.find(cacheKey);
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
  if(appearance && appearance.use_count() > 1) {
    appearance = make_shared<GLDraw::GeometryAppearance>(*appearance);
    if(!cacheKey.empty()) {
      //detach references to this' geometry
      map<string,GeometryManager::GeometryList>::iterator i=manager.cache.find(cacheKey);
      Assert(i != manager.cache.end());
      for(size_t j=0;j<i->second.geoms.size();j++) {
        if(i->second.geoms[j]->appearance->geom == geometry.get())
          i->second.geoms[j]->appearance->Set(*i->second.geoms[j]->geometry);
      }
    }
  }
}

const ManagedGeometry& ManagedGeometry::operator = (const ManagedGeometry& rhs)
{
  RemoveFromCache();

  geometry = rhs.geometry;
  appearance = rhs.appearance;
  appearance->geom = geometry.get();
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
  //Assert(appearance->geom == geometry.get());
  if(appearance->geom == NULL)
    appearance->Set(*geometry);
  appearance->DrawGL();
}

void ManagedGeometry::DrawGLOpaque(bool opaque)
{
  if(!geometry) return;
  Assert(appearance->geom != NULL);
  //Assert(appearance->geom == geometry.get());
  if(appearance->geom == NULL)
    appearance->Set(*geometry);
  GLDraw::GeometryAppearance::Element e = (opaque ? GLDraw::GeometryAppearance::ALL_OPAQUE : GLDraw::GeometryAppearance::ALL_TRANSPARENT);
  appearance->DrawGL(e);
}

bool ManagedGeometry::IsDynamicGeometry() const
{
  return !dynamicGeometrySource.empty();
}

bool ManagedGeometry::DynamicGeometryUpdate()
{
  if(0==strncmp(dynamicGeometrySource.c_str(),"ros://",6)) {
    //strip out the ros:/ part
    if(ROSHadUpdate(dynamicGeometrySource.substr(5,dynamicGeometrySource.length()-5).c_str())) {
      OnGeometryChange();
      return true;
    }
  }
  return false;
}


GeometryManager ManagedGeometry::manager;

} //namespace Klampt