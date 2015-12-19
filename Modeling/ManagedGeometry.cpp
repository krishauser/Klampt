#include "ManagedGeometry.h"
#include "IO/ROS.h"
#include <meshing/PointCloud.h>
#include <string.h>
#include <Timer.h>
#include <utils/stringutils.h>
using namespace Math3D;

ManagedGeometry::ManagedGeometry()
{
  appearance = new GLDraw::GeometryAppearance;
}

ManagedGeometry::ManagedGeometry(const ManagedGeometry& rhs)
{
  operator = (rhs);
}

ManagedGeometry::~ManagedGeometry()
{
  RemoveFromCache();
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
    cachedGeoms[filename].geoms.push_back(this);
    return true;
  }

  if(LoadNoCache(filename)) {
    cacheKey = filename;
    cachedGeoms[filename].geoms.push_back(this);
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
      geometry = new Geometry::AnyCollisionGeometry3D();
      if(!geometry->Load(fn)) {
        fprintf(stderr,"ManagedGeometry: Error loading geometry file %s\n",fn);
	geometry = NULL;
        return false;
      }
      if(geometry->type == Geometry::AnyGeometry3D::TriangleMesh) {
	if(geometry->TriangleMeshAppearanceData() != NULL) {
	  printf("ManagedGeometry: Got texture information with file %s\n",filename.c_str());
	  appearance = new GLDraw::GeometryAppearance(*geometry->TriangleMeshAppearanceData());
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
  std::map<std::string,ManagedGeometry::GeometryInfo>::const_iterator i=cachedGeoms.find(filename);
  if(i==cachedGeoms.end()) return NULL;
  if(i->second.geoms.empty()) return NULL;
  return i->second.geoms[0];
}

void ManagedGeometry::AddToCache(const std::string& filename)
{
  if(!cacheKey.empty()) {
    if(cacheKey != filename)
      printf("ManagedGeometry::AddToCache(): warning, item was previously cached as %s, now being asked to be cached as %s?\n",cacheKey.c_str(),filename.c_str());
    return;
  }
  cacheKey = filename;
  cachedGeoms[cacheKey].geoms.push_back(this);
}

void ManagedGeometry::RemoveFromCache()
{
  if(cacheKey.empty()) return;
  std::map<std::string,ManagedGeometry::GeometryInfo>::iterator i=cachedGeoms.find(cacheKey);
  if(i==cachedGeoms.end()) {
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
      if(i->second.geoms.empty())
	cachedGeoms.erase(i);
      cacheKey.clear();
      return;
    }
  }
  printf("ManagedGeometry::RemoveFromCache(): warning, item %s pointer was not previously cached?\n",cacheKey.c_str());
  cacheKey.clear();
}

void ManagedGeometry::TransformGeometry(const Math3D::Matrix4& xform)
{
  if(geometry) {
    RemoveFromCache();
    geometry->Transform(xform);
    geometry->ClearCollisionData();
  }
}

ManagedGeometry::AppearancePtr ManagedGeometry::Appearance()
{
  return appearance; 
}

bool ManagedGeometry::IsAppearanceShared() const
{ 
  if(cacheKey.empty()) return false;
  std::map<std::string,ManagedGeometry::GeometryInfo>::const_iterator i=cachedGeoms.find(cacheKey);
  if(i==cachedGeoms.end()) 
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
  if(appearance) {
    appearance = new GLDraw::GeometryAppearance(*appearance);
    if(geometry)
      appearance->Set(*geometry);
  }
}

const ManagedGeometry& ManagedGeometry::operator = (const ManagedGeometry& rhs)
{
  RemoveFromCache();

  geometry = rhs.geometry;
  appearance = rhs.appearance;
  appearance->geom = geometry;
  cacheKey = rhs.cacheKey;
  if(!cacheKey.empty())
    cachedGeoms[cacheKey].geoms.push_back(this);
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
      appearance->Set(*geometry);
      return true;
    }
  }
  return false;
}


std::map<std::string,ManagedGeometry::GeometryInfo> ManagedGeometry::cachedGeoms;
