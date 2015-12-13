#include "Environment.h"
#include <Timer.h>
#include <meshing/IO.h>
#include <meshing/PointCloud.h>
#include <utils/SimpleFile.h>
#include <utils/stringutils.h>
#include <string.h>
#include <fstream>
#include "IO/ROS.h"

void Environment::InitCollisions()
{
  Timer timer;
  geometry.InitCollisionData();
  double t = timer.ElapsedTime();
  if(t > 0.2) 
    printf("Initialized environment %s collision data structures in time %gs\n",geomFile.c_str(),t);
}

bool Environment::Load(const char* fn)
{
  const char* ext=FileExtension(fn);
  if(ext && 0==strcmp(ext,"env")) {
    SimpleFile f(fn);
    if(!f) {
      fprintf(stderr,"SimpleFile read failed\n");
      return false;
    }
    if(f.count("mesh")==0) {
      fprintf(stderr,"Environment file doesn't contain a mesh file\n");
      return false;
    }
    if(!f.CheckSize("mesh",1,fn)) return false;
    string fnPath = GetFilePath(fn);
    string geomfn = fnPath + f["mesh"][0].AsString();
    if(!LoadGeometry(geomfn.c_str())) return false;
    Timer timer;
    //TESTING: don't need this with dynamic initialization
    //geometry.InitCollisions();
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s collision init took time %gs\n",geomfn.c_str(),timer.ElapsedTime());
    if(f.count("kFriction")!=0) {
      if(!f.CheckType("kFriction",PrimitiveValue::Double,fn)) return false;
      vector<double> values=f.AsDouble("kFriction");
      if(values.size() == 1)
	SetUniformFriction(values[0]);
      else if(values.size() == geometry.NumElements()) {
	kFriction = values;
      }
      else {
	fprintf(stderr,"Environment file doesn't contain the right number of friction values\n");
	return false;
      }
    }
    return true;
  }
  else 
    return LoadGeometry(fn);
  return false;
}

bool Environment::LoadGeometry(const char* fn)
{
  this->geomFile = fn;

  if(0==strncmp(fn,"ros://",6)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    this->geomFile = fn;
    this->geometry = Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
    Meshing::PointCloud3D& pc = this->geometry.AsPointCloud();
    printf("Environment subscribing to point cloud on ROS topic %s\n",fn+5);
    return ROSSubscribePointCloud(pc,fn+5);
  }
  else if(0==strncmp(fn,"ros://PointCloud/",17)) {
    //it's a ROS topic
    if(!ROSInit()) return false;
    this->geomFile = fn;
    this->geometry = Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
    Meshing::PointCloud3D& pc = this->geometry.AsPointCloud();
    printf("Environment subscribing to point cloud on ROS topic %s\n",fn+16);
    return ROSSubscribePointCloud(pc,fn+16);
  }
  //TODO: ROS Mesh messages?

  Timer timer;
  const char* ext=FileExtension(fn);
  if(ext && Geometry::AnyGeometry3D::CanLoadExt(ext)) {
    this->geomFile = fn;
    Timer timer;
    if(!geometry.Load(fn)) {
      cout<<"Environment::Load error loading geometry from "<<fn<<endl;
      return false;
    }
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s load took time %gs\n",fn,timer.ElapsedTime());
    timer.Reset();
    //geometry.InitCollisions();
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s collision init took time %gs\n",fn,timer.ElapsedTime());
    SetUniformFriction(0.5);
    return true;
  }
}

//bool Environment::Save(const char* fn);
