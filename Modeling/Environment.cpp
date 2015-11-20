#include "Environment.h"
#include <Timer.h>
#include <meshing/IO.h>
#include <utils/SimpleFile.h>
#include <utils/stringutils.h>
#include <string.h>
#include <fstream>


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
  if(0==strcmp(ext,"env")) {
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
    this->geomFile = geomfn;
    Timer timer;
    if(!geometry.Load(geomfn.c_str())) {
      cout<<"Environment::Load error loading "<<geomfn<<endl;
      return false;
    }
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s load took time %gs\n",fn,timer.ElapsedTime());
    timer.Reset();
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
  else if(Geometry::AnyGeometry3D::CanLoadExt(ext)) {
    this->geomFile = fn;
    Timer timer;
    if(!geometry.Load(fn)) return false;
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s load took time %gs\n",fn,timer.ElapsedTime());
    timer.Reset();
    //geometry.InitCollisions();
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s collision init took time %gs\n",fn,timer.ElapsedTime());
    SetUniformFriction(0.5);
    return true;
  }
  return false;
}

//bool Environment::Save(const char* fn);
