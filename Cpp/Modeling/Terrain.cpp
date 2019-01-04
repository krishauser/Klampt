#include "Terrain.h"
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/utils/SimpleFile.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/ioutils.h>
#include <string.h>
#include <fstream>
#include "IO/ROS.h"
#include "View/Texturizer.h"

void Terrain::InitCollisions()
{
  Timer timer;
  geometry->InitCollisionData();
  double t = timer.ElapsedTime();
  if(t > 0.2) 
    printf("Initialized terrain %s collision data structures in time %gs\n",geomFile.c_str(),t);
}

bool Terrain::Load(const char* fn)
{
  const char* ext=FileExtension(fn);
  if(ext && 0==strcmp(ext,"env")) {
    SimpleFile f(fn);
    if(!f) {
      fprintf(stderr,"SimpleFile read failed\n");
      return false;
    }
    if(f.count("mesh")==0) {
      fprintf(stderr,"Terrain file doesn't contain a mesh file\n");
      return false;
    }
    if(!f.CheckSize("mesh",1,fn)) return false;
    string fnPath = GetFilePath(fn);
    string geomfn = fnPath + f["mesh"][0].AsString();
    if(!LoadGeometry(geomfn.c_str())) return false;
    Timer timer;
    if(timer.ElapsedTime() > 1.0)
      printf("Env %s collision init took time %gs\n",geomfn.c_str(),timer.ElapsedTime());
    if(f.count("kFriction")!=0) {
      if(!f.CheckType("kFriction",PrimitiveValue::Double,fn)) return false;
      vector<double> values=f.AsDouble("kFriction");
      if(values.size() == 1)
	SetUniformFriction(values[0]);
      else if(values.size() == geometry->NumElements()) {
	kFriction = values;
      }
      else {
	fprintf(stderr,"Terrain file doesn't contain the right number of friction values\n");
	return false;
      }
    }
    else
      SetUniformFriction(0.5);
    return true;
  }
  else {
    if(!LoadGeometry(fn)) return false;
    //printf("Terrain %s: Setting uniform friction 0.5\n",fn);
    SetUniformFriction(0.5);
    return true;
  }
  return false;
}

bool Terrain::LoadGeometry(const char* fn)
{
  geomFile = fn;
  if(geometry.Load(geomFile)) {
    if(!geometry.Appearance()->tex1D && !geometry.Appearance()->tex2D) {
      geometry.Appearance()->faceColor.set(0.8,0.6,0.2);
      geometry.Appearance()->texWrap = true;
      Texturizer tex;
      tex.texture = "checker";
      tex.Set(geometry);
    }
    return true;
  }
  return false;
}

bool Terrain::Save(const char* fn)
{
  ofstream out(fn);
  if(!out) return false;
  out<<"mesh ";
  SafeOutputString(out,geomFile);
  out<<endl;
  if(kFriction.size() > 0) {
    bool nonuniform = false;
    for(size_t i=1;i<kFriction.size();i++)
      if(kFriction[i] != kFriction[0]) nonuniform = true;
    if(nonuniform) {
      out<<"friction ";
      for(size_t i=0;i<kFriction.size();i++)
        out<<kFriction[i]<<" ";
      out<<endl;
    }
    else
      out<<"friction "<<kFriction[0]<<endl;
  }
  out.close();
  return true;
}

void Terrain::DrawGL()
{
  if(!geometry) return;

  geometry.DrawGL();
}
