#include "RigidObject.h"
#include <KrisLibrary/Timer.h>
#include "Mass.h"
#include "IO/ROS.h"
#include <KrisLibrary/robotics/Inertia.h>
#include <KrisLibrary/utils/SimpleFile.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/meshing/IO.h>
#include <string.h>
#include <fstream>
using namespace Math3D;
using namespace std;
namespace Klampt {

//defined in XmlWorld.cpp
string ResolveFileReference(const string& path,const string& fn);
string MakeURLLocal(const string& url,const char* url_resolution_path="klampt_downloads");

RigidObjectModel::RigidObjectModel()
{
  T.setIdentity();
  w.setZero();
  v.setZero();
  mass = 1;
  com.setZero();
  inertia.setIdentity();
  kFriction = 0.5;
  kRestitution = 0.0;
  kStiffness = kDamping = Inf;
}


bool RigidObjectModel::Load(const char* fn)
{
  string localpath = MakeURLLocal(fn);
  const char* ext=FileExtension(fn);
  if(ext && strcmp(ext,"obj")==0) {
    SimpleFile f;
    f.AllowItem("mesh");
    f.AllowItem("geomscale");
    f.AllowItem("geomtranslate");
    f.AllowItem("T");
    f.AllowItem("position");
    f.AllowItem("velocity");
    f.AllowItem("angularVelocity");
    f.AllowItem("mass");
    f.AllowItem("inertia");
    f.AllowItem("com");
    f.AllowItem("kFriction");
    f.AllowItem("kRestitution");
    f.AllowItem("kStiffness");
    f.AllowItem("kDamping");
    f.AllowItem("autoMass");
    if(!f.Load(localpath.c_str())) return false;

    if(!f.CheckSize("mesh",1,fn)) return false;
    if(!f.CheckType("mesh",PrimitiveValue::String,fn)) return false;

    string fnPath = GetFilePath(fn);
    geomFile = f["mesh"][0].AsString();
    string geomfn = ResolveFileReference(fnPath,geomFile);
    if(!LoadGeometry(geomfn.c_str()))
      return false;
    f.erase("mesh");

    Matrix4 ident; ident.setIdentity();
    Matrix4 geomT=ident; 
    if(f.count("geomscale") != 0) {
      if(!f.CheckType("geomscale",PrimitiveValue::Double,fn)) return false;
      vector<double> scale = f.AsDouble("geomscale");
      if(scale.size()==1) { geomT(0,0)=geomT(1,1)=geomT(2,2)=scale[0]; }
      else if(scale.size()==3) {
        geomT(0,0)=scale[0];
        geomT(1,1)=scale[1];
        geomT(2,2)=scale[2];
      }
      else {
        fprintf(stderr,"Invalid number of geomscale components in %s\n",fn);
        return false;
      }
      f.erase("geomscale");
    }
    if(f.count("geomtranslate") != 0) {
      if(!f.CheckType("geomtranslate",PrimitiveValue::Double,fn)) return false;
      if(!f.CheckSize("geomtranslate",3,fn)) return false;
      vector<double> trans = f.AsDouble("geomtranslate");
      geomT(0,3)=trans[0];
      geomT(1,3)=trans[1];
      geomT(2,3)=trans[2];
      f.erase("geomtranslate");
    }
    if(!(ident == geomT)) {
      geometry.TransformGeometry(geomT);  
    }
    if(f.count("T")==0) { T.setIdentity(); }
    else {
      if(!f.CheckType("T",PrimitiveValue::Double,fn)) return false;
      vector<double> items = f.AsDouble("T");
      if(items.size()==12) { //read 4 columns of 3, row major then translation, just like RigidTransform
        Vector3 x(items[0],items[3],items[6]);
        Vector3 y(items[1],items[4],items[7]);
        Vector3 z(items[2],items[5],items[8]);
        Vector3 t(items[9],items[10],items[11]);
        T.R.set(x,y,z); T.t=t;
      }
      else if(items.size()==16) { //read 4 rows of a 4 x 4 transform matrix
        Vector3 x(items[0],items[4],items[8]);
        Vector3 y(items[1],items[5],items[9]);
        Vector3 z(items[2],items[6],items[10]);
        Vector3 t(items[3],items[7],items[11]);
        T.R.set(x,y,z); T.t=t;
      }
      else {
        fprintf(stderr,"Invalid number of transformation components in %s\n",fn);
        return false;
      }
      f.erase("T");
    }
    if(f.count("position") != 0) {
      if(!f.CheckType("position",PrimitiveValue::Double,fn)) return false;
      if(!f.CheckSize("position",3,fn)) return false;
      vector<double> trans = f.AsDouble("position");
      T.t.set(trans[0],trans[1],trans[2]);
      f.erase("position");
    }
    if(f.count("velocity") != 0) {
      if(!f.CheckType("velocity",PrimitiveValue::Double,fn)) return false;
      if(!f.CheckSize("velocity",3,fn)) return false;
      vector<double> trans = f.AsDouble("velocity");
      v.set(trans[0],trans[1],trans[2]);
      f.erase("velocity");
    }
    if(f.count("angularVelocity") != 0) {
      if(!f.CheckType("angularVelocity",PrimitiveValue::Double,fn)) return false;
      if(!f.CheckSize("angularVelocity",3,fn)) return false;
      vector<double> trans = f.AsDouble("angularVelocity");
      w.set(trans[0],trans[1],trans[2]);
      f.erase("angularVelocity");
    }
    if(f.count("mass")==0) { mass=1.0;  }
    else {
      if(!f.CheckSize("mass",1)) return false;
      if(!f.CheckType("mass",PrimitiveValue::Double)) return false;
      mass = f["mass"][0].AsDouble();
      f.erase("mass");
    }
    bool hasCOM = false;
    if(f.count("com")==0) { com.setZero();  }
    else {
      if(!f.CheckSize("com",3)) return false;
      if(!f.CheckType("com",PrimitiveValue::Double)) return false;
      hasCOM = true;
      com.set(f["com"][0].AsDouble(),f["com"][1].AsDouble(),f["com"][2].AsDouble());
      f.erase("com");
    }
    if(f.count("inertia")==0) inertia.setIdentity();
    else {
      if(!f.CheckType("inertia",PrimitiveValue::Double,fn)) return false;
      vector<double> items = f.AsDouble("inertia");
      if(items.size() == 3) inertia.setDiagonal(Vector3(items[0],items[1],items[2]));
      else if(items.size() == 9) {
        inertia(0,0)=items[0];  inertia(0,1)=items[1];  inertia(0,2)=items[2];
        inertia(1,0)=items[3];  inertia(1,1)=items[4];  inertia(1,2)=items[5];
        inertia(2,0)=items[6];  inertia(2,1)=items[7];  inertia(2,2)=items[8];
      }
      else {
        fprintf(stderr,"Invalid number of inertia matrix components in %s\n",fn);
        return false;
      }
      f.erase("inertia");
    }
    if(f.count("kFriction")==0) kFriction = 0.5;
    else {
      if(!f.CheckSize("kFriction",1,fn)) return false;
      if(!f.CheckType("kFriction",PrimitiveValue::Double,fn)) return false;
      kFriction = f.AsDouble("kFriction")[0];
      f.erase("kFriction");
    }
    if(f.count("kRestitution")==0) kRestitution = 0.5;
    else {
      if(!f.CheckSize("kRestitution",1,fn)) return false;
      if(!f.CheckType("kRestitution",PrimitiveValue::Double,fn)) return false;
      kRestitution = f.AsDouble("kRestitution")[0];
      f.erase("kRestitution");
    }
    if(f.count("kStiffness")==0) kStiffness=Inf;
    else {
      if(!f.CheckSize("kStiffness",1,fn)) return false;
      if(!f.CheckType("kStiffness",PrimitiveValue::Double,fn)) return false;
      kStiffness = f.AsDouble("kStiffness")[0];
      f.erase("kStiffness");
    }
    if(f.count("kDamping")==0) kDamping=Inf;
    else {
      if(!f.CheckSize("kDamping",1,fn)) return false;
      if(!f.CheckType("kDamping",PrimitiveValue::Double,fn)) return false;
      kDamping = f.AsDouble("kDamping")[0];
      f.erase("kDamping");
    }
    if(f.count("autoMass")!=0) {
      Real surfaceFraction = 1.0;
      if(f.CheckSize("autoMass",1,fn,0) && f.CheckType("autoMass",PrimitiveValue::Double,fn,0))
        surfaceFraction = f.AsDouble("autoMass")[0];
      if(hasCOM) //com specified, compute inertia about given com
        inertia = Inertia(*geometry,com,mass,surfaceFraction);
      else
        SetMassFromGeometry(mass,surfaceFraction);
      f.erase("autoMass");
    }
    if(!f.empty()) {
      for(map<string,vector<PrimitiveValue> >::const_iterator i=f.entries.begin();i!=f.entries.end();i++)
        fprintf(stderr,"Unknown entry %s in object file %s\n",i->first.c_str(),fn);
    }
    return true;
  }
  else {
    if(!LoadGeometry(fn)) {
      printf("LoadGeometry %s failed\n",fn);
      return false;
    }
    T.setIdentity();
    mass=1.0;
    com.setZero();
    inertia.setZero();
    kFriction = 0.5;
    kRestitution = 0.5;
    kStiffness=Inf;
    kDamping=Inf;
    if(ext)
      fprintf(stderr,"Warning, loading object from .%s file %s.  Setting COM and inertia matrix from geometry.\n",ext,fn);
    else
      fprintf(stderr,"Warning, loading object from file %s.  Setting COM and inertia matrix from geometry.\n",fn);
    SetMassFromGeometry(1.0);
    return true;
  }
}

bool RigidObjectModel::LoadGeometry(const char* fn)
{
  geomFile = fn;
  //default appearance options
  geometry.Appearance()->faceColor.set(0.4f,0.2f,0.8f);
  //add a little shininess
  geometry.Appearance()->shininess = 20;
  if(geometry.Load(geomFile)) {
    return true;
  }
  return false;
}

bool RigidObjectModel::Save(const char* fn)
{
  ofstream out(fn);
  if(!out) return false;
  out<<"mesh "<<geomFile<<endl;
  out<<"T "<<T.R(0,0)<<" "<<T.R(0,1)<<" "<<T.R(0,2)<<" "<<T.R(1,0)<<" "<<T.R(1,1)<<" "<<T.R(1,2)<<" "<<T.R(2,0)<<" "<<T.R(2,1)<<" "<<T.R(2,2)<<" "<<T.t<<endl;
  if(!v.isZero())
    out<<"velocity "<<v<<endl;
  if(!w.isZero())
    out<<"angularVelocity "<<w<<endl;
  out<<"mass "<<mass<<endl;
  out<<"com "<<com<<endl;
  out<<"inertia "<<inertia(0,0)<<" "<<inertia(0,1)<<" "<<inertia(0,2)<<" "<<inertia(1,0)<<" "<<inertia(1,1)<<" "<<inertia(1,2)<<" "<<inertia(2,0)<<" "<<inertia(2,1)<<" "<<inertia(2,2)<<endl;
  out<<"kFriction "<<kFriction<<endl;
  out<<"kRestitution "<<kRestitution<<endl;
  out<<"kStiffness "<<kStiffness<<endl;
  out<<"kDamping "<<kDamping<<endl;
  out.close();
  return true;
}

void RigidObjectModel::SetMassFromGeometry(Real totalMass,Real surfaceFraction)
{
  mass = totalMass;
  com = CenterOfMass(*geometry,surfaceFraction);
  inertia = Inertia(*geometry,com,mass,surfaceFraction);
}

void RigidObjectModel::SetMassFromBB(Real totalMass)
{
  AABB3D bb=geometry->GetAABB();
  mass = totalMass;
  com = 0.5*(bb.bmin+bb.bmax);
  BoxInertiaMatrix(bb.bmax.x-bb.bmin.x,bb.bmax.y-bb.bmin.y,bb.bmax.z-bb.bmin.z,mass,inertia);
}

void RigidObjectModel::InitCollisions()
{
  Timer timer;
  geometry->InitCollisionData();
  double t = timer.ElapsedTime();
  if(t > 0.2) 
    printf("Initialized rigid object %s collision data structures in time %gs\n",geomFile.c_str(),t);
}

void RigidObjectModel::UpdateGeometry()
{
  geometry->SetTransform(T);
}

void RigidObjectModel::DrawGL()
{
  if(!geometry) return;

  glDisable(GL_CULL_FACE);
  glPushMatrix();
  GLDraw::glMultMatrix(Matrix4(T));

  geometry.DrawGL();

  glPopMatrix();
  glEnable(GL_CULL_FACE);
}

void RigidObjectModel::DrawGLOpaque(bool opaque)
{
  if(!geometry) return;

  glDisable(GL_CULL_FACE);
  glPushMatrix();
  GLDraw::glMultMatrix(Matrix4(T));

  geometry.DrawGLOpaque(opaque);

  glPopMatrix();
  glEnable(GL_CULL_FACE);
}

} //namespace Klampt