#include "ODERigidObject.h"
#include "ODECommon.h"
#include "ODECustomGeometry.h"
#include "Settings.h"
#include <ode/ode.h>
#include <KrisLibrary/errors.h>
#include <iostream>

using namespace Klampt;

DECLARE_LOGGER(ODESimulator)

double ODERigidObject::defaultPadding = gDefaultRigidObjectPadding;
ODESurfaceProperties ODERigidObject::defaultSurface = {0.1,0.5,Inf,Inf};

ODERigidObject::ODERigidObject(RigidObjectModel& _obj)
  :obj(_obj),bodyID(0),geometry(0),spaceID(0)
{}

ODERigidObject::~ODERigidObject()
{
  Clear();
}

void ODERigidObject::Create(dWorldID worldID,dSpaceID space,bool useBoundaryLayer)
{
  Clear(); 
  spaceID = space;
  bodyID = dBodyCreate(worldID);

  dMass mass;
  mass.mass = obj.mass;
  //NOTE: in ODE, COM must be zero vector! -- we take care of this by shifting geometry
  //CopyVector3(mass.c,obj.com);
  mass.c[0] = mass.c[1] = mass.c[2] = 0; mass.c[3] = 1.0;
  CopyMatrix(mass.I,obj.inertia);
  int res=dMassCheck(&mass);
  if(res != 1) {
    LOG4CXX_ERROR(GET_LOGGER(ODESimulator),"Uh... rigid object mass is not considered to be valid by ODE?");
    LOG4CXX_ERROR(GET_LOGGER(ODESimulator),"  Inertia: "<<obj.inertia);
  }
  dBodySetMass(bodyID,&mass);
  
  geometry = new ODEGeometry;
  geometry->Create(&*obj.geometry,spaceID,-obj.com,useBoundaryLayer);
  dGeomSetBody(geometry->geom(),bodyID);
  dGeomSetData(geometry->geom(),(void*)-1);
  geometry->SetPadding(defaultPadding);
  geometry->surf().kRestitution = obj.kRestitution;
  geometry->surf().kFriction = obj.kFriction;
  geometry->surf().kStiffness = obj.kStiffness;
  geometry->surf().kDamping = obj.kDamping;

  SetTransform(obj.T);
}

void ODERigidObject::Clear()
{
  SafeDeleteProc(bodyID,dBodyDestroy);
  SafeDelete(geometry);
}

void ODERigidObject::SetTransform(const RigidTransform& T)
{
  Vector3 comPos = T*obj.com;
  dBodySetPosition(bodyID,comPos.x,comPos.y,comPos.z);
  dMatrix3 rot;
  CopyMatrix(rot,T.R);
  dBodySetRotation(bodyID,rot);
}


void ODERigidObject::GetTransform(RigidTransform& T) const
{
  const dReal* pos = dBodyGetPosition(bodyID);
  Vector3 comPos(pos[0],pos[1],pos[2]);
  const dReal* rot = dBodyGetRotation(bodyID);
  CopyMatrix(T.R,rot);
  T.t = comPos - T.R*obj.com;
}


void ODERigidObject::SetVelocity(const Vector3& w,const Vector3& v)
{
  //v is the body origin velocity, need the velocity at the body's com
  //v = x' + w x R*(cm - origin) = v = x' + w x R*cm
  Vector3 vcom;
  if(obj.com.maxAbsElement() > 0) {
    RigidTransform T;
    GetTransform(T);
    vcom = v+cross(w,T.R*obj.com);
  }
  else
    vcom = v;

  dBodySetLinearVel(bodyID,vcom.x,vcom.y,vcom.z);
  dBodySetAngularVel(bodyID,w.x,w.y,w.z);
}

void ODERigidObject::GetVelocity(Vector3& w,Vector3& v) const
{
  //TODO: get velocity at com offset
  CopyVector(v,dBodyGetLinearVel(bodyID));
  CopyVector(w,dBodyGetAngularVel(bodyID));

  //v is now the velocity at the body's com, do a correction to get
  //body origin
  //v = x' + w x R*(cm - origin) = v = x' + w x R*cm
  //x' = v - w x R*cm 
  if(obj.com.maxAbsElement() > 0) {
    RigidTransform T;
    GetTransform(T);
    v = v-cross(w,T.R*obj.com);
  }
}


Real ODERigidObject::GetKineticEnergy() const
{
  Vector3 w,v,wlocal;
  RigidTransform T;
  CopyVector(v,dBodyGetLinearVel(bodyID));
  CopyVector(w,dBodyGetAngularVel(bodyID));
  GetTransform(T);
  T.R.mulTranspose(w,wlocal);
  return obj.mass*v.normSquared() + wlocal.dot(obj.inertia*wlocal);
}

bool ODERigidObject::ReadState(File& f)
{
  Vector3 w,v;
  dReal pos[3];
  dReal q[4];
  dReal frc[3];
  dReal trq[3];
  if(!ReadArrayFile(f,pos,3)) return false;
  if(!ReadArrayFile(f,q,4)) return false;
  if(!ReadFile(f,w)) return false;
  if(!ReadFile(f,v)) return false;
  if(!ReadArrayFile(f,frc,3)) return false;
  if(!ReadArrayFile(f,trq,3)) return false;

  dBodySetPosition(bodyID,pos[0],pos[1],pos[2]);
  dBodySetQuaternion(bodyID,q);
  dBodySetForce(bodyID,frc[0],frc[1],frc[2]);
  dBodySetTorque(bodyID,trq[0],trq[1],trq[2]);
  SetVelocity(w,v);
  return true;
}

bool ODERigidObject::WriteState(File& f) const
{
  //TODO: use body quaternion
  Vector3 w,v;
  const dReal* pos=dBodyGetPosition(bodyID);
  const dReal* q=dBodyGetQuaternion(bodyID);
  GetVelocity(w,v);
  //do we need this? (yes)
  const dReal* frc=dBodyGetForce(bodyID);
  const dReal* trq=dBodyGetTorque(bodyID);
    
  if(!WriteArrayFile(f,pos,3)) return false;
  if(!WriteArrayFile(f,q,4)) return false;
  if(!WriteFile(f,w)) return false;
  if(!WriteFile(f,v)) return false;
  if(!WriteArrayFile(f,frc,3)) return false;
  if(!WriteArrayFile(f,trq,3)) return false;
  return true;
}
