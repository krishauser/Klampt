#include "ODECustomGeometry.h"
#include "ODECommon.h"
#include <KrisLibrary/geometry/CollisionPointCloud.h>
#include <ode/collision.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/meshing/IO.h>
#include <iostream>
using namespace std;

DECLARE_LOGGER(ODESimulator)

namespace Klampt {

static bool gCustomGeometryMeshesIntersect = false;

int gdCustomGeometryClass = 0;


dGeomID dCreateCustomGeometry(AnyCollisionGeometry3D* geometry,Real outerMargin)
{
  dGeomID geom = dCreateGeom(gdCustomGeometryClass);
  CustomGeometryData* data = dGetCustomGeometryData(geom);
  data->geometry = geometry;
  data->outerMargin = outerMargin;
  data->odeOffset.setZero();
  dGeomSetCategoryBits(geom,0xffffffff);
  dGeomSetCollideBits(geom,0xffffffff);
  dGeomEnable(geom);
  return geom;
}

CustomGeometryData* dGetCustomGeometryData(dGeomID o)
{
  return (CustomGeometryData*)dGeomGetClassData(o);
}


int dCustomGeometryCollide (dGeomID o1, dGeomID o2, int flags,
                           dContactGeom *contact, int skip)
{
  int m = (flags&0xffff);
  if(m == 0) m=1;
  //printf("CustomGeometry collide\n");
  CustomGeometryData* d1 = dGetCustomGeometryData(o1);
  CustomGeometryData* d2 = dGetCustomGeometryData(o2);
  RigidTransform T1;
  RigidTransform T2;
  CopyMatrix(T1.R,dGeomGetRotation(o1));
  CopyVector(T1.t,dGeomGetPosition(o1));
  CopyMatrix(T2.R,dGeomGetRotation(o2));
  CopyVector(T2.t,dGeomGetPosition(o2));
  T1.t += T1.R*d1->odeOffset;
  T2.t += T2.R*d2->odeOffset;
  d1->geometry->SetTransform(T1);
  d2->geometry->SetTransform(T2);

  AnyContactsQuerySettings settings;
  settings.padding1 = d1->outerMargin;
  settings.padding2 = d2->outerMargin;
  settings.maxcontacts = m;
  AnyContactsQueryResult res = d1->geometry->Contacts(*d2->geometry,settings);
  int k=0;
  for(const auto& c:res.contacts) {
    Vector3 x = (c.p1+c.p2)*0.5;
    ///the convention between AnyGeometry and ODE is reversed
    Vector3 n; n.setNegative(c.n);
    x.get(contact[k].pos);
    n.get(contact[k].normal);
    contact[k].depth = c.depth;
    contact[k].g1 = o1;
    contact[k].g2 = o2;
    if(c.unreliable)
      gCustomGeometryMeshesIntersect = true;
    k++;
    if(k >= m) break;
  }
  return k;
}


dColliderFn * dCustomGeometryGetColliderFn (int num)
{
  if(num == gdCustomGeometryClass) return dCustomGeometryCollide;
  else return NULL;
}


void dCustomGeometryAABB(dGeomID o,dReal aabb[6])
{
  CustomGeometryData* d = dGetCustomGeometryData(o);
  AABB3D bb;
  RigidTransform T;
  CopyMatrix(T.R,dGeomGetRotation(o));
  CopyVector(T.t,dGeomGetPosition(o));  
  T.t += T.R*d->odeOffset;
  d->geometry->SetTransform(T);
  bb = d->geometry->GetAABB();
  bb.bmin -= Vector3(d->outerMargin,d->outerMargin,d->outerMargin);
  bb.bmax += Vector3(d->outerMargin,d->outerMargin,d->outerMargin);
  aabb[0] = bb.bmin.x;
  aabb[1] = bb.bmax.x;
  aabb[2] = bb.bmin.y;
  aabb[3] = bb.bmax.y;
  aabb[4] = bb.bmin.z;
  aabb[5] = bb.bmax.z;
}



void dCustomGeometryDtor(dGeomID o)
{
}

void InitODECustomGeometry()
{
  dGeomClass mmclass;
  mmclass.bytes = sizeof(CustomGeometryData);
  mmclass.collider = dCustomGeometryGetColliderFn;
  mmclass.aabb = dCustomGeometryAABB;
  mmclass.aabb_test = NULL;
  mmclass.dtor = dCustomGeometryDtor;
  gdCustomGeometryClass = dCreateGeomClass(&mmclass);
}

bool GetCustomGeometryCollisionReliableFlag()
{
  return !gCustomGeometryMeshesIntersect;
}

void ClearCustomGeometryCollisionReliableFlag()
{
  gCustomGeometryMeshesIntersect = false;
}

} //namespace Klampt