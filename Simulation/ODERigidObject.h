#ifndef ODE_RIGID_OBJECT_H
#define ODE_RIGID_OBJECT_H

#include "Modeling/RigidObject.h"
#include "ODEGeometry.h"
#include <math/vector.h>
#include <ode/common.h>
#include <myfile.h>
using namespace Math;

/** @ingroup Simulation
 * @brief An ODE-simulated rigid object
 */
class ODERigidObject
{
 public:
  static double defaultPadding;
  static ODESurfaceProperties defaultSurface;

  ODERigidObject(const RigidObject& obj);
  ~ODERigidObject();
  void Create(dWorldID worldID,dSpaceID space,bool useBoundaryLayer=true);
  void Clear();
  void SetTransform(const RigidTransform& T);
  void GetTransform(RigidTransform& T) const;
  void SetVelocity(const Vector3& w,const Vector3& v);
  void GetVelocity(Vector3& w,Vector3& v) const;
  bool ReadState(File& f);
  bool WriteState(File& f) const;
  
  dBodyID body() { return bodyID; }
  dGeomID geom() { return geometry->geom(); }
  dSpaceID space() { return spaceID; }
  ODEGeometry* triMesh() { return geometry; }

  const RigidObject& obj;

 private:
  dBodyID bodyID;
  ODEGeometry* geometry;
  dSpaceID spaceID;
};

#endif
