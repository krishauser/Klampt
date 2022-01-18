#ifndef ODE_RIGID_OBJECT_H
#define ODE_RIGID_OBJECT_H

#include <Klampt/Modeling/RigidObject.h>
#include "ODEGeometry.h"
#include <KrisLibrary/math/vector.h>
#include <ode/common.h>
#include <KrisLibrary/File.h>

namespace Klampt {
  using namespace Math;

/** @ingroup Simulation
 * @brief An ODE-simulated rigid object.
 *
 * Note: if you manually set the object's velocities, make sure to disable
 * instability correction in the simulator for that time step.
 */
class ODERigidObject
{
 public:
  static double defaultPadding;
  static ODESurfaceProperties defaultSurface;

  ODERigidObject(RigidObjectModel& obj);
  ~ODERigidObject();
  void Create(dWorldID worldID,dSpaceID space,bool useBoundaryLayer=true);
  void Clear();
  void SetTransform(const RigidTransform& T);
  void GetTransform(RigidTransform& T) const;
  void SetVelocity(const Vector3& w,const Vector3& v);
  void GetVelocity(Vector3& w,Vector3& v) const;
  Real GetKineticEnergy() const;
  bool ReadState(File& f);
  bool WriteState(File& f) const;
  
  dBodyID body() { return bodyID; }
  dGeomID geom() { return geometry->geom(); }
  dSpaceID space() { return spaceID; }
  ODEGeometry* triMesh() { return geometry; }

  RigidObjectModel& obj;

 private:
  dBodyID bodyID;
  ODEGeometry* geometry;
  dSpaceID spaceID;
};

} //namespace Klampt

#endif
