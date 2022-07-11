#ifndef ODE_INTERFACE_TRI_MESH_H
#define ODE_INTERFACE_TRI_MESH_H

#include "ODESurface.h"
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <ode/common.h>
#include <ode/collision_trimesh.h>

namespace Klampt {
  using namespace Math3D;
  using namespace Geometry;

/** @ingroup Simulation
 * @brief An ODE collision geometry.
 *
 * The pointer to the AnyCollisionGeometry3D passed into Create must live
 * throughout the duration of the use of this geometry.
 */
class ODEGeometry
{
 public:
  ODEGeometry();
  ~ODEGeometry();

  void Create(AnyCollisionGeometry3D* geom,dSpaceID space,Vector3 offset=Vector3(0.0),bool useCustomMesh = true);
  ///Deletes the geometry and removes it from the space
  void Clear();
  ///Debugging: draws the points of the trimesh (only works if useCustomMesh=false)
  void DrawGL();
  ///Sets the boundary layer thickness for this geometry
  void SetPadding(Real outerMargin);
  ///Returns the boundadry layer thickness for this geometry
  Real GetPadding();
  ///Sets the boundary layer thickness AND shrinks the original geometry.
  ///May lead to closer contact behavior to the original mesh.
  ///If inplace is set to true, then the original geometry is actually modified
  ///(this may help verify whether the geometry was shrunk correctly)
  ///
  ///Note: does not work with point clouds (never) or geometric primitives
  ///(not at the moment, but may be supported in the future). Interpenetrating
  ///meshes may be created because a relatively naive method is used to
  ///shrink the geometry. Do this only with small margins or else you risk
  ///creating artifacts (a good rule of thumb is no smaller than half the
  ///width of your smallest triangle)
  ///
  ///Note: destroys the collisionGeometry pointer and rebuilds the geometry.
  ///
  ///Returns the new geometry.
  AnyCollisionGeometry3D* SetPaddingWithPreshrink(Real outerMargin,bool inplace=false);

  dGeomID geom() const { return geomID; }
  dTriMeshDataID triMeshData() const { return triMeshDataID; }
  ODESurfaceProperties& surf() { return surface; }

 private:
  dTriMeshDataID triMeshDataID;
  dGeomID geomID;
  
  dReal* verts;
  int* indices;
  dReal* normals;
  int numVerts;
  int numTris;
  int numVertComponents;

  AnyCollisionGeometry3D* collisionGeometry;
  ODESurfaceProperties surface;
  bool geometrySelfAllocated;
};

} //namespace Klampt

#endif
