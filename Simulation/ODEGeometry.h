#ifndef ODE_INTERFACE_TRI_MESH_H
#define ODE_INTERFACE_TRI_MESH_H

#include "ODESurface.h"
#include <geometry/AnyGeometry.h>
#include <ode/common.h>
#include <ode/collision_trimesh.h>
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
  void Clear();
  void DrawGL();
  void SetPadding(Real outerMargin);
  Real GetPadding();

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
};

#endif
