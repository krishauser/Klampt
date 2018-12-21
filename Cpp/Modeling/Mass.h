#ifndef OBJECT_MASS_H
#define OBJECT_MASS_H

#include <KrisLibrary/geometry/AnyGeometry.h>
using namespace Math3D;

/** @addtogroup Modeling */
/*@{*/

///Computes the first moment integrated over the mesh (assuming the mesh
///is a hollow shell)
Vector3 CenterOfMass(const Meshing::TriMesh& mesh);
///Computes the first moment integrated over the point cloud
Vector3 CenterOfMass(const Meshing::PointCloud3D& pc);
///Computes the first moment integrated over the volume grid (assuming < 0 
///is the interior)
Vector3 CenterOfMass(const Meshing::VolumeGrid& grid);
///Computes the first moment integrated over the geometry
Vector3 CenterOfMass(const Geometry::AnyGeometry3D& geom);

///Computes the second moment integrated over the geometry
Matrix3 Covariance(const Meshing::TriMesh& mesh,const Vector3& center);
Matrix3 Covariance(const Meshing::PointCloud3D& pc,const Vector3& center);
Matrix3 Covariance(const Meshing::VolumeGrid& vol,const Vector3& center);
Matrix3 Covariance(const Geometry::AnyGeometry3D& geom,const Vector3& center);
inline Matrix3 Covariance(const Meshing::TriMesh& mesh) { return Covariance(mesh,CenterOfMass(mesh)); }
inline Matrix3 Covariance(const Meshing::PointCloud3D& mesh) { return Covariance(mesh,CenterOfMass(mesh)); }
inline Matrix3 Covariance(const Meshing::VolumeGrid& mesh) { return Covariance(mesh,CenterOfMass(mesh)); }
inline Matrix3 Covariance(const Geometry::AnyGeometry3D& mesh) { return Covariance(mesh,CenterOfMass(mesh)); }

///Computes the inertia tensor by integrating over the mesh
Matrix3 Inertia(const Meshing::TriMesh& mesh,const Vector3& center,Real mass);
Matrix3 Inertia(const Meshing::PointCloud3D& mesh,const Vector3& center,Real mass);
Matrix3 Inertia(const Meshing::VolumeGrid& mesh,const Vector3& center,Real mass);
Matrix3 Inertia(const Geometry::AnyGeometry3D& mesh,const Vector3& center,Real mass);
inline Matrix3 Inertia(const Meshing::TriMesh& mesh,Real mass) { return Inertia(mesh,CenterOfMass(mesh),mass); }
inline Matrix3 Inertia(const Meshing::PointCloud3D& mesh,Real mass) { return Inertia(mesh,CenterOfMass(mesh),mass); }
inline Matrix3 Inertia(const Meshing::VolumeGrid& mesh,Real mass) { return Inertia(mesh,CenterOfMass(mesh),mass); }
inline Matrix3 Inertia(const Geometry::AnyGeometry3D& mesh,Real mass) { return Inertia(mesh,CenterOfMass(mesh),mass); }

///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Vector3 CenterOfMass_Solid(const Meshing::TriMesh& mesh,Real gridRes);
///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center);
inline Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes)
{
  return Covariance_Solid(mesh,gridRes,CenterOfMass_Solid(mesh,gridRes));
}
///Computes the inertia integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Inertia_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center,Real mass);
inline Matrix3 Inertia_Solid(const Meshing::TriMesh& mesh,Real gridRes,Real mass)
{
  return Inertia_Solid(mesh,gridRes,CenterOfMass_Solid(mesh,gridRes),mass);
}

/*@}*/

#endif
