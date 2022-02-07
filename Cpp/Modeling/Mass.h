#ifndef OBJECT_MASS_H
#define OBJECT_MASS_H

#include <KrisLibrary/geometry/AnyGeometry.h>

namespace Klampt {
    using namespace Math3D;

/** @addtogroup Modeling */
/*@{*/

///Computes the first moment integrated over the mesh.  surfaceFraction indicates how much
///of the mass is concentrated at mesh considered as a hollow shell vs a solid.
Vector3 CenterOfMass(const Meshing::TriMesh& mesh,Real surfaceFraction=1.0);
///Computes the first moment integrated over the point cloud
Vector3 CenterOfMass(const Meshing::PointCloud3D& pc,Real surfaceFraction=1.0);
///Computes the first moment integrated over the volume grid (assuming < 0 
///is the interior)
Vector3 CenterOfMass(const Meshing::VolumeGrid& grid,Real surfaceFraction=1.0);
///Computes the first moment integrated over the geometry
Vector3 CenterOfMass(const Geometry::AnyGeometry3D& geom,Real surfaceFraction=1.0);

///Computes the second moment integrated over the geometry
Matrix3 Covariance(const Meshing::TriMesh& mesh,const Vector3& center,Real surfaceFraction=1.0);
Matrix3 Covariance(const Meshing::PointCloud3D& pc,const Vector3& center,Real surfaceFraction=1.0);
Matrix3 Covariance(const Meshing::VolumeGrid& vol,const Vector3& center,Real surfaceFraction=1.0);
Matrix3 Covariance(const Geometry::AnyGeometry3D& geom,const Vector3& center,Real surfaceFraction=1.0);
inline Matrix3 Covariance(const Meshing::TriMesh& mesh,Real surfaceFraction=1.0) { return Covariance(mesh,CenterOfMass(mesh,surfaceFraction),surfaceFraction); }
inline Matrix3 Covariance(const Meshing::PointCloud3D& mesh,Real surfaceFraction=1.0) { return Covariance(mesh,CenterOfMass(mesh,surfaceFraction),surfaceFraction); }
inline Matrix3 Covariance(const Meshing::VolumeGrid& mesh,Real surfaceFraction=1.0) { return Covariance(mesh,CenterOfMass(mesh,surfaceFraction),surfaceFraction); }
inline Matrix3 Covariance(const Geometry::AnyGeometry3D& mesh,Real surfaceFraction=1.0) { return Covariance(mesh,CenterOfMass(mesh,surfaceFraction),surfaceFraction); }

///Computes the inertia tensor by integrating over the mesh
Matrix3 Inertia(const Meshing::TriMesh& mesh,const Vector3& center,Real mass,Real surfaceFraction=1.0);
Matrix3 Inertia(const Meshing::PointCloud3D& mesh,const Vector3& center,Real mass,Real surfaceFraction=1.0);
Matrix3 Inertia(const Meshing::VolumeGrid& mesh,const Vector3& center,Real mass,Real surfaceFraction=1.0);
Matrix3 Inertia(const Geometry::AnyGeometry3D& mesh,const Vector3& center,Real mass,Real surfaceFraction=1.0);
inline Matrix3 Inertia(const Meshing::TriMesh& mesh,Real mass,Real surfaceFraction=1.0) { return Inertia(mesh,CenterOfMass(mesh,surfaceFraction),mass,surfaceFraction); }
inline Matrix3 Inertia(const Meshing::PointCloud3D& mesh,Real mass,Real surfaceFraction=1.0) { return Inertia(mesh,CenterOfMass(mesh,surfaceFraction),mass,surfaceFraction); }
inline Matrix3 Inertia(const Meshing::VolumeGrid& mesh,Real mass,Real surfaceFraction=1.0) { return Inertia(mesh,CenterOfMass(mesh,surfaceFraction),mass,surfaceFraction); }
inline Matrix3 Inertia(const Geometry::AnyGeometry3D& mesh,Real mass,Real surfaceFraction=1.0) { return Inertia(mesh,CenterOfMass(mesh,surfaceFraction),mass,surfaceFraction); }

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

} //namespace Klampt

#endif
