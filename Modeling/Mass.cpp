#include "Mass.h"
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/Voxelize.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/errors.h>
using namespace Meshing;
using namespace Geometry;

const static Real third = 1.0/3.0;

//m = m+xy^t
inline void AddOuterProduct(Matrix3& m,const Vector3& x,const Vector3& y)
{
  //note that the matrix data variable is column-major, not row major
  m.data[0][0]+=x.x*y.x;
  m.data[0][1]+=x.y*y.x;
  m.data[0][2]+=x.z*y.x;

  m.data[1][0]+=x.x*y.y;
  m.data[1][1]+=x.y*y.y;
  m.data[1][2]+=x.z*y.y;

  m.data[2][0]+=x.x*y.z;
  m.data[2][1]+=x.y*y.z;
  m.data[2][2]+=x.z*y.z;
}


//first and second moments integrated over mesh
Vector3 CenterOfMass(const TriMesh& mesh)
{
  Triangle3D tri;
  Vector3 center(Zero);
  if(mesh.tris.empty()) return center;
  Real sumArea=Zero;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    Real area=tri.area();
    center.madd(tri.a+tri.b+tri.c,area*third);
    sumArea += area;
  }
  Assert(sumArea != Zero);
  center /= sumArea;
  return center;
}

Vector3 CenterOfMass(const Math3D::GeometricPrimitive3D& g)
{
  AABB3D bbox = g.GetAABB();
  return (bbox.bmin + bbox.bmax)*0.5;
}


Vector3 CenterOfMass(const Meshing::PointCloud3D& pc)
{
  Vector3 center(Zero);
  for(size_t i=0;i<pc.points.size();i++)
    center += pc.points[i];
  return center / pc.points.size();
}

Vector3 CenterOfMass(const Meshing::VolumeGrid& grid)
{
  Vector3 mean(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    if(*i <= 0) {
      Vector3 c;
      i.getCellCenter(c);
      sum += 1.0;
      mean += *c;
    }
  }
  if(sum == 0) return mean;
  return mean / sum;
}


Vector3 CenterOfMass(const vector<AnyGeometry3D>& group)
{
  Vector3 sum(0.0);
  if(group.empty()) return sum;
  for(size_t i=0;i<group.size();i++)
    sum += CenterOfMass(group[i]);
  return sum / group.size();
}


Vector3 CenterOfMass(const AnyGeometry3D& geom)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return CenterOfMass(geom.AsPrimitive());
  case AnyGeometry3D::TriangleMesh:
    return CenterOfMass(geom.AsTriangleMesh());
  case AnyGeometry3D::PointCloud:
    return CenterOfMass(geom.AsPointCloud());
  case AnyGeometry3D::ImplicitSurface:
    return CenterOfMass(geom.AsImplicitSurface());
  case AnyGeometry3D::Group:
    return CenterOfMass(geom.AsGroup());
  }
  return Vector3(0.0);
}

Matrix3 Covariance(const TriMesh& mesh,const Vector3& center)
{
  Triangle3D tri;
  Real sumArea=Zero;
  Matrix3 A,temp;
  A.setZero();
  if(mesh.tris.empty()) return A;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    tri.a-=center;
    tri.b-=center;
    tri.c-=center;
    temp.setZero();
    AddOuterProduct(temp,tri.a,Two*tri.a+tri.b+tri.c);
    AddOuterProduct(temp,tri.b,Two*tri.b+tri.a+tri.c);
    AddOuterProduct(temp,tri.c,Two*tri.c+tri.a+tri.b);
    Real area = tri.area();
    temp *= area;
    A += temp;
    sumArea += area;
  }
  Assert(sumArea != Zero);
  A *= 1.0/sumArea;
  return A;
}

Matrix3 Covariance(const PointCloud3D& pc,const Vector3& center)
{
  Matrix3 A;
  A.setZero();
  if(pc.points.empty()) return A;
  for(size_t i=0;i<pc.points.size();i++) {
    AddOuterProduct(A,pc.points[i]-center,pc.points[i]-center);
  }
  A *= 1.0/pc.points.size();
  return A;
}

Matrix3 Covariance(const VolumeGrid& grid,const Vector3& center)
{
  Matrix3 cov(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    if(*i <= 0) {
      Vector3 c;
      i.getCellCenter(c);
      Matrix3 temp; temp.setZero();
      AddOuterProduct(temp,c-center,c-center);
      cov += temp;
      sum += 1.0;
    }
  }
  if(sum == 0) return cov;
  cov *= 1.0/sum;
  return cov;
}


Matrix3 Covariance(const GeometricPrimitive3D& g,const Vector3& center)
{
  AABB3D bbox = g.GetAABB();
  Matrix3 res(0.0);
  res(0,0) = Sqr(bbox.bmax[0] - bbox.bmin[0])/12.0;
  res(1,1) = Sqr(bbox.bmax[1] - bbox.bmin[1])/12.0;
  res(2,2) = Sqr(bbox.bmax[2] - bbox.bmin[2])/12.0;
  return res;
}


Matrix3 Covariance(const vector<AnyGeometry3D>& group,const Vector3& center)
{
  Matrix3 res(0.0); 
  for(size_t i=0;i<group.size();i++) 
    res += Covariance(group[i],center);
  return res;
}


Matrix3 Covariance(const AnyGeometry3D& geom,const Vector3& center)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return Covariance(geom.AsPrimitive(),center);
  case AnyGeometry3D::TriangleMesh:
    return Covariance(geom.AsTriangleMesh(),center);
  case AnyGeometry3D::PointCloud:
    return Covariance(geom.AsPointCloud(),center);
  case AnyGeometry3D::ImplicitSurface:
    return Covariance(geom.AsImplicitSurface(),center);
  case AnyGeometry3D::Group:
    return Covariance(geom.AsGroup(),center);
  }
  return Matrix3();
}

Matrix3 Inertia(const Math3D::GeometricPrimitive3D& geom,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(geom,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}


Matrix3 Inertia(const TriMesh& mesh,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(mesh,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}

Matrix3 Inertia(const PointCloud3D& mesh,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(mesh,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}

Matrix3 Inertia(const VolumeGrid& mesh,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(mesh,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}


Matrix3 Inertia(const vector<AnyGeometry3D>& group,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(group,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}



Matrix3 Inertia(const AnyGeometry3D& geom,const Vector3& center,Real mass)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return Inertia(geom.AsPrimitive(),center,mass);
  case AnyGeometry3D::TriangleMesh:
    return Inertia(geom.AsTriangleMesh(),center,mass);
  case AnyGeometry3D::PointCloud:
    return Inertia(geom.AsPointCloud(),center,mass);
  case AnyGeometry3D::ImplicitSurface:
    return Inertia(geom.AsImplicitSurface(),center,mass);
  case AnyGeometry3D::Group:
    return Inertia(geom.AsGroup(),center,mass);
  }
  return Matrix3();
}



///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Vector3 CenterOfMass_Solid(const Meshing::TriMesh& mesh,Real gridRes)
{
  VolumeGrid grid;
  mesh.GetAABB(grid.bb.bmin,grid.bb.bmax);
  //expand slightly
  grid.bb.bmin -= Vector3(0.5*gridRes);
  grid.bb.bmax += Vector3(0.5*gridRes);
  grid.ResizeByResolution(Vector3(gridRes));
  DensityEstimate_CenterShooting(mesh,grid.value,grid.bb);
  Vector3 mean(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    Vector3 c;
    i.getCellCenter(c);
    sum += *i;
    mean += *i*c;
  }
  if(sum == 0) return mean;
  mean /= sum;
  return mean;
}

///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center)
{
  VolumeGrid grid;
  mesh.GetAABB(grid.bb.bmin,grid.bb.bmax);
  //expand slightly
  grid.bb.bmin -= Vector3(0.5*gridRes);
  grid.bb.bmax += Vector3(0.5*gridRes);
  grid.ResizeByResolution(Vector3(gridRes));
  DensityEstimate_CenterShooting(mesh,grid.value,grid.bb);
  Matrix3 cov(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    Vector3 c;
    i.getCellCenter(c);
    Matrix3 temp; temp.setZero();
    AddOuterProduct(temp,c-center,c-center);
    temp *= *i;
    cov += temp;
    sum += *i;
  }
  if(sum == 0) return cov;
  cov *= 1.0/sum;
  return cov;
}

Matrix3 Inertia_Solid(const TriMesh& mesh,Real gridRes,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance_Solid(mesh,gridRes,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}
