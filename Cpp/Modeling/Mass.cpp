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
  m(0,0)+=x.x*y.x;
  m(0,1)+=x.x*y.y;
  m(0,2)+=x.x*y.z;
  
  m(1,0)+=x.y*y.x;
  m(1,1)+=x.y*y.y;
  m(1,2)+=x.y*y.z;
  
  m(2,0)+=x.z*y.x;
  m(2,1)+=x.z*y.y;
  m(2,2)+=x.z*y.z;
}


//first and second moments integrated over mesh
Vector3 CenterOfMass(const TriMesh& mesh,Real surfaceFraction)
{
  Triangle3D tri;
  Vector3 center(Zero);
  if(mesh.tris.empty()) {
    return center;
  }

  if(surfaceFraction != 0) {
    Real sumArea=Zero;
    for(size_t i=0;i<mesh.tris.size();i++) {
      mesh.GetTriangle(i,tri);
      Real area=tri.area();
      center.madd(tri.a+tri.b+tri.c,area*third);
      sumArea += area;
    }
    Assert(sumArea != Zero);
    center *= surfaceFraction/sumArea;
  }

  if(surfaceFraction != 1) {
    //use tetrahedral integration
    Vector3 center2(Zero);
    Vector3 centroid(Zero);
    for(size_t i=0;i<mesh.verts.size();i++)
      centroid += mesh.verts[i];
    centroid /= mesh.verts.size();
    Real sumVolume=Zero;
    Real sixth = 1.0/6.0;
    for(const auto& t:mesh.tris) {
      Matrix3 mat;
      mat.setCol1(mesh.verts[t.a] - centroid);
      mat.setCol2(mesh.verts[t.b] - centroid);
      mat.setCol3(mesh.verts[t.c] - centroid);
      Real volume = mat.determinant()*sixth;
      center2.madd(mesh.verts[t.a]+mesh.verts[t.b]+mesh.verts[t.c]+centroid,volume*0.25);
      sumVolume += volume;
    }
    center2 *= (1.0-surfaceFraction)/sumVolume;
    center += center2;
  }
  return center;
}

Vector3 CenterOfMass(const Math3D::GeometricPrimitive3D& g,Real surfaceFraction)
{
  AABB3D bbox = g.GetAABB();
  return (bbox.bmin + bbox.bmax)*0.5;
}


Vector3 CenterOfMass(const Meshing::PointCloud3D& pc,Real surfaceFraction)
{
  Vector3 center(Zero);
  for(size_t i=0;i<pc.points.size();i++)
    center += pc.points[i];
  return center / pc.points.size();
}

Vector3 CenterOfMass(const Meshing::VolumeGrid& grid,Real surfaceFraction)
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


Vector3 CenterOfMass(const vector<AnyGeometry3D>& group,Real surfaceFraction)
{
  Vector3 sum(0.0);
  if(group.empty()) return sum;
  for(size_t i=0;i<group.size();i++)
    sum += CenterOfMass(group[i],surfaceFraction);
  return sum / group.size();
}


Vector3 CenterOfMass(const AnyGeometry3D& geom,Real surfaceFraction)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return CenterOfMass(geom.AsPrimitive(),surfaceFraction);
  case AnyGeometry3D::TriangleMesh:
    return CenterOfMass(geom.AsTriangleMesh(),surfaceFraction);
  case AnyGeometry3D::PointCloud:
    return CenterOfMass(geom.AsPointCloud(),surfaceFraction);
  case AnyGeometry3D::ImplicitSurface:
    return CenterOfMass(geom.AsImplicitSurface(),surfaceFraction);
  case AnyGeometry3D::Group:
    return CenterOfMass(geom.AsGroup(),surfaceFraction);
  case AnyGeometry3D::ConvexHull:
    {
      AnyGeometry3D mesh;
      geom.Convert(AnyGeometry3D::TriangleMesh,mesh);
      return CenterOfMass(mesh.AsTriangleMesh(),surfaceFraction);
    }
    break;
  }
  return Vector3(0.0);
}

Matrix3 Covariance(const TriMesh& mesh,const Vector3& center,Real surfaceFraction)
{
  Triangle3D tri;
  Real sumArea=Zero;
  Matrix3 A,temp;
  A.setZero();
  if(mesh.tris.empty()) {
    return A;
  }
  if(surfaceFraction != 0) {
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
      temp *= area/12.0;
      A += temp;
      sumArea += area;
    }
    Assert(sumArea != Zero);
    A *= surfaceFraction/sumArea;
  }
  if(surfaceFraction != 1) {
    Matrix3 A2;
    Vector3 diag(Zero);
    Vector3 offd(Zero);
    Real sumVolume=0;
    Real sixth = 1.0/6.0;
    for(size_t i=0;i<mesh.tris.size();i++) {
      mesh.GetTriangle(i,tri);
      tri.a-=center;
      tri.b-=center;
      tri.c-=center;
      Matrix3 mat;
      mat.setCol1(tri.a);
      mat.setCol2(tri.b);
      mat.setCol3(tri.c);
      Real volume = mat.determinant()*sixth;
      sumVolume += volume;
      for(int j=0;j < 3;j++)
      {
        int j1=(j+1)%3;   
        int j2=(j+2)%3;   
        diag[j] += (tri.a[j]*tri.b[j] + tri.b[j]*tri.c[j] + tri.c[j]*tri.a[j] + 
                    tri.a[j]*tri.a[j] + tri.b[j]*tri.b[j] + tri.c[j]*tri.c[j]  ) *volume; // divide by 60 later
        offd[j] += ((tri.a[j1]*tri.b[j2]  + tri.b[j1]*tri.c[j2]  + tri.c[j1]*tri.a[j2]  +
                    tri.a[j1]*tri.c[j2]  + tri.b[j1]*tri.a[j2]  + tri.c[j1]*tri.b[j2])*0.5  +
                    tri.a[j1]*tri.a[j2]+ tri.b[j1]*tri.b[j2]+ tri.c[j1]*tri.c[j2] ) *volume; // divide by 60 later
      }
    }
    diag /= sumVolume*(10.0);
    offd /= sumVolume*(10.0);
    A2(0,0) = diag.x;
    A2(1,1) = diag.y;
    A2(2,2) = diag.z;
    A2(0,1) = offd.z;
    A2(1,0) = offd.z;
    A2(1,2) = offd.x;
    A2(2,1) = offd.x;
    A2(0,2) = offd.y;
    A2(2,0) = offd.y;
    A2 *= (1.0-surfaceFraction);
    A += A2;
  }
  return A;
}

Matrix3 Covariance(const PointCloud3D& pc,const Vector3& center,Real surfaceFraction)
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

Matrix3 Covariance(const VolumeGrid& grid,const Vector3& center,Real surfaceFraction)
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


Matrix3 Covariance(const GeometricPrimitive3D& g,const Vector3& center,Real surfaceFraction)
{
  AABB3D bbox = g.GetAABB();
  Matrix3 res(0.0);
  res(0,0) = Sqr(bbox.bmax[0] - bbox.bmin[0])/12.0;
  res(1,1) = Sqr(bbox.bmax[1] - bbox.bmin[1])/12.0;
  res(2,2) = Sqr(bbox.bmax[2] - bbox.bmin[2])/12.0;
  return res;
}


Matrix3 Covariance(const vector<AnyGeometry3D>& group,const Vector3& center,Real surfaceFraction)
{
  Matrix3 res(0.0); 
  for(size_t i=0;i<group.size();i++) 
    res += Covariance(group[i],center,surfaceFraction);
  return res;
}


Matrix3 Covariance(const AnyGeometry3D& geom,const Vector3& center,Real surfaceFraction)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return Covariance(geom.AsPrimitive(),center,surfaceFraction);
  case AnyGeometry3D::TriangleMesh:
    return Covariance(geom.AsTriangleMesh(),center,surfaceFraction);
  case AnyGeometry3D::PointCloud:
    return Covariance(geom.AsPointCloud(),center,surfaceFraction);
  case AnyGeometry3D::ImplicitSurface:
    return Covariance(geom.AsImplicitSurface(),center,surfaceFraction);
  case AnyGeometry3D::Group:
    return Covariance(geom.AsGroup(),center,surfaceFraction);
  case AnyGeometry3D::ConvexHull:
    {
      AnyGeometry3D mesh;
      geom.Convert(AnyGeometry3D::TriangleMesh,mesh);
      return Covariance(mesh.AsTriangleMesh(),surfaceFraction);
    }
    break;
  }
  return Matrix3();
}

Matrix3 Inertia(const Math3D::GeometricPrimitive3D& geom,const Vector3& center,Real mass,Real surfaceFraction)
{
  Matrix3 cov=Covariance(geom,center,surfaceFraction);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}


Matrix3 Inertia(const TriMesh& mesh,const Vector3& center,Real mass,Real surfaceFraction)
{
  Matrix3 cov=Covariance(mesh,center,surfaceFraction);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}

Matrix3 Inertia(const PointCloud3D& mesh,const Vector3& center,Real mass,Real surfaceFraction)
{
  Matrix3 cov=Covariance(mesh,center,surfaceFraction);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}

Matrix3 Inertia(const VolumeGrid& mesh,const Vector3& center,Real mass,Real surfaceFraction)
{
  Matrix3 cov=Covariance(mesh,center,surfaceFraction);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}


Matrix3 Inertia(const vector<AnyGeometry3D>& group,const Vector3& center,Real mass,Real surfaceFraction)
{
  Matrix3 cov=Covariance(group,center,surfaceFraction);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}



Matrix3 Inertia(const AnyGeometry3D& geom,const Vector3& center,Real mass,Real surfaceFraction)
{
  switch(geom.type) {
  case AnyGeometry3D::Primitive:
    return Inertia(geom.AsPrimitive(),center,mass,surfaceFraction);
  case AnyGeometry3D::TriangleMesh:
    return Inertia(geom.AsTriangleMesh(),center,mass,surfaceFraction);
  case AnyGeometry3D::PointCloud:
    return Inertia(geom.AsPointCloud(),center,mass,surfaceFraction);
  case AnyGeometry3D::ImplicitSurface:
    return Inertia(geom.AsImplicitSurface(),center,mass,surfaceFraction);
  case AnyGeometry3D::Group:
    return Inertia(geom.AsGroup(),center,mass,surfaceFraction);
  case AnyGeometry3D::ConvexHull:
    {
      AnyGeometry3D mesh;
      geom.Convert(AnyGeometry3D::TriangleMesh,mesh);
      return Inertia(mesh.AsTriangleMesh(),surfaceFraction);
    }
    break;
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
