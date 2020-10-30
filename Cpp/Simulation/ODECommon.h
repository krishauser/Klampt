#ifndef ODE_INTERFACE_COMMON_H
#define ODE_INTERFACE_COMMON_H

#include <KrisLibrary/math3d/primitives.h>
using namespace Math3D;

inline void CopyVector(dVector4 x,const Vector3& v)
{
  x[0] = v.x;
  x[1] = v.y;
  x[2] = v.z;
  x[3] = 1;
}

inline void CopyVector3(dReal* x,const Vector3& v)
{
  x[0] = v.x;
  x[1] = v.y;
  x[2] = v.z;
}

inline void CopyVector(Vector3& x,const dVector4 v)
{
  x.set(v[0],v[1],v[2]);
}

inline void CopyMatrix(dMatrix3 x,const Matrix3& v)
{
  //column major 4x3?
  for(int i=0;i<3;i++) 
    for(int j=0;j<3;j++)
      x[i*4+j] = v(i,j);
  x[3] = x[7] = x[11] = 0;
}

inline void CopyMatrix(Matrix3& x,const dMatrix3 v)
{
  for(int i=0;i<3;i++) 
    for(int j=0;j<3;j++)
      x(i,j) = v[i*4+j];
}

//column matrix format
inline void CopyMatrixCM(dReal* x,const Matrix4& v)
{
  //column major 4x4?
  for(int i=0;i<4;i++) 
    for(int j=0;j<4;j++)
      x[i+j*4] = v(i,j);
}

//column matrix format
inline void CopyMatrixCM(Matrix4& x,const dReal* v)
{
  for(int i=0;i<4;i++) 
    for(int j=0;j<4;j++)
      x(i,j) = v[i+j*4];
}


#endif
