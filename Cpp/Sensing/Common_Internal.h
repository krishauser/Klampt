#ifndef KLAMPT_SENSING_COMMON_H
#define KLAMPT_SENSING_COMMON_H

#include <KrisLibrary/File.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/math.h>
#include <KrisLibrary/math3d/primitives.h>
#include <sstream>

using namespace Math;
using namespace Math3D;

#ifdef WIN32
static inline double round(double val) { return floor(val + 0.5); }
#endif //WIN32

//emulates a process that discretizes a continuous value into a digital one
//with resolution resolution, and variance variance
inline Real Discretize(Real value,Real resolution,Real variance)
{
  if(variance>0)
    value += RandGaussian()*Sqrt(variance);
  if(resolution>0)
    value = round(value/resolution)*resolution;
  return value;
}

inline Vector3 Discretize(const Vector3& value,const Vector3& resolution,const Vector3& variance)
{
  Vector3 res;
  res.x = Discretize(value.x,resolution.x,variance.x);
  res.y = Discretize(value.y,resolution.y,variance.y);
  res.z = Discretize(value.z,resolution.z,variance.z);
  return res;
}


inline bool WriteFile(File& f,const string& s)
{
  size_t n=s.length();
  if(!WriteFile(f,n)) return false;
  if(n > 0)
    if(!WriteArrayFile(f,&s[0],s.length())) return false;
  return true;
}

inline bool ReadFile(File& f,string& s)
{
  size_t n;
  if(!ReadFile(f,n)) return false;
  s.resize(n);
  if(n > 0)
    if(!ReadArrayFile(f,&s[0],n)) return false;
  return true;
}

template <class T>
inline bool WriteFile(File& f,const vector<T>& v)
{
  if(!WriteFile(f,(int)v.size())) return false;
  if(!v.empty())
    if(!WriteArrayFile(f,&v[0],v.size())) return false;
  return true;
}

template <class T>
inline bool ReadFile(File& f,vector<T>& v)
{
  int n;
  if(!ReadFile(f,n)) return false;
  v.resize(0);
  if(n > 0) {
    v.resize(n);
    if(!ReadArrayFile(f,&v[0],n)) return false;
    return true;
  }
  return false;
}

#endif