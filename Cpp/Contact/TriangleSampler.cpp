#include "TriangleSampler.h"
#include <KrisLibrary/math/sample.h>
#include <iostream>
#include <stdio.h>

using namespace Klampt;

void Triangle2DSampler::InitAreas()
{
  areas.resize(tris.size());
  sumAreas.resize(tris.size());
  for(size_t i=0;i<areas.size();i++) {
    areas[i] = tris[i].area();
    if(i==0) sumAreas[i] = areas[i];
    else sumAreas[i] = sumAreas[i-1]+areas[i];
  }
}  

void Triangle2DSampler::SamplePointOnTri(int tri,Vector2& x) const
{
  //now sample the triangle
  Vector2 uv;
  SampleTriangle(uv.x,uv.y);
  x = tris[tri].planeCoordsToPoint(uv);
}

void Triangle2DSampler::SamplePoint(Vector2& x) const
{
  if(tris.empty()) {
    cerr<<"Triangle2DSampler: tris are empty!"<<endl;
    cerr<<"Press any key to continue"<<endl;
    getchar();
    x.setZero();
    return;
  }
  //Assert(!tris.empty());
  int t=SampleTri();
  SamplePointOnTri(t,x);
}

void Triangle2DSampler::SamplePoints(int num,std::vector<Vector2>& pts) const
{
  pts.resize(num);
  if(num < 20 && num < (int)areas.size()) {
    for(int i=0;i<num;i++)
      SamplePoint(pts[i]);
  }
  else {
    vector<int> counts(areas.size());
    RandomAllocate(counts,num,areas);
    int k=0;
    for(size_t i=0;i<counts.size();i++) {
      for(int s=0;s<counts[i];s++) {
	SamplePointOnTri(i,pts[k]);
	k++;
      }
    }
  }
}

void Triangle2DSampler::SamplePoints(int num,std::vector<int>& tris,std::vector<Vector2>& pts) const
{
  tris.resize(num);
  pts.resize(num);
  if(num < 20 && num < (int)areas.size()) {
    for(int i=0;i<num;i++) {
      tris[i] = SampleTri();
      SamplePointOnTri(tris[i],pts[i]);
    }
  }
  else {
    vector<int> counts(areas.size());
    RandomAllocate(counts,num,areas);
    int k=0;
    for(size_t i=0;i<counts.size();i++) {
      for(int s=0;s<counts[i];s++) {
	tris[k] = i;
	SamplePointOnTri(i,pts[k]);
	k++;
      }
    }
  }
}


void Triangle3DSampler::InitAreas()
{
  areas.resize(tris.size());
  sumAreas.resize(tris.size());
  for(size_t i=0;i<areas.size();i++) {
    areas[i] = tris[i].area();
    if(i==0) sumAreas[i] = areas[i];
    else sumAreas[i] = sumAreas[i-1]+areas[i];
  }
}  

void Triangle3DSampler::SamplePointOnTri(int tri,Vector3& x) const
{
  //now sample the triangle
  Vector2 uv;
  SampleTriangle(uv.x,uv.y);
  x = tris[tri].planeCoordsToPoint(uv);
}

void Triangle3DSampler::SamplePoint(Vector3& x) const
{
  if(tris.empty()) {
    cerr<<"Triangle3DSampler: tris are empty!"<<endl;
    cerr<<"Press any key to continue"<<endl;
    getchar();
    x.setZero();
    return;
  }
  //Assert(!tris.empty());
  int t=SampleTri();
  SamplePointOnTri(t,x);
}

void Triangle3DSampler::SamplePoints(int num,std::vector<Vector3>& pts) const
{
  pts.resize(num);
  if(num < 20 && num < (int)areas.size()) {
    for(int i=0;i<num;i++)
      SamplePoint(pts[i]);
  }
  else {
    vector<int> counts(areas.size());
    RandomAllocate(counts,num,areas);
    int k=0;
    for(size_t i=0;i<counts.size();i++) {
      for(int s=0;s<counts[i];s++) {
	SamplePointOnTri(i,pts[k]);
	k++;
      }
    }
  }
}


void Triangle3DSampler::SamplePoints(int num,std::vector<int>& tris,std::vector<Vector3>& pts) const
{
  tris.resize(num);
  pts.resize(num);
  if(num < 20 && num < (int)areas.size()) {
    for(int i=0;i<num;i++) {
      tris[i] = SampleTri();
      SamplePointOnTri(tris[i],pts[i]);
    }
  }
  else {
    vector<int> counts(areas.size());
    RandomAllocate(counts,num,areas);
    int k=0;
    for(size_t i=0;i<counts.size();i++) {
      for(int s=0;s<counts[i];s++) {
	tris[k] = i;
	SamplePointOnTri(i,pts[k]);
	k++;
      }
    }
  }
}
