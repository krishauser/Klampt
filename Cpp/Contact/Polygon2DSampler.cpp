#include "Polygon2DSampler.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/errors.h>
#include <iostream>
using namespace Math;
using namespace Klampt;

//rayBound caps rays at the given distance

//sort points so rays are first c, then b
void Order(const PointRay2D*& a,const PointRay2D*& b,const PointRay2D*& c)
{
  if(a->isRay) {
    if(b->isRay) {
      if(!c->isRay) std::swap(a,c);
    }
    else {
      if(c->isRay) std::swap(a,b);
      else std::swap(a,c); 
    }
  }
  else {
    if(b->isRay && !c->isRay) std::swap(b,c);
  }
}


void Polygon2DSampler::Set(const vector<Vector2>& poly)
{
  //split the polygon into triangles, pick a triangle
  //with probability ~ area, pick a sample uniformly in the tri
  //triangle i consists of vertex 0,i+1,i+2

  tris.resize(0);
  Triangle2D t;
  for(size_t i=1;i+1<poly.size();i++) {
    t.a = poly[0];
    t.b = poly[i];
    t.c = poly[i+1];
    tris.push_back(t);
  }

  InitAreas();
}

void Polygon2DSampler::Set(const vector<PointRay2D>& poly, Real rayBound)
{
  //split the polygon into triangles, pick a triangle
  //with probability ~ area, pick a sample uniformly in the tri
  //triangle i consists of vertex 0,i+1,i+2

  tris.resize(0);
  Triangle2D t;

  if(poly.size()==0) {
    cout<<"Polygon2DSampler:: Warning, empty polygon!"<<endl;
    t.a = t.b = t.c = Vector2(Zero);
    tris.push_back(t);
  }
  else if(poly.size()==1) {
    t.a = t.b = t.c = poly[0];
    tris.push_back(t);
  }
  else if(poly.size()==2) {
    t.a = t.b = poly[0];
    t.c = poly[1];
    tris.push_back(t);
  }
  else {
    for(size_t i=1;i+1<poly.size();i++) {
      const PointRay2D* a=&poly[0];
      const PointRay2D* b=&poly[i];
      const PointRay2D* c=&poly[i+1];
      Order(a,b,c);
      if(a->isRay) {
	Assert(b->isRay && c->isRay);
	continue;
      }
      else if(b->isRay) {
	Assert(c->isRay);
	t.a = *a;
	t.b = *a; t.b.madd(*b,rayBound);
	t.c = *a; t.c.madd(*c,rayBound);
	tris.push_back(t);
      }
      else if(c->isRay) {
	t.a = *a;
	t.b = *b;
	t.c = *a; t.c.madd(*c,rayBound);
	tris.push_back(t);
	t.a = *b;
	t.b = *b; t.b.madd(*c,rayBound);
	t.c = *a; t.c.madd(*c,rayBound);
	tris.push_back(t);
      }
      else {
	t.a = *a;
	t.b = *b;
	t.c = *c;
	tris.push_back(t);
      }
    }
  }

  InitAreas();
}  



void Polygon3DSampler::Set(const vector<Vector3>& poly)
{
  tris.resize(0);
  Triangle3D t;
  for(size_t i=0;i+1<poly.size();i++) {
    t.a = poly[0];
    t.b = poly[i];
    t.c = poly[i+1];
    tris.push_back(t);
  }

  InitAreas();
}
