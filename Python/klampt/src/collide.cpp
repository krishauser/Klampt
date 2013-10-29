#include "collide.h"
#include "pyerr.h"
#include <vector>
#include <list>
#include <geometry/CollisionMesh.h>
#include <utils/AnyValue.h>
#include <utils/SmartPointer.h>
using namespace std;
using namespace Math3D;
using namespace Geometry;

static vector<AnyValue> geoms;
static list<int> geomsDeleteList;

struct GeomCollisionQuery
{
  int geom1,geom2;
  CollisionMeshQuery meshQuery;
};

static vector<GeomCollisionQuery> queries;
static list<int> queriesDeleteList;

int newGeom()
{
  if(geomsDeleteList.empty()) {
    geoms.push_back(AnyValue());
    return (int)(geoms.size()-1);
  }
  else {
    int index = geomsDeleteList.front();
    geomsDeleteList.erase(geomsDeleteList.begin());
    geoms[index] = AnyValue();
    return index;
  }
}

void destroyGeom(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = AnyValue();
  geomsDeleteList.push_back(geom);
}

void makeTriMeshGeom(int geom,const char* fn)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = CollisionMesh();
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  assert(mesh!=NULL);
  if(!LoadMultipleTriMeshes(fn,*mesh)) {
    throw PyException("Unable to read tri mesh file");
  }
  mesh->InitCollisions();
}

void makeTriMeshGeom(int geom,const double* verts,const int* inds,int nv,int nt)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = CollisionMesh();
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  assert(mesh!=NULL);
  mesh->verts.resize(nv);
  mesh->tris.resize(nt);
  for(int i=0;i<nt*3;i++) 
    assert(0 <= inds[i] && inds[i] < nv);
  for(int i=0;i<nv;i++) 
    mesh->verts[i].set(verts[i*3],verts[i*3+1],verts[i*3+2]);
  for(int i=0;i<nt;i++) 
    mesh->tris[i].set(inds[i*3],inds[i*3+1],inds[i*3+2]);
  mesh->InitCollisions();
}


void setTriMeshTranslation(int geom,const double t[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  mesh->currentTransform.t.set(t);
}

void setTriMeshRotation(int geom,const double r[9])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  mesh->currentTransform.R.set(Matrix3(r));
}

void getTriMeshTranslation(int geom,double t[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  mesh->currentTransform.t.get(t);
}

void getTriMeshRotation(int geom,double r[9])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  mesh->currentTransform.R.get(r);
}

void getTriMeshBB(int geom,double bmin[3],double bmax[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  Box3D bb;
  GetBB(*mesh,bb);
  AABB3D aabb;
  bb.getAABB(aabb);
  aabb.bmin.get(bmin);
  aabb.bmax.get(bmax);
}

int getTriMeshNumVerts(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  return (int)mesh->verts.size();
}

int getTriMeshNumTris(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  return (int)mesh->tris.size();
}

double* getTriMeshVerts(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  return &mesh->verts[0].x;
}

int* getTriMeshTris(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* mesh=AnyCast<CollisionMesh>(&geoms[geom]);
  if(mesh==NULL) {
    throw PyException("Geom is not a tri mesh");
  }
  return &mesh->tris[0][0];
}

void makePointGeom(int geom,const double x[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = Point3D();  
  Point3D* s=AnyCast<Point3D>(&geoms[geom]);
  assert(s!=NULL);
  s->set(x);
}

void makeSphereGeom(int geom,const double c[3],double r)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = Sphere3D();  
  Sphere3D* s=AnyCast<Sphere3D>(&geoms[geom]);
  assert(s!=NULL);
  s->center.set(c);
  s->radius = r;
}

void makeRayGeom(int geom,const double s[3],const double d[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = Ray3D();  
  Ray3D* r=AnyCast<Ray3D>(&geoms[geom]);
  assert(r!=NULL);
  r->source.set(s);
  r->direction.set(d);
}

void makeLineGeom(int geom,const double s[3],const double d[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = Line3D();  
  Line3D* l=AnyCast<Line3D>(&geoms[geom]);
  assert(l!=NULL);
  l->source.set(s);
  l->direction.set(d);
}

void makeSegmentGeom(int geom,const double a[3],const double b[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = Segment3D();  
  Segment3D* s=AnyCast<Segment3D>(&geoms[geom]);
  assert(s!=NULL);
  s->a.set(a);
  s->b.set(b);
}

void makeAABBGeom(int geom,const double bmin[3],const double bmax[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = AABB3D();  
  AABB3D* s=AnyCast<AABB3D>(&geoms[geom]);
  assert(s!=NULL);
  s->bmin.set(bmin);
  s->bmax.set(bmax);  
}

bool checkCircularReference(int geom,int checkIndex)
{
  vector<int>* subgroup = AnyCast<vector<int> >(&geoms[checkIndex]);
  if(subgroup == NULL) return false;
  for(size_t i=0;i<subgroup->size();i++) {
    if((*subgroup)[i] == geom) return true;
    if(checkCircularReference(geom,(*subgroup)[i])) return true;
  }
  return false;
}

void makeGroupGeom(int geom,int* elements,int numelements)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  for(int i=0;i<numelements;i++) {
    if(elements[i] < 0 || elements[i] >= (int)geoms.size())
      throw PyException("Invalid sub-geom index");
    if(checkCircularReference(geom,elements[i]))
      throw PyException("Circular reference in group geom");
  }
  vector<int> geomlist(elements,elements+numelements);
  geoms[geom] = geomlist;
}

/********************** Point ********************/
inline bool Collide(const Point3D& p1,const Point3D& p2) { return p1.isEqual(p2,Epsilon); }
inline bool Collide(const Point3D& p,const Sphere3D& s) { return s.contains(p);}
inline bool Collide(const Point3D& p,const Segment3D& s) {
  Vector3 pt;
  s.closestPoint(p,pt);
  return p.isEqual(pt,Epsilon);
}
inline bool Collide(const Point3D& p,const Ray3D& r) { return FuzzyZero(r.distance(p)); }
inline bool Collide(const Point3D& p,const Line3D& l) { return FuzzyZero(l.distance(p)); }
inline bool Collide(const Point3D& p,const AABB3D& a) { return a.contains(p); }
inline bool Collide(const Point3D& p,const CollisionMesh& m) {
  Sphere3D s;
  s.center = p;
  s.radius = Epsilon;
  return Collide(m,s);
}

/********************** Sphere ********************/
inline bool Collide(const Sphere3D& s,const Point3D& p) { return Collide(p,s); }
inline bool Collide(const Sphere3D& s1,const Sphere3D& s2) { return s1.intersects(s2); }
inline bool Collide(const Sphere3D& s,const Segment3D& seg)
{
  Vector3 pt;
  seg.closestPoint(s.center,pt);
  return s.contains(pt);
}
inline bool Collide(const Sphere3D& s,const Ray3D& r) { return Abs(r.distance(s.center))<=s.radius; }
inline bool Collide(const Sphere3D& s,const Line3D& r) { return Abs(r.distance(s.center))<=s.radius; }
inline bool Collide(const Sphere3D& s,const AABB3D& a) { return a.distance(s.center) <= s.radius; }
inline bool Collide(const Sphere3D& s,const CollisionMesh& m) { return Collide(m,s); }

/********************** Segment ********************/
inline bool Collide(const Segment3D& s,const Point3D& p) { return Collide(p,s); }
inline bool Collide(const Segment3D& seg,const Sphere3D& s) { return Collide(s,seg); }
inline bool Collide(const Segment3D& s1,const Segment3D& s2) {
  Real t,u;
  s1.closestPoint(s2,t,u);
  Vector3 p1,p2;
  s1.eval(t,p1);
  s2.eval(u,p2);
  return p1.isEqual(p2,Epsilon); 
}
inline bool Collide(const Segment3D& s,const Ray3D& r) {
  Line3D sl;
  s.getLine(sl);
  Real t,u;
  sl.closestPoint(r,t,u);
  t = Clamp(t,0.0,1.0);
  u = Max(0.0,u);
  Vector3 p1,p2;
  sl.eval(t,p1);
  r.eval(u,p2);
  return p1.isEqual(p2,Epsilon);
}
inline bool Collide(const Segment3D& s,const Line3D& l) {
  Line3D sl;
  s.getLine(sl);
  Real t,u;
  sl.closestPoint(l,t,u);
  t = Clamp(t,0.0,1.0);
  Vector3 p1,p2;
  sl.eval(t,p1);
  l.eval(u,p2);
  return p1.isEqual(p2,Epsilon);
}
inline bool Collide(const Segment3D& seg,const AABB3D& a) { return seg.intersects(a); }
inline bool Collide(const Segment3D& s,const CollisionMesh& m) { Vector3 pt;  return Collide(m,s,pt)>=0; }

/********************** Ray ********************/
inline bool Collide(const Ray3D& r,const Point3D& p) { return Collide(p,r); }
inline bool Collide(const Ray3D& r,const Sphere3D& s) { return Collide(s,r); }
inline bool Collide(const Ray3D& r,const Segment3D& s) { return Collide(s,r); }
inline bool Collide(const Ray3D& r1,const Ray3D& r2) {
  Real t,u;
  r1.closestPoint(r2,t,u);
  t=Max(t,0.0);
  u=Max(u,0.0);
  Vector3 p1,p2;
  r1.eval(t,p1);
  r2.eval(u,p2);
  return p1.isEqual(p2,Epsilon);
}
inline bool Collide(const Ray3D& r,const Line3D& l) {
  Real t,u;
  r.closestPoint(l,t,u);
  t=Max(t,0.0);
  Vector3 p1,p2;
  r.eval(t,p1);
  l.eval(u,p2);
  return p1.isEqual(p2,Epsilon);
}
inline bool Collide(const Ray3D& r,const AABB3D& a) { return r.lineIntersects(a); }
inline bool Collide(const Ray3D& r,const CollisionMesh& m) { Vector3 pt; return RayCast(m,r,pt)>=0; }

/********************** Line ********************/
inline bool Collide(const Line3D& l,const Point3D& p) { return Collide(p,l); }
inline bool Collide(const Line3D& l,const Sphere3D& s) { return Collide(s,l); }
inline bool Collide(const Line3D& l,const Segment3D& s) { return Collide(s,l); }
inline bool Collide(const Line3D& l,const Ray3D& r) { return Collide(r,l); }
inline bool Collide(const Line3D& l1,const Line3D& l2) {
  Real t,u;
  l1.closestPoint(l2,t,u);
  Vector3 p1,p2;
  l1.eval(t,p1);
  l2.eval(u,p2);
  return p1.isEqual(p2,Epsilon);
}
inline bool Collide(const Line3D& l,const AABB3D& a) { return l.lineIntersects(a); }
inline bool Collide(const Line3D& l,const CollisionMesh& m)
{
   Ray3D fwd,rev;
   fwd.source = l.source;
   fwd.direction = l.direction;
   rev.source = l.source;
   rev.direction = -l.direction;
   Vector3 pt;
   return RayCast(m,fwd,pt)>=0 || RayCast(m,rev,pt)>=0;
}

/********************** AABB ********************/
inline bool Collide(const AABB3D& a,const Point3D& p) { return Collide(p,a); }
inline bool Collide(const AABB3D& a,const Sphere3D& s) { return Collide(s,a); }
inline bool Collide(const AABB3D& a,const Segment3D& seg) { return Collide(seg,a);}
inline bool Collide(const AABB3D& a,const Ray3D& r) { return Collide(r,a); }
inline bool Collide(const AABB3D& a,const Line3D& l) { return Collide(l,a); }
inline bool Collide(const AABB3D& a1,const AABB3D& a2) { return a1.intersects(a2); }
inline bool Collide(const AABB3D& a,const CollisionMesh& m) { return Collide(m,a); }

/********************** CollisionMesh ********************/

inline bool Collide(const CollisionMesh& m,const Point3D& p) { return Collide(p,m); }
inline bool Collide(const CollisionMesh& m,const Segment3D& s) { Vector3 pt;  return Collide(m,s,pt)>=0; }
inline bool Collide(const CollisionMesh& m,const Ray3D& r) { return Collide(r,m); }
inline bool Collide(const CollisionMesh& m,const Line3D& l) { return Collide(l,m); }
//rest are defined in CollisionMesh.h



bool collideMesh(CollisionMesh* m1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m1,*m2); }
  else if(p2) { return Collide(*m1,*p2);  }
  else if(s2) { return Collide(*m1,*s2);  }
  else if(seg2) { return Collide(*m1,*seg2); }
  else if(r2) { return Collide(*m1,*r2); }
  else if(l2) { return Collide(*m1,*l2);  }
  else if(a2) { return Collide(*m1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideMesh(m1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index");  }
  return false;
}

bool collidePoint(Point3D* p1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*p1);  }
  else if(p2) { return Collide(*p1,*p2);  }
  else if(s2) { return Collide(*s2,*p1);  }
  else if(seg2) { return Collide(*p1,*seg2);  }
  else if(r2) { return Collide(*p1,*r2);  }
  else if(l2) { return Collide(*p1,*l2);  }
  else if(a2) { return Collide(*p1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collidePoint(p1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index"); }
  return false;
}


bool collideSphere(Sphere3D* s1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*s1);  }
  else if(p2) { return Collide(*s1,*p2);  }
  else if(s2) { return Collide(*s1,*s2);  }
  else if(seg2) { return Collide(*s1,*seg2);  }
  else if(r2) { return Collide(*s1,*r2);  }
  else if(l2) { return Collide(*s1,*l2);  }
  else if(a2) {  return Collide(*s1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideSphere(s1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index");  }
  return false;
}

bool collideSegment(Segment3D* s1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*s1);  }
  else if(p2) { return Collide(*s1,*p2);  }
  else if(s2) { return Collide(*s1,*s2);  }
  else if(seg2) { return Collide(*s1,*seg2);  }
  else if(r2) { return Collide(*s1,*r2);  }
  else if(l2) { return Collide(*s1,*l2);  }
  else if(a2) {  return Collide(*s1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideSegment(s1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index");  }
  return false;
}

bool collideRay(Ray3D* s1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*s1);  }
  else if(p2) { return Collide(*s1,*p2);  }
  else if(s2) { return Collide(*s1,*s2);  }
  else if(seg2) { return Collide(*s1,*seg2);  }
  else if(r2) { return Collide(*s1,*r2);  }
  else if(l2) { return Collide(*s1,*l2);  }
  else if(a2) {  return Collide(*s1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideRay(s1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index");  }
  return false;
}

bool collideLine(Line3D* s1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*s1);  }
  else if(p2) { return Collide(*s1,*p2);  }
  else if(s2) { return Collide(*s1,*s2);  }
  else if(seg2) { return Collide(*s1,*seg2);  }
  else if(r2) { return Collide(*s1,*r2);  }
  else if(l2) { return Collide(*s1,*l2);  }
  else if(a2) {  return Collide(*s1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideLine(s1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index");  }
  return false;
}




bool collideAABB(AABB3D* a1,int geom2)
{
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m2=AnyCast<CollisionMesh>(&geoms[geom2]);
  Point3D* p2=AnyCast<Point3D>(&geoms[geom2]);
  Sphere3D* s2=AnyCast<Sphere3D>(&geoms[geom2]);
  Segment3D* seg2=AnyCast<Segment3D>(&geoms[geom2]);
  Ray3D* r2=AnyCast<Ray3D>(&geoms[geom2]);
  Line3D* l2=AnyCast<Line3D>(&geoms[geom2]);
  AABB3D* a2=AnyCast<AABB3D>(&geoms[geom2]);
  vector<int>* g2=AnyCast<vector<int> >(&geoms[geom2]);
  if(m2) { return Collide(*m2,*a1);  }
  else if(p2) { return Collide(*a1,*p2);  }
  else if(s2) { return Collide(*a1,*s2);  }
  else if(seg2) { return Collide(*a1,*seg2);  }
  else if(r2) { return Collide(*a1,*r2);  }
  else if(l2) { return Collide(*a1,*l2);  }
  else if(a2) { return Collide(*a1,*a2);  }
  else if(g2) { for(size_t i=0;i<g2->size();i++) return collideAABB(a1,(*g2)[i]);  }
  else { throw PyException("Invalid geom index"); }
  return false;
}

bool collideGroup(const vector<int>& group,int geom2)
{
  for(size_t i=0;i<group.size();i++)
    if(collide(group[i],geom2)) return true;
  return false;
}

bool collide(int geom1,int geom2)
{
  if(geom1 < 0 || geom1 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m1=AnyCast<CollisionMesh>(&geoms[geom1]);
  Point3D* p1=AnyCast<Point3D>(&geoms[geom1]);
  Sphere3D* s1=AnyCast<Sphere3D>(&geoms[geom1]);
  Segment3D* seg1=AnyCast<Segment3D>(&geoms[geom1]);
  Ray3D* r1=AnyCast<Ray3D>(&geoms[geom1]);
  Line3D* l1=AnyCast<Line3D>(&geoms[geom1]);
  AABB3D* a1=AnyCast<AABB3D>(&geoms[geom1]);
  vector<int>* g1=AnyCast<vector<int> >(&geoms[geom1]);
  if(m1) {
    return collideMesh(m1,geom2);
  }
  else if(p1) {
    return collidePoint(p1,geom2);
  }
  else if(s1) {
    return collideSphere(s1,geom2);
  }
  else if(seg1) {
    return collideSegment(seg1,geom2);
  }
  else if(r1) {
    return collideRay(r1,geom2);
  }
  else if(l1) {
    return collideLine(l1,geom2);
  }
  else if(a1) {
    return collideAABB(a1,geom2);
  }
  else if(g1) {
    return collideGroup(*g1,geom2);
  }
  else {
    throw PyException("Invalid geom index");
  }
}

bool withinTolerance(int geom1,int geom2,double tol)
{
  throw PyException("Not done yet");
}

double distance(int geom1,int geom2,double relErr,double absErr)
{
  throw PyException("Not done yet");
}

void closestPoints(int geom1,int geom2,double p1[3],double p2[3])
{
  throw PyException("Not done yet");
}

bool rayCast(int geom,const double s[3],const double d[3],double out[3])
{
  Ray3D r;
  r.source.set(s);
  r.direction.set(d);
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  CollisionMesh* m1=AnyCast<CollisionMesh>(&geoms[geom]);
  Point3D* p1=AnyCast<Point3D>(&geoms[geom]);
  Sphere3D* s1=AnyCast<Sphere3D>(&geoms[geom]);
  Segment3D* seg1=AnyCast<Segment3D>(&geoms[geom]);
  Ray3D* r1=AnyCast<Ray3D>(&geoms[geom]);
  Line3D* l1=AnyCast<Line3D>(&geoms[geom]);
  AABB3D* a1=AnyCast<AABB3D>(&geoms[geom]);
  vector<int>* g1=AnyCast<vector<int> >(&geoms[geom]);
  if(m1) {
    Vector3 pt;
    int tri=RayCast(*m1,r,pt);
    if(tri >= 0) {
      pt.get(out);
      return true;
    }
    else return false;
  }
  else {
    throw PyException("Non-mesh ray-casting not done yet");
    return false;
  }
}

int makeCollQuery(int geom1,int geom2)
{
  GeomCollisionQuery q;
  q.geom1 = geom1;
  q.geom2 = geom2;
  CollisionMesh* c1 = AnyCast<CollisionMesh>(&geoms[geom1]);
  CollisionMesh* c2 = AnyCast<CollisionMesh>(&geoms[geom2]);
  if(c1 && c2) {
    q.meshQuery = CollisionMeshQuery(*c1,*c2);
  }
  if(queriesDeleteList.empty()) {
    queries.push_back(q);
    return (int)(queries.size()-1);
  }
  else {
    int index = queriesDeleteList.front();
    queriesDeleteList.erase(queriesDeleteList.begin());
    queries[index] = q;
    return index;
  }  
}

void destroyCollQuery(int query)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  queries[query] = GeomCollisionQuery();
  queriesDeleteList.push_back(query);
}

bool queryCollide(int query)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  if(queries[query].meshQuery.m1) 
    return queries[query].meshQuery.Collide();
  return collide(queries[query].geom1,queries[query].geom2);
}

bool queryWithinTolerance(int query,double tol)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  if(queries[query].meshQuery.m1) 
    return queries[query].meshQuery.WithinDistance(tol);
  return withinTolerance(queries[query].geom1,queries[query].geom2,tol);
}

double queryDistance(int query,double relErr,double absErr)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  if(queries[query].meshQuery.m1) 
    return queries[query].meshQuery.Distance(relErr,absErr);
  return distance(queries[query].geom1,queries[query].geom2,relErr,absErr);
}

void queryClosestPoints(int query,double p1[3],double p2[3])
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  if(queries[query].meshQuery.m1) {
    Vector3 v1,v2;
    queries[query].meshQuery.ClosestPoints(v1,v2);
    v1.get(p1);
    v2.get(p2);
  }
  else
    closestPoints(queries[query].geom1,queries[query].geom2,p1,p2);
}

void queryTolerancePoints(int query,double p1[3],double p2[3])
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  if(queries[query].meshQuery.m1) {
    Vector3 v1,v2;
    queries[query].meshQuery.TolerancePoints(v1,v2);
    v1.get(p1);
    v2.get(p2);
  }
  else
    closestPoints(queries[query].geom1,queries[query].geom2,p1,p2);
}

void destroy()
{
  geoms.resize(0);
  geomsDeleteList.resize(0);
  queries.resize(0);
  queriesDeleteList.resize(0);
}
