#include "collide.h"
#include "pyerr.h"
#include <vector>
#include <list>
#include <geometry/CollisionMesh.h>
#include <geometry/AnyGeometry.h>
#include <meshing/IO.h>
#include <utils/AnyValue.h>
#include <utils/SmartPointer.h>
using namespace std;
using namespace Math3D;
using namespace Geometry;

static vector<AnyCollisionGeometry3D> geoms;
static list<int> geomsDeleteList;

struct GeomCollisionQuery
{
  int geom1,geom2;
  AnyCollisionQuery query;
};

static vector<GeomCollisionQuery> queries;
static list<int> queriesDeleteList;

int newGeom()
{
  if(geomsDeleteList.empty()) {
    geoms.push_back(AnyCollisionGeometry3D());
    return (int)(geoms.size()-1);
  }
  else {
    int index = geomsDeleteList.front();
    geomsDeleteList.erase(geomsDeleteList.begin());
    geoms[index] = AnyCollisionGeometry3D();
    return index;
  }
}

void destroyGeom(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = AnyCollisionGeometry3D();
  geomsDeleteList.push_back(geom);
}

bool loadGeom(int geom,const char* fn)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(!geoms[geom].Load(fn)) return false;
  geoms[geom].InitCollisions();
  return true;
}

void makeTriMeshGeom(int geom,const char* fn)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(!geoms[geom].Load(fn)) 
    throw PyException("Failed to load triangle mesh");
  geoms[geom].InitCollisions();
}

void makeTriMeshGeom(int geom,const double* verts,const int* inds,int nv,int nt)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  Meshing::TriMesh mesh;
  mesh.verts.resize(nv);
  mesh.tris.resize(nt);
  for(int i=0;i<nt*3;i++) 
    assert(0 <= inds[i] && inds[i] < nv);
  for(int i=0;i<nv;i++) 
    mesh.verts[i].set(verts[i*3],verts[i*3+1],verts[i*3+2]);
  for(int i=0;i<nt;i++) 
    mesh.tris[i].set(inds[i*3],inds[i*3+1],inds[i*3+2]);
  geoms[geom] = AnyCollisionGeometry3D(mesh);
  geoms[geom].InitCollisions();
}


void setTriMeshTranslation(int geom,const double t[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  RigidTransform T = geoms[geom].GetTransform();
  T.t.set(t);
  geoms[geom].SetTransform(T);
}

void setTriMeshRotation(int geom,const double r[9])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  RigidTransform T = geoms[geom].GetTransform();
  T.R.set(Matrix3(r));
  geoms[geom].SetTransform(T);
}

void getTriMeshTranslation(int geom,double t[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  RigidTransform T = geoms[geom].GetTransform();
  T.t.get(t);
}

void getTriMeshRotation(int geom,double r[9])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  RigidTransform T = geoms[geom].GetTransform();
  T.R.get(r);
}

void getTriMeshBB(int geom,double bmin[3],double bmax[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  AABB3D aabb = geoms[geom].GetAABB();
  aabb.bmin.get(bmin);
  aabb.bmax.get(bmax);
}

int getTriMeshNumVerts(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geoms[geom].type != AnyGeometry3D::TriangleMesh) {
    throw PyException("Geom is not a tri mesh");
  }
  return geoms[geom].AsTriangleMesh().verts.size();
}

int getTriMeshNumTris(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geoms[geom].type != AnyGeometry3D::TriangleMesh) {
    throw PyException("Geom is not a tri mesh");
  }
  return geoms[geom].AsTriangleMesh().tris.size();
}

double* getTriMeshVerts(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geoms[geom].type != AnyGeometry3D::TriangleMesh) {
    throw PyException("Geom is not a tri mesh");
  }
  return &geoms[geom].AsTriangleMesh().verts[0].x;
}

int* getTriMeshTris(int geom)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geoms[geom].type != AnyGeometry3D::TriangleMesh) {
    throw PyException("Geom is not a tri mesh");
  }
  return &geoms[geom].AsTriangleMesh().tris[0][0];
}

void makePointGeom(int geom,const double x[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  geoms[geom] = GeometricPrimitive3D(x);
}

void makeSphereGeom(int geom,const double c[3],double r)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  Sphere3D s;
  s.center.set(c);
  s.radius = r;
  geoms[geom] = GeometricPrimitive3D(s);
}

void makeRayGeom(int geom,const double s[3],const double d[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  throw PyException("Rays not supported yet");
  /*
  Ray3D r;
  r.source.set(s);
  r.direction.set(d);
  geoms[geom] = GeometricPrimitive3D(r); 
  */
}

void makeLineGeom(int geom,const double s[3],const double d[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  throw PyException("Lines not supported yet");
  /*
  Line3D l;
  l.source.set(s);
  l.direction.set(d);
  geoms[geom] = GeometricPrimitive3D(l);
  */
}

void makeSegmentGeom(int geom,const double a[3],const double b[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  Segment3D s;
  s.a.set(a);
  s.b.set(b);
  geoms[geom] = GeometricPrimitive3D(s); 
}

void makeAABBGeom(int geom,const double bmin[3],const double bmax[3])
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  AABB3D s;
  s.bmin.set(bmin);
  s.bmax.set(bmax);  
  geoms[geom] = GeometricPrimitive3D(s); 
}

void makeGroupGeom(int geom,int* elements,int numelements)
{
  if(geom < 0 || geom >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  vector<AnyGeometry3D> geomlist(numelements);
  for(int i=0;i<numelements;i++) {
    if(elements[i] < 0 || elements[i] >= (int)geoms.size())
      throw PyException("Invalid sub-geom index");
    geomlist[i] = geoms[elements[i]];
    geomlist[i].Transform(geoms[elements[i]].GetTransform());
  }
  geoms[geom] = AnyCollisionGeometry3D(geomlist);
}


bool collide(int geom1,int geom2)
{
  if(geom1 < 0 || geom1 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  return geoms[geom1].Collides(geoms[geom2]);
}

bool withinTolerance(int geom1,int geom2,double tol)
{
  if(geom1 < 0 || geom1 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  if(geom2 < 0 || geom2 >= (int)geoms.size()) 
    throw PyException("Invalid geom index");
  return geoms[geom1].WithinDistance(geoms[geom2],tol);
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
  Real distance;
  bool res=geoms[geom].RayCast(r,&distance);
  if(res) {
    Vector3 pt = (r.source + r.direction*distance);
    pt.get(out);
  }
  return res;
}

int makeCollQuery(int geom1,int geom2)
{
  GeomCollisionQuery q;
  q.geom1 = geom1;
  q.geom2 = geom2;
  q.query = AnyCollisionQuery(geoms[geom1],geoms[geom2]);
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
  return queries[query].query.Collide();
}

bool queryWithinTolerance(int query,double tol)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  return queries[query].query.WithinDistance(tol);
}

double queryDistance(int query,double relErr,double absErr)
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  return queries[query].query.Distance(relErr,absErr);
}

void queryClosestPoints(int query,double p1[3],double p2[3])
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  vector<Vector3> pts1,pts2;
  queries[query].query.InteractingPoints(pts1,pts2);
  if(pts1.size() != 1)
    throw PyException("Internal error");
  pts1[0].get(p1);
  pts2[0].get(p2);
}

void queryTolerancePoints(int query,double p1[3],double p2[3])
{
  if(query < 0 || query >= (int)queries.size()) 
    throw PyException("Invalid query index");
  vector<Vector3> pts1,pts2;
  queries[query].query.InteractingPoints(pts1,pts2);
  if(pts1.size() != 1)
    throw PyException("Internal error");
  pts1[0].get(p1);
  pts2[0].get(p2);
}

void destroy()
{
  geoms.resize(0);
  geomsDeleteList.resize(0);
  queries.resize(0);
  queriesDeleteList.resize(0);
}
