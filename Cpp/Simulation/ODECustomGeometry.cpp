#include "ODECustomGeometry.h"
#include "ODECommon.h"
#include <KrisLibrary/geometry/CollisionPointCloud.h>
#include <ode/collision.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/meshing/IO.h>
#include <iostream>
using namespace std;

DECLARE_LOGGER(ODESimulator)

//if a normal has this length then it is ignored
const static Real gZeroNormalTolerance = 1e-4;

//if two contact points are closer than this threshold, will try to look
//at the local geometry to derive a contact normal
const static Real gNormalFromGeometryTolerance = 1e-5;
//const static Real gNormalFromGeometryTolerance = 1e-2;

//if a barycentric coordinate is within this tolerance of zero, it will be
//considered a zero
const static Real gBarycentricCoordZeroTolerance = 1e-3;

//if true, takes the ODE tolerance points and performs additional contact
//checking -- useful for flat contacts
const static bool gDoTriangleTriangleCollisionDetection = false;

//doesn't consider unique contact points if they are between this tolerance
const static Real cptol=1e-5;

static bool gCustomGeometryMeshesIntersect = false;

int gdCustomGeometryClass = 0;

void ReverseContact(dContactGeom& contact)
{
  std::swap(contact.g1,contact.g2);
  for(int k=0;k<3;k++) contact.normal[k]*=-1.0;
  std::swap(contact.side1,contact.side2);
}

dGeomID dCreateCustomGeometry(AnyCollisionGeometry3D* geometry,Real outerMargin)
{
  dGeomID geom = dCreateGeom(gdCustomGeometryClass);
  CustomGeometryData* data = dGetCustomGeometryData(geom);
  data->geometry = geometry;
  data->outerMargin = outerMargin;
  data->odeOffset.setZero();
  dGeomSetCategoryBits(geom,0xffffffff);
  dGeomSetCollideBits(geom,0xffffffff);
  dGeomEnable(geom);
  return geom;
}

CustomGeometryData* dGetCustomGeometryData(dGeomID o)
{
  return (CustomGeometryData*)dGeomGetClassData(o);
}



//1 = pt, 2 = edge, 3 = face
inline int FeatureType(const Vector3& b) 
{
  int type=0;
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) type++;
  return 3-type;
}

int EdgeIndex(const Vector3& b)
{
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

int VertexIndex(const Vector3& b)
{
  if(FuzzyEquals(b.x,One,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyEquals(b.y,One,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyEquals(b.z,One,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

Vector3 VertexNormal(CollisionMesh& m,int tri,int vnum)
{
  if(m.incidentTris.empty()) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"VertexNormal: mesh is not properly initialized with incidentTris array?");
    m.CalcIncidentTris();
    //return Vector3(0.0);
    //FatalError("VertexNormal: mesh is not properly initialized with incidentTris array?");
  }
  Assert(vnum >= 0 && vnum < 3);
  int v=m.tris[tri][vnum];
  Assert(v >= 0 && v < m.incidentTris.size());
  if(m.incidentTris[v].empty()) return Vector3(0.0);
  Vector3 n(Zero);
  for(size_t i=0;i<m.incidentTris[v].size();i++)
    n += m.TriangleNormal(m.incidentTris[v][i]);
  n.inplaceNormalize();
  return m.currentTransform.R*n;
}

Vector3 EdgeNormal(CollisionMesh& m,int tri,int e)
{
  if(m.triNeighbors.empty()) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"EdgeNormal: Warning, mesh is not properly initialized with triNeighbors");
    m.CalcTriNeighbors();
    //return Vector3(0.0);
  }
  Assert(!m.triNeighbors.empty());
  Vector3 n=m.TriangleNormal(tri);
  if(m.triNeighbors[tri][e] != -1) {
    n += m.TriangleNormal(m.triNeighbors[tri][e]);
    n.inplaceNormalize();
  }
  return m.currentTransform.R*n;
}

///Compute normal from mesh geometry: returns the local normal needed for
///triangle 1 on m1 to get out of triangle 2 on m2.
///p1 and p2 are given in local coordinates
Vector3 ContactNormal(CollisionMesh& m1,CollisionMesh& m2,const Vector3& p1,const Vector3& p2,int t1,int t2)
{
  Triangle3D tri1,tri2;
  m1.GetTriangle(t1,tri1);
  m2.GetTriangle(t2,tri2);
  Vector3 b1=tri1.barycentricCoords(p1);
  Vector3 b2=tri2.barycentricCoords(p2);
  int type1=FeatureType(b1),type2=FeatureType(b2);
  switch(type1) {
  case 1:  //pt
    switch(type2) {
    case 1:  //pt
      //get the triangle normals
      {
        //printf("ODECustomMesh: Point-point contact\n");
        Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
        Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
        n2 -= n1;
        n2.inplaceNormalize();
        return n2;
      }
      break;
    case 2:  //edge
      {
        //printf("ODECustomMesh: Point-edge contact\n");
        Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
        int e = EdgeIndex(b2);
        Segment3D s = tri2.edge(e);
        Vector3 ev = m2.currentTransform.R*(s.b-s.a);
        Vector3 n2 = EdgeNormal(m2,t2,e);
        n2-=(n1-ev*ev.dot(n1)/ev.dot(ev)); //project onto normal
        n2.inplaceNormalize();
        return n2;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 2:  //edge
    switch(type2) {
    case 1:  //pt
      {
        //printf("ODECustomMesh: Edge-point contact\n");
        Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
        int e = EdgeIndex(b1);
        Segment3D s = tri1.edge(e);
        Vector3 ev = m1.currentTransform.R*(s.b-s.a);
        Vector3 n1 = EdgeNormal(m1,t1,e);
        n2 = (n2-ev*ev.dot(n2)/ev.dot(ev))-n1; //project onto normal
        n2.inplaceNormalize();
        return n2;
      }
      break;
    case 2:  //edge
      {
        //printf("ODECustomMesh: Edge-edge contact\n");
        int e = EdgeIndex(b1);
        Segment3D s1 = tri1.edge(e);
        Vector3 ev1 = m1.currentTransform.R*(s1.b-s1.a);
        ev1.inplaceNormalize();
        e = EdgeIndex(b2);
        Segment3D s2 = tri2.edge(e);
        Vector3 ev2 = m2.currentTransform.R*(s2.b-s2.a);
        ev2.inplaceNormalize();
        Vector3 n; 
        n.setCross(ev1,ev2);
        Real len = n.length();
        if(len < gZeroNormalTolerance) {
          //hmm... edges are parallel?
        }
        n /= len;
        //make sure the normal direction points into m1 and out of m2
        if(n.dot(m1.currentTransform*s1.a) < n.dot(m2.currentTransform*s2.a))
          n.inplaceNegative();
        /*
        if(n.dot(m1.currentTransform.R*tri1.normal()) > 0.0) {
          if(n.dot(m2.currentTransform.R*tri2.normal()) > 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
          n.inplaceNegative();
        }
        else {
          if(n.dot(m2.currentTransform.R*tri2.normal()) < 0.0) {
            printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
          }
        }
        */
        //cout<<"Edge vector 1 "<<ev1<<", vector 2" <<ev2<<", normal: "<<n<<endl;
        return n;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 3:  //face
    if(type2 == 3) {
      //printf("ODECustomMesh: Warning, face-face contact?\n");
    }
    return m1.currentTransform.R*(-tri1.normal());
  }
  static int warnedCount = 0;
  if(warnedCount % 10000 == 0) 
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODECustomMesh: Warning, degenerate triangle, types "<<type1<<" "<<type2);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

//Returns a contact normal for the closest point to the triangle t.  p is the point on the triangle.
//The direction is the one in which triangle 1 can move to get away from closestpt
Vector3 ContactNormal(CollisionMesh& m,const Vector3& p,int t,const Vector3& closestPt)
{
  Triangle3D tri;
  m.GetTriangle(t,tri);
  Vector3 b=tri.barycentricCoords(p);
  int type=FeatureType(b);
  switch(type) {
  case 1:  //pt
    //get the triangle normal
    {
      Vector3 n = VertexNormal(m,t,VertexIndex(b));
      n.inplaceNegative();
      return n;
    }
    break;
  case 2:  //edge
    {
      int e = EdgeIndex(b);
      Vector3 n = EdgeNormal(m,t,e);
      n.inplaceNegative();
      return n;
    }
    break;
  case 3:  //face
    return m.currentTransform.R*(-tri.normal());
  }
  static int warnedCount = 0;
  if(warnedCount % 10000 == 0) 
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODECustomMesh: Warning, degenerate triangle, types "<<type);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

int MeshMeshCollide(CollisionMesh& m1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  CollisionMeshQuery q(m1,m2);
  bool res=q.WithinDistanceAll(outerMargin1+outerMargin2);
  if(!res) {
    return 0;
  }

  vector<int> t1,t2;
  vector<Vector3> cp1,cp2;
  q.TolerancePairs(t1,t2);
  q.TolerancePoints(cp1,cp2);

  const RigidTransform& T1 = m1.currentTransform;
  const RigidTransform& T2 = m2.currentTransform;
  RigidTransform T21; T21.mulInverseA(T1,T2);
  RigidTransform T12; T12.mulInverseA(T2,T1);
  Real tol = outerMargin1+outerMargin2;
  Real tol2 = Sqr(tol);

  size_t imax=t1.size();
  Triangle3D tri1,tri2,tri1loc,tri2loc;
  if(gDoTriangleTriangleCollisionDetection) {
    //test if more triangle vertices are closer than tolerance
    for(size_t i=0;i<imax;i++) {
      m1.GetTriangle(t1[i],tri1);
      m2.GetTriangle(t2[i],tri2);
      
      tri1loc.a = T12*tri1.a;
      tri1loc.b = T12*tri1.b;
      tri1loc.c = T12*tri1.c;
      tri2loc.a = T21*tri2.a;
      tri2loc.b = T21*tri2.b;
      tri2loc.c = T21*tri2.c;
      bool usecpa,usecpb,usecpc,usecpa2,usecpb2,usecpc2;
      Vector3 cpa = tri1.closestPoint(tri2loc.a);
      Vector3 cpb = tri1.closestPoint(tri2loc.b);
      Vector3 cpc = tri1.closestPoint(tri2loc.c);
      Vector3 cpa2 = tri2.closestPoint(tri1loc.a);
      Vector3 cpb2 = tri2.closestPoint(tri1loc.b);
      Vector3 cpc2 = tri2.closestPoint(tri1loc.c);
      usecpa = (cpa.distanceSquared(tri2loc.a) < tol2);
      usecpb = (cpb.distanceSquared(tri2loc.b) < tol2);
      usecpc = (cpc.distanceSquared(tri2loc.c) < tol2);
      usecpa2 = (cpa2.distanceSquared(tri1loc.a) < tol2);
      usecpb2 = (cpb2.distanceSquared(tri1loc.b) < tol2);
      usecpc2 = (cpc2.distanceSquared(tri1loc.c) < tol2);
      //if already existing, disable it
      if(usecpa && cpa.isEqual(cp1[i],cptol)) usecpa=false;
      if(usecpb && cpb.isEqual(cp1[i],cptol)) usecpb=false;
      if(usecpc && cpc.isEqual(cp1[i],cptol)) usecpc=false;
      if(usecpa2 && cpa2.isEqual(cp2[i],cptol)) usecpa2=false;
      if(usecpb2 && cpb2.isEqual(cp2[i],cptol)) usecpb2=false;
      if(usecpc2 && cpc2.isEqual(cp2[i],cptol)) usecpc2=false;
      
      if(usecpa) {
        if(usecpb && cpb.isEqual(cpa,cptol)) usecpb=false;
        if(usecpc && cpc.isEqual(cpa,cptol)) usecpc=false;
      }
      if(usecpb) {
        if(usecpc && cpc.isEqual(cpb,cptol)) usecpc=false;
      }
      if(usecpa2) {
        if(usecpb2 && cpb2.isEqual(cpa2,cptol)) usecpb2=false;
        if(usecpc2 && cpc2.isEqual(cpa2,cptol)) usecpc2=false;
      }
      if(usecpb) {
        if(usecpc2 && cpc.isEqual(cpb2,cptol)) usecpc2=false;
      }
      
      if(usecpa) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpa);
        cp2.push_back(tri2.a);
      }
      if(usecpb) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpb);
        cp2.push_back(tri2.b);
      }
      if(usecpc) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(cpc);
        cp2.push_back(tri2.c);
      }
      if(usecpa2) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.a);
        cp2.push_back(cpa2);
      }
      if(usecpb2) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.b);
        cp2.push_back(cpb2);
      }
      if(usecpc2) {
        t1.push_back(t1[i]);
        t2.push_back(t2[i]);
        cp1.push_back(tri1.c);
        cp2.push_back(cpc2);
      }
    }
    /*
    if(t1.size() != imax)
      printf("ODECustomMesh: Triangle vert checking added %d points\n",t1.size()-imax);
    */
    //getchar();
  }

  imax = t1.size();
  static int warnedCount = 0;
  for(size_t i=0;i<imax;i++) {
    m1.GetTriangle(t1[i],tri1);
    m2.GetTriangle(t2[i],tri2);

    //tri1loc.a = T12*tri1.a;
    //tri1loc.b = T12*tri1.b;
    //tri1loc.c = T12*tri1.c;
    tri2loc.a = T21*tri2.a;
    tri2loc.b = T21*tri2.b;
    tri2loc.c = T21*tri2.c;
    Segment3D s;
    //this is here to avoid degenerate triangles
    bool collides;
    Vector3 n1,n2;
    n1.setCross(tri1.b-tri1.a,tri1.c-tri1.a);
    n2.setCross(tri2.b-tri2.a,tri2.c-tri2.a);
    if(n2.normSquared() > n1.normSquared())
      collides = tri2loc.intersects(tri1,s);
    else
      collides = tri1.intersects(tri2loc,s);
    if(collides) { 
      gCustomGeometryMeshesIntersect = true;
      if(warnedCount % 1000 == 0) {
        LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODECustomMesh: Triangles penetrate margin "<<outerMargin1<<"+"<<outerMargin2<<": can't trust contact detector");
      }
      /*
      cout<<"Triangle 1"<<endl;
      cout<<"  "<<tri1.a<<endl;
      cout<<"  "<<tri1.b<<endl;
      cout<<"  "<<tri1.c<<endl;
      cout<<"intersects triangle 2"<<endl;
      cout<<"  "<<tri2loc.a<<endl;
      cout<<"  "<<tri2loc.b<<endl;
      cout<<"  "<<tri2loc.c<<endl;
      */
      warnedCount++;
      /*
      //the two triangles intersect! can't trust results of PQP
      t1[i] = t1.back();
      t2[i] = t2.back();
      cp1[i] = cp1.back();
      cp2[i] = cp2.back();
      i--;
      imax--;
      */
    }
  }
  if(t1.size() != imax) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"ODECustomMesh: "<<t1.size()-imax<<" candidate points were removed due to mesh collision");
    t1.resize(imax);
    t2.resize(imax);
    cp1.resize(imax);
    cp2.resize(imax);
  }
  
  int k=0;  //count the # of contact points added
  for(size_t i=0;i<cp1.size();i++) {
    Vector3 p1 = T1*cp1[i];
    Vector3 p2 = T2*cp2[i];
    Vector3 n=p1-p2;
    Real d = n.norm();
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      n = ContactNormal(m1,m2,cp1[i],cp2[i],t1[i],t2[i]);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      printf("Skipping contact due to irregular distance between points %g\n",d);
      cout<<"  cp 1 "<<p1<<endl;
      cout<<"  cp 2 "<<p2<<endl;
      cout<<"  local cp 1 "<<cp1[i]<<endl;
      cout<<"  local cp 2 "<<cp2[i]<<endl;
      continue;
    }
    else n /= d;
    //check for invalid normals
    Real len=n.length();
    if(len < gZeroNormalTolerance || !IsFinite(len)) {
      printf("Skipping contact due to irregular normal length %g\n",len);
      continue;
    }
    //cout<<"Local Points "<<cp1[i]<<", "<<cp2[i]<<endl;
    //cout<<"Points "<<p1<<", "<<p2<<endl;
    //Real utol = (tol)*0.5/d + 0.5;
    //CopyVector(contact[k].pos,p1+utol*(p2-p1));
    CopyVector(contact[k].pos,0.5*(p1+p2) + ((outerMargin2 - outerMargin1)*0.5)*n);
    CopyVector(contact[k].normal,n);
    contact[k].depth = tol - d;
    if(contact[k].depth < 0) contact[k].depth = 0;
    //cout<<"Normal "<<n<<", depth "<<contact[i].depth<<endl;
    //getchar();
    k++;
    if(k == maxcontacts) break;
  }
  return k;
}

//defined in KrisLibrary/geometry/AnyGeometry.cpp
bool Collides(const CollisionPointCloud& a,Real margin,const CollisionMesh& b,
              vector<int>& elements1,vector<int>& elements2,size_t maxContacts);


int MeshPointCloudCollide(CollisionMesh& m1,Real outerMargin1,CollisionPointCloud& pc2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  Real tol=outerMargin1+outerMargin2;
  vector<int> points;
  vector<int> tris;
  if(!Collides(pc2,tol,m1,points,tris,maxcontacts)) return 0;
  Assert(points.size()==tris.size());
  Triangle3D tri,triw;
  int k=0;
  for(size_t i=0;i<points.size();i++) {
    Vector3 pw = pc2.currentTransform*pc2.points[points[i]];
    m1.GetTriangle(tris[i],tri);
    triw.a = m1.currentTransform*tri.a;
    triw.b = m1.currentTransform*tri.b;
    triw.c = m1.currentTransform*tri.c;
    Vector3 cp = triw.closestPoint(pw);
    Vector3 n = cp - pw;
    Real d = n.length();
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp,plocal);
      n = ContactNormal(m1,plocal,tris[i],pw);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      continue;
    }
    else n /= d;
    //migrate the contact point to the center of the overlap region
    CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
    CopyVector(contact[k].normal,n);
    contact[k].depth = tol - d;
    k++;
    if(k == maxcontacts) break;
  }
  /*
  Real tol = outerMargin1 + outerMargin2;
  Box3D mbb,mbb_pclocal;
  GetBB(m1,mbb);
  RigidTransform Tw_pc;
  Tw_pc.setInverse(pc2.currentTransform);
  mbb_pclocal.setTransformed(mbb,Tw_pc);
  AABB3D maabb_pclocal;
  mbb_pclocal.getAABB(maabb_pclocal);
  maabb_pclocal.bmin -= Vector3(tol);
  maabb_pclocal.bmax += Vector3(tol);
  maabb_pclocal.setIntersection(pc2.bblocal);
  list<void*> nearpoints;
  pc2.grid.BoxItems(Vector(3,maabb_pclocal.bmin),Vector(3,maabb_pclocal.bmax),nearpoints);
  int k=0;
  vector<int> tris;
  Triangle3D tri,triw;
  for(list<void*>::iterator i=nearpoints.begin();i!=nearpoints.end();i++) {
    Vector3 pcpt = *reinterpret_cast<Vector3*>(*i);
    Vector3 pw = pc2.currentTransform*pcpt;
    NearbyTriangles(m1,pw,tol,tris,maxcontacts-k);
    for(size_t j=0;j<tris.size();j++) {   
      m1.GetTriangle(tris[j],tri);
      triw.a = m1.currentTransform*tri.a;
      triw.b = m1.currentTransform*tri.b;
      triw.c = m1.currentTransform*tri.c;
      Vector3 cp = triw.closestPoint(pw);
      Vector3 n = cp - pw;
      Real d = n.length();
      if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
        Vector3 plocal;
        m1.currentTransform.mulInverse(cp,plocal);
        n = ContactNormal(m1,plocal,tris[j],pw);
      }
      else if(d > tol) {  //some penetration -- we can't trust the result of PQP
        continue;
      }
      else n /= d;
      //migrate the contact point to the center of the overlap region
      CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
      CopyVector(contact[k].normal,n);
      contact[k].depth = tol - d;
      k++;
      if(k == maxcontacts) break;
    }
  }
  return k;
  */
  return k;
}

int PointCloudMeshCollide(CollisionPointCloud& pc1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  int num = MeshPointCloudCollide(m2,outerMargin2,pc1,outerMargin1,contact,maxcontacts);
  for(int i=0;i<num;i++) ReverseContact(contact[i]);
  return num;
}

int MeshSphereCollide(CollisionMesh& m1,Real outerMargin1,const Sphere3D& s,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  Real tol = outerMargin1 + outerMargin2;
  Triangle3D tri;
  vector<int> tris;
  int k=0;
  NearbyTriangles(m1,s.center,s.radius+tol,tris,maxcontacts);
  for(size_t j=0;j<tris.size();j++) {   
    m1.GetTriangle(tris[j],tri);
    tri.a = m1.currentTransform*tri.a;
    tri.b = m1.currentTransform*tri.b;
    tri.c = m1.currentTransform*tri.c;

    Vector3 cp = tri.closestPoint(s.center);
    Vector3 n = cp - s.center;
    Real nlen = n.length();
    Real d = nlen-s.radius;
    Vector3 pw = s.center;
    if(s.radius > 0)
      //adjust pw to the sphere surface
      pw += n*(s.radius/nlen);
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp,plocal);
      n = ContactNormal(m1,plocal,tris[j],pw);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      continue;
    }
    else n /= nlen;
    //migrate the contact point to the center of the overlap region
    CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
    CopyVector(contact[k].normal,n);
    contact[k].depth = tol - d;
    k++;
    if(k == maxcontacts) break;
  }
  return k;
}

int MeshPrimitiveCollide(CollisionMesh& m1,Real outerMargin1,GeometricPrimitive3D& g2,const RigidTransform& T2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  GeometricPrimitive3D gworld=g2;
  gworld.Transform(T2);
  
  if(gworld.type == GeometricPrimitive3D::Point) {
    Sphere3D s;
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
    return MeshSphereCollide(m1,outerMargin1,s,outerMargin2,contact,maxcontacts);
  }
  else if(gworld.type == GeometricPrimitive3D::Sphere) {
    const Sphere3D& s = *AnyCast<Sphere3D>(&gworld.data);
    return MeshSphereCollide(m1,outerMargin1,s,outerMargin2,contact,maxcontacts);
  }
  else {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"Distance computations between Triangles and "<<gworld.TypeName()<<" not supported");
    return 0;
  }
}

int PointCloudPrimitiveCollide(CollisionPointCloud& pc1,Real outerMargin1,GeometricPrimitive3D& g2,const RigidTransform& T2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  if(g2.type == GeometricPrimitive3D::Empty) return 0;
  if(!g2.SupportsDistance(GeometricPrimitive3D::Point)) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"Cannot do contact checking on point cloud vs primitive "<<g2.TypeName()<<" yet");
    return 0;
  }

  GeometricPrimitive3D gworld=g2;
  gworld.Transform(T2);
    
  Real tol = outerMargin1 + outerMargin2;
  vector<int> points;
  int k=0;
  NearbyPoints(pc1,gworld,tol,points,maxcontacts);
  for(size_t j=0;j<points.size();j++) {   
    Vector3 pw = pc1.currentTransform*pc1.points[points[j]];
    if(gworld.Distance(pw) <= tol) {
      vector<double> u = gworld.ClosestPointParameters(pw);
      Vector3 cp = gworld.ParametersToPoint(u);
      Vector3 n = pw - cp;
      Real d = n.length();
      if(d < gNormalFromGeometryTolerance) {  //too close?
        continue;
      }
      else if(d > tol) {  //some penetration -- we can't trust the result of PQP
        continue;
      }
      else n /= d;
      //migrate the contact point to the center of the overlap region
      CopyVector(contact[k].pos,0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n);
      CopyVector(contact[k].normal,n);
      contact[k].depth = tol - d;
      k++;
      if(k == maxcontacts) break;
    }
  }
  return k;
}

int PrimitivePrimitiveCollide(GeometricPrimitive3D& g1,const RigidTransform& T1,Real outerMargin1,GeometricPrimitive3D& g2,const RigidTransform& T2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  if(maxcontacts==0) return 0;
  if(!g1.SupportsDistance(g2.type)) {
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"TODO: primitive collisions of type "<<g1.TypeName()<<" to "<<g2.TypeName());
    return 0;
  }
  if((g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Point) && (g2.type == GeometricPrimitive3D::Point || g2.type == GeometricPrimitive3D::Point)) {
    //do this the other way around
    int res = PrimitivePrimitiveCollide(g2,T2,outerMargin2,g1,T1,outerMargin1,contact,maxcontacts);
    for(int i=0;i<res;i++)
      ReverseContact(contact[i]);
    return res;
  }
  GeometricPrimitive3D tg1=g1,tg2=g2;
  tg1.Transform(T1);
  tg2.Transform(T2);
  if(g1.type != GeometricPrimitive3D::Point && g1.type != GeometricPrimitive3D::Sphere) {
    //TODO: try copying into ODE data structures?
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"Contact computations between primitives "<<g1.TypeName()<<" and "<<g2.TypeName()<<" not yet supported");
    return 0;
  }
  else {
    Sphere3D s;
    if(g1.type == GeometricPrimitive3D::Point) {
      s.center = *AnyCast<Point3D>(&tg1.data);
      s.radius = 0;
    }
    else {
      s = *AnyCast<Sphere3D>(&tg1.data);
    }
    if(tg2.Distance(s.center) > s.radius + outerMargin1 + outerMargin2) return 0;
    vector<double> params = tg2.ClosestPointParameters(s.center);
    Vector3 p2 = tg2.ParametersToPoint(params);
    //normal out from sphere to g2
    Vector3 n = p2 - s.center;
    Real d = n.norm();
    if(FuzzyZero(d)) { 
      //penetrating all the way to center?
      n = tg2.ParametersToNormal(params);
    }
    else
      n /= d;
    Vector3 p1 = s.center + n*s.radius;
    p2 -= outerMargin2*n;
    p1 += outerMargin1*n;
    contact[0].depth = p1.distance(p2);
    CopyVector(contact[0].pos,(p1+p2)*0.5);
    CopyVector(contact[0].normal,-n);
    return 1;
  }
}


int PrimitiveMeshCollide(GeometricPrimitive3D& g1,const RigidTransform& T1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  int num = MeshPrimitiveCollide(m2,outerMargin2,g1,T1,outerMargin1,contact,maxcontacts);
  for(int i=0;i<num;i++) ReverseContact(contact[i]);
  return num;
}


int PrimitivePointCloudCollide(GeometricPrimitive3D& g1,const RigidTransform& T1,Real outerMargin1,CollisionPointCloud& pc2,Real outerMargin2,dContactGeom* contact,int maxcontacts)
{
  int num = PointCloudPrimitiveCollide(pc2,outerMargin2,g1,T1,outerMargin1,contact,maxcontacts);
  for(int i=0;i<num;i++) ReverseContact(contact[i]);
  return num;
}


int PrimitiveGeometryCollide(GeometricPrimitive3D& g1,const RigidTransform& T, Real outerMargin1,Geometry::AnyCollisionGeometry3D& g2,Real outerMargin2,dContactGeom* contact,int m)
{
  switch(g2.type) {
  case AnyGeometry3D::Primitive:
    return PrimitivePrimitiveCollide(g1,T,outerMargin1,
        g2.AsPrimitive(),g2.PrimitiveCollisionData(),g2.margin+outerMargin2,contact,m);
    break;
  case AnyGeometry3D::TriangleMesh:
    return PrimitiveMeshCollide(g1,T,outerMargin1,
                                g2.TriangleMeshCollisionData(),g2.margin+outerMargin2,contact,m);
  case AnyGeometry3D::PointCloud:
    return PrimitivePointCloudCollide(g1,T,outerMargin1,
                                      g2.PointCloudCollisionData(),g2.margin+outerMargin2,contact,m);
  case AnyGeometry3D::ImplicitSurface:
    LOG4CXX_WARN(GET_LOGGER(ODESimulator),"TODO: primitive-implicit surface collisions");
    break;
  case AnyGeometry3D::Group:
    {
      vector<Geometry::AnyCollisionGeometry3D>& items = g2.GroupCollisionData();
      int n=0;
      for(size_t i=0;i<items.size();i++) {
        n += PrimitiveGeometryCollide(g1,T,outerMargin1,items[i],g2.margin+outerMargin2,contact+n,m-n);
        if(n >= m) return n;
      }
      return n;
    }
    break;
  }
  return 0;
}

int MeshGeometryCollide(CollisionMesh& m1,Real outerMargin1,Geometry::AnyCollisionGeometry3D& g2,Real outerMargin2,dContactGeom* contact,int m)
{
  switch(g2.type) {
  case AnyGeometry3D::Primitive:
    return MeshPrimitiveCollide(m1,outerMargin1,
                                g2.AsPrimitive(),g2.PrimitiveCollisionData(),g2.margin+outerMargin2,
                                contact,m);
  case AnyGeometry3D::TriangleMesh:
    return MeshMeshCollide(m1,outerMargin1,
                           g2.TriangleMeshCollisionData(),g2.margin+outerMargin2,
                           contact,m);
  case AnyGeometry3D::PointCloud:
    return MeshPointCloudCollide(m1,outerMargin1,
                                 g2.PointCloudCollisionData(),g2.margin+outerMargin2,
                                 contact,m);
    break;
  case AnyGeometry3D::ImplicitSurface:
    fprintf(stderr,"TODO: triangle mesh-implicit surface collisions\n");
    break;
  case AnyGeometry3D::Group:
    {
      vector<Geometry::AnyCollisionGeometry3D>& items = g2.GroupCollisionData();
      int n=0;
      for(size_t i=0;i<items.size();i++) {
        n += MeshGeometryCollide(m1,outerMargin1,items[i],g2.margin+outerMargin2,contact+n,m-n);
        if(n >= m) return n;
      }
      return n;
    }
    break;
  }
  return 0;
}

//m is max number of contacts
int GeometryGeometryCollide(Geometry::AnyCollisionGeometry3D& g1,Real outerMargin1,
                            Geometry::AnyCollisionGeometry3D& g2,Real outerMargin2,
                            dContactGeom* contact,int m)
{
  g1.InitCollisionData();
  g2.InitCollisionData();
  switch(g1.type) {
  case AnyGeometry3D::Primitive:
    return PrimitiveGeometryCollide(g1.AsPrimitive(),g1.PrimitiveCollisionData(),g1.margin+outerMargin1,g2,outerMargin2,contact,m);
  case AnyGeometry3D::TriangleMesh:
    return MeshGeometryCollide(g1.TriangleMeshCollisionData(),g1.margin+outerMargin1,g2,outerMargin2,contact,m);
  case AnyGeometry3D::PointCloud:
    switch(g2.type) {
    case AnyGeometry3D::Primitive:
      return PointCloudPrimitiveCollide(g1.PointCloudCollisionData(),g1.margin+outerMargin1,g2.AsPrimitive(),g2.PrimitiveCollisionData(),g2.margin+outerMargin2,contact,m);
      break;
    case AnyGeometry3D::TriangleMesh:
      return PointCloudMeshCollide(*AnyCast<CollisionPointCloud>(&g1.collisionData),g1.margin+outerMargin1,
                                   *AnyCast<CollisionMesh>(&g2.collisionData),g2.margin+outerMargin2,
                                   contact,m);
      break;
    case AnyGeometry3D::PointCloud:
      fprintf(stderr,"TODO: point cloud-point cloud collisions\n");
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: point cloud-implicit surface collisions\n");
      break;
    case AnyGeometry3D::Group:
      fprintf(stderr,"TODO: point cloud-group collisions\n");
      break;
    }
    break;
  case AnyGeometry3D::ImplicitSurface:
    switch(g2.type) {
    case AnyGeometry3D::Primitive:
      fprintf(stderr,"TODO: implicit surface-primitive collisions\n");
      break;
    case AnyGeometry3D::TriangleMesh:
      fprintf(stderr,"TODO: implicit surface-triangle mesh collisions\n");
      break;
    case AnyGeometry3D::PointCloud:
      fprintf(stderr,"TODO: implicit surface-point cloud collisions\n");
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: implicit surface-implicit surface collisions\n");
      break;
    case AnyGeometry3D::Group:
      fprintf(stderr,"TODO: implicit surface-group collisions\n");
      break;
    }
    break;
  case AnyGeometry3D::Group:
    {
      vector<Geometry::AnyCollisionGeometry3D>& items = g1.GroupCollisionData();
      int n=0;
      for(size_t i=0;i<items.size();i++) {
        n += GeometryGeometryCollide(items[i],g1.margin+outerMargin1,g2,outerMargin2,contact+n,m-n);
        if(n >= m) return n;
      }
      return n;
    }
    break;
  }
  return 0;
}

int dCustomGeometryCollide (dGeomID o1, dGeomID o2, int flags,
                           dContactGeom *contact, int skip)
{
  int m = (flags&0xffff);
  if(m == 0) m=1;
  //printf("CustomGeometry collide\n");
  CustomGeometryData* d1 = dGetCustomGeometryData(o1);
  CustomGeometryData* d2 = dGetCustomGeometryData(o2);
  RigidTransform T1;
  RigidTransform T2;
  CopyMatrix(T1.R,dGeomGetRotation(o1));
  CopyVector(T1.t,dGeomGetPosition(o1));
  CopyMatrix(T2.R,dGeomGetRotation(o2));
  CopyVector(T2.t,dGeomGetPosition(o2));
  T1.t += T1.R*d1->odeOffset;
  T2.t += T2.R*d2->odeOffset;
  d1->geometry->SetTransform(T1);
  d2->geometry->SetTransform(T2);

  int n=GeometryGeometryCollide(*d1->geometry,d1->outerMargin,*d2->geometry,d2->outerMargin,contact,m);

  for(int k=0;k<n;k++) {
    contact[k].g1 = o1;
    contact[k].g2 = o2;
  }
  return n;
}


dColliderFn * dCustomGeometryGetColliderFn (int num)
{
  if(num == gdCustomGeometryClass) return dCustomGeometryCollide;
  else return NULL;
}


void dCustomGeometryAABB(dGeomID o,dReal aabb[6])
{
  CustomGeometryData* d = dGetCustomGeometryData(o);
  AABB3D bb;
  RigidTransform T;
  CopyMatrix(T.R,dGeomGetRotation(o));
  CopyVector(T.t,dGeomGetPosition(o));  
  T.t += T.R*d->odeOffset;
  d->geometry->SetTransform(T);
  bb = d->geometry->GetAABB();
  bb.bmin -= Vector3(d->outerMargin,d->outerMargin,d->outerMargin);
  bb.bmax += Vector3(d->outerMargin,d->outerMargin,d->outerMargin);
  aabb[0] = bb.bmin.x;
  aabb[1] = bb.bmax.x;
  aabb[2] = bb.bmin.y;
  aabb[3] = bb.bmax.y;
  aabb[4] = bb.bmin.z;
  aabb[5] = bb.bmax.z;
}



void dCustomGeometryDtor(dGeomID o)
{
}

void InitODECustomGeometry()
{
  dGeomClass mmclass;
  mmclass.bytes = sizeof(CustomGeometryData);
  mmclass.collider = dCustomGeometryGetColliderFn;
  mmclass.aabb = dCustomGeometryAABB;
  mmclass.aabb_test = NULL;
  mmclass.dtor = dCustomGeometryDtor;
  gdCustomGeometryClass = dCreateGeomClass(&mmclass);
}

bool GetCustomGeometryCollisionReliableFlag()
{
  return !gCustomGeometryMeshesIntersect;
}

void ClearCustomGeometryCollisionReliableFlag()
{
  gCustomGeometryMeshesIntersect = false;
}
