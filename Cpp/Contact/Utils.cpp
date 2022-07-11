#include "Utils.h"
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/geometry/ConvexHull2D.h>
#include <KrisLibrary/statistics/KMeans.h>
#include <KrisLibrary/utils/EquivalenceMap.h>
using namespace Geometry;
using namespace Meshing;

#include <ode/common.h>
#include <ode/collision.h>
#include <Klampt/Modeling/World.h>

namespace Klampt {

//Produces a list of contacts as though the robot were standing on a plane.
//tol is the tolerance with which minimum-distance points are generated.
//All contacts are given zero friction and in the local frame of the robot.
void GetFlatContacts(RobotWithGeometry& robot,Real tol,ContactFormation& contacts)
{
  vector<AABB3D> bbs(robot.geometry.size());
  vector<pair<Real,int> > order;
  robot.UpdateGeometry();
  for(size_t i=0;i<robot.geometry.size();i++)
    if(robot.parents[i] >= 0 && !robot.IsGeometryEmpty(i)) {
      AABB3D aabb = robot.geometry[i]->GetAABB();
      bbs[i] = aabb;
      order.push_back(pair<Real,int>(aabb.bmin.z,(int)i));
    }
  sort(order.begin(),order.end());
  Real best = Inf;
  for(size_t i=0;i<order.size();i++) {
    if(order[i].first > best) break; //done
    int k=order[i].second;
    switch(robot.geometry[k]->type) {
    case AnyGeometry3D::Primitive:
      FatalError("Can't get flat contacts for primitives");
      break;
    case AnyGeometry3D::ImplicitSurface:
      FatalError("Can't get flat contacts for implicit surfaces");
      break;
    case AnyGeometry3D::Group:
      FatalError("Can't get flat contacts for geometry group");
      break;
    case AnyGeometry3D::ConvexHull:
      FatalError("Can't get flat contacts for convex hull");
      break;
    case AnyGeometry3D::TriangleMesh:
      {
	const TriMesh& m=robot.geometry[k]->AsTriangleMesh();
	for(size_t v=0;v<m.verts.size();v++) {
	  Vector3 pw = robot.links[k].T_World*m.verts[v];
	  if(pw.z < best)
	    best = pw.z;
	  assert(pw.z >= order[i].first);
	}
      }
      break;
    case AnyGeometry3D::PointCloud:
      {
	const PointCloud3D& pc=robot.geometry[k]->AsPointCloud();
	for(size_t v=0;v<pc.points.size();v++) {
	  Vector3 pw = robot.links[k].T_World*pc.points[v];
	  if(pw.z < best)
	    best = pw.z;
	  assert(pw.z >= order[i].first);
	}
      }
      break;
    }
  }
  
  //got the plane height, now output the vertices
  ContactPoint cp;
  cp.kFriction = 0.0;
  contacts.links.resize(0);
  contacts.contacts.resize(0);
  for(size_t i=0;i<order.size();i++) {
    if(order[i].first > best+tol) break; //done
    int k=order[i].second;
    contacts.links.resize(contacts.links.size()+1);
    contacts.links.back()=k;
    contacts.contacts.resize(contacts.contacts.size()+1);
    vector<Vector3> pts;
    switch(robot.geometry[k]->type) {
    case AnyGeometry3D::TriangleMesh:
      {
	const TriMesh& m=robot.geometry[k]->AsTriangleMesh();
	for(size_t v=0;v<m.verts.size();v++) {
	  Vector3 pw = robot.links[k].T_World*m.verts[v];
	  if(pw.z < best+tol) {
	    pts.push_back(pw);
	  }
	}
      }
      break;
    case AnyGeometry3D::PointCloud:
      {
	const PointCloud3D& pc=robot.geometry[k]->AsPointCloud();
	for(size_t v=0;v<pc.points.size();v++) {
	  Vector3 pw = robot.links[k].T_World*pc.points[v];
	  if(pw.z < best+tol) {
	    pts.push_back(pw);
	  }
	}
      }
      break;
    default:
      break;
    }
    //get the convex hull of those points
    vector<Vector2> pts2(pts.size());
    vector<Vector2> hull(pts.size());
    vector<int> hindex(pts.size());
    for(size_t v=0;v<pts.size();v++)
      pts2[v].set(pts[v].x,pts[v].y);
    int num = ConvexHull2D_Chain_Unsorted( &pts2[0], pts2.size(), &hull[0], &hindex[0]);
    contacts.contacts.back().resize(num);
    for(int v=0;v<num;v++) {
      //local estimate
      robot.links[k].T_World.mulInverse(pts[hindex[v]],cp.x);
      robot.links[k].T_World.R.mulTranspose(Vector3(0,0,1),cp.n);
      contacts.contacts.back()[v] = cp;
    }
    if(contacts.contacts.back().empty()) {
      //bound was below threshold, but no contacts below threshold
      contacts.links.resize(contacts.links.size()-1);
      contacts.contacts.resize(contacts.contacts.size()-1);
    }
  }
}

void GetFlatContacts(RobotWithGeometry& robot,int link,Real tol,vector<ContactPoint>& contacts)
{
  Real best = Inf;
  const vector<Vector3>* points = NULL;
  switch(robot.geometry[link]->type) {
  case AnyGeometry3D::Primitive:
    FatalError("Can't get flat contacts for primitives");
    break;
  case AnyGeometry3D::ImplicitSurface:
    FatalError("Can't get flat contacts for implicit surfaces");
    break;
  case AnyGeometry3D::Group:
    FatalError("Can't get flat contacts for geometry group");
    break;
  case AnyGeometry3D::ConvexHull:
    FatalError("Can't get flat contacts for convex hull");
    break;
  case AnyGeometry3D::TriangleMesh:
    points = &robot.geometry[link]->AsTriangleMesh().verts;
    break;
  case AnyGeometry3D::PointCloud:
    points = &robot.geometry[link]->AsPointCloud().points;
    break;
  }

  for(size_t v=0;v<points->size();v++) {
    Vector3 pw = robot.links[link].T_World*(*points)[v];
    if(pw.z < best)
      best = pw.z;
  }
  
  //got the plane height, now output the vertices
  ContactPoint cp;
  cp.kFriction = 0.0;
  contacts.resize(0);
  vector<Vector3> pts;
  for(size_t v=0;v<points->size();v++) {
    Vector3 pw = robot.links[link].T_World*(*points)[v];
    if(pw.z < best+tol) {
      pts.push_back(pw);
    }
  }
  //get the convex hull of those points
  vector<Vector2> pts2(pts.size());
  vector<Vector2> hull(pts.size());
  vector<int> hindex(pts.size());
  for(size_t v=0;v<pts.size();v++)
    pts2[v].set(pts[v].x,pts[v].y);
  int num = ConvexHull2D_Chain_Unsorted( &pts2[0], pts2.size(), &hull[0], &hindex[0]);
  contacts.resize(num);
  for(int v=0;v<num;v++) {
    //local estimate
    robot.links[link].T_World.mulInverse(pts[hindex[v]],cp.x);
    robot.links[link].T_World.R.mulTranspose(Vector3(0,0,1),cp.n);
    contacts[v] = cp;
  }
}

void GetFlatStance(RobotWithGeometry& robot,Real tol,Stance& s,Real kFriction)
{
  ContactFormation formation;
  GetFlatContacts(robot,tol,formation);
  LocalContactsToStance(formation,robot,s);
  for(Stance::iterator i=s.begin();i!=s.end();i++)
    for(size_t j=0;j<i->second.contacts.size();j++)
      i->second.contacts[j].kFriction = kFriction;
}


void GetNearbyContacts(RobotWithGeometry& robot,WorldModel& world,Real tol,ContactFormation& contacts)
{
  contacts.links.resize(0);
  contacts.contacts.resize(0);
  for(int i=0;i<(int)robot.links.size();i++) {
    if(robot.parents[i] < 0) continue; //fixed link
    vector<ContactPoint> cps;
    GetNearbyContacts(robot,i,world,tol,cps);
    if(!cps.empty()) {
      contacts.links.push_back(i);
      contacts.contacts.push_back(cps);
    }
  }
}


void GetNearbyContacts(RobotWithGeometry& robot,int link,WorldModel& world,Real tol,vector<ContactPoint>& contacts)
{
  contacts.resize(0);
  if(robot.IsGeometryEmpty(link)) {
    return;
  }
  AnyContactsQuerySettings settings;
  settings.padding1 = 0;
  settings.padding2 = tol;

  Geometry::AnyCollisionGeometry3D& g1 = *robot.geometry[link];
  vector<Geometry::AnyCollisionGeometry3D*> geomsToCheck;
  for(size_t i=0;i<world.terrains.size();i++) {
    if(world.terrains[i]->geometry.Empty()) continue;
    geomsToCheck.push_back(&*world.terrains[i]->geometry);
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    if(world.rigidObjects[i]->geometry.Empty()) {
      continue;
    }
    geomsToCheck.push_back(&*world.rigidObjects[i]->geometry);
    world.rigidObjects[i]->UpdateGeometry();
  }
  //now do the tolerance checks and add to the contacts list
  for(size_t i=0;i<geomsToCheck.size();i++) {
    AnyContactsQueryResult res = g1.Contacts(*geomsToCheck[i],settings);
    size_t start = contacts.size();
    contacts.resize(start+res.contacts.size());
    for(size_t j=0;j<res.contacts.size();j++) {
      //contacts[j].x.set(temp[j].pos);
      contacts[start+j].x = (res.contacts[j].p1+res.contacts[j].p2)*0.5;
      contacts[start+j].n = res.contacts[j].n;
      contacts[start+j].kFriction = 0;

      //convert to local coordinates
      Vector3 localPos,localNormal;
      robot.links[link].T_World.mulInverse(contacts[start+j].x,localPos);
      robot.links[link].T_World.R.mulTranspose(contacts[start+j].n,localNormal);
      contacts[start+j].x = localPos;
      contacts[start+j].n = localNormal;
    }
  }
}

void LocalContactsToHold(const vector<ContactPoint>& contacts,int link,const RobotKinematics3D& robot,Hold& hold)
{
  hold.link = link;
  hold.contacts = contacts;
  for(size_t i=0;i<contacts.size();i++) {
    hold.contacts[i].x = robot.links[link].T_World*hold.contacts[i].x;
    hold.contacts[i].n = robot.links[link].T_World.R*hold.contacts[i].n;
  }
  MomentRotation m;
  m.setMatrix(robot.links[link].T_World.R);
  hold.SetupIKConstraint(contacts[0].x,m);
  hold.ikConstraint.destLink = -1;
}


void LocalContactsToStance(const ContactFormation& contacts,const RobotKinematics3D& robot,Stance& stance)
{
  stance.clear();
  for(size_t i=0;i<contacts.contacts.size();i++) {
    Hold h;
    LocalContactsToHold(contacts.contacts[i],contacts.links[i],robot,h);
    if(!contacts.targets.empty())
      h.ikConstraint.destLink = contacts.targets[i];
    stance.insert(h);
  }
}



void ClusterContacts(vector<ContactPoint>& contacts,int numClusters,Real normalScale,Real frictionScale)
{
  if((int)contacts.size() <= numClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].x.x;
    pts[i][1] = contacts[i].x.y;
    pts[i][2] = contacts[i].x.z;
    pts[i][3] = contacts[i].n.x*normalScale;
    pts[i][4] = contacts[i].n.y*normalScale;
    pts[i][5] = contacts[i].n.z*normalScale;
    pts[i][6] = contacts[i].kFriction*frictionScale;
  }

  Statistics::KMeans kmeans(pts,numClusters);
  kmeans.RandomInitialCenters();
  int iters=20;
  kmeans.Iterate(iters);
  contacts.resize(kmeans.centers.size());
  vector<int> degenerate;
  for(size_t i=0;i<contacts.size();i++) {
    contacts[i].x.x = kmeans.centers[i][0];
    contacts[i].x.y = kmeans.centers[i][1];
    contacts[i].x.z = kmeans.centers[i][2];
    contacts[i].n.x = kmeans.centers[i][3];
    contacts[i].n.y = kmeans.centers[i][4];
    contacts[i].n.z = kmeans.centers[i][5];
    Real len = contacts[i].n.length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ClusterContacts: Warning, clustered normal became zero/infinite\n");
      //pick any in the cluster
      int found = -1;
      for(size_t k=0;k<kmeans.labels.size();k++) {
	if(kmeans.labels[k] == (int)i) {
	  found = (int)k;
	  break;
	}
      }
      if(found < 0) {
	//strange -- degenerate cluster?
	degenerate.push_back(i);
	continue;
      }
      contacts[i].x.x = pts[found][0];
      contacts[i].x.y = pts[found][1];
      contacts[i].x.z = pts[found][2];
      contacts[i].n.x = pts[found][3];
      contacts[i].n.y = pts[found][4];
      contacts[i].n.z = pts[found][5];
      Real len = contacts[i].n.length();
      contacts[i].n /= len;
      contacts[i].kFriction = pts[found][6]/frictionScale;
      Assert(contacts[i].kFriction >= 0);
      continue;
    }
    contacts[i].n /= len;
    //cout<<"Clustered contact "<<contacts[i].pos[0]<<" "<<contacts[i].pos[1]<<" "<<contacts[i].pos[2]<<endl;
    //cout<<"Clustered normal "<<contacts[i].normal[0]<<" "<<contacts[i].normal[1]<<" "<<contacts[i].normal[2]<<endl;
    contacts[i].kFriction = kmeans.centers[i][6]/frictionScale;
    Assert(contacts[i].kFriction >= 0);
  }

  //erase backward down degenerate list 
  reverse(degenerate.begin(),degenerate.end());
  for(size_t i=0;i<degenerate.size();i++) {
    contacts.erase(contacts.begin()+degenerate[i]);
  }
}







void CHContactsPlane(vector<ContactPoint>& cp,const Vector3& n,const Vector3& ori,Real ntol,Real xtol)
{
  Assert(!cp.empty());
  Real ofs = dot(n,ori);
  //all points must be on a plane
  for(size_t i=0;i<cp.size();i++) {
    if(!cp[i].n.isEqual(n,ntol)) {
      cout<<"CHContactsPlane: Warning: non-equal normal"<<endl;
    }
    if(!FuzzyEquals(n.dot(cp[i].x),ofs,xtol)) {
      cout<<"CHContactsPlane: Warning: non-equal offset: "<<ofs<<" vs "<<n.dot(cp[i].x)<<endl;
    }
  }
  Vector3 x,y;
  GetCanonicalBasis(n,x,y);
  Point2D* planePts = new Point2D[cp.size()];
  Point2D* chPts = new Point2D[cp.size()+1];
  //project points on plane
  Real xofs = x.dot(ofs);
  Real yofs = y.dot(ofs);
  for(size_t i=0;i<cp.size();i++) {
    planePts[i].x = x.dot(cp[i].x)-xofs;
    planePts[i].y = y.dot(cp[i].x)-yofs;
  }
  int k=ConvexHull2D_Chain_Unsorted(planePts,cp.size(),chPts);
  Assert(k <= (int)cp.size());
  Assert(k > 0);

  cp.resize(k);
  for(int i=0;i<k;i++) {
    cp[i].x.mul(n,ofs);
    cp[i].x.madd(x,chPts[i].x+xofs);
    cp[i].x.madd(y,chPts[i].y+yofs);
  }

  delete [] planePts;
  delete [] chPts;
}

void GetPlane(const vector<ContactPoint>& cp,Vector3& n,Vector3& origin)
{
  //just a simple average...
  n.setZero();
  origin.setZero();
  for(size_t i=0;i<cp.size();i++) {
    n += cp[i].n;
    origin += cp[i].x;
  }
  origin /= (Real)cp.size();
  n.inplaceNormalize();
}

struct EqualCP
{
  EqualCP(Real _xtol,Real _ntol) :xtol(_xtol),ntol(_ntol) {}
  bool operator()(const ContactPoint& a,const ContactPoint& b) const
  {
    if(a.x.isEqual(b.x,xtol)) {
      if(a.n.isEqual(b.n,ntol)) {
	if(FuzzyEquals(a.kFriction,b.kFriction,ntol)) {
	  return true;
	}
      }
    }
    return false;
  }

  Real xtol,ntol;
};

struct EqualPlane
{
  EqualPlane(Real _ntol,Real _xtol) :ntol(_ntol),xtol(_xtol) {}
  bool operator()(const ContactPoint& a,const ContactPoint& b) const
  {
    if(a.n.isEqual(b.n,ntol)) {
      if(FuzzyEquals(a.n.dot(a.x),a.n.dot(b.x),xtol) &&
	 FuzzyEquals(b.n.dot(a.x),b.n.dot(b.x),xtol)) {
	return true;
      }
    }
    return false;
  }

  Real ntol,xtol;
};

void CHContacts(vector<ContactPoint>& cp,Real ntol,Real xtol)
{
  EqualPlane eq(ntol,xtol);
  //EqualNormal eq(ntol);
  vector<vector<int> > sets;
  EquivalenceMap(cp,sets,eq);

  vector<ContactPoint> newCp;
  for(size_t i=0;i<sets.size();i++) {
    //set temp = sets[i]
    vector<int>& s=sets[i];
    vector<ContactPoint> temp(s.size());
    for(size_t j=0;j<s.size();j++) {
      Assert(0<=s[j] && s[j]<(int)cp.size());
      temp[j] = cp[s[j]];
    }
    size_t oldSize = temp.size();
    //average the plane
    Vector3 n,origin;
    GetPlane(temp,n,origin);
    CHContactsPlane(temp,n,origin,ntol,xtol);

    //append temp onto newCps
    oldSize=newCp.size();
    newCp.resize(oldSize + temp.size());
    //copy(temp.begin(),temp.end(),newCp.begin()+oldSize);
    copy_backward(temp.begin(),temp.end(),newCp.end());
  }
  swap(cp,newCp);
}

void CleanupContacts(vector<ContactPoint>& cp,Real tol)
{
  EqualCP eq(tol,tol*10.0);

  vector<vector<int> > sets;
  EquivalenceMap(cp,sets,eq);

  vector<ContactPoint> cpNew;
  cpNew.resize(sets.size());
  for(size_t i=0;i<sets.size();i++) {
    vector<int>& avgPoints = sets[i];
    Assert(!avgPoints.empty());
    cpNew[i].x.setZero();
    cpNew[i].n.setZero();
    cpNew[i].kFriction = Zero;
    for(size_t j=0;j<avgPoints.size();j++) {
      cpNew[i].x += cp[avgPoints[j]].x;
      cpNew[i].n += cp[avgPoints[j]].n;
      cpNew[i].kFriction += cp[avgPoints[j]].kFriction;
    }
    cpNew[i].x /= (Real)avgPoints.size();
    cpNew[i].n /= (Real)avgPoints.size();
    cpNew[i].kFriction /= (Real)avgPoints.size();
    cpNew[i].n.inplaceNormalize();
  }
  swap(cp,cpNew);
}

int ClosestContact(const ContactPoint& p,const Meshing::TriMesh& mesh,ContactPoint& pclose,Real normalScale)
{
  int closest = -1;
  Real closestDist2 = Inf;
  Triangle3D tri;
  Plane3D plane;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    //first check distance to supporting plane, since it's a lower bound
    tri.getPlane(plane);
    Real dxmin = plane.distance(p.x);
    Real dn = normalScale*plane.normal.distanceSquared(p.n);
    if(dn + Sqr(dxmin) < closestDist2) {
      //has potential to be closer than previous
      Vector3 cp = tri.closestPoint(p.x);
      Real d = cp.distanceSquared(p.x) + dn;
      if(d < closestDist2) {
	closest = (int)i;
	closestDist2 = d;
	pclose.x = cp;
	pclose.n = plane.normal;
	pclose.kFriction = p.kFriction;
      }
    }
  }
  return closest;
}

} //namespace Klampt