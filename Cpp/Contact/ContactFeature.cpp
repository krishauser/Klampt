#include "ContactFeature.h"
#include "TriangleSampler.h"
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math3d/Cylinder3D.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/math/random.h>
#include "LineReader.h"
#include "Polygon2DSampler.h"
#include <fstream>
#include <sstream>

namespace Klampt {

/*
//TODO: replace this with a true likelihood function
Real TrianglePlacementLikelihood(const Triangle3D& tri,Real kFriction)
{
  Vector3 n=tri.normal();
  Real scale=(n.z+One)*Half+Atan(kFriction)/Pi_2;
  return tri.area()*scale;
}
*/

void PointContactFeature::GetHold(const Vector3& x,Hold& h) const
{
  h.link = h.ikConstraint.link = link;
  h.ikConstraint.localPosition = p;
  h.ikConstraint.SetFixedPosition(x);
  h.ikConstraint.SetFreeRotation();
  h.contacts.resize(1);
  h.contacts[0].x = x;
  h.contacts[0].n.setZero();
  h.contacts[0].kFriction = 0;
}

void EdgeContactFeature::GetHold(const Vector3& x,const Vector3& axis,Hold& h) const
{
  h.link = h.ikConstraint.link = link;
  h.ikConstraint.localPosition = p1;
  h.ikConstraint.SetFixedPosition(x);
  Vector3 locAxis=p2-p1;
  if(locAxis.isZero()) {
    cerr<<"Warning: edge contact feature is degenerate, setting a point hold"<<endl;
    h.ikConstraint.SetFreeRotation();
    h.contacts.resize(1);
    h.contacts[0].x = p1;
    h.contacts[0].n.setZero();
    h.contacts[0].kFriction = 0;
  }
  else {
    locAxis.inplaceNormalize();
    h.ikConstraint.SetAxisRotation(locAxis,axis);
    h.contacts.resize(2);
    h.contacts[0].x = p1;
    h.contacts[1].x = p2;
    h.contacts[0].n.setZero(); h.contacts[1].n.setZero();
    h.contacts[0].kFriction = h.contacts[1].kFriction = 0;
  }
}

void EdgeContactFeature::GetHold(const Vector3& x,const Vector3& n,Real theta,Hold& h) const
{
  h.link = h.ikConstraint.link = link;
  h.ikConstraint.localPosition = p1;
  h.ikConstraint.SetFixedPosition(x);
  Vector3 locAxis = p2-p1;
  if(locAxis.isZero()) {
    cerr<<"Warning: edge contact feature is degenerate, setting a point hold"<<endl;
    h.ikConstraint.SetFreeRotation();
    h.contacts.resize(1);
    h.contacts[0].x = p1;
    h.contacts[0].n = n;
    h.contacts[0].kFriction = 0;
  }
  else {
    locAxis.inplaceNormalize();
    //Get rotation that makes local axis orthogonal to n
    //could do this, or just project locAxis onto plane normal to n
    //Next, rotate around n by angle theta
    Matrix3 R,Rtheta;
    GetMinimalRotationToPlane(locAxis,n,R);
    AngleAxisRotation aa;
    aa.axis = n;
    aa.angle = theta;
    aa.getMatrix(Rtheta);
    Vector3 axis = Rtheta*R*locAxis;
    h.ikConstraint.SetAxisRotation(locAxis,axis);
    h.contacts.resize(2);
    h.contacts[0].x = p1;
    h.contacts[1].x = p2;
    h.contacts[0].n = h.contacts[1].n = n;
    h.contacts[0].kFriction = h.contacts[1].kFriction = 0;
  }
}

void FaceContactFeature::GetPlane(Plane3D& p) const
{
  Polygon3D::getPlaneFit(p);
  Plane3D temp;
  Polygon3D::getPlane(0,temp);
  if(dot(temp.normal,p.normal) < 0) {
    p.normal.inplaceNegative();
    p.offset = -p.offset;
  }
}

void FaceContactFeature::GetCentroid(Vector3& p) const
{
  p = centroidConvex();
  /*
  p.setZero();
  for(size_t i=0;i<vertices.size();i++)
    p += vertices[i];
  p /= vertices.size();
  */
}

void FaceContactFeature::GetHold(const Vector3& x,const Vector3& n,Real theta,Hold& h) const
{
  Vector3 centroid;
  GetCentroid(centroid);
  GetHold(centroid,x,n,theta,h);
}

void FaceContactFeature::GetHold(const Vector3& localPos,const Vector3& x,const Vector3& n,Real theta,Hold& h) const
{
  h.link=h.ikConstraint.link=link;
  Plane3D plane;
  GetPlane(plane);
  //get rotation
  Matrix3 R; //get a matrix to rotate to the world frame
  GetMinimalRotation(-plane.normal,n,R);
  //rotate contact's hold about the normal
  AngleAxisRotation aa;
  aa.axis=n;
  aa.angle=theta;
  Matrix3 Raxis;
  aa.getMatrix(Raxis);
  RigidTransform T;
  T.R = Raxis*R;
  //h.ikConstraint.localPosition = vertices[0];
  //GetRotationAboutLocalPoint(localPos,x,T.R,T);
  //T.mulPoint(localPos,h.ikConstraint.endPosition);
  h.ikConstraint.localPosition = localPos;
  h.ikConstraint.SetFixedPosition(x);
  T.t = x - T.R*localPos;

  h.ikConstraint.SetFixedRotation(T.R);
  //transform contact point positions
  h.contacts.resize(vertices.size());
  for(size_t i=0;i<vertices.size();i++) {
    T.mulPoint(vertices[i],h.contacts[i].x);
    h.contacts[i].n = n;
    h.contacts[i].kFriction=0;
  }
}

void WheelContactFeature::GetHold(const Vector3& x,const Vector3& n,Real theta,Hold& h) const
{
  h.link=h.ikConstraint.link=link;

  Vector3 loccenter=GetCenter();
  Real radius = GetRadius();
  if(FuzzyZero(radius)) {
    FatalError("Error, wheel feature radius is zero!");
  }
  //affix the center of the wheel to x+n*radius
  Vector3 worldcenter = x + n*radius;
  h.ikConstraint.localPosition = loccenter;
  h.ikConstraint.SetFixedPosition(worldcenter);

  //get the rotation that moves the local axis to be orthogonal to n
  //then multiply it by the rotation theta
  Matrix3 R,Rtheta;
  GetMinimalRotationToPlane(axis.direction,n,R);
  AngleAxisRotation aa(theta,n);  aa.getMatrix(Rtheta);
  Vector3 worldaxis = Rtheta*R*axis.direction;
  h.ikConstraint.SetAxisRotation(axis.direction,worldaxis);

  //get the contact points -- they're offsets along the axis direction
  h.contacts.resize(2);
  //h.contacts[0].x = worldcenter + Half*width*worldaxis;
  //h.contacts[1].x = worldcenter - Half*width*worldaxis;
  h.contacts[0].x = x + Half*width*worldaxis;
  h.contacts[1].x = x - Half*width*worldaxis;
  h.contacts[0].n = h.contacts[1].n = n;
  h.contacts[0].kFriction = h.contacts[1].kFriction = 0;
}

void WheelContactFeature::GetFixedHold(const Vector3& x,const Vector3& n,Real theta,Real roll,Hold& h) const
{
  h.link=h.ikConstraint.link=link;

  Vector3 loccenter=GetCenter();
  Real radius = GetRadius();
  if(FuzzyZero(radius)) {
    FatalError("Error, wheel feature radius is zero!");
  }
  //affix the center of the wheel to x+n*radius
  Vector3 worldcenter = x + n*radius;
  h.ikConstraint.localPosition = loccenter;
  h.ikConstraint.SetFixedPosition(worldcenter);

  //get the rotation that moves the local axis to be orthogonal to n
  //then multiply it by the rotation theta
  Matrix3 R,Rtheta;
  GetMinimalRotationToPlane(axis.direction,n,R);
  AngleAxisRotation aa(theta,n);  aa.getMatrix(Rtheta);
  Vector3 worldaxis = Rtheta*R*axis.direction;

  //Form a local frame at center of wheel, z pointing up from contact points
  Matrix3 Rloc;
  Vector3 z = loccenter-p; z.inplaceNormalize();
  Assert(FuzzyZero(axis.direction.dot(z)));
  Vector3 y; y.setCross(z,axis.direction);  y.inplaceNormalize();
  Rloc.set(axis.direction,y,z);
  //rotate Rloc by theta about the x axis
  Rtheta.setRotateX(roll);
  Rloc = Rloc*Rtheta;

  //Form a local frame at the fixed hold
  y.setCross(n,worldaxis);
  R.set(worldaxis,y,n);

  Rtheta.setInverse(Rloc);
  h.ikConstraint.SetFixedRotation(R*Rtheta);

  //get the contact points -- they're offsets along the axis direction
  h.contacts.resize(2);
  h.contacts[0].x = x + Half*width*worldaxis;
  h.contacts[1].x = x - Half*width*worldaxis;
  h.contacts[0].n = h.contacts[1].n = n;
  h.contacts[0].kFriction = h.contacts[1].kFriction = 0;

  //testing
  RigidTransform T;
  h.ikConstraint.GetFixedGoalTransform(T);
  Vector3 worldCenter = T*loccenter;
  Assert(worldCenter.distance(h.contacts[0].x) < radius*1.5);
  Assert(worldCenter.distance(h.contacts[1].x) < radius*1.5);
}

void SampleHold(const Vector3& x,const Vector3& n,const ContactFeatureBase* contact,Hold& h)
{
  ContactFeatureBase::Type type=contact->GetType();
  switch(type) {
  case ContactFeatureBase::Point:
    dynamic_cast<const PointContactFeature*>(contact)->GetHold(x,h);
    h.contacts[0].n = n;
    break;
  case ContactFeatureBase::Edge:
    dynamic_cast<const EdgeContactFeature*>(contact)->GetHold(x,n,Rand()*TwoPi,h);
    break;
  case ContactFeatureBase::Face:
    dynamic_cast<const FaceContactFeature*>(contact)->GetHold(x,n,Rand()*TwoPi,h);
    break;
  case ContactFeatureBase::Wheel:
    dynamic_cast<const WheelContactFeature*>(contact)->GetHold(x,n,Rand()*TwoPi,h);
    break;
  case ContactFeatureBase::MultipleFaces:
    FatalError("Not done with sampling multiple-face contact feature");
    break;
  }
}

void SampleHold(const Vector3& x,const Vector3& n,const vector<ContactFeature>& contacts,Hold& h)
{
  Real totalWeight=0;
  for(size_t i=0;i<contacts.size();i++)
    totalWeight += contacts[i]->weight;
  Real t=Rand()*totalWeight;
  for(size_t i=0;i<contacts.size();i++) {
    t -= contacts[i]->weight;
    if(t<=Zero) {
      SampleHold(x,n,contacts[i].get(),h);
      return;
    }
  }
  AssertNotReached();
}


void SampleHold(Triangle3DSampler& smp,const ContactFeatureBase* contact,Hold& h)
{
  int t=smp.SampleTri();
  Vector3 x;
  smp.SamplePointOnTri(t,x);
  Vector3 n=smp.tris[t].normal();
  SampleHold(x,n,contact,h);
}

void SampleHold(Triangle3DSampler& smp,const vector<ContactFeature>& contacts,Hold& h)
{
  Real totalWeight=0;
  for(size_t i=0;i<contacts.size();i++)
    totalWeight += contacts[i]->weight;
  Real t=Rand()*totalWeight;
  for(size_t i=0;i<contacts.size();i++) {
    t -= contacts[i]->weight;
    if(t<=Zero) {
      SampleHold(smp,contacts[i].get(),h);
      return;
    }
  }
  AssertNotReached();
}




class ContactFeatureReader : public LineReader
{
public:
  ContactFeatureReader(istream& in)
    :LineReader(in),c(NULL)
  {}
  virtual bool Begin(const string& name,stringstream& args)
  {
    c = NULL;
    link = -1;
    weight = 1;
    this->name = "Unnamed";
    faces.clear();
    center.setZero(); 
    axis.setZero();
    if(name == "point") { type = ContactFeatureBase::Point; return true; }
    if(name == "edge") { type = ContactFeatureBase::Edge; return true; }
    if(name == "face") { type = ContactFeatureBase::Face; return true; }
    if(name == "wheel") { type = ContactFeatureBase::Wheel; return true; } 
    if(name == "faces") { type = ContactFeatureBase::MultipleFaces; return true; }
    return false;
  }
  virtual bool Assign(const string& item,stringstream& rhs)
  {
    if(item == "weight") { rhs >> weight; return true; }
    else if(item == "link") { rhs >> link; return true; }
    else if(item == "name") { name.clear(); return ::InputQuotedString(rhs,name); }
    else if(item == "pts") { 
      vector<Vector3> pts;
      Vector3 v;
      rhs >> v;
      while(rhs) {
        pts.push_back(v);
        rhs >> v;
      }
      faces.push_back(pts);
      return true; 
    }
    else if(item == "axis") { rhs >> center >> axis; return true; }
    cout<<"Unknown item "<<item<<endl;
    return false;
  }
  virtual bool End() 
  { 
    if(faces.empty()) {
      cerr<<"Must have at least one set of points"<<endl;
      return false;
    }
    //most types of features only have one set of points
    const vector<Vector3>& pts=faces[0];
    if(type != ContactFeatureBase::MultipleFaces) {
      if(faces.size() > 1) {
        cerr<<"Warning: feature should only have one set of points, ignoring all but the first set"<<endl;
      }
    }

    switch(type) {
    case ContactFeatureBase::Point:
      {
        PointContactFeature* f=new PointContactFeature;
        c.reset(f);
        if(pts.size() > 1) {
          cerr<<"Only one contact point can be specified for point features"<<endl;
          return false;
        }
        if(pts.size() == 0) {
          cerr<<"Must specify one contact point for a point feature"<<endl;
          return false;
        }
        f->p = pts[0];
      }
      break;
    case ContactFeatureBase::Edge:
      {
        EdgeContactFeature* f=new EdgeContactFeature;
        c.reset(f);
        if(pts.size() > 2) {
          cerr<<"Only two contact points can be specified for edge features"<<endl;
          return false;
        }
        if(pts.size() < 2) {
          cerr<<"Must specify two contact points for a edge feature"<<endl;
          return false;
        }
        f->p1 = pts[0];
        f->p2 = pts[1];
      }
      break;
    case ContactFeatureBase::Face:
      {
        FaceContactFeature* f=new FaceContactFeature;
        c.reset(f);
        if(pts.size() < 3) {
          cerr<<"At least contact points must be specified for face features"<<endl;
          return false;
        }
        f->vertices = pts;
      }
      break;
    case ContactFeatureBase::Wheel:
      {
        WheelContactFeature* f=new WheelContactFeature;
        c.reset(f);
        if(pts.size() > 2) {
          cerr<<"At most two contact points can be specified for wheel features"<<endl;
          return false;
        }
        else if(pts.size() == 0) {
          cerr<<"Must specify more than one contact point for a wheel feature"<<endl;
          return false;
        }
        else if(pts.size() == 1) {
          f->p = pts[0];
          f->width = 0;
        }
        else {
          f->p = Half*(pts[0]+pts[1]);
          f->width = pts[0].distance(pts[1]);
        }
        if(axis.isZero()) {
          cerr<<"Didn't specify axis for wheel hold"<<endl;
          return false;
        }
        f->axis.source = center;
        f->axis.direction = axis;
        if(!FuzzyEquals(f->axis.direction.norm(),One)) {
          cerr<<"Warning, axis is not normalized"<<endl;
          f->axis.direction.inplaceNormalize();
        }
      }
      break;
    case ContactFeatureBase::MultipleFaces:
      cerr<<"Not done with multiple face contact"<<endl;
      return false;
    }
    Assert(c != NULL);
    c->link = link;
    c->weight = weight;
    c->name = name;
    return true;
  }

  ContactFeatureBase::Type type;
  int link;
  Real weight;
  string name;
  vector<vector<Vector3> > faces;
  Vector3 center,axis; //for wheel holds
  ContactFeature c;  //output pointer
};

istream& operator >> (istream& in,ContactFeature& c)
{
  ContactFeatureReader reader(in);
  if(!reader.Read()) {
    in.setstate(ios::failbit);
    return in;
  }
  c = reader.c;
  return in;
}

ostream& operator << (ostream& out,const ContactFeature& _c)
{
  const ContactFeatureBase* c = _c.get();
  out<<"begin ";
  switch(c->GetType()) {
  case ContactFeatureBase::Point: out<<"point"; break;
  case ContactFeatureBase::Edge: out<<"edge"; break;
  case ContactFeatureBase::Face: out<<"face"; break;
  case ContactFeatureBase::Wheel: out<<"wheel"; break;
  case ContactFeatureBase::MultipleFaces: out<<"faces"; break;
  }
  out<<endl;
  out<<"link = "<<c->link<<endl;
  out<<"name = "; OutputQuotedString(out,c->name); out<<endl;
  switch(c->GetType()) {
  case ContactFeatureBase::Point: 
    out<<"pts = "<<dynamic_cast<const PointContactFeature*>(c)->p<<endl;
    break;
  case ContactFeatureBase::Edge:
    out<<"pts = "<<dynamic_cast<const EdgeContactFeature*>(c)->p1<<"   "<<dynamic_cast<const EdgeContactFeature*>(c)->p2<<endl;
    break;
  case ContactFeatureBase::Face:
    { const FaceContactFeature* f = dynamic_cast<const FaceContactFeature*>(c);
      out<<"pts = ";
      for(size_t i=0;i<f->vertices.size();i++) {
        if(i > 0) out<<"      ";
        out<<f->vertices[i];
        if(i+1 < f->vertices.size()) out<<" \\";
        out<<endl;
      }
    }
    break;
  case ContactFeatureBase::Wheel:
    {
      const WheelContactFeature* f = dynamic_cast<const WheelContactFeature*>(c);
      out<<"pts = "<<f->p-f->axis.direction*f->width*Half<<"   "<<f->p+f->axis.direction*f->width*Half<<endl;
      out<<"axis = "<<f->axis.source<<"   "<<f->axis.direction<<endl;
    }
    break;
  case ContactFeatureBase::MultipleFaces:
    FatalError("Not done outputting multiple-face features");
    break;
  }
  out<<"weight = "<<c->weight<<endl;
  out<<"end"<<endl;
  return out;
}

bool LoadContactFeatures(const char* fn,vector<ContactFeature>& ccs)
{
  ContactFeature c;
  ifstream in(fn);
  if(!in) return false;
  in >> c;
  while(in) {
    ccs.push_back(c);
    in >> c;
  }
  if(in.bad()) return false;
  return true;
}

bool SaveContactFeatures(const char* fn,const vector<ContactFeature>& ccs)
{
  ofstream out(fn);
  if(!out) return false;
  for(size_t i=0;i<ccs.size();i++) {
    out << ccs[i] <<endl;
  }
  return true;
}

bool FeatureRayIntersection(const ContactFeatureBase* f,const Ray3D& ray,Real tol)
{
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
    return ray.distance(dynamic_cast<const PointContactFeature*>(f)->p) <= tol;
  case ContactFeatureBase::Edge:
    {
      Line3D s;
      s.source=dynamic_cast<const EdgeContactFeature*>(f)->p1;
      s.direction=dynamic_cast<const EdgeContactFeature*>(f)->p2 - s.source;
      Real t,u;
      s.closestPoint(ray,t,u);
      t = Clamp(t,Zero,One);
      if(u < Zero) u=Zero;
      Vector3 sp,rp;
      s.eval(t,sp);
      ray.eval(u,rp);
      return sp.distanceSquared(rp) <= Sqr(tol);
    }
  case ContactFeatureBase::Face:
    {
      const FaceContactFeature* ff=dynamic_cast<const FaceContactFeature*>(f);
      Matrix4 T,Tinv;
      Polygon2D poly;
      Plane3D plane;
      ff->getPlaneFit(plane);
      Real t;
      if(!plane.intersectsRay(ray,&t)) return false;
      Vector3 pt;
      ray.eval(t,pt);

      ff->getPlanarPolygon(poly,T);
      Tinv.setInverse(T);
      Vector3 pt2d = Tinv*pt;
      int contains=poly.residue(Vector2(pt2d.x,pt2d.y));
      return ((contains&0x1) == 1);
    }
  case ContactFeatureBase::Wheel:
    {
      const WheelContactFeature* wf=dynamic_cast<const WheelContactFeature*>(f);
      Cylinder3D c;
      c.center = wf->GetCenter();
      c.radius = wf->GetRadius();
      c.axis = wf->axis.direction;
      c.center -= c.axis*wf->width*Half;
      c.height = wf->width;
      Real tmin,tmax;
      if(c.intersects(ray,&tmin,&tmax))
        return tmin > 0 || tmax > 0;
      return false;
    }
  default:
    FatalError("Not done yet");
  }
  return false;
}


/// Returns the centroid of the feature contact region
Vector3 FeatureContactPoint(const ContactFeatureBase* f)
{
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
    return dynamic_cast<const PointContactFeature*>(f)->p;
  case ContactFeatureBase::Edge:
    {
      const EdgeContactFeature*ef=dynamic_cast<const EdgeContactFeature*>(f);
      return (ef->p1+ef->p2)*Half;
    }
  case ContactFeatureBase::Face:
    {
      Vector3 c;
      dynamic_cast<const FaceContactFeature*>(f)->GetCentroid(c);
      return c;
    }
  case ContactFeatureBase::Wheel:
    return dynamic_cast<const WheelContactFeature*>(f)->p;
  default:
    FatalError("Can't do that type of feature yet");
    break;
  }
  return Vector3(Zero);
}

/// Returns the local normal that is mapped to the environment contact normal.
/// The zero vector is returned if there is no such normal defined
/// (e.g. point and edge features)
Vector3 FeatureContactNormal(const ContactFeatureBase* f)
{
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
  case ContactFeatureBase::Edge:
    return Vector3(Zero);  //none
  case ContactFeatureBase::Face:
    {
      Plane3D p;
      dynamic_cast<const FaceContactFeature*>(f)->GetPlane(p);
      return -p.normal;
    }
  case ContactFeatureBase::Wheel:
    {
      const WheelContactFeature* w=dynamic_cast<const WheelContactFeature*>(f);
      Vector3 n = w->GetCenter()-w->p;
      n.inplaceNormalize();
      return n;
    }
  default:
    FatalError("Can't do that type of feature yet");
    break;
  }
  return Vector3(Zero);
}

/// Uniformly samples a contact point in the local contact region of f
Vector3 SampleFeatureContactPoint(const ContactFeatureBase* f)
{
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
    return dynamic_cast<const PointContactFeature*>(f)->p;
  case ContactFeatureBase::Edge:
    {
      const EdgeContactFeature*ef=dynamic_cast<const EdgeContactFeature*>(f);
      Real u = Rand();
      return (ef->p1+ef->p2)*u;
    }
  case ContactFeatureBase::Face:
    {
      const FaceContactFeature*ff=dynamic_cast<const FaceContactFeature*>(f);
      Assert(ff != NULL);
      Assert(ff->vertices.size() >= 3);
      Polygon3DSampler sampler;
      sampler.Set(ff->vertices);
      Assert(!sampler.IsEmpty());
      Vector3 c;
      sampler.Sample(c);
      return c;
    }
  case ContactFeatureBase::Wheel:
    {
      const WheelContactFeature* wf=dynamic_cast<const WheelContactFeature*>(f);
      return wf->p + Rand(-Half,Half)*wf->axis.direction;
    }
  default:
    FatalError("Can't do that type of feature yet");
    break;
  }
  return Vector3(Zero);
}

void HoldFromTransform(const ContactFeatureBase* f,const RigidTransform& T,Hold& h)
{
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
    {
      const PointContactFeature* pf = dynamic_cast<const PointContactFeature*>(f);
      pf->GetHold(T*pf->p,h);
    }
    break;
  case ContactFeatureBase::Edge:
    {
      const EdgeContactFeature* ef = dynamic_cast<const EdgeContactFeature*>(f);
      Vector3 dir = ef->p2-ef->p1;
      dir.inplaceNormalize();
      ef->GetHold(T*ef->p1,T*dir,h);
    }
    break;
  case ContactFeatureBase::Face:
    {
      const FaceContactFeature* ff = dynamic_cast<const FaceContactFeature*>(f);
      h.link = ff->link;
      h.ikConstraint.localPosition.setZero();
      h.ikConstraint.SetFixedPosition(T.t);
      h.ikConstraint.SetFixedRotation(T.R);
      h.contacts.resize(ff->vertices.size());
      Plane3D p;
      ff->GetPlane(p);
      for(size_t i=0;i<h.contacts.size();i++) {
        h.contacts[i].x = T*ff->vertices[i];
        h.contacts[i].n = T.R*p.normal;
        h.contacts[i].kFriction = 0;
      }
    }
    break;
  default:
    FatalError("HoldFromTransform: Not done matching feature to transform for that type of contact feature");
    break;
  }
}

} //namespace Klampt