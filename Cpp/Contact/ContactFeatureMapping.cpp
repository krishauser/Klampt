#include "ContactFeatureMapping.h"
#include "KrisLibrary/robotics/Rotation.h"
#include <KrisLibrary/math3d/Plane3D.h>
#include <KrisLibrary/math3d/misc.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/angle.h>
#include "LineReader.h"
#include <KrisLibrary/utils/ioutils.h>
#include <sstream>
using namespace std;

namespace Klampt {

bool MatchHoldToFeatureMapping(const Hold& h,const ContactFeature& f,ContactFeatureMapping& m,Real& error);

ContactFeatureMapping::ContactFeatureMapping()
  :angle(0),fixedWheel(false),wheelRoll(0),localOffset(Zero)
{}

void ContactFeatureMapping::GetLocalFrame(RigidTransform& T) const
{
  Hold temp;
  GetHold(temp);
  switch(temp.ikConstraint.rotConstraint) {
  case IKGoal::RotFixed:
    temp.ikConstraint.GetFixedGoalTransform(T);
    break;
  case IKGoal::RotAxis:
    temp.ikConstraint.GetEdgeGoalTransform(0,T);
    break;
  case IKGoal::RotNone:
    //what should we do? identity transform?
    T.t = temp.ikConstraint.endPosition - temp.ikConstraint.localPosition;
    T.R.setIdentity();
    break;
  default:
    FatalError("Not done yet");
    break;
  }
}

void ContactFeatureMapping::GetHold(Hold& h) const
{
  ContactFeatureBase::Type type=feature->GetType();
  const ContactFeatureBase* fp = feature.get();
  switch(type) {
  case ContactFeatureBase::Point:
    dynamic_cast<const PointContactFeature*>(fp)->GetHold(contact.x,h);
    h.contacts[0].n = contact.n;
    break;
  case ContactFeatureBase::Edge:
    dynamic_cast<const EdgeContactFeature*>(fp)->GetHold(contact.x,contact.n,angle,h);
    break;
  case ContactFeatureBase::Face:
    dynamic_cast<const FaceContactFeature*>(fp)->GetHold(contact.x,contact.n,angle,h);
    break;
  case ContactFeatureBase::Wheel:
    if(fixedWheel)
      dynamic_cast<const WheelContactFeature*>(fp)->GetFixedHold(contact.x,contact.n,angle,wheelRoll,h);
    else
      dynamic_cast<const WheelContactFeature*>(fp)->GetHold(contact.x,contact.n,angle,h);
    break;
  case ContactFeatureBase::MultipleFaces:
    FatalError("Not done with creating multiple-face contact feature");
    break;
  }
  h.ikConstraint.localPosition += localOffset;
  Assert(h.link == feature->link);
  for(size_t i=0;i<h.contacts.size();i++)
    h.contacts[i].kFriction = contact.kFriction;
}


class ContactFeatureMappingReader : public LineReader
{
public:
  ContactFeatureMappingReader(istream& in,const vector<ContactFeature>& _features)
    :LineReader(in),features(_features)
  {}
  virtual bool Begin(const string& name,stringstream& args)
  {
    featureName.erase();
    fm.contact.x.set(0,0,0);
    fm.contact.n.set(0,0,1);
    fm.contact.kFriction = 1;
    fm.angle=0;
    fm.fixedWheel=false;
    fm.wheelRoll=0;
    if(name == "featuremapping") { return true; }
    return false;
  }
  virtual bool Assign(const string& item,stringstream& rhs)
  {
    if(item == "feature") { return ::InputQuotedString(rhs,featureName); }
    else if(item == "x") { rhs >> fm.contact.x; return true; }
    else if(item == "n") { rhs >> fm.contact.n; return true; }
    else if(item == "friction") { rhs >> fm.contact.kFriction; return true; }
    else if(item == "angle") { rhs >> fm.angle; return true; }
    else if(item == "fixedwheel") { fm.fixedWheel=true; return true; }
    else if(item == "wheelroll") { rhs >> fm.wheelRoll; return true; }
    cout<<"Unknown item "<<item<<endl;
    return false;
  }
  virtual bool End() 
  { 
    fm.feature = NULL;
    for(size_t i=0;i<features.size();i++) {
      if(featureName == features[i]->name)
	fm.feature = features[i];
    }
    if(fm.feature == NULL) {
      cerr<<"Invalid feature named: "<<featureName<<endl;
      return false;
    }
    return true;
  }

  string featureName;
  ContactFeatureMapping fm;
  const vector<ContactFeature>& features;
};

bool LoadContactFeatureMapping(istream& in,const vector<ContactFeature>& features,ContactFeatureMapping& m)
{
  ContactFeatureMappingReader reader(in,features);
  if(reader.Read() && in) {
    m = reader.fm;
    Assert(m.feature != NULL);
    return true;
  }
  else {
    in.setstate(ios::failbit);
    return false;
  }
}

void SaveContactFeatureMapping(ostream& out,ContactFeatureMapping& m)
{
  out<<"begin featuremapping"<<endl;
  out<<"  feature = \""<<m.feature->name<<"\""<<endl;
  out<<"  x = "<<m.contact.x<<endl;
  out<<"  n = "<<m.contact.n<<endl;
  out<<"  friction = "<<m.contact.kFriction<<endl;
  if(m.feature->GetType() != ContactFeatureBase::Point) 
    out<<"  angle = "<<m.angle<<endl;
  if(m.feature->GetType() == ContactFeatureBase::Wheel) {
    out<<"  fixedwheel = "<<m.fixedWheel<<endl;
    if(m.fixedWheel)
      out<<"  wheelroll = "<<m.wheelRoll<<endl;
  }
  out<<"end"<<endl;
}

bool MatchHoldToFeatureMapping(const Hold& h,const ContactFeature& f,ContactFeatureMapping& m,Real& error)
{
  const ContactFeatureBase* _f=f.get();
  if(h.link != f->link) return false;
  if(h.contacts.empty()) {
    cerr<<"MatchHoldToFeatureMapping: hold has no contacts??"<<endl;
    return false;
  }
  switch(f->GetType()) {
  case ContactFeatureBase::Point:
    if(h.ikConstraint.rotConstraint!=IKGoal::RotNone) return false;
    if(h.contacts.size() != 1) {
      cerr<<"Hmm... point-constrained hold with more than one contact point?"<<endl;
      return false;
    }
    {
      const PointContactFeature* pf=dynamic_cast<const PointContactFeature*>(_f);
      error = pf->p.distance(h.ikConstraint.localPosition);
      m.feature = f;
      m.contact = h.contacts[0];
      return true;
    }
    break;
  case ContactFeatureBase::Edge:
    if(h.ikConstraint.rotConstraint!=IKGoal::RotAxis) return false;
    if(h.contacts.size() != 2) {
      cerr<<"Hmm... edge-constrained hold with more than two contact points?"<<endl;
      return false;
    }
    {
      const EdgeContactFeature* ef=dynamic_cast<const EdgeContactFeature*>(_f);
      Line3D featureAxis;
      featureAxis.source = ef->p1;
      featureAxis.direction = ef->p2-ef->p1;
      featureAxis.direction.inplaceNormalize();
      //closeness has to make sure it's not a wheel hold?
      //check the distance from the feature edge axis to the constraint local pos
      Real axisMatch = featureAxis.direction.dot(h.ikConstraint.localAxis);
      Real axisDist = featureAxis.distance(h.ikConstraint.localPosition);
      error = Acos(axisMatch) + axisDist;  //TODO: weights?

      m.feature = f;
      //pick the x that matches p1 to the hold local position
      m.contact.x = h.ikConstraint.endPosition;
      m.contact.n = h.contacts[0].n+h.contacts[1].n;
      m.contact.n.inplaceNormalize();
      m.contact.kFriction = Half*(h.contacts[0].kFriction+h.contacts[1].kFriction);
      //1. Rotate the feature's axis to the plane
      //2. Pick the angle that rotates it to the world axis
      Matrix3 R;
      GetMinimalRotationToPlane(featureAxis.direction,m.contact.n,R);
      Vector3 planeAxis = R*featureAxis.direction;
      //get the angle that rotates planeAxis to h.ikConstraint.endRotation (about the normal)
      Vector3 xb = planeAxis; xb.inplaceNormalize();
      Vector3 yb; yb.setCross(m.contact.n,xb);
      Real x = dot(xb,h.ikConstraint.endRotation);
      Real y = dot(yb,h.ikConstraint.endRotation);
      m.angle = Atan2(y,x);
      return true;
    }
    break;
  case ContactFeatureBase::Face:
    if(h.ikConstraint.rotConstraint != IKGoal::RotFixed) return false;
    if(h.contacts.size() < 3) return false;  //could be a wheel hold
    {
      const FaceContactFeature* ff=dynamic_cast<const FaceContactFeature*>(_f);
      //make sure the local plane lines up with the hold contact points
      Plane3D p,pworld;
      ff->GetPlane(p);
      RigidTransform T;
      h.ikConstraint.GetFixedGoalTransform(T);
      pworld.setTransformed(p,T);
      Real dist=0;
      for(size_t k=0;k<h.contacts.size();k++) {
	Real pdiff = pworld.distance(h.contacts[k].x);
	dist += Sqr(pdiff);
      }
      dist /= h.contacts.size();
      dist = Sqrt(dist);

	/*
	//check that the contacts make a fairly good planar polygon
	Polygon3D hpoly;
	hpoly.vertices.resize(h.contacts.size());
	for(size_t k=0;k<h.contacts.size();k++) 
	  hpoly.vertices[k] = h.contacts[k].x;
	Plane3D hp;
	hpoly.getPlaneFit(hp);
	cout<<"Fitted plane normal: "<<hp.normal<<endl;
	for(size_t k=0;k<h.contacts.size();k++) {
	  if(Abs(h.contacts[k].n.dot(hp.normal)) < 0.9) {
	    cout<<"The normal at contact "<<k<<" deviates from the fit plane normal:"<<endl;
	    cout<<h.contacts[k].n<<endl;
	  }
	}
	*/

      Vector3 avgNormal(Zero);
      for(size_t k=0;k<h.contacts.size();k++)
	avgNormal += h.contacts[k].n;
      avgNormal.inplaceNormalize();
      Real nangle = Acos(-avgNormal.dot(pworld.normal));
      dist += 0.1*nangle; 
      error = dist;

      m.feature = f;
      //map the centroid of the feature to the contact point
      Vector3 centroid(Zero);
      for(size_t i=0;i<ff->vertices.size();i++)
	centroid += ff->vertices[i];
      centroid /= (Real)ff->vertices.size();
      m.contact.x = T * centroid;
      m.contact.n = avgNormal;
      //average friction
      m.contact.kFriction = 0;
      for(size_t k=0;k<h.contacts.size();k++)
	m.contact.kFriction += h.contacts[k].kFriction;
      m.contact.kFriction /= (Real)h.contacts.size();
      
      //get rotation angle
      //1. get the base matrix that rotates to the world frame
      //2. find the angle about the normal that rotates to the goal rotation
      Matrix3 R0; 
      GetMinimalRotation(-p.normal,m.contact.n,R0);
      //R = R(normal,theta)*R0
      Matrix3 Raxis; Raxis.mulTransposeB(T.R,R0);
      AngleAxisRotation aa;
      Assert(IsFinite(Raxis));
      aa.setMatrix(Raxis);
      //axis should be +/- normal
      if(FuzzyZero(aa.angle,(Real)1e-3)) {
	m.angle = 0;
      }
      else if(FuzzyEquals(aa.angle,Pi,(Real)1e-3)) {
	m.angle = Pi;
      }
      else {
	/*
	  if(!FuzzyEquals(Abs(aa.axis.dot(m.contact.n)),One,(Real)5e-2)) {
	  cerr<<"contact feature "<<ff->name<<endl;
	  cerr<<"Rotation axis does not match with normal!"<<endl;
	  cerr<<"Feature normal "<<p.normal<<endl;
	  cerr<<"Terrain normal "<<m.contact.n<<endl;
	  cerr<<"end Rotation"<<endl<<T.R<<endl<<endl;
	  cerr<<"base rotation"<<endl<<R0<<endl<<endl;
	  cerr<<"axis rotation"<<endl<<Raxis<<endl<<endl;
	  cerr<<"angle "<<aa.angle<<", axis "<<aa.axis<<endl;
	  cerr<<"Probably shouldn't use this feature"<<endl;
	  getchar();
	  }
	*/
	if(aa.axis.dot(m.contact.n) < 0) //reverse
	  m.angle = -aa.angle;
	else
	  m.angle = aa.angle;
      }
      return true;
    }
    break;
  case ContactFeatureBase::Wheel:
    if(h.contacts.size() == 2 && h.ikConstraint.rotConstraint==IKGoal::RotAxis) {   //free wheel
      const WheelContactFeature* wf=dynamic_cast<const WheelContactFeature*>(_f);
      Real dist=wf->axis.distance(h.ikConstraint.localPosition);
      Real axisDiff=wf->axis.direction.distance(h.ikConstraint.localAxis);
      Line3D worldAxis; worldAxis.source=h.ikConstraint.endPosition; worldAxis.direction=h.ikConstraint.endRotation;
      Real contactDiff=worldAxis.distance(h.contacts[0].x);
      error = dist + 0.1*axisDiff+contactDiff;  //weights?
      
      m.feature = f;
      m.contact.n = h.contacts[0].n+h.contacts[1].n;
      m.contact.n.inplaceNormalize();
      //ik.endPosition = x + r*n
      m.contact.x = h.ikConstraint.endPosition - contactDiff*m.contact.n;
      m.contact.kFriction = Half*(h.contacts[0].kFriction+h.contacts[1].kFriction);
      m.fixedWheel = false;
      //1. Rotate the feature's axis to the plane
      //2. Pick the angle that rotates it to the world axis
      Matrix3 R;
      GetMinimalRotationToPlane(wf->axis.direction,m.contact.n,R);
      Vector3 planeAxis = R*wf->axis.direction;
      //get the angle that rotates planeAxis to h.ikConstraint.endRotation (about the normal)
      Vector3 xb = planeAxis; xb.inplaceNormalize();
      Vector3 yb; yb.setCross(m.contact.n,xb);
      Real x = dot(xb,h.ikConstraint.endRotation);
      Real y = dot(yb,h.ikConstraint.endRotation);
      m.angle = Atan2(y,x);
      return true;
    }
    else if(h.ikConstraint.rotConstraint==IKGoal::RotFixed) {   //fixed wheel
      const WheelContactFeature* wf=dynamic_cast<const WheelContactFeature*>(_f);

      Plane3D p,plocal;
      h.GetNormal(p.normal);
      Vector3 c=h.GetCentroid();
      p.offset = p.normal.dot(c);
      RigidTransform T,Tinv;
      h.ikConstraint.GetFixedGoalTransform(T);
      Tinv.setInverse(T);
      plocal.setTransformed(p,Tinv);
      Real r = wf->axis.distance(wf->p);
      Vector3 wheelCenter; wf->axis.closestPoint(wf->p,wheelCenter);
      Real axisDiff = Abs(p.normal.dot(T.R*wf->axis.direction));
      Real rimDist = Abs(plocal.distance(wheelCenter)-r);
      if(rimDist > 0.05) {
	stringstream ss;
	ss<<"Large distance in matching wheel feature!"<<endl;
	ss<<"Wheel contact centroid "<<c<<", normal "<<p.normal<<endl;
	ss<<"Transform: "<<T<<endl;
	ss<<"Transform of wheel center: "<<T*wheelCenter<<endl;
	ss<<"wheel plane distance from center: "<<plocal.distance(wheelCenter)<<endl;
	ss<<"AxisDiff "<<axisDiff<<", RimDist "<<rimDist;
	cerr<<ss.str()<<endl;
      }
      axisDiff=0;
      error=axisDiff+rimDist;

      m.feature = f;
      m.contact.n = p.normal;
      //T*wheel center = x + r*n
      m.contact.x = T*wheelCenter - r*m.contact.n;
      m.contact.kFriction = Zero;
      for(size_t i=0;i<h.contacts.size();i++)
	m.contact.kFriction += h.contacts[i].kFriction;
      m.contact.kFriction /= Real(h.contacts.size());
      m.fixedWheel = true;
      //1. Rotate the feature's axis to the plane
      //2. Pick the angle that rotates it to the world axis
      Matrix3 R;
      GetMinimalRotationToPlane(wf->axis.direction,m.contact.n,R);
      Vector3 planeAxis = R*wf->axis.direction;
      Vector3 worldAxis = T.R*wf->axis.direction;
      //get the angle that rotates planeAxis to worldAxis (about the normal)
      Vector3 xb = planeAxis; xb.inplaceNormalize();
      Vector3 yb; yb.setCross(m.contact.n,xb);
      Real x = dot(xb,worldAxis);
      Real y = dot(yb,worldAxis);
      m.angle = Atan2(y,x);
      //Pick the wheel roll that rotates x to p
      Vector3 xloc = Tinv*m.contact.x;
      xb = wf->p-wheelCenter; xb.inplaceNormalize();
      yb.setCross(wf->axis.direction,xb);
      x=dot(xloc-wheelCenter,xb);
      y=dot(xloc-wheelCenter,yb);
      m.wheelRoll = Atan2(y,x);
      return true;
    }
    break;
  default:
    break;
  }
  return false;
}

bool FeatureMappingFromHold(const Hold& h,const vector<ContactFeature>& features, ContactFeatureMapping& m)
{
  int closestMatch=-1;
  Real closestDist=Inf;
  ContactFeatureMapping temp;
  for(size_t i=0;i<features.size();i++) {
    Real dist;
    if(MatchHoldToFeatureMapping(h,features[i],temp,dist)) {
      if(dist < closestDist) {
	if(!IsInf(closestDist)) {
	  if(dist*100.0 > closestDist) {
	    stringstream ss;
	    ss<<"Choosing between two features: "<<m.feature->name<<" and "<<temp.feature->name<<endl;
	    ss<<"Differences "<<closestDist<<" and "<<dist<<endl;
	    cerr<<ss.str()<<endl;
	  }
	}
	m = temp;
	closestDist = dist;
	closestMatch = (int)i;
      }
    }
  }
  if(closestMatch == -1) return false;
  else if(closestDist > 0.05) {
    stringstream ss;
    ss<<"Matching distance "<<closestDist<<" exceeded lower bound of 0.05"<<endl;
    for(size_t i=0;i<features.size();i++) {
      Real dist;
      if(MatchHoldToFeatureMapping(h,features[i],temp,dist)) {
	ss<<"  Feature "<<features[i]->name<<" has distance "<<dist<<endl;
      }
      else {
	ss<<"  Feature "<<features[i]->name<<" couldn't be matched"<<endl;
      }
    }
    cerr<<ss.str()<<endl;
    //TEMP: this is here for things that assert holds map to contact features?
    return true;
    return false;
  }
  return true;
}




Real BestFeatureMappingAngle(const ContactFeatureMapping& feature,const Matrix3& Rdes)
{
  Hold h;
  feature.GetHold(h);
  if(h.ikConstraint.rotConstraint == IKGoal::RotFixed) {
    RigidTransform T;
    h.ikConstraint.GetFixedGoalTransform(T);
    //find the angle that matches T to the current transform when rotated about the contact normal
    Matrix3 Rrel;  //transform of feature relative to contact
    Rrel.mulTransposeA(T.R,Rdes);
    Vector3 nrel;
    T.R.mulTranspose(feature.contact.n,nrel);
    Real angleDelta = -MatrixAngleAboutAxis(Rrel,nrel);
    Real bestAngle = angleDelta + feature.angle;
    return bestAngle;

    /*
    ContactFeature fm;
    fm = feature;
    fm.angle = bestAngle;
    The following commented out code tests to make sure that this is the correct angle...
    AngleAxisRotation aa;    
    aa.setMatrix(Rrel);
    cout<<"Theta=0 orientation error: "<<aa.angle<<endl;

    Matrix3 Rangle;
    aa.axis = nrel;
    aa.angle = -fm.angle;
    aa.getMatrix(Rangle);
    Rrel = Rangle*Rrel;
    aa.setMatrix(Rrel);
    cout<<"Theoretical theta="<<fm.angle<<" orientation error: "<<aa.angle<<endl;

    fm.GetHold(h);
    h.ikConstraint.GetFixedGoalTransform(T);
    Rrel.mulTransposeA(T.R,Rdes);
    aa.setMatrix(Rrel);
    cout<<"Theta="<<fm.angle<<" orientation error: "<<aa.angle<<endl;
    fm.angle += 0.1;
    fm.GetHold(h);
    h.ikConstraint.GetFixedGoalTransform(T);
    Rrel.mulTransposeA(T.R,robot.links[feature->link].T_World.R);
    aa.setMatrix(Rrel);
    cout<<"Theta="<<fm.angle<<" orientation error: "<<aa.angle<<endl;
    fm.angle -= 0.2;
    fm.GetHold(h);
    h.ikConstraint.GetFixedGoalTransform(T);
    Rrel.mulTransposeA(T.R,robot.links[feature->link].T_World.R);
    aa.setMatrix(Rrel);
    cout<<"Theta="<<fm.angle<<" orientation error: "<<aa.angle<<endl;
    fm.angle += 0.1;
    getchar();
    */
  }
  else if(h.ikConstraint.rotConstraint == IKGoal::RotAxis) { 
    cerr<<"TODO: BestFeatureMappingAngle for axis-type feature mappings"<<endl;
    //TODO: something smarter
    return Rand()*TwoPi;
  }
  else {  //angle doesn't matter
    return 0;
  }
}

//finds the rotation angle about axis that minimizes the distance between
//R*a and b (where R is a rotation matrix about axis)
Real BestRotationAngle(const Vector3& axis,const Vector3& a,const Vector3& b)
{
  /* min ||R*a-b||^2
     = |a|^2 + |b|^2 - 2b^t*R*a
     is reached when -2b^t*R*a is minimized
     extremum when 0 = b^t*R'*a where R' is the derivative w.r.t. angle

     R(axis=r,angle=q) = s[r]-c[r][r]+rrt = s[r]-c(rrt-I)+rrt = cI + rrt(1-c) + s[r]
     so R' = -sI + rrts + c[r]
     and b^t*R'*a = -s b.a + (b.r)(a.r)(s) + c b.(rxa) 
     => s*((b.r)(a.r)-b.a) + c*(b.(rxa)) = 0

     minimum v.s maximum is found by requiring positive 2nd derivative
     of -2b^t*R*a, which means b^t*R''*a is negative
     R'' = c(rrt-I) - s[r]
     meaning c*((b.r)(a.r)-a.b) - s*(b.(rxa)) < 0
  */
  Real ar=dot(a,axis),br=dot(b,axis);
  Real triple = dot(b,cross(axis,a));
  Real ab = dot(a,b);
  Real q1,q2;
  bool res=SolveCosSinEquation(triple,ar*br-ab,0,q1,q2);
  if(!res) {
    cerr<<"Error solving best rotation angle"<<endl;
    return Rand()*TwoPi;
  }
  Real c=Cos(q1), s=Sin(q1);
  if(c*(ar*br-ab) - s*triple < 0) return q1;
  else return q2;
}

void SetFeatureMappingOrientation(ContactFeatureMapping& feature,const Matrix3& Rdes)
{
  if(feature.feature->GetType() != ContactFeatureBase::Wheel) {
    feature.angle = BestFeatureMappingAngle(feature,Rdes);
    return;
  }

  const WheelContactFeature* wf=dynamic_cast<const WheelContactFeature*>(feature.feature.get());
  //orient axis by setting the normal rotation angle
  Matrix3 R_axis_to_n;
  GetMinimalRotationToPlane(wf->axis.direction,feature.contact.n,R_axis_to_n);
  Vector3 baseAxis = R_axis_to_n*wf->axis.direction;
  Vector3 axisDes = Rdes*wf->axis.direction;
  //find the angle rotating around n, that maps baseAxis to axisDes
  feature.angle = BestRotationAngle(feature.contact.n,baseAxis,axisDes);

  //find the best roll angle
  Matrix3 R_about_n;
  AngleAxisRotation aa(feature.angle,feature.contact.n);
  aa.getMatrix(R_about_n);
  Matrix3 R_no_roll = R_about_n*R_axis_to_n;
  baseAxis = R_no_roll*wf->axis.direction;  //roll rotates about this direction

  Vector3 locz = wf->GetCenter()-wf->p;
  locz.inplaceNormalize();
  Vector3 basez = feature.contact.n;//R_no_roll*locz;
  Vector3 desz = Rdes*locz;
  feature.wheelRoll = BestRotationAngle(baseAxis,basez,desz);
}

} //namespace Klampt