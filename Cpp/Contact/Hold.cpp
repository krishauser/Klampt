#include "Hold.h"
#include "HoldReader.h"
#include <KrisLibrary/math3d/rotation.h>
//#include <Robotics/IKUtils.h>
#include <iostream>
#include <sstream>

namespace Klampt {

void Hold::Transform(const RigidTransform& T)
{
  for(size_t i=0;i<contacts.size();i++) {
    contacts[i].x = T*contacts[i].x;
    contacts[i].n = T.R*contacts[i].n;
  }
  ikConstraint.Transform(T);
}


void Hold::SetupFixedIKConstraint(const Vector3& localPos0, const Matrix3& rot)
{
  ikConstraint.link = link;
  Assert(!contacts.empty());
  
  ikConstraint.localPosition = localPos0;
  ikConstraint.SetFixedPosition(contacts[0].x);
  ikConstraint.SetFixedRotation(rot);
}

void Hold::SetupPointIKConstraint(const Vector3& localPos0)
{
  ikConstraint.link = link;
  Assert(!contacts.empty());
  
  ikConstraint.localPosition = localPos0;
  ikConstraint.SetFixedPosition(contacts[0].x);
  ikConstraint.SetFreeRotation();
}

void Hold::SetupAxisIKConstraint(const Vector3& localPos0, const Vector3& axis_l, const Vector3& axis_w)
{
  ikConstraint.link = link;
  Assert(!contacts.empty());
  
  ikConstraint.localPosition = localPos0;
  ikConstraint.SetFixedPosition(contacts[0].x);
  ikConstraint.SetAxisRotation( axis_l, axis_w);
}

void Hold::SetupIKConstraint(const Vector3& localPos0, const Vector3& rot) 
{
  ikConstraint.link = link;
  Assert(!contacts.empty());

  ikConstraint.localPosition = localPos0;
  ikConstraint.SetFixedPosition(contacts[0].x);

  //any rotation constraints?
  if(contacts.size()==1) {  //point contact
    ikConstraint.SetFreeRotation();
    ikConstraint.endRotation = rot;
  }
  else if(contacts.size()==2) {  //line contact
    //cout<<"Setting edge rotation example moment "<<rot<<endl;
    Vector3 axis = contacts[1].x-contacts[0].x;
    if(axis.normSquared()==0) { //degenerate, point contact
      //cout<<"Warning: degenerate point contact"<<endl;
      ikConstraint.SetFreeRotation();
      ikConstraint.endRotation = rot;
      return;
    }
    MomentRotation mr(rot);
    Matrix3 R;
    mr.getMatrix(R);
    R.inplaceTranspose();

    axis.inplaceNormalize();
    //cout<<"world-space rotation axis: "<<axis<<endl;
    //cout<<"local-space rotation axis: "<<R*axis<<endl;
    ikConstraint.SetAxisRotation(R*axis,axis);
  }
  else {  //plane contact
    ikConstraint.SetFixedRotation(rot);
  }
}

#define CMPRETURN(x,y) { if(x<y) return -1; if(x>y) return 1; }

int vec3cmp (const Vector3& a,const Vector3& b)
{
  if(a.x<b.x) return -1;
  if(a.x>b.x) return 1;
  if(a.y<b.y) return -1;
  if(a.y>b.y) return 1;
  if(a.z<b.z) return -1;
  if(a.z>b.z) return 1;
  return 0;
}

int vec3cmp_fuzzy (const Vector3& a,const Vector3& b,Real eps=Epsilon)
{
  if(a.x<b.x-eps) return -1;
  if(a.x>b.x+eps) return 1;
  if(a.y<b.y-eps) return -1;
  if(a.y>b.y+eps) return 1;
  if(a.z<b.z-eps) return -1;
  if(a.z>b.z+eps) return 1;
  return 0;
}

int mat3cmp_fuzzy (const Matrix3& a,const Matrix3& b,Real eps=Epsilon)
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) {
      if(a(i,j) < b(i,j)-eps) return -1;
      if(a(i,j) > b(i,j)+eps) return 1;
    }
  return 0;
}

int ikcmp (const IKGoal& a,const IKGoal& b)
{
  Assert(a.posConstraint==IKGoal::PosFixed && b.posConstraint==IKGoal::PosFixed);
  int cmp;
  if(int(a.rotConstraint) < int(b.rotConstraint)) return -1;
  if(int(a.rotConstraint) > int(b.rotConstraint)) return 1;
  if(a.rotConstraint == IKGoal::RotFixed) {
    //only the transform matters
    RigidTransform Ta,Tb;
    a.GetFixedGoalTransform(Ta);
    b.GetFixedGoalTransform(Tb);
    if((cmp=vec3cmp_fuzzy(Ta.t,Tb.t))!=0) return cmp;
    if((cmp=mat3cmp_fuzzy(Ta.R,Tb.R))!=0) return cmp;
  }
  else {
    if((cmp=vec3cmp_fuzzy(a.endPosition,b.endPosition))!=0) return cmp;
    if((cmp=vec3cmp_fuzzy(a.localPosition,b.localPosition))!=0) return cmp;
    if(a.rotConstraint != IKGoal::RotNone) {
      if((cmp=vec3cmp_fuzzy(a.endRotation,b.endRotation))!=0) return cmp;
      if((cmp=vec3cmp_fuzzy(a.localAxis,b.localAxis))!=0) return cmp;
    }
  }
  return 0;
}

int holdcmp(const Hold& a,const Hold& b)
{
  if(a.link < b.link) return -1;
  if(a.link > b.link) return 1;
  if(a.contacts.size() < b.contacts.size()) return -1;
  if(a.contacts.size() > b.contacts.size()) return 1;
  int cmp=ikcmp(a.ikConstraint,b.ikConstraint);
  if(cmp!=0) return cmp;
  for(size_t i=0;i<a.contacts.size();i++) {
    if((cmp=vec3cmp_fuzzy(a.contacts[i].x,b.contacts[i].x)) != 0) return cmp;
    if((cmp=vec3cmp_fuzzy(a.contacts[i].n,b.contacts[i].n)) != 0) return cmp;
    if(a.contacts[i].kFriction < b.contacts[i].kFriction) return -1;
    if(a.contacts[i].kFriction > b.contacts[i].kFriction) return 1;
  }
  return 0;
}

bool operator == (const Hold& a,const Hold& b)
{
  return (holdcmp(a,b)==0);
}

bool operator < (const Hold& a,const Hold& b)
{
  return (holdcmp(a,b)<0);
}

bool operator > (const Hold& a,const Hold& b)
{
  return (holdcmp(a,b)>0);
}




HoldReader::HoldReader(istream& in)
  :LineReader(in)
{}

bool HoldReader::Begin(const string& name,stringstream& args)
{
  h.link = -1;
  h.contacts.clear();
  h.ikConstraint.SetFreePosition();
  h.ikConstraint.SetFreeRotation();
  if(name == "hold") { return true; }
  return false;
}

bool HoldReader::Assign(const string& item,stringstream& rhs)
{
  if(item == "link") { rhs >> h.link; return true; }
  else if(item == "destLink") { rhs >> h.ikConstraint.destLink; return true; }
  else if(item == "contacts") {
    ContactPoint cp;
    while(rhs >> cp.x) {
      rhs >> cp.n >> cp.kFriction;
      if(rhs.bad()) {
	cerr<<"Error reading contact point #"<<h.contacts.size()<<endl;
	return false;
      }
      h.contacts.push_back(cp);
    }
    if(!rhs.eof()) {
      cerr<<"Error reading contact points?!?!"<<endl;
      cerr<<"rhs = "<<rhs.str()<<endl;
      return false;
    }
    //cout<<"Read "<<h.contacts.size()<<" contacts"<<endl;
    return true;
  }
  else if(item == "position") {
    Vector3 localPos,worldPos;
    rhs >> localPos;
    rhs >> worldPos;
    if(rhs.bad()) return false;
    h.ikConstraint.localPosition = localPos;
    h.ikConstraint.SetFixedPosition(worldPos);
    return true;
  }
  else if(item == "axis") {
    Vector3 localAxis,worldAxis;
    rhs >> localAxis;
    rhs >> worldAxis;
    if(rhs.bad()) return false;
    h.ikConstraint.SetAxisRotation(localAxis,worldAxis);
    return true;
  }
  else if(item == "rotation") {
    Vector3 rot;
    rhs >> rot;
    if(rhs.bad()) return false;
    h.ikConstraint.SetFixedRotation(rot);
    return true;
  }
  else if(item == "ikparams") {
    Vector3 localPos0,rot;
    rhs >> localPos0;
    rhs >> rot;
    h.SetupIKConstraint(localPos0,rot);
  }
  cout<<"Unknown item "<<item<<endl;
  return false;
}

bool HoldReader::End() 
{ 
  if(h.link < 0) {
    cerr<<"Invalid link!"<<endl;
    return false;
  }
  h.ikConstraint.link = h.link;
  if(h.ikConstraint.posConstraint==IKGoal::PosNone) {
    cerr<<"Read no ik constraints!"<<endl;
    return false;
  }
  if(h.contacts.empty()) {
    cerr<<"Hold Load: Warning, empty contacts"<<endl;
    return true;
  }
  return true;
}



ostream& operator << (ostream& out, const Hold& h)
{
  out<<"begin hold"<<endl;
  out<<"link = "<<h.link<<endl;
  if(h.ikConstraint.destLink >= 0)
    out<<"destLink = "<<h.ikConstraint.destLink<<endl;
  out<<"contacts = ";
  for(size_t i=0;i<h.contacts.size();i++) {
    if(i != 0)
      out<<"           "; //nice spacing
    out<<h.contacts[i].x<<"\t\t"<<h.contacts[i].n<<"\t\t"<<h.contacts[i].kFriction;
    if(i+1 != h.contacts.size()) out<<"  \\"<<endl;
  }
  out<<endl;
  out<<"position = "<<h.ikConstraint.localPosition<<"  \\"<<endl;
  out<<"           "<<h.ikConstraint.endPosition<<endl;
  if(h.ikConstraint.rotConstraint==IKGoal::RotAxis) {
    out<<"axis = "<<h.ikConstraint.localAxis<<"  \\"<<endl;
    out<<"       "<<h.ikConstraint.endRotation<<endl;
  }
  else if(h.ikConstraint.rotConstraint==IKGoal::RotFixed) {
    out<<"rotation = "<<h.ikConstraint.endRotation<<endl;
  }
  out<<"end"<<endl;
  return out;
}

istream& operator >> (istream& in, Hold& h)
{
  HoldReader reader(in);
  if(!reader.Read()) {
    printf("Failed while reading hold\n");
    in.setstate(ios::failbit);
    return in;
  }
  h = reader.h;
  return in;
}

} //namespace Klampt