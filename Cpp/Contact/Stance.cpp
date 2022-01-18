#include "Stance.h"
#include "HoldReader.h"
#include <KrisLibrary/statistics/OLS.h>
#include <KrisLibrary/utils/SimpleParser.h>
#include <string>
#include <iostream>

namespace Klampt {

bool Stance::remove(int link)
{
  iterator i=find(link);
  if(i == end()) return false;
  erase(i);
  return true;
}

bool Stance::isValid() const
{
  for(const_iterator i=begin();i!=end();i++) {
    if(i->first != i->second.link) return false;
  }
  return true;
}



struct StanceReader : public SimpleParser
{
  StanceReader(istream& in) : SimpleParser(in),holdReader(in),mode(0) {}
  virtual Result InputToken(const string& word)
  {
    //cout<<"InputToken("<<word<<"), mode="<<mode<<endl;
    switch(mode) {
    case 0:
      if(word != "begin") {
	cerr<<"StanceReader: Error reading begin"<<endl;
	return Error;
      }
      mode=1;
      break;
    case 1:
      if(word != "stance") {
	cerr<<"StanceReader: Error reading begin stance"<<endl;
	return Error;
      }
      mode=2;
      break;
    case 2:
      if(word == "begin") {
	holdReader.lineno = lineno;
	holdReader.InputToken(word);
	mode=3;
      }
      else if(word == "end") {
	return Stop;
      }
      else { //anything else?
	cerr<<"StanceReader: Some weird word during stance: "<<word<<endl;
	return Error;
      }
      break;
    case 3:
      {
	holdReader.lineno = lineno;
	Result res=holdReader.InputToken(word);
	if(res == Stop) {
	  stance.insert(holdReader.h);
	  mode=2;
	  return Continue;
	}
	return res;
      }
    }
    return Continue;
  }
  virtual Result InputPunct(const string& punct)
  {
    //cout<<"InputPunct("<<punct<<"), mode="<<mode<<endl;
    if(mode==3) {
      holdReader.lineno = lineno;
      return holdReader.InputPunct(punct);
    }
    return Error;
  }
  virtual Result InputEndLine()
  {
    //cout<<"InputEndline(), mode="<<mode<<endl;
    if(mode==3) {
      holdReader.lineno = lineno;
      return holdReader.InputEndLine();
    }
    return Continue;
  }

  HoldReader holdReader;
  //0 none
  //1 read begin
  //2 read begin stance
  //3 reading hold
  int mode;
  Stance stance;
};

istream& operator >> (istream& in,Stance& stance)
{
  StanceReader reader(in);
  if(!reader.Read()) {
    in.setstate(ios::failbit);
    return in;
  }
  stance = reader.stance;
  return in;
}

ostream& operator << (ostream& out,const Stance& stance)
{
  out<<"begin stance"<<endl;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) 
    out<<i->second;
  out<<"end"<<endl;
  return out;
}



bool CopyStance(const vector<int>& indices,const vector<Hold>& holds,Stance& s)
{
  bool res=true;
  s.clear();
  for(size_t i=0;i<indices.size();i++) {
    int h=indices[i];
    if(h < 0 || h >= (int)holds.size()) res=false;
    else if(s.contains(holds[h].link)) res=false;
    else s.insert(holds[h]);
  }
  return res;
}


const Hold& DifferingHold(const Stance& a,const Stance& b)
{
  assert(a.size()+1 == b.size());
  Stance::const_iterator res=b.end();
  for(Stance::const_iterator i=b.begin();i!=b.end();i++)
    if(!a.contains(i->first)) {
      Assert(res == b.end());
      res = i;
    }
  Assert(res != b.end());
  return res->second;
}

bool IsSimilarStance(const Stance& a,const Stance& b)
{
  for(Stance::const_iterator i=a.begin();i!=a.end();i++)
    if(!b.contains(i->first)) return false;
  for(Stance::const_iterator i=b.begin();i!=b.end();i++)
    if(!a.contains(i->first)) return false;
  return true;
}

bool IsSubset(const Stance& a,const Stance& b)
{
  for(Stance::const_iterator i=a.begin();i!=a.end();i++) {
    Stance::const_iterator ib = b.find(i->first);
    if(ib == b.end()) return false;
    if(!(ib->second == i->second)) return false;
  }
  return true;
}

void GetDifference(const Stance& a,const Stance& b,vector<Hold>& d)
{
  d.clear();
  for(Stance::const_iterator i=a.begin();i!=a.end();i++) {
    Stance::const_iterator ib = b.find(i->first);
    if(ib == b.end()) d.push_back(i->second);
    else if(!(ib->second == i->second)) d.push_back(i->second);
  }
}

//TODO: put definition in header file
void TransformStance(Stance& s,const RigidTransform& T)
{
  for(Stance::iterator i=s.begin();i!=s.end();i++)
    i->second.Transform(T);
}



Vector3 GetCentroid(const Stance& s,Real L)
{
  Vector3 c(Zero);
  int n=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
    c += i->second.GetCentroid();
    if(L != 0) c.madd(i->second.GetNormal(),L);
    n++;
  }
  if(n!=0) c /= n;
  return c;
}

Real GetStdDev(const Stance& s)
{
  return GetStdDev(s,GetCentroid(s));
}

Real GetStdDev(const Stance& s,const Vector3& c)
{
  Real d=0;
  int n=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
    const Hold& h=i->second;
    for(size_t j=0;j<h.contacts.size();j++)
      d += (h.contacts[j].x-c).normSquared();
    n+=h.contacts.size();
  }
  if(n==0) return 0;
  return Sqrt(d)/n;
}

int NumContactPoints(const Stance& s)
{
  int n=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++)
    n += (int)i->second.contacts.size();
  return n;
}

void GetContactPoints(const Stance& s,vector<ContactPoint>& cps)
{
  cps.resize(NumContactPoints(s));
  int n=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
    for(size_t k=0;k<i->second.contacts.size();k++)
      cps[n+k] = i->second.contacts[k];
    n += (int)i->second.contacts.size();
  }
}

const ContactPoint& GetContactPoint(const Stance& s,int k)
{
  for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
    if(k < (int)i->second.contacts.size())
      return i->second.contacts[k];
    k -= i->second.contacts.size();
  }
  AssertNotReached();
  return s.begin()->second.contacts[0];
}

ContactPoint& GetContactPoint(Stance& s,int k)
{
  for(Stance::iterator i=s.begin();i!=s.end();i++) {
    if(k < (int)i->second.contacts.size())
      return i->second.contacts[k];
    k -= i->second.contacts.size();
  }
  AssertNotReached();
  return s.begin()->second.contacts[0];
}


void GetPlaneFit(const Stance& s,Plane3D& p)
{
  int np = NumContactPoints(s);
  Assert(np >= 0);
  if(np < 3) {
    cerr<<"GetPlaneFit(): Grasp contains less than 3 contact points, returning +z plane"<<endl;
    p.normal.setZero(); p.normal.z = 1;
    p.offset=0;
    return;
  }
  std::vector<Vector> v(np);
  int k=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
    const Hold& h = i->second;
    for(size_t j=0;j<h.contacts.size();j++,k++) {
      Assert(k < np);
      v[k].resize(3);
      v[k].copy(h.contacts[j].x);
    }
  }
  Assert(k == (int)v.size());
  int d; //dependent variable
  Vector coeffs;
  coeffs.resize(3);
  bool res=Statistics::LeastSquaresPickDependent(v,d,coeffs);
  if(!res) {
    cerr<<"GetPlaneFit(): Warning, least squares failed!"<<endl;
    p.normal.setZero(); p.normal.z=1;
    p.offset=Zero;
    return;
  }
  Assert(coeffs.n == 3);
  //cout<<"Dependent variable: "<<d<<endl;
  //if d == 0, x = c0 + c1*y + c2*z
  //if d == 1, y = c0*x + c1 + c2*z
  //if d == 2, z = c0*x + c1*y + c2
  Assert(d >= 0 && d <= 2);
  p.normal.set(coeffs(0),coeffs(1),coeffs(2));
  p.offset = -p.normal[d];
  p.normal[d] = -1;
  Real scale = One/p.normal.norm();
  if(p.normal.z < 0) scale = -scale;
  p.normal *= scale;
  p.offset *= scale;
}

void GetPlaneFit(const Stance& s,const Vector3& c,Plane3D& p)
{
  GetPlaneFit(s,p);
}


void ToContactFormation(const Stance& stance,ContactFormation& contacts)
{
  contacts.links.resize(stance.size());
  contacts.contacts.resize(stance.size());
  int k=0;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    contacts.links[k]=i->first;
    contacts.contacts[k] = i->second.contacts;
    //if any self-contacts, set the targets member
    if(i->second.ikConstraint.destLink >= 0) {
      contacts.targets.resize(stance.size(),-1);
      contacts.targets[k] = i->second.ikConstraint.destLink;
    }
    k++;
  }
}

void ToIKProblem(const Stance& stance,vector<IKGoal>& constraints)
{
  constraints.resize(stance.size());
  int k=0;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    constraints[k] = i->second.ikConstraint;
    k++;
  }
}

} //namespace Klampt