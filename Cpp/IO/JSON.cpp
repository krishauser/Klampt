#include "JSON.h"

namespace Klampt {

template <> void Convert(const Vector& v,AnyCollection& c)
{
  Convert(vector<Real>(v),c);
}

template <> bool Convert(const AnyCollection& c,Vector& v)
{
  if(c.as(v)) return true;
  vector<Real> array;
  if(!Convert(c,array)) return false;
  v = array;
  return true;
}

template <> void Convert(const Vector3& x,AnyCollection& c)
{
  vector<double> v(3);
  x.get(v[0],v[1],v[2]);
  c = v;
}

template <> bool Convert(const AnyCollection& c,Vector3& x)
{
  if(c.as(x)) return true;
  vector<double> v;
  if(!c.asvector<double>(v))
    return false;
  if(v.size() != 3) 
    return false;
  x.set(v[0],v[1],v[2]);
  return true;
}

template <> void Convert(const IKGoal& g,AnyCollection& c)
{
  c["link"]=g.link;
  if(g.destLink >= 0)
    c["destLink"]=g.destLink;
  switch(g.posConstraint) {
  case IKGoal::PosNone:
    break;
  case IKGoal::PosPlanar:
    c["posConstraint"] = string("planar");
    Convert(g.localPosition,c["localPosition"]);
    Convert(g.endPosition,c["endPosition"]);
    Convert(g.direction,c["direction"]);
    break;
  case IKGoal::PosLinear:
    c["posConstraint"] = string("linear");
    Convert(g.localPosition,c["localPosition"]);
    Convert(g.endPosition,c["endPosition"]);
    Convert(g.direction,c["direction"]);
    break;
  case IKGoal::PosFixed:
    c["posConstraint"] = string("fixed");
    Convert(g.localPosition,c["localPosition"]);
    Convert(g.endPosition,c["endPosition"]);
    break;
  default:
    break;
  }
  switch(g.rotConstraint) {
  case IKGoal::RotNone:
    break;
  case IKGoal::RotTwoAxis:
    c["rotConstraint"] = string("twoaxis");
    Convert(g.localAxis,c["localAxis"]);
    Convert(g.endRotation,c["endRotation"]);
    break;
  case IKGoal::RotAxis:
    c["rotConstraint"] = string("axis");
    Convert(g.localAxis,c["localAxis"]);
    Convert(g.endRotation,c["endRotation"]);
    break;
  case IKGoal::RotFixed:
    c["rotConstraint"] = string("fixed");
    Convert(g.endRotation,c["endRotation"]);
    break;
  }
}

template <> void Convert(const Hold& h,AnyCollection& c)
{
  Convert(h.ikConstraint,c["ik"]);
  c["contacts"].resize(h.contacts.size());
  for(size_t i=0;i<h.contacts.size();i++) {
    Convert(h.contacts[i].x,c["contacts"][(int)i]["x"]);
    Convert(h.contacts[i].n,c["contacts"][(int)i]["n"]);
    c["contacts"][(int)i]["kFriction"] = h.contacts[i].kFriction;
  }
}

template <> void Convert(const Grasp& g,AnyCollection& c)
{
  c["objectIndex"] = g.objectIndex;
  c["robotIndex"] = g.robotIndex;
  c["constraints"].resize(g.constraints.size());
  for(size_t i=0;i<g.constraints.size();i++)
    Convert(g.constraints[i],c["constraints"][(int)i]);
  c["fixedValues"] = g.fixedValues;
  c["fixedDofs"] = g.fixedDofs;
  c["contacts"].resize(g.contacts.size());
  for(size_t i=0;i<g.contacts.size();i++) {
    Convert(g.contacts[i].x,c["contacts"][(int)i]["x"]);
    Convert(g.contacts[i].n,c["contacts"][(int)i]["n"]);
    c["contacts"][(int)i]["kFriction"] = g.contacts[i].kFriction;
  }
  c["contactLinks"] = g.contactLinks;
  c["forces"].resize(g.forces.size());
  for(size_t i=0;i<g.forces.size();i++)
    Convert(g.forces[i],c["forces"][(int)i]);
}

template <> void Convert(const Stance& s,AnyCollection& c)
{
  c["holds"].resize(s.size());
  int k=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++,k++) {
    Convert(i->second,c["holds"][k]);
  }
}
 
template <> bool Convert(const AnyCollection& c,IKGoal& g)
{
  if(!c["link"].as(g.link)) return false;
  if(c.find("destLink") == NULL || !c["destLink"].as(g.destLink))
    g.destLink = -1;
  string s;
  if(c.find("posConstraint") == NULL || !c["posConstraint"].as(s))
    g.SetFreePosition();
  else {
    if(s == "fixed") 
      g.posConstraint = IKGoal::PosFixed;
    else if(s == "planar")
      g.posConstraint = IKGoal::PosPlanar; 
    else if(s=="linear") 
      g.posConstraint = IKGoal::PosLinear;
    else {
      fprintf(stderr,"AnyCollection to IKGoal: Invalid posConstraint type %s\n",s.c_str());
      return false;
    }
    if(s == "fixed" || s == "planar" || s=="linear") {
      if(!Convert(c["endPosition"],g.endPosition)) return false;
      if(!Convert(c["localPosition"],g.localPosition)) return false;
    }
    if(s == "planar" || s=="linear") {
      if(!Convert(c["direction"],g.direction)) return false;
    }
  }
  if(c.find("rotConstraint") == NULL || !c["rotConstraint"].as(s))
    g.SetFreeRotation();
  else {
    if(s == "fixed") 
      g.rotConstraint = IKGoal::RotFixed;
    else if(s == "axis")
      g.rotConstraint = IKGoal::RotAxis;
    else if(s=="twoaxis") 
      g.rotConstraint = IKGoal::RotTwoAxis;
    else {
      fprintf(stderr,"AnyCollection to IKGoal: Invalid rotConstraint type %s\n",s.c_str());
      return false;
    }
    if(s == "fixed" || s == "axis" || s=="twoaxis") {
      if(!Convert(c["endRotation"],g.endRotation)) return false;
    }
    if(s == "axis" || s=="twoaxis") {
      if(!Convert(c["localAxis"],g.localAxis)) return false;
    }
  }
  return true;
}

template <> bool Convert(const AnyCollection& c,Hold& h)
{
  if(!Convert(c["ik"],h.ikConstraint)) return false;
  h.contacts.resize(c["contacts"].size());
  for(size_t i=0;i<h.contacts.size();i++) {
    if(!Convert(c["contacts"][(int)i]["x"],h.contacts[i].x)) return false;
    if(!Convert(c["contacts"][(int)i]["n"],h.contacts[i].n)) return false;
    if(!Convert(c["contacts"][(int)i]["kFriction"],h.contacts[i].kFriction)) return false;
  }
  return true;
}


template <> bool Convert(const AnyCollection& c,Grasp& g)
{
  g.objectIndex = c["objectIndex"];
  g.robotIndex = c["robotIndex"];
  g.constraints.resize(c["constraints"].size());
  for(size_t i=0;i<g.constraints.size();i++)
    if(!Convert(c["constraints"][(int)i],g.constraints[i])) return false;
  if(!c["fixedValues"].asvector(g.fixedValues)) return false;
  if(!c["fixedDofs"].asvector(g.fixedDofs)) return false;
  g.contacts.resize(c["contacts"].size());
  for(size_t i=0;i<g.contacts.size();i++) {
    if(!Convert(c["contacts"][(int)i]["x"],g.contacts[i].x)) return false;
    if(!Convert(c["contacts"][(int)i]["n"],g.contacts[i].n)) return false;
    g.contacts[i].kFriction = c["contacts"][(int)i]["kFriction"];
  }
  if(!c["contactLinks"].asvector(g.contactLinks)) return false;
  g.forces.resize(c["forces"].size());
  for(size_t i=0;i<g.forces.size();i++)
    if(!Convert(c["forces"],g.forces[i])) return false;
  return true;
}

template <> bool Convert(const AnyCollection& c,Stance& s)
{
  vector<shared_ptr<AnyCollection> > holds;
  s.clear();
  c["holds"].enumerate(holds);
  for(size_t i=0;i<holds.size();i++) {
    HoldResource h;
    if(!h.Load(*holds[i])) {
      fprintf(stderr,"Convert(AnyCollection,Stance): Error reading hold %d\n",(int)i);
      return false;
    }
    s.insert(h.hold);
  }
  return true;
}

} //namespace Klampt