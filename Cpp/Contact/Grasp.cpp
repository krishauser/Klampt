#include "Grasp.h"
#include <tinyxml.h>

using namespace Klampt;

Grasp::Grasp()
  :objectIndex(-1),robotIndex(0)
{}

void Grasp::GetHold(Hold& h) const
{
  h.link = constraints[0].link;
  h.ikConstraint = constraints[0];
  h.contacts = contacts;
}

void Grasp::GetStance(Stance& s) const
{
  s.clear();
  for(size_t i=0;i<constraints.size();i++)
    s[constraints[i].link].ikConstraint = constraints[i];
  for(size_t i=0;i<contacts.size();i++) 
    s[contactLinks[i]].contacts.push_back(contacts[i]);
}

void Grasp::GetContactFormation(ContactFormation& s) const
{
  map<int,vector<ContactPoint> > cmap;
  for(size_t i=0;i<contacts.size();i++) 
    cmap[contactLinks[i]].push_back(contacts[i]);
  s.links.resize(cmap.size());
  s.contacts.resize(cmap.size());
  size_t k=0;
  for(map<int,vector<ContactPoint> >::const_iterator i=cmap.begin();i!=cmap.end();i++,k++) {
    s.links[k] = i->first;
    s.contacts[k] = i->second;
  }
}

void Grasp::SetHold(const Hold& h)
{
  constraints.resize(1);
  constraints[0] = h.ikConstraint;
  fixedDofs.resize(0);
  fixedValues.resize(0);
  contacts = h.contacts;
  contactLinks.resize(h.contacts.size());
  fill(contactLinks.begin(),contactLinks.end(),h.ikConstraint.link);
}

void Grasp::SetStance(const Stance& s)
{
  constraints.resize(s.size());
  contacts.resize(0);
  contactLinks.resize(0);
  int k=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++,k++) {
    constraints[k] = i->second.ikConstraint;
    contacts.insert(contacts.end(),i->second.contacts.begin(),i->second.contacts.end());
    contactLinks.resize(contacts.size(),i->first);
  }
  fixedDofs.resize(0);
  fixedValues.resize(0);
}

void Grasp::Add(const Grasp& grasp)
{
  for(size_t i=0;i<constraints.size();i++)
    for(size_t j=0;j<grasp.constraints.size();j++)
      Assert(constraints[i].link != grasp.constraints[j].link);
  for(size_t i=0;i<fixedDofs.size();i++)
    for(size_t j=0;j<grasp.fixedDofs.size();j++)
      if(fixedDofs[i] == grasp.fixedDofs[j])
	Assert(fixedValues[i] == grasp.fixedValues[j]);
  constraints.insert(constraints.end(),grasp.constraints.begin(),grasp.constraints.end());
  fixedDofs.insert(fixedDofs.end(),grasp.fixedDofs.begin(),grasp.fixedDofs.end());
  fixedValues.insert(fixedValues.end(),grasp.fixedValues.begin(),grasp.fixedValues.end());
  
  contacts.insert(contacts.end(),grasp.contacts.begin(),grasp.contacts.end());
  contactLinks.insert(contactLinks.end(),grasp.contactLinks.begin(),grasp.contactLinks.end());
  forces.insert(forces.end(),grasp.forces.begin(),grasp.forces.end());
}

void Grasp::Transform(const RigidTransform& T)
{
  for(size_t i=0;i<constraints.size();i++)
    constraints[i].Transform(T);
  for(size_t i=0;i<contacts.size();i++) {
    contacts[i].x = T*contacts[i].x;
    contacts[i].n = T.R*contacts[i].n;
  }
}

void Grasp::SetFixed(Vector& q) const
{
  for(size_t i=0;i<fixedDofs.size();i++)
    q(fixedDofs[i]) = fixedValues[i];
}


bool Grasp::Load(TiXmlElement* in)
{
  if(0!=strcmp(in->Value(),"Grasp")) return false;
  objectIndex = -1;
  robotIndex = 0;
  constraints.clear();
  fixedDofs.clear();
  fixedValues.clear();
  contacts.clear();
  contactLinks.clear();
  forces.clear();
  if(in->Attribute("fixed")!=NULL) {
    stringstream ss(in->Attribute("fixed"));
    while(ss) {
      int dof;
      Real value;
      ss >> dof >> value;
      if(ss) {
	fixedDofs.push_back(dof);
	fixedValues.push_back(value);
      }
    }
  }
  if(in->QueryIntAttribute("object",&objectIndex)!=TIXML_SUCCESS) {
  }
  if(in->QueryIntAttribute("robot",&robotIndex)!=TIXML_SUCCESS) {
  }
  TiXmlElement* c=in->FirstChildElement();
  while(c != NULL) {
    if(0==strcmp(c->Value(),"IKGoal")) {
      if(!c->Attribute("data")) {
	fprintf(stderr,"IKGoal does not have data attribute\n");
	return false;
      }
      stringstream ss(c->Attribute("data"));
      IKGoal g;
      ss >> g;
      if(ss.bad()) {
	fprintf(stderr,"Error loading IKGoal from data attribute\n");
	return false;
      }
      constraints.push_back(g);
    }
    else if(0==strcmp(c->Value(),"contact")) {
      ContactPoint pt;
      int link;
      bool hasForce = false;
      Vector3 force;
      if(c->QueryIntAttribute("link",&link)!=TIXML_SUCCESS) {
	fprintf(stderr,"Grasp contact does not have link attribute\n");
	return false;
      }
      if(c->QueryValueAttribute("x",&pt.x)!=TIXML_SUCCESS) {
	fprintf(stderr,"Grasp contact does not have x attribute\n");
	return false;
      }
      if(c->QueryValueAttribute("n",&pt.n)!=TIXML_SUCCESS) {
	fprintf(stderr,"Grasp contact does not have n attribute\n");
	return false;
      }
      if(c->QueryValueAttribute("kFriction",&pt.kFriction)!=TIXML_SUCCESS) {
	fprintf(stderr,"Grasp contact does not have kFriction attribute\n");
	return false;
      }
      if(c->QueryValueAttribute("force",&force)!=TIXML_SUCCESS) {
	hasForce=false;
      }
      else hasForce=true;
      contacts.push_back(pt);
      contactLinks.push_back(link);
      if(hasForce)
	forces.push_back(force);
    }
    c = c->NextSiblingElement();
  }
  return true;
}

bool Grasp::Save(TiXmlElement* e)
{
  e->SetValue("Grasp");
  e->SetAttribute("object",objectIndex);
  e->SetAttribute("robot",robotIndex);
  if(!fixedDofs.empty()) {
    stringstream ss;
    for(size_t i=0;i<fixedDofs.size();i++)
      ss<<fixedDofs[i]<<" "<<fixedValues[i]<<"  ";
    e->SetAttribute("fixed",ss.str().c_str());
  }
  for(size_t i=0;i<constraints.size();i++) {
    TiXmlElement* c=new TiXmlElement("IKGoal");
    stringstream ss;
    ss<<constraints[i];
    c->SetAttribute("data",ss.str().c_str());
    e->LinkEndChild(c);
  }
  for(size_t i=0;i<contacts.size();i++) {
    TiXmlElement* c=new TiXmlElement("contact");
    c->SetAttribute("link",contactLinks[i]);
    {
      stringstream ss;
      ss<<contacts[i].x;
      c->SetAttribute("x",ss.str().c_str());
    }
    {
      stringstream ss;
      ss<<contacts[i].n;
      c->SetAttribute("n",ss.str().c_str());
    }
    {
      stringstream ss;
      ss<<contacts[i].kFriction;
      c->SetAttribute("kFriction",ss.str().c_str());
    }
    if(i < forces.size()) {
      stringstream ss;
      ss<<forces[i];
      c->SetAttribute("force",ss.str().c_str());
    }
    e->LinkEndChild(c);
  }
  return true;
}
