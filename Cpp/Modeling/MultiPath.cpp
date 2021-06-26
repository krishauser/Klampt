#include "MultiPath.h"
//#include "Resources.h"
#include <tinyxml.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/spline/TimeSegmentation.h>
#include <KrisLibrary/spline/Hermite.h>
#include <fstream>

namespace Klampt {

ostream& operator << (ostream& out,const MultiPath& path)
{
  TiXmlElement node("multipath");
  path.Save(&node);
  out<<node<<endl;
  return out;
}


bool MultiPath::Load(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"multipath")) {
    fprintf(stderr,"MultiPath Load XML: node \"%s\" not of multipath type\n",node->Value());
    return false;
  }

  holdSet.clear();
  holdSetNames.clear();
  sections.clear();
  settings.Load(node);
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) {
    if(0==strcmp(e->Value(),"section")) {
      sections.resize(sections.size()+1);
      sections.back().settings.Load(e);
      MultiPath::PathSection& section=sections.back();

      TiXmlElement* c=e->FirstChildElement();
      while(c != NULL) {
	if(0==strcmp(c->Value(),"ikgoal")) {
	  if(!c->GetText()) return false;
	  stringstream ss(c->GetText());
	  section.ikGoals.resize(section.ikGoals.size()+1);
	  ss >> section.ikGoals.back();
	  if(!ss) {
	    fprintf(stderr,"MultiPath Load XML: error loading ikgoal %d of section %d\n",section.ikGoals.size()-1,sections.size()-1);
	    return false;
	  }
	}
	else if(0==strcmp(c->Value(),"hold")) {
	  int index;
	  if(c->QueryValueAttribute("index",&index) == TIXML_SUCCESS) {
	    section.holdIndices.push_back(index);
	  }
	  else if(c->Attribute("name") != NULL) {
	    section.holdNames.push_back(c->Attribute("name"));
	  }
	  else {
	    if(c->GetText()==NULL) {
	      fprintf(stderr,"Hold has no text\n");
	      return false;
	    }
	    stringstream ss(c->GetText());
	    section.holds.resize(section.holds.size()+1);
	    ss>>section.holds.back();
	    if(!ss) {
	      fprintf(stderr,"MultiPath Load XML: unable to read hold %d of section %d\n",section.holds.size()-1,sections.size()-1);
	      return false;
	    }
	  }
	}
	else if(0==strcmp(c->Value(),"milestone")) {
	  section.milestones.resize(section.milestones.size()+1);
	  if(c->Attribute("config")==NULL) {
	    fprintf(stderr,"Need a config attribute in milestone\n");
	    return false;
	  }
	  if(c->QueryValueAttribute("config",&section.milestones.back()) != TIXML_SUCCESS) {
	    fprintf(stderr,"Error loading config attribute in milestone\n");
	    return false;	    
	  }
	  if(c->Attribute("time")!=NULL) {
	    section.times.resize(section.milestones.size());
	    if(c->Attribute("time",&section.times.back()) == NULL) {
	      fprintf(stderr,"Error loading time attribute in milestone\n");
	      return false;	    
	    }
	  }
	  if(c->Attribute("velocity")!=NULL) {
	    section.velocities.resize(section.milestones.size());
	    if(c->QueryValueAttribute("velocity",&section.velocities.back()) != TIXML_SUCCESS) {
	      fprintf(stderr,"Error loading velocity attribute in milestone\n");
	      return false;   
	    }
	  }
	}
	c = c->NextSiblingElement();
      }
    }
    else if(0==strcmp(e->Value(),"hold")) {
      holdSet.resize(holdSet.size()+1);
      if(e->Attribute("name") != NULL) {
	holdSetNames.resize(holdSet.size());
	holdSetNames.back() = e->Attribute("name");
      }
      if(e->GetText()==NULL) {
	fprintf(stderr,"Hold has no text child\n");
	return false;
      }
      stringstream ss(e->GetText());
      ss>>holdSet.back();
      if(!ss) {
	fprintf(stderr,"MultiPath Load XML: unable to read hold %d of hold set\n",holdSet.size()-1);
	return false;
      }
    }
    e = e->NextSiblingElement();
  }
  return true;

}

bool MultiPath::Save(TiXmlElement* node) const
{
  node->SetValue("multipath");
  settings.Save(node);
  for(size_t i=0;i<sections.size();i++) {
    TiXmlElement c("section");
    sections[i].settings.Save(&c);

    //write the section data
    for(size_t j=0;j<sections[i].ikGoals.size();j++) {
      TiXmlElement cc("ikgoal");
      stringstream ss;
      ss<<sections[i].ikGoals[j];
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    for(size_t j=0;j<sections[i].holds.size();j++) {
      TiXmlElement cc("hold");
      stringstream ss;
      ss<<sections[i].holds[j];
      TiXmlText text(ss.str().c_str());
      //text.SetCDATA(true);
      cc.InsertEndChild(text);
      c.InsertEndChild(cc);
    }
    for(size_t j=0;j<sections[i].holdIndices.size();j++) {
      TiXmlElement cc("hold");
      cc.SetAttribute("index",sections[i].holdIndices[j]);
      c.InsertEndChild(cc);
    }
    for(size_t j=0;j<sections[i].holdNames.size();j++) {
      TiXmlElement cc("hold");
      cc.SetAttribute("name",sections[i].holdNames[j].c_str());
      c.InsertEndChild(cc);
    }
    for(size_t j=0;j<sections[i].milestones.size();j++) {
      TiXmlElement cc("milestone");
      if(j<sections[i].times.size())
	cc.SetDoubleAttribute("time",sections[i].times[j]);
      stringstream ss;
      ss<<sections[i].milestones[j];
      cc.SetAttribute("config",ss.str());
      if(j<sections[i].velocities.size()) {
	stringstream ss;
	ss<<sections[i].velocities[j];
	cc.SetAttribute("velocity",ss.str());
      }
      c.InsertEndChild(cc);
    }

    node->InsertEndChild(c);
  }
  for(size_t i=0;i<holdSet.size();i++) {
    TiXmlElement cc("hold");
    if(i < holdSetNames.size() && !holdSetNames[i].empty()) 
      cc.SetAttribute("name",holdSetNames[i].c_str());
    stringstream ss;
    ss<<holdSet[i];
    TiXmlText text(ss.str().c_str());
    //text.SetCDATA(true);
    cc.InsertEndChild(text);
    node->InsertEndChild(cc);
  }
  return true;
}

bool MultiPath::IsValid() const
{
  for(size_t i=0;i<sections.size();i++) {
    for(size_t j=0;j<sections[i].holdIndices.size();j++)
      if(sections[i].holdIndices[j] < 0 || sections[i].holdIndices[j] >= (int)holdSet.size()) {
	fprintf(stderr,"Invalid hold reference %d on path section %d\n",sections[i].holdIndices[j],i);
	return false;
      }
    for(size_t j=0;j<sections[i].holdNames.size();j++) {
      int index=-1;
      for(size_t k=0;k<holdSetNames.size();k++) {
	if(sections[i].holdNames[j] == holdSetNames[k]) {
	  index = (int)k;
	  break;
	}
      }
      if(index < 0) {
	fprintf(stderr,"Invalid hold name %s on path section %d\n",sections[i].holdNames[j].c_str(),i);
	return false;
      }
    }

    if(sections[i].milestones.empty()) {
      fprintf(stderr,"Empty milestone list on path section %d\n",i);
      return false;
    }
    if(sections[i].milestones.size()==1) {
      fprintf(stderr,"Singleton milestone on path section %d\n",i);
      return false;
    }
    if(!sections[i].times.empty() && sections[i].times.size() != sections[i].milestones.size()) {
      fprintf(stderr,"Invalid number of times on path section %d\n",i);
      return false;
    }
    if(!sections[i].velocities.empty() && sections[i].velocities.size() != sections[i].milestones.size()) {
      fprintf(stderr,"Invalid number of velocities on path section %d\n",i);
      return false;
    }
  }
  return true;
}

bool MultiPath::IsContinuous(Real tol) const
{
  for(size_t i=0;i+1<sections.size();i++) {
    if(!sections[i].milestones.back().isEqual( sections[i+1].milestones.front(),tol)) {
      fprintf(stderr,"MultiPath: Discontinuity at section %d to %d\n",i,i+1);
      return false;
    }
    if(!sections[i].times.empty() && !sections[i+1].times.empty()) {
      if(!FuzzyEquals(sections[i].times.back(),sections[i+1].times.front(),tol)) return false;
    }
    if(!sections[i].velocities.empty()) {
      if(!sections[i+1].velocities.empty()) {
	if(!sections[i].velocities.back().isEqual(sections[i+1].velocities.front(),tol)) return false;
      }
      else {
	if(!sections[i].velocities.back().isZero(tol)) return false;
      }
    }
    else if(!sections[i+1].velocities.empty()) {
      if(!sections[i+1].velocities.front().isZero(tol)) return false;
    }
  }
  return true;
}

bool MultiPath::Load(const string& fn)
{
  //whitespace needs to be preserved for holds
  TiXmlBase::SetCondenseWhiteSpace(false);

  TiXmlDocument doc;
  if(!doc.LoadFile(fn.c_str())) {
    return false;
  }
  TiXmlElement* e=doc.RootElement();
  return Load(e);
}

bool MultiPath::Save(const string& fn) const
{
  TiXmlDocument doc;
  TiXmlElement node("multipath");
  doc.InsertEndChild(node);
  Save(doc.RootElement());
  return doc.SaveFile(fn.c_str());
}

bool MultiPath::HasTiming(int s) const
{
  if(s < 0 || s >= (int)sections.size()) return false;
  return !sections[s].times.empty();
}

bool MultiPath::HasVelocity(int s) const
{
  if(s < 0 || s >= (int)sections.size()) return false;
  return !sections[s].velocities.empty();
}

bool MultiPath::HasConstraints(int s) const
{
  if(s < 0 || s >= (int)sections.size()) return false;
  return !sections[s].ikGoals.empty() || HasContacts(s);
}

bool MultiPath::HasContacts(int s) const
{
  if(s < 0 || s >= (int)sections.size()) return false;
  return (sections[s].holds.size()+sections[s].holdNames.size()+sections[s].holdIndices.size() > 0);
}

void MultiPath::GetMilestones(vector<Vector>& milestones,int s) const
{
  milestones = sections[s].milestones;
}

void MultiPath::GetTimedMilestones(vector<Real>& times,vector<Vector>& milestones,int s) const
{
  milestones = sections[s].milestones;
  if(HasTiming(s))
    times = sections[s].times;
  else {
    times.resize(milestones.size());
    for(size_t i=0;i<milestones.size();i++)
      times[i] = Real(i)/Real(milestones.size());
  }
}

bool MultiPath::GetDynamicPath(ParabolicRamp::DynamicPath& path,int s) const
{
  if(HasTiming(s)) {
    //need to do SolveMinTimes
    path.ramps.resize(0);
    ParabolicRamp::DynamicPath subpath;
    swap(subpath.xMin,path.xMin);
    swap(subpath.xMax,path.xMax);
    swap(subpath.velMax,path.velMax);
    swap(subpath.accMax,path.accMax);
    ParabolicRamp::Vector zero(sections[s].milestones[0].n,0.0);
    bool res=true;
    for(size_t i=0;i+1<sections[s].milestones.size();i++) {
      Real dt=sections[s].times[i+1]-sections[s].times[i];
      if(HasVelocity(s)) {
	if(!subpath.SolveMinAccel(sections[s].milestones[i],sections[s].velocities[i],
				  sections[s].milestones[i+1],sections[s].velocities[i+1],dt)) {
	  res=false;
	  break;
	}
      }
      else {
	if(!subpath.SolveMinAccel(sections[s].milestones[i],zero,
				  sections[s].milestones[i+1],zero,dt)) {
	  res=false;
	  break;
	}
      }
      path.Concat(subpath);
    }
    swap(subpath.xMin,path.xMin);
    swap(subpath.xMax,path.xMax);
    swap(subpath.velMax,path.velMax);
    swap(subpath.accMax,path.accMax);
    return res;
  }
  else if(HasVelocity(s)) {
    vector<ParabolicRamp::Vector> milestones(sections[s].milestones.size()),velocities(sections[s].milestones.size());
    copy(sections[s].milestones.begin(),sections[s].milestones.end(),milestones.begin());
    copy(sections[s].velocities.begin(),sections[s].velocities.end(),velocities.begin());
    return path.SetMilestones(milestones,velocities);
  }
  else {
    vector<ParabolicRamp::Vector> milestones(sections[s].milestones.size());
    copy(sections[s].milestones.begin(),sections[s].milestones.end(),milestones.begin());
    return path.SetMilestones(milestones);
  }
}

void MultiPath::SetMilestones(const vector<Vector>& milestones,int s)
{
  if(s == (int)sections.size()) sections.resize(s+1);
  Assert(s < (int)sections.size());
  sections[s].milestones = milestones;
  sections[s].times.clear();
  sections[s].velocities.clear();
}

void MultiPath::SetTimedMilestones(const vector<Real>& times,const vector<Vector>& milestones,int s)
{
  if(s == (int)sections.size()) sections.resize(s+1);
  Assert(s < (int)sections.size());
  sections[s].milestones = milestones;
  sections[s].times = times;
  sections[s].velocities.clear();
}

void MultiPath::SetDynamicPath(const ParabolicRamp::DynamicPath& path,int s)
{
  if(s == (int)sections.size()) sections.resize(s+1);
  Assert(s < (int)sections.size());
  sections[s].times.clear();
  sections[s].milestones.resize(path.ramps.size()+1);
  sections[s].velocities.resize(path.ramps.size()+1);
  sections[s].milestones[0] = path.ramps[0].x0;
  sections[s].velocities[0] = path.ramps[0].dx0;
  for(size_t i=0;i<path.ramps.size();i++) {
    sections[s].milestones[i+1] = path.ramps[i].x1;
    sections[s].velocities[i+1] = path.ramps[i].dx1;
  }
}

Real MultiPath::Duration() const
{
  if(!HasTiming()) return 1.0;
  if(sections.empty()) return 0.0;
  return sections.back().times.back()-sections.front().times.front();
}

Real MultiPath::StartTime() const
{
  if(!HasTiming()) return 0;
  if(sections.empty()) return 0.0;
  return sections.front().times.front();
}

Real MultiPath::EndTime() const
{
  if(!HasTiming()) return 1;
  if(sections.empty()) return 0.0;
  return sections.back().times.back();
}

void MultiPath::SetDuration(Real duration,bool uniformSectionTime)
{
  if(!HasTiming()) {
    //assign section durations
    vector<Real> sectionTiming(sections.size()+1);
    sectionTiming[0] = 0;
    if(uniformSectionTime) {
      for(size_t i=0;i<sections.size();i++) 
	sectionTiming[i+1] = Real(i+1)*duration/sections.size();
    }
    else {
      for(size_t i=0;i<sections.size();i++) 
	sectionTiming[i+1] = sectionTiming[i] + (sections[i].milestones.size()-1);
      for(size_t i=0;i<sections.size();i++) 
	sectionTiming[i+1] = sectionTiming[i+1]*duration/sectionTiming.back();
    }
    //set linear timing within each section
    for(size_t i=0;i<sections.size();i++) {
      Real tstart=sectionTiming[i],tend=sectionTiming[i+1];
      sections[i].times.resize(sections[i].milestones.size());
      for(size_t j=0;j<sections[i].milestones.size();j++) {
	sections[i].times[j] = tstart + Real(j)/Real(sections[i].milestones.size()-1)*(tend-tstart);
      }
    }
  }
  else {
    Real scale = duration / Duration();
    Real t0 = StartTime();
    for(size_t i=0;i<sections.size();i++) {
      Assert(!sections[i].times.empty());
      for(size_t j=0;j<sections[i].times.size();j++)
	sections[i].times[j] *= scale;
    }
  }
}

void MultiPath::SetSmoothTiming(Real duration,bool uniformSectionTime)
{
  //assign section durations
  vector<Real> sectionTiming(sections.size()+1);
  sectionTiming[0] = 0;
  if(uniformSectionTime) {
    for(size_t i=0;i<sections.size();i++) 
      sectionTiming[i+1] = Real(i+1)*duration/sections.size();
  }
  else {
    for(size_t i=0;i<sections.size();i++) 
      sectionTiming[i+1] = sectionTiming[i] + (sections[i].milestones.size()-1);
    for(size_t i=0;i<sections.size();i++) 
      sectionTiming[i+1] = sectionTiming[i+1]*duration/sectionTiming.back();
  }
  
  for(size_t i=0;i<sections.size();i++) {
    sections[i].times.resize(sections[i].milestones.size());
    //accelerate and decelerate to use up time T
    //s(t) is such that x'(t(s)) = x'(s) s'(t) is linear inc. in the
    // first half, linearly dec. in the second.
    //x'(t(s)) = c*t(s) = s'(t) => s'(t) = ct =>
    //s(t) = d+1/2ct^2
    //t(s) = sqrt(2s/c)
    //we want to find t(s+1)-t(s)
    Real tstart=sectionTiming[i];
    Real T = sectionTiming[i+1]-sectionTiming[i];
    size_t n=sections[i].milestones.size()-1;
    for(size_t j=0;j<sections[i].milestones.size();j++) {
      if(j*2 < n)
	sections[i].times[j] = tstart+T/2*Sqrt(2*Real(j)/n);
      else
	sections[i].times[j] = tstart+T-T/2*Sqrt(2*Real(n-j)/n);
    }
  }
}

void MultiPath::Concat(const MultiPath& suffix,bool relative)
{
  //TODO: merge the settings
  if(settings.empty())
    settings = suffix.settings;

  if(sections.empty())
    sections = suffix.sections;
  else {
    Real tofs = 0;
    if(relative) tofs = EndTime();
    if(HasTiming()) Assert(suffix.HasTiming());
    size_t i=sections.size();
    sections.insert(sections.end(),suffix.sections.begin(),suffix.sections.end());
    if(tofs != 0) {
      for(;i<sections.size();i++)
	for(size_t j=0;j<sections[i].times.size();j++)
	  sections[i].times[j] += tofs;
    }
  }
}

void MultiPath::SetIKProblem(const vector<IKGoal>& goals,int s)
{
  if(s == (int)sections.size()) sections.resize(s+1);
  Assert(s < (int)sections.size());
  sections[s].ikGoals = goals;
  sections[s].holds.clear();
  sections[s].holdIndices.clear();
  sections[s].holdNames.clear();
}

void MultiPath::GetIKProblem(vector<IKGoal>& goals,int s) const
{
  goals=sections[s].ikGoals;
  for(size_t i=0;i<sections[s].holds.size();i++) 
    goals.push_back(sections[s].holds[i].ikConstraint);
  for(size_t i=0;i<sections[s].holdIndices.size();i++) 
    goals.push_back(holdSet[sections[s].holdIndices[i]].ikConstraint);
  for(size_t i=0;i<sections[s].holdNames.size();i++) {
    Hold h;
    bool res=GetHold(sections[s].holdNames[i],h);
    Assert(res==true);
    goals.push_back(h.ikConstraint);
  }
}

void MultiPath::SetStance(const Stance& stance,int s)
{
  if(s == (int)sections.size()) sections.resize(s+1);
  Assert(s < (int)sections.size());
  sections[s].ikGoals.resize(0);
  sections[s].holds.resize(0);
  sections[s].holdIndices.resize(0);
  sections[s].holdNames.resize(0);
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++)
    sections[s].holds.push_back(i->second);
}

void MultiPath::GetStance(Stance& stance,int s) const
{
  stance.clear();
  for(size_t i=0;i<sections[s].holds.size();i++) {
    if(stance.contains(sections[s].holds[i].link)) {
      FatalError("TODO: merge holds");
    }
    stance.insert(sections[s].holds[i]);
  }
  for(size_t i=0;i<sections[s].holdIndices.size();i++) {
    if(stance.contains(holdSet[sections[s].holdIndices[i]].link)) {
      FatalError("TODO: merge holds");
    }
    stance.insert(holdSet[sections[s].holdIndices[i]]);
  }
  for(size_t i=0;i<sections[s].holdNames.size();i++) {
    Hold h;
    GetHold(sections[s].holdNames[i],h);
    if(stance.contains(h.link)) {
      FatalError("TODO: merge holds");
    }
    stance.insert(h);
  }
  for(size_t i=0;i<sections[s].ikGoals.size();i++) {
    if(stance.contains(sections[s].ikGoals[i].link)) {
      FatalError("TODO: merge holds");
    }
    Hold h;
    h.link = sections[s].ikGoals[i].link;
    h.ikConstraint = sections[s].ikGoals[i];
    stance.insert(h);
  }
}

void MultiPath::SetHold(const string& str,const Hold& h)
{
  holdSet.push_back(h);
  holdSetNames.resize(holdSet.size());
  holdSetNames.back()=str;
}

bool MultiPath::GetHold(const string& str,Hold& h) const
{
  for(size_t i=0;i<holdSetNames.size();i++)
    if(str == holdSetNames[i]) {
      h = holdSet[i];
      return true;
    }
  return false;
}

int MultiPath::TimeToSection(Real time) const
{
  if(!HasTiming()) {
    //untimed
    for(size_t i=0;i<sections.size();i++)
      Assert(sections[i].times.empty());
    if(time < 0) return -1;
    else if(time > 1) return sections.size();
    else if(time >= 1) return (int)sections.size()-1;
    return (int)Floor(time*sections.size());
  }
  //timed
  for(size_t i=0;i<sections.size();i++)
    Assert(!sections[i].times.empty());
  if(time < sections[0].times[0]) return -1;
  if(time >= sections.back().times.back()) return sections.size();
  for(size_t i=0;i<sections.size();i++) {
    if(time <= sections[i].times.back()) return(int)i;
  }
  AssertNotReached();
  return 0;
}

int MultiPath::Evaluate(Real time,GeneralizedCubicBezierCurve& curve,Real& duration,Real& param,InterpPolicy policy) const
{
  int seg = TimeToSection(time);
  if(seg < 0) { 
    curve.x0 = curve.x1 = curve.x2 = curve.x3 = sections[0].milestones[0]; 
    duration = param = 0;
    return -1; 
  }
  else if(seg == (int)sections.size()) { 
    curve.x0 = curve.x1 = curve.x2 = curve.x3 = sections.back().milestones.back(); 
    duration = param = 0;
    return seg; 
  }
  if(sections[seg].times.empty()) {
    //untimed
    Real u = time*sections.size() - seg;
    int config = (int)Floor(u*(sections[seg].milestones.size()-1));
    duration = 1.0/(sections.size()*(sections[seg].milestones.size()-1));
    param = u*(sections[seg].milestones.size()-1) - config;
    if(config+1==(int)sections[seg].milestones.size()) {
      config--;
      param = 1;
    }
    Assert(config >= 0);
    Assert(config+1 < (int)sections[seg].milestones.size());

    curve.x0=sections[seg].milestones[config];
    curve.x3=sections[seg].milestones[config+1];
    if(policy == InterpLinear)
      curve.SetSmoothTangents(NULL,NULL);
    else if(!sections[seg].velocities.empty()) {
      Assert(config+1 < (int)sections[seg].velocities.size());
      curve.SetNaturalTangents(sections[seg].velocities[config],sections[seg].velocities[config+1]);
    }
    else {
      const Vector *prev=NULL,*next=NULL;
      if(config > 0) 
	prev = &sections[seg].milestones[config-1];
      if(config+2<(int)sections[seg].milestones.size()) 
	next = &sections[seg].milestones[config+2];
      curve.SetSmoothTangents(prev,next);
    }
  }
  else {
    int config=Spline::TimeSegmentation::Map(sections[seg].times,time,param);
    Assert(config >= 0);
    Assert(config+1 < (int)sections[seg].milestones.size());
    curve.x0=sections[seg].milestones[config];
    curve.x3=sections[seg].milestones[config+1];
    Real t1=sections[seg].times[config];
    Real t2=sections[seg].times[config+1];
    duration = t2-t1;
    if(policy == InterpLinear)
      curve.SetSmoothTangents(NULL,NULL);
    else if(!sections[seg].velocities.empty()) {
      //use hermite interpolation
      Assert(config+1 < (int)sections[seg].velocities.size());
      const Vector& v1=sections[seg].velocities[config];
      const Vector& v2=sections[seg].velocities[config+1];
      curve.SetNaturalTangents(v1*(t2-t1),v2*(t2-t1));
    }
    else {
      const Vector *prev=NULL,*next=NULL;
      if(config > 0) 
	prev = &sections[seg].milestones[config-1];
      if(config+2<(int)sections[seg].milestones.size()) 
	next = &sections[seg].milestones[config+2];

      Real scalep = (t2-t1)/(t2-sections[seg].times[config-1]);
      Real scalen = (t2-t1)/(sections[seg].times[config+2]-t1);
      curve.SetSmoothTangents(prev,next);
      //TODO: scale tangents by timing scale
    }
  }
  return seg;
}

int MultiPath::Evaluate(Real time,Vector& q,InterpPolicy policy) const
{
  GeneralizedCubicBezierCurve curve;
  Real duration,param;
  int seg=Evaluate(time,curve,duration,param,policy);
  curve.Eval(param,q);
  return seg;
}

int MultiPath::Evaluate(Real time,Vector& q,Vector& v,InterpPolicy policy) const
{
  GeneralizedCubicBezierCurve curve;
  Real duration,param;
  int seg=Evaluate(time,curve,duration,param,policy);
  curve.Eval(param,q);
  curve.Deriv(param,v);
  v /= duration;
  return seg;
}

} //namespace Klampt