#include "Resources.h"
#include <utils/stringutils.h>
#include <utils/fileutils.h>
#include <tinyxml.h>
#include "IO/XmlWorld.h"
#include <sstream>

template class BasicResource<Config>;
template class BasicResource<Vector3>;
template class BasicResource<Matrix3>;
template class BasicResource<Matrix>;
template class BasicResource<RigidTransform>;
template class BasicResource<Hold>;
template class BasicResource<Meshing::TriMesh>;
template <> const char* BasicResource<Config>::className = "Config";
template <> const char* BasicResource<Vector3>::className = "Vector3";
template <> const char* BasicResource<Matrix3>::className = "Matrix3";
template <> const char* BasicResource<Matrix>::className = "Matrix";
template <> const char* BasicResource<RigidTransform>::className = "RigidTransform";
template <> const char* BasicResource<GeometricPrimitive3D>::className = "GeometricPrimitive3D";
template <> const char* BasicResource<IKGoal>::className = "IKGoal";
template <> const char* BasicResource<Hold>::className = "Hold";
template <> const char* BasicResource<Meshing::TriMesh>::className = "TriMesh";


void MakeRobotResourceLibrary(ResourceLibrary& library)
{
  //whitespace needs to be preserved for some items, e.g. multipaths
  TiXmlBase::SetCondenseWhiteSpace(false);

  library.AddLoader<IntArrayResource>("ints");
  library.AddLoader<FloatArrayResource>("floats");
  library.AddLoader<ConfigResource>("config");
  library.AddLoader<MatrixResource>("matrix");
  library.AddLoader<HoldResource>("hold");
  library.AddLoader<StanceResource>("stance");
  library.AddLoader<GraspResource>("grasp");
  library.AddLoader<ConfigsResource>("configs");
  library.AddLoader<PointCloudResource>("pcd");
  library.AddLoader<TriMeshResource>("tri");
  library.AddLoader<RobotResource>("rob");
  library.AddLoader<RigidObjectResource>("obj");
  library.AddLoader<WorldResource>("xml");
  library.AddLoader<LinearPathResource>("path");
  library.AddLoader<MultiPathResource>("xml");
  library.AddLoader<MultiPathResource>("multipath");
  library.AddLoader<GraspResource>("xml");
}


bool ConfigsResource::Load(istream& in)
{
  configs.resize(0);
  Config temp;
  while(in) {
    in >> temp;
    if(in) configs.push_back(temp);
  }
  return true;
}

bool ConfigsResource::Save(ostream& out)
{
  for(size_t i=0;i<configs.size();i++)
    out<<configs[i]<<endl;
  return true;
}

bool PointCloudResource::Load(istream& in)
{
  return pointCloud.LoadPCL(in);
}

bool PointCloudResource::Save(ostream& out)
{
  return pointCloud.SavePCL(out);
}

bool RigidObjectResource::Load(const string& fn)
{
  return object.Load(fn.c_str());
}

bool RigidObjectResource::Save(const string& fn)
{
  string path = GetFilePath(fn);
  if(!path.empty()) {
    if(!FileUtils::IsDirectory(path.c_str()))
      FileUtils::CreateDirectory(path.c_str());
  }
  path += "/";
  path += name + "/" + name;
  if(!object.Save(fn.c_str())) return false;
  return false;
}

bool RobotResource::Load(const string& fn)
{
  return robot.Load(fn.c_str());
}

bool RobotResource::Save(const string& fn)
{
  string path = GetFilePath(fn);
  if(!path.empty()) {
    if(!FileUtils::IsDirectory(path.c_str()))
      FileUtils::CreateDirectory(path.c_str());
  }
  path += "/";
  path += name + "/";
  if(!robot.Save(fn.c_str())) return false;
  if(!robot.SaveGeometry(path.c_str())) return false;
  return false;
}

bool WorldResource::Load(const string& fn)
{
  XmlWorld f;
  if(!f.Load(fn)) return false;
  if(!f.GetWorld(world)) return false;
  return true;
}

bool WorldResource::Save(const string& fn)
{
  return false;
}

bool LinearPathResource::Load(istream& in)
{
  times.clear();
  milestones.clear();
  Real t;
  Vector x;
  while(in) {
    in >> t >> x;
    if(in) {
      times.push_back(t);
      milestones.push_back(x);
    }
  }
  if(in.bad()) {
    return false;
  }
  return true;
}


bool LinearPathResource::Save(ostream& out)
{
  Assert(times.size()==milestones.size());
  for(size_t i=0;i<times.size();i++) {
    out<<times[i]<<"    "<<milestones[i]<<endl;
  }
  return true;
}

bool MultiPathResource::Load(const std::string& fn)
{
  return path.Load(fn);
}

bool MultiPathResource::Save(const std::string& fn)
{
  return path.Save(fn);
}

bool MultiPathResource::Load(TiXmlElement* in)
{
  return path.Load(in);
}

bool MultiPathResource::Save(TiXmlElement* out)
{
  return path.Save(out);
}

void Convert(const IKGoal& g,AnyCollection& c)
{
  c["link"]=g.link;
  if(g.destLink >= 0)
    c["destLink"]=g.destLink;
  switch(g.posConstraint) {
  case IKGoal::PosNone:
    break;
  case IKGoal::PosPlanar:
    c["posConstraint"] = string("planar");
    c["localPosition"] = g.localPosition;
    c["endPosition"] = g.endPosition;
    c["direction"] = g.direction;
    break;
  case IKGoal::PosLinear:
    c["posConstraint"] = string("linear");
    c["localPosition"] = g.localPosition;
    c["endPosition"] = g.endPosition;
    c["direction"] = g.direction;
    break;
  case IKGoal::PosFixed:
    c["posConstraint"] = string("fixed");
    c["localPosition"] = g.localPosition;
    c["endPosition"] = g.endPosition;
    break;
  default:
    break;
  }
  switch(g.rotConstraint) {
  case IKGoal::RotNone:
    break;
  case IKGoal::RotTwoAxis:
    c["posConstraint"] = string("twoaxis");
    c["localAxis"] = g.localAxis;
    c["endRotation"] = g.endRotation;
    break;
  case IKGoal::RotAxis:
    c["posConstraint"] = string("axis");
    c["localAxis"] = g.localAxis;
    c["endRotation"] = g.endRotation;
    break;
  case IKGoal::RotFixed:
    c["posConstraint"] = string("fixed");
    c["endRotation"] = g.endRotation;
    break;
  }

}
void Convert(const Hold& h,AnyCollection& c)
{
  Convert(h.ikConstraint,c["ik"]);
  c["contacts"].resize(h.contacts.size());
  for(size_t i=0;i<h.contacts.size();i++) {
    c["contacts"][(int)i]["x"] = h.contacts[i].x;
    c["contacts"][(int)i]["n"] = h.contacts[i].n;
    c["contacts"][(int)i]["kFriction"] = h.contacts[i].kFriction;
  }
}

void Convert(const Grasp& g,AnyCollection& c)
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
    c["contacts"][(int)i]["x"] = g.contacts[i].x;
    c["contacts"][(int)i]["n"] = g.contacts[i].n;
    c["contacts"][(int)i]["kFriction"] = g.contacts[i].kFriction;
  }
  c["contactLinks"] = g.contactLinks;
  c["forces"].resize(g.forces.size());
  for(size_t i=0;i<g.forces.size();i++)
    c["forces"] = g.forces[i];
}

bool Convert(const AnyCollection& c,IKGoal& g)
{
  if(!c["link"].as(g.link)) return false;
  if(!c["destLink"].as(g.destLink))
    g.destLink = -1;
  string s;
  if(!c["posConstraint"].as(s))
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
      if(!c["endPosition"].as(g.endPosition)) return false;
      if(!c["localPosition"].as(g.localPosition)) return false;
    }
    if(s == "planar" || s=="linear") {
      if(!c["direction"].as(g.direction)) return false;
    }
  }
  if(!c["rotConstraint"].as(s))
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
      if(!c["endRotation"].as(g.endRotation)) return false;
    }
    if(s == "axis" || s=="twoaxis") {
      if(!c["localAxis"].as(g.localAxis)) return false;
    }
  }
  return true;
}

bool Convert(const AnyCollection& c,Hold& h)
{
  if(!Convert(c["ik"],h.ikConstraint)) return false;
  h.contacts.resize(c["contacts"].size());
  for(size_t i=0;i<h.contacts.size();i++) {
    if(!c["contacts"][(int)i]["x"].as(h.contacts[i].x)) return false;
    if(!c["contacts"][(int)i]["n"].as(h.contacts[i].n)) return false;
    if(!c["contacts"][(int)i]["kFriction"].as(h.contacts[i].kFriction)) return false;
  }
  return true;
}


bool Convert(const AnyCollection& c,Grasp& g)
{
  g.objectIndex = c["objectIndex"];
  g.robotIndex = c["robotIndex"];
  g.constraints.resize(c["constraints"].size());
  for(size_t i=0;i<g.constraints.size();i++)
    Convert(c["constraints"][(int)i],g.constraints[i]);
  if(!c["fixedValues"].asvector(g.fixedValues)) return false;
  if(!c["fixedDofs"].asvector(g.fixedDofs)) return false;
  g.contacts.resize(c["contacts"].size());
  for(size_t i=0;i<g.contacts.size();i++) {
    g.contacts[i].x = c["contacts"][(int)i]["x"];
    g.contacts[i].n = c["contacts"][(int)i]["n"];
    g.contacts[i].kFriction = c["contacts"][(int)i]["kFriction"];
  }
  if(!c["contactLinks"].asvector(g.contactLinks)) return false;
  g.forces.resize(c["forces"].size());
  for(size_t i=0;i<g.forces.size();i++)
    g.forces[i] = c["forces"];
  return true;
}


bool IKGoalResource::Load(AnyCollection& c)
{
  return Convert(c,data);
}

bool IKGoalResource::Save(AnyCollection& c)
{
  Convert(data,c);
  return true;
}


bool HoldResource::Load(AnyCollection& c)
{
  return Convert(c,data);
}

bool HoldResource::Save(AnyCollection& c)
{
  Convert(data,c);
  return true;
}

StanceResource::StanceResource(const Stance& val)
  :stance(val)
{}

bool StanceResource::Load(istream& in)
{
  in>>stance;
  return (in);
}

bool StanceResource::Save(ostream& out)
{
  out<<stance;
  return (out);
}

bool StanceResource::Load(TiXmlElement* in)
{
  stance.clear();
  TiXmlElement* c=in->FirstChildElement();
  while(c != NULL) {
    if(0==strcmp(c->Value(),"Hold")) {
      HoldResource h;
      ResourceBase* res=&h;
      if(!res->Load(c)) return false;
      stance[h.data.link] = h.data;
    }
    c = c->NextSiblingElement();
  }
  return true;
}

bool StanceResource::Save(TiXmlElement* out)
{
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    HoldResource h(i->second);
    stringstream ss;
    ss<<"hold_"<<i->first;
    h.name = ss.str();
    ResourceBase* res=&h;
    TiXmlElement* c = new TiXmlElement("Hold");
    if(!res->Save(c)) {
      delete c;
      return false;
    }
    out->LinkEndChild(c);
  }
  return true;
}

bool StanceResource::Load(AnyCollection& c)
{
  vector<SmartPointer<AnyCollection> > holds;
  stance.clear();
  c["holds"].enumerate(holds);
  for(size_t i=0;i<holds.size();i++) {
    HoldResource h;
    if(!h.Load(*holds[i])) {
      fprintf(stderr,"StanceResource: Error reading hold %d\n",i);
      return false;
    }
    stance.insert(h.data);
  }
  return true;
}

bool StanceResource::Save(AnyCollection& c)
{
  c["holds"].resize(stance.size());
  int k=0;
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++,k++) {
    HoldResource h(i->second);
    if(!h.Save(c["holds"][k])) return false;
  }
  return true;
}



GraspResource::GraspResource(const Grasp& val)
  :grasp(val)
{}


bool GraspResource::Load(TiXmlElement* in)
{
  return grasp.Load(in);
}

bool GraspResource::Save(TiXmlElement* out)
{
  return grasp.Save(out);
}

bool GraspResource::Load(AnyCollection& c)
{
  return Convert(c,grasp);
}

bool GraspResource::Save(AnyCollection& c)
{
  Convert(grasp,c);
  return true;
}





ResourcePtr MakeResource(const string& name,const vector<int>& vals)
{
  return new IntArrayResource(vals,name);
}

ResourcePtr MakeResource(const string& name,const vector<double>& vals)
{
  return new FloatArrayResource(vals,name);
}

ResourcePtr MakeResource(const string& name,const Config& q)
{
  return (new ConfigResource(q,name));
}

ResourcePtr MakeResource(const string& name,const vector<Config>& qs)
{
  ConfigsResource* r=new ConfigsResource;
  r->configs = qs;
  r->name = name;
  return (r);
}

ResourcePtr MakeResource(const string& name,const vector<Real>& ts,const vector<Config>& qs)
{
  LinearPathResource* r=new LinearPathResource;
  r->name = name;
  r->times = ts;
  r->milestones = qs;
  return (r);
}

ResourcePtr MakeResource(const string& name,const MultiPath& path)
{
  MultiPathResource* r=new MultiPathResource;
  r->name = name;
  r->path = path;
  return (r);
}

ResourcePtr MakeResource(const string& name,const Vector3& T)
{
  return (new Vector3Resource(T,name));
}

ResourcePtr MakeResource(const string& name,const Matrix3& T)
{
  return (new Matrix3Resource(T,name));
}

ResourcePtr MakeResource(const string& name,const RigidTransform& T)
{
  return (new RigidTransformResource(T,name));
}

ResourcePtr MakeResource(const string& name,const GeometricPrimitive3D& geom)
{
  return (new GeometricPrimitive3DResource(geom,name));
}

ResourcePtr MakeResource(const string& name,const IKGoal& goal)
{
  return (new IKGoalResource(goal,name));
}


ResourcePtr MakeResource(const string& name,const Meshing::TriMesh& mesh)
{
  return (new TriMeshResource(mesh,name));
}


ResourcePtr MakeResource(const string& name,const Hold& hold)
{
  return (new HoldResource(hold,name));
}

ResourcePtr MakeResource(const string& name,const Stance& stance)
{
  StanceResource* res=new StanceResource(stance);
  res->name = name;
  return (res);
}

ResourcePtr MakeResource(const string& name,const Grasp& grasp)
{
  GraspResource* res=new GraspResource(grasp);
  res->name = name;
  return (res);
}

ResourcePtr CastResource(const ResourcePtr& item,const char* type)
{
  if(0==strcmp(item->Type(),type)) return item;
  if(0==strcmp("Grasp",item->Type())) {
    const GraspResource* gr = dynamic_cast<const GraspResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Hold")) {
      Hold h;
      gr->grasp.GetHold(h);
      return new HoldResource(h);
    }
    else if(0==strcmp(type,"Stance")) {
      Stance s;
      gr->grasp.GetStance(s);
      return new StanceResource(s);
    }
    else if(0==strcmp(type,"IKGoal")) {
      if(gr->grasp.constraints.empty()) return NULL;
      return new IKGoalResource(gr->grasp.constraints.front());
    }
  }
  else if(0==strcmp("Stance",item->Type())) {
    const StanceResource* sr = dynamic_cast<const StanceResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Grasp")) {
      Grasp g;
      g.SetStance(sr->stance);
      return MakeResource(sr->name,g);
    }
  }
  else if(0==strcmp("Hold",item->Type())) {
    const HoldResource* hr = dynamic_cast<const HoldResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"IKGoal")) {
      return MakeResource(hr->name,hr->data.ikConstraint);
    }
    else if(0==strcmp(type,"Stance")) {
      Stance s;
      s.insert(hr->data);
      return MakeResource(hr->name,s);
    }
    else if(0==strcmp(type,"Grasp")) {
      Grasp g;
      g.SetHold(hr->data);
      return MakeResource(hr->name,g);
    }
  }
  else if(0==strcmp("IKGoal",item->Type())) {
    const IKGoalResource* gr = dynamic_cast<const IKGoalResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Hold")) {
      Hold h;
      h.link = gr->data.link;
      h.ikConstraint = gr->data;
      return new HoldResource(h);
    }
    else if(0==strcmp(type,"Grasp")) {
      Grasp g;
      g.objectIndex = -1;
      g.robotIndex = -1;
      g.constraints.push_back(gr->data);
      return new GraspResource(g);
    }
    else if(0==strcmp(type,"Stance")) {
      Hold h;
      h.link = gr->data.link;
      h.ikConstraint = gr->data;
      Stance s;
      s.insert(h);
      return new StanceResource(s);
    }
  }
  else if(0==strcmp("LinearPath",item->Type())) {
    const LinearPathResource* pr = dynamic_cast<const LinearPathResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Configs")) {
      return MakeResource(pr->name,pr->milestones);
    }
    else if(0==strcmp(type,"MultiPath")) {
      MultiPathResource* mp = new MultiPathResource;
      mp->name = pr->name;
      mp->path.SetTimedMilestones(pr->times,pr->milestones);
      return mp;
    }
  }
  else if(0==strcmp("Configs",item->Type())) {
    const ConfigsResource* cr = dynamic_cast<const ConfigsResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"LinearPath")) {
      LinearPathResource* p=new LinearPathResource;
      p->name = cr->name;
      p->milestones = cr->configs;
      p->times.resize(p->milestones.size());
      for(size_t i=0;i<p->times.size();i++)
	p->times[i] = i;
      return p;
    }
    else if(0==strcmp(type,"MultiPath")) {
      MultiPathResource* mp = new MultiPathResource;
      mp->name = cr->name;
      mp->path.SetMilestones(cr->configs);
      return mp;
    }
  }
  else if(0==strcmp("MultiPath",item->Type())) {
    const MultiPathResource* pr = dynamic_cast<const MultiPathResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"LinearPath")) {
      LinearPathResource* p=new LinearPathResource;
      p->name = pr->name;
      for(size_t i=0;i<pr->path.sections.size();i++) 
	p->milestones.insert(p->milestones.end(),pr->path.sections[i].milestones.begin(),pr->path.sections[i].milestones.end());
      if(pr->path.HasTiming()) {
	for(size_t i=0;i<pr->path.sections.size();i++) 
	  p->times.insert(p->times.end(),pr->path.sections[i].times.begin(),pr->path.sections[i].times.end());
      }
      else {
	p->times.resize(p->milestones.size());
	for(size_t i=0;i<p->times.size();i++)
	  p->times[i] = i;
      }
      return p;
    }
    else if(0==strcmp(type,"Configs")) {
      ConfigsResource* c = new ConfigsResource;
      c->name = pr->name;
      for(size_t i=0;i<pr->path.sections.size();i++) 
	c->configs.insert(c->configs.end(),pr->path.sections[i].milestones.begin(),pr->path.sections[i].milestones.end());
      return c;
    }
  }
  fprintf(stderr,"CastResource: No conversion from %s to %s\n",item->Type(),type);
  return NULL;
}


vector<ResourcePtr> ExtractResources(const ResourcePtr& item,const char* type)
{
  vector<ResourcePtr> res;
  if(0==strcmp(item->Type(),type)) {
    res.push_back(item);
    return res;
  }
  if(0==strcmp("Hold",item->Type())) {
    const HoldResource* hr = dynamic_cast<const HoldResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"IKGoal")) {
      res.push_back(MakeResource(hr->name,hr->data.ikConstraint));
      return res;
    }
    else if(0==strcmp(type,"vector<double>")) {
      vector<double> contact(7);
      for(size_t i=0;i<hr->data.contacts.size();i++) {
	stringstream ss;
	ss<<hr->name<<".contacts["<<i<<"]";
	hr->data.contacts[i].x.get(contact[0],contact[1],contact[2]);
	hr->data.contacts[i].n.get(contact[3],contact[4],contact[5]);
	contact[6] = hr->data.contacts[i].kFriction;
	res.push_back(MakeResource(ss.str(),contact));
      }
      return res;
    }
  }
  else if(0==strcmp("Grasp",item->Type())) {
    const GraspResource* gr = dynamic_cast<const GraspResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Hold")) {
      Stance s;
      gr->grasp.GetStance(s);
      for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
	stringstream ss;
	ss<<gr->name<<".hold["<<i->first<<"]";
	res.push_back(MakeResource(ss.str(),i->second));
      }
      return res;
    }
    else if(0==strcmp(type,"Stance")) {
      Stance s;
      gr->grasp.GetStance(s);
      res.push_back(MakeResource(gr->name,s));
      return res;
    }
    else if(0==strcmp(type,"IKGoal")) {
      for(size_t i=0;i<gr->grasp.constraints.size();i++) {
	stringstream ss;
	ss<<gr->name<<".constraints["<<i<<"]";
	res.push_back(MakeResource(ss.str(),gr->grasp.constraints[i]));
      }
      return res;
    }
  }
  else if(0==strcmp("Stance",item->Type())) {
    const StanceResource* sr = dynamic_cast<const StanceResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Hold")) {
      for(Stance::const_iterator i=sr->stance.begin();i!=sr->stance.end();i++) {
	stringstream ss;
	ss<<sr->name<<"["<<i->first<<"]";
	res.push_back(MakeResource(ss.str(),i->second));
      }
      return res;
    }
    else if(0==strcmp(type,"IKGoal")) {
      for(Stance::const_iterator i=sr->stance.begin();i!=sr->stance.end();i++) {
	stringstream ss;
	ss<<sr->name<<"["<<i->first<<"]";
	res.push_back(MakeResource(ss.str(),i->second.ikConstraint));
      }
      return res;
    }
    else if(0==strcmp(type,"Grasp")) {
      Grasp g;
      g.SetStance(sr->stance);
      res.push_back(MakeResource(sr->name,g));
      return res;
    }
  }
  else if(0==strcmp("IKGoal",item->Type())) {
    const IKGoalResource* gr = dynamic_cast<const IKGoalResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Hold")) {
      Hold h;
      h.link = gr->data.link;
      h.ikConstraint = gr->data;
      res.push_back(MakeResource(gr->name,h));
      return res;
    }
    else if(0==strcmp(type,"Grasp")) {
      Grasp g;
      g.objectIndex = -1;
      g.robotIndex = -1;
      g.constraints.push_back(gr->data);
      res.push_back(MakeResource(gr->name,g));
      return res;
    }
    else if(0==strcmp(type,"Stance")) {
      Hold h;
      h.link = gr->data.link;
      h.ikConstraint = gr->data;
      Stance s;
      s.insert(h);
      res.push_back(MakeResource(gr->name,s));
      return res;
    }
  }
  else if(0==strcmp("LinearPath",item->Type())) {
    const LinearPathResource* pr = dynamic_cast<const LinearPathResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"Configs")) {
      res.push_back(MakeResource(pr->name,pr->milestones));
      return res;
    }
    else if(0==strcmp(type,"vector<double>")) {
      stringstream ss;
      ss<<pr->name<<".times";
      res.push_back(MakeResource(pr->name,pr->times));
      return res;
    }
    else if(0==strcmp(type,"MultiPath")) {
      MultiPathResource* mp = new MultiPathResource;
      mp->name = pr->name;
      mp->path.SetTimedMilestones(pr->times,pr->milestones);
      res.push_back(mp);
      return res;
    }
    else if(0==strcmp(type,"Config")) {
      for(size_t i=0;i<pr->milestones.size();i++) {
	stringstream ss;
	ss<<pr->name<<".milestones["<<i<<"]";
	res.push_back(MakeResource(ss.str(),pr->milestones[i]));
      }
      return res;
    }
  }
  else if(0==strcmp("Configs",item->Type())) {
    const ConfigsResource* cr = dynamic_cast<const ConfigsResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"LinearPath")) {
      LinearPathResource* p=new LinearPathResource;
      p->milestones = cr->configs;
      p->times.resize(p->milestones.size());
      for(size_t i=0;i<p->times.size();i++)
	p->times[i] = i;
      p->name = item->name;
      res.push_back(p);
      return res;
    }
    else if(0==strcmp(type,"MultiPath")) {
      MultiPathResource* mp = new MultiPathResource;
      mp->path.SetMilestones(cr->configs);
      mp->name = item->name;
      res.push_back(mp);
      return res;
    }
    else if(0==strcmp(type,"Config")) {
      for(size_t i=0;i<cr->configs.size();i++) {
	stringstream ss;
	ss<<item->name<<"["<<i<<"]";
	res.push_back(MakeResource(ss.str(),cr->configs[i]));
      }
      return res;
    }
  }
  else if(0==strcmp("MultiPath",item->Type())) {
    const MultiPathResource* pr = dynamic_cast<const MultiPathResource*>((const ResourceBase*)item);
    if(0==strcmp(type,"LinearPath")) {
      for(size_t i=0;i<pr->path.sections.size();i++) {
	LinearPathResource* p=new LinearPathResource;
	p->milestones = pr->path.sections[i].milestones;
	p->times.resize(p->milestones.size());
	for(size_t j=0;j<p->times.size();j++)
	  p->times[j] = j;
	stringstream ss;
	ss<<item->name<<"["<<i<<"]";
	p->name = ss.str();
	res.push_back(p);
      }
      return res;
    }
    else if(0==strcmp(type,"Configs")) {
      for(size_t i=0;i<pr->path.sections.size();i++) {
	ConfigsResource* c = new ConfigsResource;
	c->configs = pr->path.sections[i].milestones;;
	stringstream ss;
	ss<<item->name<<"["<<i<<"]";
	c->name = ss.str();
	res.push_back(c);
      }
      return res;
    }
    else if(0==strcmp(type,"Config")) {
      for(size_t i=0;i<pr->path.sections.size();i++) {
	for(size_t j=0;j<pr->path.sections[i].milestones.size();j++) {
	  stringstream ss;
	  ss<<item->name<<"["<<i<<"]["<<j<<"]";
	  res.push_back(MakeResource(ss.str(),pr->path.sections[i].milestones[j]));
	}
      }
      return res;
    }
    else if(0==strcmp(type,"Stance")) {
      for(size_t i=0;i<pr->path.sections.size();i++) {
	Stance s;
	pr->path.GetStance(s,i);
	stringstream ss;
	ss<<item->name<<".stance["<<i<<"]";
	res.push_back(MakeResource(ss.str(),s));
      }
      return res;
    }
  }
  fprintf(stderr,"ExtractResource: No elements of type %s in %s\n",type,item->Type());
  return res;
}

