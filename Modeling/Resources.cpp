#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Resources.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/meshing/IO.h>
#include <tinyxml.h>
#include "IO/XmlWorld.h"
#include "IO/JSON.h"
#include <sstream>

template <> const char* BasicResource<Config>::className = "Config";
template <> const char* BasicResource<Vector3>::className = "Vector3";
template <> const char* BasicResource<Matrix3>::className = "Matrix3";
template <> const char* BasicResource<Matrix>::className = "Matrix";
template <> const char* BasicResource<RigidTransform>::className = "RigidTransform";
template <> const char* BasicResource<Meshing::TriMesh>::className = "TriMesh";
template <> const char* BasicResource<GeometricPrimitive3D>::className = "GeometricPrimitive3D";
template <> const char* BasicResource<Hold>::className = "Hold";
template <> const char* BasicResource<Camera::Viewport>::className = "Viewport";
template class BasicResource<Config>;
template class BasicResource<Vector3>;
template class BasicResource<Matrix3>;
template class BasicResource<Matrix>;
template class BasicResource<RigidTransform>;
template class BasicResource<Hold>;


void MakeRobotResourceLibrary(ResourceLibrary& library)
{
  //whitespace needs to be preserved for some items, e.g. multipaths
  TiXmlBase::SetCondenseWhiteSpace(false);

  library.AddType<Vector3Resource>();
  library.AddType<Matrix3Resource>();
  library.AddType<RigidTransformResource>();
  library.AddType<GeometricPrimitive3DResource>();
  library.AddType<IKGoalResource>();
  library.AddType<ViewportResource>();
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
  if(Meshing::CanLoadTriMeshExt("obj"))
    library.AddLoader<TriMeshResource>("obj");
  if(Meshing::CanLoadTriMeshExt("off"))
    library.AddLoader<TriMeshResource>("off");
  if(Meshing::CanLoadTriMeshExt("stl"))
    library.AddLoader<TriMeshResource>("stl");
  if(Meshing::CanLoadTriMeshExt("dae"))
    library.AddLoader<TriMeshResource>("dae");
  if(Meshing::CanLoadTriMeshExt("ply"))
    library.AddLoader<TriMeshResource>("ply");
  library.AddLoader<GeometricPrimitive3DResource>("geom");
  library.AddLoader<RobotResource>("rob");
  library.AddLoader<RigidObjectResource>("obj");
  library.AddLoader<WorldResource>("xml");
  library.AddLoader<LinearPathResource>("path");
  library.AddLoader<MultiPathResource>("xml");
  library.AddLoader<MultiPathResource>("multipath");
  library.AddLoader<GraspResource>("xml");
}


bool ConfigsResource::Save(AnyCollection& c) 
{ 
  c["type"] = string("Configs");
  Convert(configs,c["configs"]);
  return true;
}

bool ConfigsResource::Load(AnyCollection& c) 
{
  return Convert(c["configs"],configs);
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

ResourceBase* ConfigsResource::Copy()
{
  ConfigsResource* res = new ConfigsResource;
  res->configs = configs;
  return res;
}


vector<string> ConfigsResource::CastTypes() const
{
  vector<string> res(2);
  res[0]="LinearPath";
  res[1]="MultiPath";
  return res;
}

vector<string> ConfigsResource::SubTypes() const
{
  return vector<string> (1,"Config");
}

ResourcePtr ConfigsResource::Cast(const char* type)
{
  if(0==strcmp(type,"LinearPath")) {
    LinearPathResource* p=new LinearPathResource;
    p->name = name;
    p->milestones = configs;
    p->times.resize(p->milestones.size());
    for(size_t i=0;i<p->times.size();i++)
      p->times[i] = i;
    return p;
  }
  else if(0==strcmp(type,"MultiPath")) {
    MultiPathResource* mp = new MultiPathResource;
    mp->name = name;
    mp->path.SetMilestones(configs);
    return mp;
  }
  return NULL;
}

bool ConfigsResource::Extract(const char* subtype,vector<ResourcePtr>& res)
{
  if(0==strcmp(subtype,"LinearPath")) {
    LinearPathResource* p=new LinearPathResource;
    p->milestones = this->configs;
    p->times.resize(p->milestones.size());
    for(size_t i=0;i<p->times.size();i++)
      p->times[i] = i;
    p->name = this->name;
    res.push_back(p);
    return true;
  }
  else if(0==strcmp(subtype,"MultiPath")) {
    MultiPathResource* mp = new MultiPathResource;
    mp->path.SetMilestones(this->configs);
    mp->name = this->name;
    res.push_back(mp);
    return true;
  }
  else if(0==strcmp(subtype,"Config")) {
    Unpack(res);
    return true;
  }
  return false;
}

bool ConfigsResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  configs.resize(0);
  for(size_t i=0;i<lib.size();i++) {
    ConfigResource* config = dynamic_cast<ConfigResource*>((ResourceBase*)lib[i]);
    if(!config) {
      if(errorMessage) {
	stringstream ss;
	ss<<"Configs resource is an array of Config resources, got "<<lib[i]->Type()<<" instead";
	*errorMessage = ss.str();
      }
      return false;
    }
    configs.push_back(config->data);
  }
  return true;
}

bool ConfigsResource::Unpack(vector<ResourcePtr>& res,bool* incomplete)
{
  res.resize(0);
  for(size_t i=0;i<this->configs.size();i++) {
    stringstream ss;
    ss<<"config["<<i<<"]";
    res.push_back(MakeResource(ss.str(),this->configs[i]));
  }
  return true;
}


bool TriMeshResource::Load(const std::string& fn)
{
  bool res=Meshing::Import(fn.c_str(),data);
  if(!res) {
    LOG4CXX_INFO(KrisLibrary::logger(),"TriMeshResource::Load:  Unable to load mesh from file "<<fn.c_str());
  }
  return res;
}

bool TriMeshResource::Save(const std::string& fn)
{
  return Meshing::Export(fn.c_str(),data);
}

ResourceBase* TriMeshResource::Copy()
{
  TriMeshResource* res = new TriMeshResource;
  res->name = name;
  res->data = data;
  return res;
}

bool PointCloudResource::Load(istream& in)
{
  return pointCloud.LoadPCL(in);
}

bool PointCloudResource::Save(ostream& out)
{
  return pointCloud.SavePCL(out);
}

ResourceBase* PointCloudResource::Copy()
{
  PointCloudResource* res = new PointCloudResource;
  res->name = name;
  res->pointCloud = pointCloud;
  return res;
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
      FileUtils::MakeDirectory(path.c_str());
  }
  path += "/";
  path += name + "/" + name;
  if(!object.Save(fn.c_str())) return false;
  return false;
}

ResourceBase* RigidObjectResource::Copy()
{
  RigidObjectResource* res=new RigidObjectResource;
  res->object = object;
  return res;
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
      FileUtils::MakeDirectory(path.c_str());
  }
  path += "/";
  path += name + "/";
  if(!robot.Save(fn.c_str())) return false;
  if(!robot.SaveGeometry(path.c_str())) return false;
  return false;
}

ResourceBase* RobotResource::Copy()
{
  RobotResource* res = new RobotResource;
  res->robot = robot;
  return res;
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
    LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: saving worlds\n");
  return false;
}

ResourceBase* WorldResource::Copy()
{
  WorldResource* res = new WorldResource;
  res->world = world;
  return res;
}

vector<string> WorldResource::SubTypes() const
{
  vector<string> res(3);
  res[0] = "Robot";
  res[1] = "RigidObject";
  res[2] = "ResourceLibrary";
  return res;
}

bool WorldResource::Extract(const char* subtype,vector<ResourcePtr>& subobjects)
{
  subobjects.resize(0);
  if(0==strcmp(subtype,"Robot")) {
    for(size_t i=0;i<world.robots.size();i++) {
      RobotResource* rr = new RobotResource;
      rr->name = world.robots[i]->name;
      rr->robot = *world.robots[i];
      subobjects.push_back(rr);
    }
    return true;
  }
  else if(0==strcmp(subtype,"RigidObject")) {
    for(size_t i=0;i<world.rigidObjects.size();i++) {
      RigidObjectResource* rr = new RigidObjectResource;
      rr->name = world.rigidObjects[i]->name;
      rr->object = *world.rigidObjects[i];
      subobjects.push_back(rr);
    }
    return true;
  }
  else if(0==strcmp(subtype,"ResourceLibrary")) {
    for(size_t i=0;i<world.terrains.size();i++) {
      ResourceLibraryResource* r = new ResourceLibraryResource;
      r->name = world.terrains[i]->name;
      r->library.Add(MakeResource("geometry",*world.terrains[i]->geometry));
      r->library.Add(MakeResource("kFriction",world.terrains[i]->kFriction));
      subobjects.push_back(r);
    }
    return true;
  }
  return false;
}

bool WorldResource::Pack(vector<ResourcePtr>& subobjects,string* errorMessage)
{
  return CompoundResourceBase::Pack(subobjects,errorMessage);
}

bool WorldResource::Unpack(vector<ResourcePtr>& subobjects,bool* incomplete)
{
  subobjects.resize(0);
  for(size_t i=0;i<world.robots.size();i++) {
    RobotResource* rr = new RobotResource;
    rr->name = world.robots[i]->name;
    rr->robot = *world.robots[i];
    subobjects.push_back(rr);
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    RigidObjectResource* rr = new RigidObjectResource;
    rr->name = world.rigidObjects[i]->name;
    rr->object = *world.rigidObjects[i];
    subobjects.push_back(rr);
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    ResourceLibraryResource* r = new ResourceLibraryResource;
    r->name = world.terrains[i]->name;
    r->library.Add(MakeResource("geometry",*world.terrains[i]->geometry));
    r->library.Add(MakeResource("kFriction",world.terrains[i]->kFriction));
    subobjects.push_back(r);
  }
  if(incomplete) *incomplete = true;
  return true;
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

bool LinearPathResource::Save(AnyCollection& c)
{
  c["type"] = string("LinearPath");
  Convert(times,c["times"]); 
  Convert(milestones,c["milestones"]);
  return true; 
}

bool LinearPathResource::Load(AnyCollection& c) 
{
  return Convert(c["times"],times) && Convert(c["milestones"],milestones); 
}

ResourceBase* LinearPathResource::Copy()
{
  LinearPathResource* res= new LinearPathResource;
  res->times = times;
  res->milestones = milestones;
  return res;
}

vector<string> LinearPathResource::CastTypes() const
{
  vector<string> res(2);
  res[0] = "Configs";
  res[1] = "MultiPath";
  return res;
}

vector<string> LinearPathResource::SubTypes() const
{
  vector<string> res(2);
  res[0] = "vector<double>";
  res[1] = "Configs";
  return res;
}

vector<string> LinearPathResource::ExtractTypes() const
{
  vector<string> res(3);
  res[0] = "Configs";
  res[1] = "Config";
  res[2] = "vector<double>";
  return res;
}

ResourcePtr LinearPathResource::Cast(const char* subtype)
{
  if(0==strcmp(subtype,"Configs")) {
    return MakeResource(name,milestones);
  }
  else if(0==strcmp(subtype,"MultiPath")) {
    MultiPathResource* mp = new MultiPathResource;
    mp->name = name;
    mp->path.SetTimedMilestones(times,milestones);
    return mp;
  }
  return NULL;
}

bool LinearPathResource::Extract(const char* subtype,vector<ResourcePtr>& res) 
{
  if(0==strcmp(subtype,"Configs")) {
    res.push_back(MakeResource("milestones",this->milestones));
    return true;
  }
  else if(0==strcmp(subtype,"vector<double>")) {
    res.push_back(MakeResource("times",this->times));
    return true;
  }
  else if(0==strcmp(subtype,"Config")) {
    for(size_t i=0;i<this->milestones.size();i++) {
      stringstream ss;
      ss<<"milestone["<<i<<"]";
      res.push_back(MakeResource(ss.str(),this->milestones[i]));
    }
    return true;
  }
  return false;
}

bool LinearPathResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  vector<FloatArrayResource*> times = ResourcesByType<FloatArrayResource>(lib);
  vector<ConfigResource*> configs = ResourcesByType<ConfigResource>(lib);
  vector<ConfigsResource*> configss = ResourcesByType<ConfigsResource>(lib);
  if(times.size() != 1) {
    if(errorMessage) *errorMessage = "Need exactly 1 time vector in a linear path";
    return false;
  }
  if(configs.empty()) {
    if(configss.size() != 1) {
      if(errorMessage) *errorMessage = "Trying to pack more than 1 config set into a linear path";
      return false;
    }
    this->times = times[0]->data;
    this->milestones = configss[0]->configs;
    return true;
  }
  else {
    if(!configss.empty()) {
      if(errorMessage) *errorMessage = "Cannot combine config's and config sets into a linear path";
      return false;
    }
    if(times[0]->data.size() != configs.size()) {
      if(errorMessage) {
	stringstream ss;
	ss<<"Mismatch in time vector vs configs: "<<times[0]->data.size()<<" vs "<<configs.size();
	*errorMessage = ss.str();
      }
      return false;
    }
    this->times = times[0]->data;
    this->milestones.resize(configs.size());
    for(size_t i=0;i<configs.size();i++) {
      this->milestones[i] = configs[i]->data;
    }
    return true;
  }
}

bool LinearPathResource::Unpack(vector<ResourcePtr>& res,bool* incomplete)
{
  res.push_back(MakeResource("times",this->times));
  res.push_back(MakeResource("milestones",this->milestones));
  return true;
}



bool MultiPathResource::Load(const string& fn)
{
  return path.Load(fn);
}

bool MultiPathResource::Save(const string& fn)
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


ResourceBase* MultiPathResource::Copy()
{
  MultiPathResource* res= new MultiPathResource;
  res->path = path;
  return res;
}

vector<string> MultiPathResource::CastTypes() const
{
  vector<string> res(2);
  res[0] = "Configs";
  res[1] = "LinearPath";
  return res;
}

vector<string> MultiPathResource::SubTypes() const
{
  vector<string> res(2);
  res[0] = "LinearPath";
  res[1] = "Stance";
  return res;
}

vector<string> MultiPathResource::ExtractTypes() const
{
  vector<string> res(5);
  res[0] = "Stance";
  res[1] = "LinearPath";
  res[2] = "Configs";
  res[3] = "Config";
  return res;
}

ResourcePtr MultiPathResource::Cast(const char* subtype) 
{
  if(0==strcmp(subtype,"LinearPath")) {
    LinearPathResource* p=new LinearPathResource;
    p->name = this->name;
    for(size_t i=0;i<this->path.sections.size();i++) 
      p->milestones.insert(p->milestones.end(),this->path.sections[i].milestones.begin(),this->path.sections[i].milestones.end());
    if(this->path.HasTiming()) {
      for(size_t i=0;i<this->path.sections.size();i++) 
	p->times.insert(p->times.end(),this->path.sections[i].times.begin(),this->path.sections[i].times.end());
    }
    else {
      p->times.resize(p->milestones.size());
      for(size_t i=0;i<p->times.size();i++)
	p->times[i] = i;
    }
    return p;
  }
  else if(0==strcmp(subtype,"Configs")) {
    ConfigsResource* c = new ConfigsResource;
    c->name = this->name;
    for(size_t i=0;i<this->path.sections.size();i++) 
      c->configs.insert(c->configs.end(),this->path.sections[i].milestones.begin(),this->path.sections[i].milestones.end());
    return c;
  }
  return NULL;
}

bool MultiPathResource::Extract(const char* subtype,vector<ResourcePtr>& res) 
{
  if(0==strcmp(subtype,"LinearPath")) {
    for(size_t i=0;i<this->path.sections.size();i++) {
      LinearPathResource* p=new LinearPathResource;
      p->milestones = this->path.sections[i].milestones;
      if(this->path.HasTiming(i))
	p->times = this->path.sections[i].times;
      else {
	p->times.resize(p->milestones.size());
	for(size_t j=0;j<p->times.size();j++)
	  p->times[j] = j;
      }
      stringstream ss;
      ss<<"section["<<i<<"]";
      p->name = ss.str();
      res.push_back(p);
    }
    return true;
  }
  else if(0==strcmp(subtype,"Configs")) {
    for(size_t i=0;i<this->path.sections.size();i++) {
      ConfigsResource* c = new ConfigsResource;
      c->configs = this->path.sections[i].milestones;;
      stringstream ss;
      ss<<"configs["<<i<<"]";
      c->name = ss.str();
      res.push_back(c);
    }
    return true;
  }
  else if(0==strcmp(subtype,"Config")) {
    for(size_t i=0;i<this->path.sections.size();i++) {
      for(size_t j=0;j<this->path.sections[i].milestones.size();j++) {
	stringstream ss;
	ss<<"config["<<i<<"]["<<j<<"]";
	res.push_back(MakeResource(ss.str(),this->path.sections[i].milestones[j]));
      }
    }
    return true;
  }
  else if(0==strcmp(subtype,"Stance")) {
    for(size_t i=0;i<this->path.sections.size();i++) {
      Stance s;
      this->path.GetStance(s,i);
      stringstream ss;
      ss<<"stance["<<i<<"]";
      res.push_back(MakeResource(ss.str(),s));
    }
    return true;
  }
  return false;
}

bool MultiPathResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  vector<LinearPathResource*> linearPaths = ResourcesByType<LinearPathResource>(lib);
  vector<ConfigsResource*> configs = ResourcesByType<ConfigsResource>(lib);
  vector<StanceResource*> stances = ResourcesByType<StanceResource>(lib);
  vector<MultiPathResource*> multiPaths = ResourcesByType<MultiPathResource>(lib);
  if(linearPaths.empty() && configs.empty() && multiPaths.empty()) {
    if(errorMessage) *errorMessage = "MultiPath must consist of at least one path section";
    return true;
  }
  int cnt = (linearPaths.empty() ? 0 : 1) + (configs.empty() ? 0 : 1) + (multiPaths.empty() ? 0 : 1);
  if(cnt != 1) {
    if(errorMessage) *errorMessage = "MultiPath may not consist of multiple types of path sections";
    return false;
  }
  if(!multiPaths.empty()) {
    //concatenate paths
    for(size_t i=0;i<multiPaths.size();i++) {
      //merge settings
      for(map<string,string>::const_iterator j=multiPaths[i]->path.settings.begin();j!=multiPaths[i]->path.settings.end();j++)
	this->path.settings[j->first] = j->second;
      //can't do global holds yet
      Assert(multiPaths[i]->path.holdSet.empty());
      Assert(multiPaths[i]->path.holdSetNames.empty());
      //append sections
      this->path.sections.insert(this->path.sections.end(),multiPaths[i]->path.sections.begin(),multiPaths[i]->path.sections.end());
    }
  }
  if(!configs.empty()) {
    if(configs.size() != 1 && configs.size() != stances.size()) {
      if(errorMessage) *errorMessage = "Mismatch in number of stances and config sets";
      return false;
    }
    this->path.sections.resize(configs.size());
    for(size_t i=0;i<configs.size();i++)
      this->path.sections[i].milestones = configs[i]->configs;
    if(!stances.empty()) {
      for(size_t i=0;i<stances.size();i++)      
	this->path.SetStance(stances[i]->stance,i);
    }
  }
  if(!linearPaths.empty()) {
    if(linearPaths.size() != 1 && linearPaths.size() != stances.size()) {
      if(errorMessage) *errorMessage = "Mismatch in number of stances and LinearPaths";
      return false;
    }
    this->path.sections.resize(linearPaths.size());
    for(size_t i=0;i<linearPaths.size();i++) {
      this->path.sections[i].times = linearPaths[i]->times;
      this->path.sections[i].milestones = linearPaths[i]->milestones;
    }
    if(!stances.empty()) {
      for(size_t i=0;i<stances.size();i++)      
	this->path.SetStance(stances[i]->stance,i);
    }
  }
  return true;
}

bool MultiPathResource::Unpack(vector<ResourcePtr>& res,bool* incomplete)
{
  if(incomplete) *incomplete = true;
  for(size_t i=0;i<this->path.sections.size();i++) {
    LinearPathResource* p=new LinearPathResource;
    p->milestones = this->path.sections[i].milestones;
    if(this->path.HasTiming(i))
      p->times = this->path.sections[i].times;
    else {
      p->times.resize(p->milestones.size());
      for(size_t j=0;j<p->times.size();j++)
	p->times[j] = j;
    }
    stringstream ss;
    ss<<"section["<<i<<"]";
    p->name = ss.str();
    res.push_back(p);
  }
  for(size_t i=0;i<this->path.sections.size();i++) {
    Stance s;
    this->path.GetStance(s,i);
    stringstream ss;
    ss<<"stance["<<i<<"]";
    res.push_back(MakeResource(ss.str(),s));
  }
  return true;
}





bool IKGoalResource::Load(AnyCollection& c)
{
  return Convert(c,goal);
}

bool IKGoalResource::Save(AnyCollection& c)
{
  c["type"] = string("IKGoal");
  Convert(goal,c);
  return true;
}

vector<string> IKGoalResource::CastTypes() const
{
  vector<string> res(3);
  res[0] = "Hold";
  res[1] = "Grasp";
  res[2] = "Stance";
  return res;
}

ResourcePtr IKGoalResource::Cast(const char* subtype) const
{
  if(0==strcmp(subtype,"Hold")) {
    Hold h;
    h.link = this->goal.link;
    h.ikConstraint = this->goal;
    return new HoldResource(h);
  }
  else if(0==strcmp(subtype,"Grasp")) {
    Grasp g;
    g.objectIndex = -1;
    g.robotIndex = -1;
    g.constraints.push_back(this->goal);
    return new GraspResource(g);
  }
  else if(0==strcmp(subtype,"Stance")) {
    Hold h;
    h.link = this->goal.link;
    h.ikConstraint = this->goal;
    Stance s;
    s.insert(h);
    return new StanceResource(s);
  }
  return NULL;
}

bool HoldResource::Load(AnyCollection& c)
{
  return Convert(c,hold);
}

bool HoldResource::Save(AnyCollection& c)
{
  c["type"] = string("Hold");
  Convert(hold,c);
  return true;
}


vector<string> HoldResource::CastTypes() const
{
  vector<string> res(3);
  res[0] = "IKGoal";
  res[1] = "Grasp";
  res[2] = "Stance";
  return res;
}

vector<string> HoldResource::SubTypes() const
{
  vector<string> res(2);
  res[0] = "IKGoal";
  res[1] = "vector<double>";
  return res;
}

ResourcePtr HoldResource::Cast(const char* subtype) const
{
  if(0==strcmp(subtype,"IKGoal")) {
    return MakeResource(this->name,this->hold.ikConstraint);
  }
  else if(0==strcmp(subtype,"Stance")) {
    Stance s;
    s.insert(this->hold);
    return MakeResource(this->name,s);
  }
  else if(0==strcmp(subtype,"Grasp")) {
    Grasp g;
    g.SetHold(this->hold);
    return MakeResource(this->name,g);
  }
  return NULL;
}

bool HoldResource::Unpack(vector<ResourcePtr>& lib,bool* incomplete)
{
  lib.push_back(MakeResource("ikConstraint",hold.ikConstraint));
  for(size_t i=0;i<hold.contacts.size();i++) {
    vector<double> c(7);
    this->hold.contacts[i].x.get(&c[0]);
    this->hold.contacts[i].n.get(&c[3]);
    c[6] = this->hold.contacts[i].kFriction;
    stringstream ss;
    ss<<"contact["<<i<<"]";
    lib.push_back(MakeResource(ss.str(),c));
  }
  return true;
}

bool HoldResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  vector<IKGoalResource*> ikgoals = ResourcesByType<IKGoalResource>(lib);
  vector<FloatArrayResource*> contacts = ResourcesByType<FloatArrayResource>(lib);
  if(ikgoals.size() != 1) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Trying to pack more than 1 ik goal into a hold\n");
    return false;
  }
  this->hold.link = ikgoals[0]->goal.link;
  this->hold.ikConstraint = ikgoals[0]->goal;
  this->hold.contacts.resize(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    if(contacts[i]->data.size() != 7) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Contact point doesn't have 7 entries x y z nx ny nz kf\n");
      return false;
    }
    this->hold.contacts[i].x.set(&contacts[i]->data[0]);
    this->hold.contacts[i].n.set(&contacts[i]->data[3]);
    this->hold.contacts[i].kFriction = contacts[i]->data[6];
  }
  return true;
}




StanceResource::StanceResource(const Stance& val)
  :stance(val)
{}

bool StanceResource::Load(istream& in)
{
  in>>stance;
  return bool(in);
}

bool StanceResource::Save(ostream& out)
{
  out<<stance;
  return bool(out);
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
      stance[h.hold.link] = h.hold;
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
  Convert(c,stance);
  return true;
}

bool StanceResource::Save(AnyCollection& c)
{
  c["type"] = string("Stance");
  Convert(stance,c);
  return true;
}

ResourceBase* StanceResource::Copy()
{
  StanceResource* res = new StanceResource;
  res->stance = stance;
  return res;
}

vector<string> StanceResource::CastTypes() const
{
  vector<string> res(1);
  res[0]="Grasp";
  return res;
}

vector<string> StanceResource::ExtractTypes() const
{
  vector<string> res(3);
  res[0]="Hold";
  res[1]="Grasp";
  res[2]="IKGoal";
  return res;
}


vector<string> StanceResource::SubTypes() const
{
  return vector<string> (1,"Hold");
}


ResourcePtr StanceResource::Cast(const char* type)
{
  if(0==strcmp(type,"Grasp")) {
    Grasp g;
    g.SetStance(stance);
    return MakeResource(name,g);
  }
  return NULL;
}

bool StanceResource::Extract(const char* subtype,vector<ResourcePtr>& res)
{
  if(0==strcmp(subtype,"Hold")) {
    Unpack(res);
    return true;
  }
  if(0==strcmp(subtype,"IKGoal")) {
    for(Stance::const_iterator i=this->stance.begin();i!=this->stance.end();i++) {
      stringstream ss;
      ss<<"ikgoal["<<i->first<<"]";
      res.push_back(MakeResource(ss.str(),i->second.ikConstraint));
    }
    return true;
  }
  return false;
}

bool StanceResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  vector<HoldResource*> holds = ResourcesByType<HoldResource>(lib);
  vector<IKGoalResource*> ikgoals = ResourcesByType<IKGoalResource>(lib);
  stance.clear();
  for(size_t i=0;i<holds.size();i++)
    this->stance.insert(holds[i]->hold);
  for(size_t i=0;i<ikgoals.size();i++) {
    Hold h;
    h.link = ikgoals[i]->goal.link;
    h.ikConstraint = ikgoals[i]->goal;
    this->stance.insert(h);
  }
  return true;
}

bool StanceResource::Unpack(vector<ResourcePtr>& res,bool* incomplete)
{
  for(Stance::const_iterator i=stance.begin();i!=stance.end();i++) {
    stringstream ss;
    ss<<"hold["<<i->first<<"]";
    res.push_back(MakeResource(ss.str(),i->second));
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
  c["type"] = string("Grasp");
  Convert(grasp,c);
  return true;
}

ResourceBase* GraspResource::Copy()
{
  GraspResource* res = new GraspResource;
  res->grasp = grasp;
  return res;
}

vector<string> GraspResource::CastTypes() const
{
  vector<string> res(3);
  res[0]="Hold";
  res[1]="Stance";
  res[2]="IKGoal";
  return res;
}

vector<string> GraspResource::ExtractTypes() const
{
  vector<string> res(3);
  res[0]="Hold";
  res[1]="Stance";
  res[2]="IKGoal";
  return res;
}


vector<string> GraspResource::SubTypes() const
{
  return vector<string> (1,"Hold");
}


ResourcePtr GraspResource::Cast(const char* type)
{
  if(0==strcmp(type,"Hold")) {
    if(this->grasp.constraints.size()!=1) return NULL;
    Hold h;
    this->grasp.GetHold(h);
    return new HoldResource(h);
  }
  else if(0==strcmp(type,"Stance")) {
    Stance s;
    this->grasp.GetStance(s);
    return new StanceResource(s);
  }
  else if(0==strcmp(type,"IKGoal")) {
    if(this->grasp.constraints.size()!=1) return NULL;
    return new IKGoalResource(this->grasp.constraints.front());
  }
  return NULL;
}

bool GraspResource::Extract(const char* type,vector<ResourcePtr>& res)
{
  if(0==strcmp(type,"Hold")) {
    Stance s;
    this->grasp.GetStance(s);
    for(Stance::const_iterator i=s.begin();i!=s.end();i++) {
      stringstream ss;
      ss<<"hold["<<i->first<<"]";
      res.push_back(MakeResource(ss.str(),i->second));
    }
    return true;
  }
  else if(0==strcmp(type,"Stance")) {
    Stance s;
    this->grasp.GetStance(s);
    res.push_back(MakeResource(this->name,s));
    return true;
  }
  else if(0==strcmp(type,"IKGoal")) {
    for(size_t i=0;i<this->grasp.constraints.size();i++) {
      stringstream ss;
      ss<<"constraints["<<i<<"]";
      res.push_back(MakeResource(ss.str(),this->grasp.constraints[i]));
    }
    return true;
  }
  return false;
}

bool GraspResource::Pack(vector<ResourcePtr>& lib,string* errorMessage)
{
  //Grasp packing is not done yet
  return CompoundResourceBase::Pack(lib,errorMessage);
}

bool GraspResource::Unpack(vector<ResourcePtr>& res,bool* incomplete)
{
  Extract("Hold",res);
  if(incomplete) *incomplete = true;
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
  TriMeshResource* res=new TriMeshResource;
  res->data = mesh;
  res->name = name;
  return res;
}

ResourcePtr MakeResource(const string& name,const Geometry::AnyGeometry3D& geom)
{
  if(geom.type == Geometry::AnyGeometry3D::TriangleMesh)
    return MakeResource(name,geom.AsTriangleMesh());
  else if(geom.type == Geometry::AnyGeometry3D::Primitive)
    return MakeResource(name,geom.AsPrimitive());
  else
    return NULL;
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

ResourcePtr CastResource(ResourcePtr& item,const char* type)
{
  if(0==strcmp(item->Type(),type)) return item;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>((ResourceBase*)item);
  if(cr) return cr->Cast(type);

  if(0==strcmp("Config",item->Type())) {
    ConfigResource* cr = dynamic_cast<ConfigResource*>((ResourceBase*)item);
    if(0==strcmp(type,"vector<double>")) {
      return new FloatArrayResource(cr->data);
    }
  }
  else if(0==strcmp("vector<double>",item->Type())) {
    FloatArrayResource* ar = dynamic_cast<FloatArrayResource*>((ResourceBase*)item);
    if(0==strcmp(type,"Config")) {
      return new ConfigResource(ar->data);
    }
  }
    LOG4CXX_ERROR(KrisLibrary::logger(),"CastResource: No conversion from "<<item->Type()<<" to "<<type);
  return NULL;
}

vector<string> ExtractResourceTypes(const ResourcePtr& item)
{
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>((const ResourceBase*)item);
  if(cr) 
    return cr->ExtractTypes();
  return vector<string>();
}

vector<ResourcePtr> ExtractResources(ResourcePtr& item,const char* type)
{
  vector<ResourcePtr> res;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>((ResourceBase*)item);
  if(cr) {
    if(!cr->Extract(type,res)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"ExtractResource: No elements of type "<<type<<" in "<<item->Type());
    }
    return res;
  }
    LOG4CXX_ERROR(KrisLibrary::logger(),"ExtractResource: item "<<item->Type());
  return res;
}




///Returns true if CastResource can cast to the given type
bool CanCastResource(const ResourcePtr& item,const char* type)
{
  if(0==strcmp(item->Type(),type)) return true;
  if(0==strcmp("Config",item->Type())) {
    if(0==strcmp(type,"vector<double>")) return true;
  }
  else if(0==strcmp("vector<double>",item->Type())) {
    if(0==strcmp(type,"Config")) return true;
  }
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>((const ResourceBase*)item);
  vector<string> types = cr->CastTypes();  
  for(size_t i=0;i<types.size();i++)
    if(types[i] == type) return true;
  return false;
}

///Returns the list of types which the item is castable to
vector<string> CastResourceTypes(const ResourcePtr& item)
{
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>((const ResourceBase*)item);
  if(cr) return cr->CastTypes();
  vector<string> res;
  if(0==strcmp("Config",item->Type())) {
    res.push_back("vector<double>");
  }
  else if(0==strcmp("vector<double>",item->Type())) {
    res.push_back("Config");
  }
  return res;
}

bool IsCompoundResource(const ResourcePtr& r)
{
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>((const ResourceBase*)r);
  if(cr) return true;
  return false;
}

vector<ResourcePtr> UnpackResource(ResourcePtr r,bool* successful,bool* incomplete)
{
  vector<string> types;
  const char* type = r->Type();
  vector<ResourcePtr> res;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>((ResourceBase*)r);
  if(cr) {
    vector<ResourcePtr> res;
    bool success = cr->Unpack(res,incomplete);
    if(successful) *successful=success;
    return res;
  }
  else {
    LOG4CXX_WARN(KrisLibrary::logger(),"ResourceNode::Expand: Warning, no expansion known for type "<<type);
    if(successful) *successful = false;
    return res;
  }
 }


ResourcePtr PackResources(ResourceLibrary& lib,const string& type,string* errorMessage)
{
  if(lib.knownTypes.count(type)==0) {
    if(errorMessage) *errorMessage = string("Unknown type ")+type;
    return NULL;
  }
  vector<ResourcePtr> resources = lib.Enumerate();
  return PackResources(resources,lib.knownTypes[type][0],errorMessage);
}

ResourcePtr PackResources(vector<ResourcePtr>& lib,ResourcePtr rtemplate,string* errorMessage)
{
  CompoundResourceBase* crtemplate = dynamic_cast<CompoundResourceBase*>((ResourceBase*)rtemplate);
  if(!crtemplate) {
    if(errorMessage) {
      stringstream ss;
      ss<<"Cannot pack non-compound type "<<rtemplate->Type();
      *errorMessage = ss.str();
    }
  }
  ResourcePtr r = crtemplate->Make();
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>((ResourceBase*)r);
  if(cr->Pack(lib,errorMessage)) return r;
  return NULL;  
}




void Convert(const Vector& v,AnyCollection& c)
{
  Convert(vector<Real>(v),c);
}

bool Convert(const AnyCollection& c,Vector& v)
{
  if(c.as(v)) return true;
  vector<Real> array;
  if(!Convert(c,array)) return false;
  v = array;
  return true;
}

void Convert(const Vector3& x,AnyCollection& c)
{
  vector<double> v(3);
  x.get(v[0],v[1],v[2]);
  c = v;
}

bool Convert(const AnyCollection& c,Vector3& x)
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
void Convert(const Hold& h,AnyCollection& c)
{
  Convert(h.ikConstraint,c["ik"]);
  c["contacts"].resize(h.contacts.size());
  for(size_t i=0;i<h.contacts.size();i++) {
    Convert(h.contacts[i].x,c["contacts"][(int)i]["x"]);
    Convert(h.contacts[i].n,c["contacts"][(int)i]["n"]);
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
    Convert(g.contacts[i].x,c["contacts"][(int)i]["x"]);
    Convert(g.contacts[i].n,c["contacts"][(int)i]["n"]);
    c["contacts"][(int)i]["kFriction"] = g.contacts[i].kFriction;
  }
  c["contactLinks"] = g.contactLinks;
  c["forces"].resize(g.forces.size());
  for(size_t i=0;i<g.forces.size();i++)
    Convert(g.forces[i],c["forces"][(int)i]);
}

void Convert(const Stance& s,AnyCollection& c)
{
  c["holds"].resize(s.size());
  int k=0;
  for(Stance::const_iterator i=s.begin();i!=s.end();i++,k++) {
    Convert(i->second,c["holds"][k]);
  }
}
 
bool Convert(const AnyCollection& c,IKGoal& g)
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"AnyCollection to IKGoal: Invalid posConstraint type "<<s.c_str());
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
            LOG4CXX_ERROR(KrisLibrary::logger(),"AnyCollection to IKGoal: Invalid rotConstraint type "<<s.c_str());
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

bool Convert(const AnyCollection& c,Hold& h)
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


bool Convert(const AnyCollection& c,Grasp& g)
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

bool Convert(const AnyCollection& c,Stance& s)
{
  vector<SmartPointer<AnyCollection> > holds;
  s.clear();
  c["holds"].enumerate(holds);
  for(size_t i=0;i<holds.size();i++) {
    HoldResource h;
    if(!h.Load(*holds[i])) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Convert(AnyCollection,Stance): Error reading hold "<<(int)i);
      return false;
    }
    s.insert(h.hold);
  }
  return true;
}
