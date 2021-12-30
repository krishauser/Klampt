#include "Resources.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/meshing/IO.h>
#include <tinyxml.h>
#include "IO/XmlWorld.h"
#include "IO/JSON.h"
#include <sstream>


template <> const char* BasicResourceTypeName<Config>() { return "Config"; }
template <> const char* BasicResourceTypeName<Vector3>() { return "Vector3"; }
template <> const char* BasicResourceTypeName<Matrix3>() { return "Matrix3"; }
template <> const char* BasicResourceTypeName<Matrix>() { return "Matrix"; }
template <> const char* BasicResourceTypeName<RigidTransform>() { return "RigidTransform"; }
template <> const char* BasicResourceTypeName<Meshing::TriMesh>() { return "TriMesh"; }
template <> const char* BasicResourceTypeName<GeometricPrimitive3D>() { return "GeometricPrimitive3D"; }
template <> const char* BasicResourceTypeName<Camera::Viewport>() { return "Viewport"; }
template class BasicResource<Config>;
template class BasicResource<Vector3>;
template class BasicResource<Matrix3>;
template class BasicResource<Matrix>;
template class BasicResource<Meshing::TriMesh>;
template class BasicResource<GeometricPrimitive3D>;
template class BasicResource<RigidTransform>;
template class BasicResource<Camera::Viewport>;

namespace Klampt {


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
    return ResourcePtr(p);
  }
  else if(0==strcmp(type,"MultiPath")) {
    MultiPathResource* mp = new MultiPathResource;
    mp->name = name;
    mp->path.SetMilestones(configs);
    return ResourcePtr(mp);
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
    res.push_back(ResourcePtr(p));
    return true;
  }
  else if(0==strcmp(subtype,"MultiPath")) {
    MultiPathResource* mp = new MultiPathResource;
    mp->path.SetMilestones(this->configs);
    mp->name = this->name;
    res.push_back(ResourcePtr(mp));
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
    ConfigResource* config = dynamic_cast<ConfigResource*>(lib[i].get());
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
    printf("TriMeshResource::Load:  Unable to load mesh from file %s\n",fn.c_str());
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
  fprintf(stderr,"TODO: saving worlds\n");
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
      subobjects.push_back(ResourcePtr(rr));
    }
    return true;
  }
  else if(0==strcmp(subtype,"RigidObject")) {
    for(size_t i=0;i<world.rigidObjects.size();i++) {
      RigidObjectResource* rr = new RigidObjectResource;
      rr->name = world.rigidObjects[i]->name;
      rr->object = *world.rigidObjects[i];
      subobjects.push_back(ResourcePtr(rr));
    }
    return true;
  }
  else if(0==strcmp(subtype,"ResourceLibrary")) {
    for(size_t i=0;i<world.terrains.size();i++) {
      ResourceLibraryResource* r = new ResourceLibraryResource;
      r->name = world.terrains[i]->name;
      r->library.Add(MakeResource("geometry",*world.terrains[i]->geometry));
      r->library.Add(MakeResource("kFriction",world.terrains[i]->kFriction));
      subobjects.push_back(ResourcePtr(r));
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
    subobjects.push_back(ResourcePtr(rr));
  }
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    RigidObjectResource* rr = new RigidObjectResource;
    rr->name = world.rigidObjects[i]->name;
    rr->object = *world.rigidObjects[i];
    subobjects.push_back(ResourcePtr(rr));
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    ResourceLibraryResource* r = new ResourceLibraryResource;
    r->name = world.terrains[i]->name;
    r->library.Add(MakeResource("geometry",*world.terrains[i]->geometry));
    r->library.Add(MakeResource("kFriction",world.terrains[i]->kFriction));
    subobjects.push_back(ResourcePtr(r));
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
    return ResourcePtr(mp);
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
    return ResourcePtr(p);
  }
  else if(0==strcmp(subtype,"Configs")) {
    ConfigsResource* c = new ConfigsResource;
    c->name = this->name;
    for(size_t i=0;i<this->path.sections.size();i++) 
      c->configs.insert(c->configs.end(),this->path.sections[i].milestones.begin(),this->path.sections[i].milestones.end());
    return ResourcePtr(c);
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
      res.push_back(ResourcePtr(p));
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
      res.push_back(ResourcePtr(c));
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
    res.push_back(ResourcePtr(p));
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

ResourcePtr IKGoalResource::Cast(const char* subtype)
{
  if(0==strcmp(subtype,"Hold")) {
    Hold h;
    h.link = this->goal.link;
    h.ikConstraint = this->goal;
    return ResourcePtr(new HoldResource(h));
  }
  else if(0==strcmp(subtype,"Grasp")) {
    Grasp g;
    g.objectIndex = -1;
    g.robotIndex = -1;
    g.constraints.push_back(this->goal);
    return ResourcePtr(new GraspResource(g));
  }
  else if(0==strcmp(subtype,"Stance")) {
    Hold h;
    h.link = this->goal.link;
    h.ikConstraint = this->goal;
    Stance s;
    s.insert(h);
    return ResourcePtr(new StanceResource(s));
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

ResourcePtr HoldResource::Cast(const char* subtype)
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
    fprintf(stderr,"Trying to pack more than 1 ik goal into a hold\n");
    return false;
  }
  this->hold.link = ikgoals[0]->goal.link;
  this->hold.ikConstraint = ikgoals[0]->goal;
  this->hold.contacts.resize(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    if(contacts[i]->data.size() != 7) {
      fprintf(stderr,"Contact point doesn't have 7 entries x y z nx ny nz kf\n");
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
    return ResourcePtr(new HoldResource(h));
  }
  else if(0==strcmp(type,"Stance")) {
    Stance s;
    this->grasp.GetStance(s);
    return ResourcePtr(new StanceResource(s));
  }
  else if(0==strcmp(type,"IKGoal")) {
    if(this->grasp.constraints.size()!=1) return NULL;
    return ResourcePtr(new IKGoalResource(this->grasp.constraints.front()));
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
  return ResourcePtr(new IntArrayResource(vals,name));
}

ResourcePtr MakeResource(const string& name,const vector<double>& vals)
{
  return ResourcePtr(new FloatArrayResource(vals,name));
}

ResourcePtr MakeResource(const string& name,const Config& q)
{
  return ResourcePtr(new ConfigResource(q,name));
}

ResourcePtr MakeResource(const string& name,const vector<Config>& qs)
{
  ConfigsResource* r=new ConfigsResource;
  r->configs = qs;
  r->name = name;
  return ResourcePtr(r);
}

ResourcePtr MakeResource(const string& name,const vector<Real>& ts,const vector<Config>& qs)
{
  LinearPathResource* r=new LinearPathResource;
  r->name = name;
  r->times = ts;
  r->milestones = qs;
  return ResourcePtr(r);
}

ResourcePtr MakeResource(const string& name,const MultiPath& path)
{
  MultiPathResource* r=new MultiPathResource;
  r->name = name;
  r->path = path;
  return ResourcePtr(r);
}

ResourcePtr MakeResource(const string& name,const Vector3& T)
{
  return ResourcePtr(new Vector3Resource(T,name));
}

ResourcePtr MakeResource(const string& name,const Matrix3& T)
{
  return ResourcePtr(new Matrix3Resource(T,name));
}

ResourcePtr MakeResource(const string& name,const RigidTransform& T)
{
  return ResourcePtr(new RigidTransformResource(T,name));
}

ResourcePtr MakeResource(const string& name,const GeometricPrimitive3D& geom)
{
  return ResourcePtr(new GeometricPrimitive3DResource(geom,name));
}

ResourcePtr MakeResource(const string& name,const IKGoal& goal)
{
  return ResourcePtr(new IKGoalResource(goal,name));
}


ResourcePtr MakeResource(const string& name,const Meshing::TriMesh& mesh)
{
  TriMeshResource* res=new TriMeshResource;
  res->data = mesh;
  res->name = name;
  return ResourcePtr(res);
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
  return ResourcePtr(new HoldResource(hold,name));
}

ResourcePtr MakeResource(const string& name,const Stance& stance)
{
  StanceResource* res=new StanceResource(stance);
  res->name = name;
  return ResourcePtr(res);
}

ResourcePtr MakeResource(const string& name,const Grasp& grasp)
{
  GraspResource* res=new GraspResource(grasp);
  res->name = name;
  return ResourcePtr(res);
}

ResourcePtr CastResource(ResourcePtr& item,const char* type)
{
  if(0==strcmp(item->Type(),type)) return item;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>(item.get());
  if(cr) return cr->Cast(type);

  if(0==strcmp("Config",item->Type())) {
    ConfigResource* cr = dynamic_cast<ConfigResource*>(item.get());
    if(0==strcmp(type,"vector<double>")) {
      return ResourcePtr(new FloatArrayResource(cr->data));
    }
  }
  else if(0==strcmp("vector<double>",item->Type())) {
    FloatArrayResource* ar = dynamic_cast<FloatArrayResource*>(item.get());
    if(0==strcmp(type,"Config")) {
      return ResourcePtr(new ConfigResource(Vector(ar->data)));
    }
  }
  fprintf(stderr,"CastResource: No conversion from %s to %s\n",item->Type(),type);
  return NULL;
}

vector<string> ExtractResourceTypes(const ResourcePtr& item)
{
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>(item.get());
  if(cr) 
    return cr->ExtractTypes();
  return vector<string>();
}

vector<ResourcePtr> ExtractResources(ResourcePtr& item,const char* type)
{
  vector<ResourcePtr> res;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>(item.get());
  if(cr) {
    if(!cr->Extract(type,res)) {
      fprintf(stderr,"ExtractResource: No elements of type %s in %s\n",type,item->Type());
    }
    return res;
  }
  fprintf(stderr,"ExtractResource: item %s is not compound\n",item->Type());
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
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>(item.get());
  vector<string> types = cr->CastTypes();  
  for(size_t i=0;i<types.size();i++)
    if(types[i] == type) return true;
  return false;
}

///Returns the list of types which the item is castable to
vector<string> CastResourceTypes(const ResourcePtr& item)
{
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>(item.get());
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
  const CompoundResourceBase* cr = dynamic_cast<const CompoundResourceBase*>(r.get());
  if(cr) return true;
  return false;
}

vector<ResourcePtr> UnpackResource(ResourcePtr r,bool* successful,bool* incomplete)
{
  vector<string> types;
  const char* type = r->Type();
  vector<ResourcePtr> res;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>(r.get());
  if(cr) {
    vector<ResourcePtr> res;
    bool success = cr->Unpack(res,incomplete);
    if(successful) *successful=success;
    return res;
  }
  else {
    printf("ResourceNode::Expand: Warning, no expansion known for type %s\n",type);
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
  CompoundResourceBase* crtemplate = dynamic_cast<CompoundResourceBase*>(rtemplate.get());
  if(!crtemplate) {
    if(errorMessage) {
      stringstream ss;
      ss<<"Cannot pack non-compound type "<<rtemplate->Type();
      *errorMessage = ss.str();
    }
  }
  ResourcePtr r(crtemplate->Make());
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>(r.get());
  if(cr->Pack(lib,errorMessage)) return r;
  return NULL;  
}


} //namespace Klampt