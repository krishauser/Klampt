#include "Modeling/Resources.h"
#include <utils/stringutils.h>
#include <fstream>
#include <algorithm>
#include <map>
#include <vector>
#include <string.h>
using namespace std;

map<string,vector<string> > unpackTypes;
map<string,vector<string> > packTypes;
map<string,int> packPriorities;

void MakeCompoundTypes()
{
  unpackTypes["Hold"].push_back("IKGoal");
  unpackTypes["Hold"].push_back("vector<double>");

  unpackTypes["Stance"].push_back("Hold");

  unpackTypes["LinearPath"].push_back("vector<double>");
  unpackTypes["LinearPath"].push_back("Configs");

  unpackTypes["MultiPath"].push_back("LinearPath");
  unpackTypes["MultiPath"].push_back("Configs");
  unpackTypes["MultiPath"].push_back("Stance");

  unpackTypes["Configs"].push_back("Config");

  unpackTypes["World"].push_back("Robot");
  unpackTypes["World"].push_back("RigidObject");
  unpackTypes["World"].push_back("TriMesh");

  packTypes["Hold"].push_back("IKGoal");
  packTypes["Hold"].push_back("vector<double>");
  packPriorities["Hold"] = 0;

  packTypes["Stance"].push_back("Hold");
  packTypes["Stance"].push_back("IKGoal");
  packPriorities["Stance"] = 1;


  packTypes["Configs"].push_back("Config");
  packPriorities["LinearPath"] = 2;

  packTypes["LinearPath"].push_back("vector<double>");
  packTypes["LinearPath"].push_back("Config");
  packTypes["LinearPath"].push_back("Configs");
  packPriorities["LinearPath"] = 3;

  packTypes["MultiPath"].push_back("Config");
  packTypes["MultiPath"].push_back("Configs");
  packTypes["MultiPath"].push_back("LinearPath");
  packTypes["MultiPath"].push_back("MultiPath");
  packTypes["MultiPath"].push_back("Stance");
  packPriorities["MultiPath"] = 4;
  //packTypes["MultiPath"].push_back("Hold");
  //packTypes["MultiPath"].push_back("IKGoal");

  packTypes["World"].push_back("Robot");
  packTypes["World"].push_back("RigidObject");
  packTypes["World"].push_back("TriMesh");
  packPriorities["MultiPath"] = 5;
}

bool Unpack(const char* fn,const char* path=NULL)
{
  ResourceLibrary lib;
  MakeRobotResourceLibrary(lib);
  ResourcePtr r = lib.LoadItem(fn);
  if(!r) return false;
  if(unpackTypes.find(r->Type()) == unpackTypes.end()) {
    printf("Type %s is not a compound type\n",r->Type());
    return false;
  }
  cout<<"Resource of type "<<r->Type()<<endl;
  vector<string> s = unpackTypes[r->Type()];
  if(0==strcmp(r->Type(),"MultiPath")) {
    //conditional unpack -- if untimed, unpack sections as Configs.  If timed,
    //unpack sections as LinearPaths
    MultiPathResource* mp = dynamic_cast<MultiPathResource*>(&(*r));
    if(mp->path.HasTiming()) 
      s.erase(find(s.begin(),s.end(),"Configs"));
    else
      s.erase(find(s.begin(),s.end(),"LinearPath"));
  }
  for(size_t i=0;i<s.size();i++) {
    vector<ResourcePtr> res = ExtractResources(r,s[i].c_str());
    //change "name." to "name/"
    for(size_t j=0;j<res.size();j++) {
      int n = res[j]->name.find('.');
      if(n == string::npos)
	res[j]->fileName = r->name+"/"+lib.DefaultFileName(res[j]);
      else {
	res[j]->fileName = lib.DefaultFileName(res[j]);
	res[j]->fileName.replace(n,1,"/");
      }
      if(path != NULL) {
	res[j]->fileName = string(path)+"/"+res[j]->fileName;
      }
      else {
	char path[1024];
	GetFilePath(fn,path);
	res[j]->fileName = string(path) + res[j]->fileName;
      }
      cout<<"Extracted item "<<res[j]->name<<", saving to "<<res[j]->fileName<<endl;
      if(!res[j]->Save()) {
	printf("Could not save %s to %s\n",res[j]->name.c_str(),res[j]->fileName.c_str());
	return false;
      }
    }
  }
  return true;
}

string AutoTypePack(ResourceLibrary& lib)
{
  vector<string> libtypes;
  for(ResourceLibrary::Map::iterator i=lib.itemsByType.begin();i!=lib.itemsByType.end();i++) libtypes.push_back(i->first);

  vector<string> candidates;
  for(map<string,vector<string> >::iterator i=packTypes.begin();i!=packTypes.end();i++) {
    const vector<string>& itypes = i->second;
    //if libtypes is a subsets of itypes, add it to candidates
    bool cand = true;
    for(size_t j=0;j<libtypes.size();j++)
      if(find(itypes.begin(),itypes.end(),libtypes[j]) == itypes.end()) {
	cand = false;
	break;
      }
    if(cand) candidates.push_back(i->first);
  }
  if(candidates.empty()) return "";
  if(candidates.size()==1) return candidates[0];

  vector<string> bestCandidates;
  int bestPriority = 100;
  for(size_t i=0;i<candidates.size();i++) {
    if(packPriorities[candidates[i]] == bestPriority) 
      bestCandidates.push_back(candidates[i]);
    else if(packPriorities[candidates[i]] < bestPriority) {
      bestCandidates.resize(1);
      bestCandidates[0] = candidates[i];
      bestPriority = packPriorities[candidates[i]];
    }
  }
  if(bestCandidates.size()==1) 
    return bestCandidates[0];

  //otherwise, have an opportunity for multiple candidates
  cout<<"Ambiguous pack, items can be composed into types:"<<endl;
  for(size_t i=0;i<bestCandidates.size();i++)
    cout<<bestCandidates[i]<<endl;
  return "";
}

ConfigsResource* PackConfigs(ResourceLibrary& lib)
{
  std::vector<ConfigResource*> configs = lib.GetPtrsByType<ConfigResource>();
  ConfigsResource* c=new ConfigsResource;
  for(size_t i=0;i<configs.size();i++)
    c->configs.push_back(configs[i]->data);
  return c;
}

StanceResource* PackStance(ResourceLibrary& lib)
{
  std::vector<HoldResource*> holds = lib.GetPtrsByType<HoldResource>();
  std::vector<IKGoalResource*> ikgoals = lib.GetPtrsByType<IKGoalResource>();
  StanceResource* s=new StanceResource;
  for(size_t i=0;i<holds.size();i++)
    s->stance.insert(holds[i]->data);
  for(size_t i=0;i<ikgoals.size();i++) {
    Hold h;
    h.link = ikgoals[i]->data.link;
    h.ikConstraint = ikgoals[i]->data;
    s->stance.insert(h);
  }
  return s;
}


HoldResource* PackHold(ResourceLibrary& lib)
{
  std::vector<IKGoalResource*> ikgoals = lib.GetPtrsByType<IKGoalResource>();
  std::vector<FloatArrayResource*> contacts = lib.GetPtrsByType<FloatArrayResource>();
  if(ikgoals.size() != 1) {
    fprintf(stderr,"Trying to pack more than 1 ik goal into a hold\n");
    return false;
  }
  HoldResource* h=new HoldResource;
  h->data.link = ikgoals[0]->data.link;
  h->data.ikConstraint = ikgoals[0]->data;
  h->data.contacts.resize(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    if(contacts[i]->data.size() != 7) {
      fprintf(stderr,"Contact point doesn't have 7 entries x y z nx ny nz kf\n");
      return false;
    }
    h->data.contacts[i].x.set(&contacts[i]->data[0]);
    h->data.contacts[i].n.set(&contacts[i]->data[3]);
    h->data.contacts[i].kFriction = contacts[i]->data[6];
  }
  return h;
}


LinearPathResource* PackLinearPath(ResourceLibrary& lib)
{
  std::vector<FloatArrayResource*> times = lib.GetPtrsByType<FloatArrayResource>();
  std::vector<ConfigResource*> configs = lib.GetPtrsByType<ConfigResource>();
  std::vector<ConfigsResource*> configss = lib.GetPtrsByType<ConfigsResource>();
  if(times.size() != 1) {
    fprintf(stderr,"Trying to pack more than 1 time vector into a linear path\n");
    return false;
  }
  if(configs.empty()) {
    if(configss.size() != 1) {
      fprintf(stderr,"Trying to pack more than 1 config set into a linear path\n");
      return false;
    }
    LinearPathResource* p=new LinearPathResource;
    p->times = times[0]->data;
    p->milestones = configss[0]->configs;
    return p;
  }
  else {
    if(!configss.empty()) {
      fprintf(stderr,"Cannot combine config's and config sets into a linear path\n");
      return false;
    }
    if(times[0]->data.size() != configs.size()) {
      fprintf(stderr,"Mismatch in time vector vs configs: %d vs %d\n",times[0]->data.size(),configs.size());
      return false;
    }
    LinearPathResource* p=new LinearPathResource;
    p->times = times[0]->data;
    for(size_t i=0;i<configs.size();i++) {
      p->milestones.push_back(configs[i]->data);
    }
    return p;
  }
}

MultiPathResource* PackMultiPath(ResourceLibrary& lib)
{
  std::vector<LinearPathResource*> linearPaths = lib.GetPtrsByType<LinearPathResource>();
  std::vector<ConfigsResource*> configs = lib.GetPtrsByType<ConfigsResource>();
  std::vector<StanceResource*> stances = lib.GetPtrsByType<StanceResource>();
  std::vector<MultiPathResource*> multiPaths = lib.GetPtrsByType<MultiPathResource>();
  if(linearPaths.empty() && configs.empty() && multiPaths.empty()) {
    fprintf(stderr,"MultiPath must consist of at least one path section\n");
    return false;
  }
  int cnt = (linearPaths.empty() ? 0 : 1) + (configs.empty() ? 0 : 1) + (multiPaths.empty() ? 0 : 1);
  if(cnt != 1) {
    fprintf(stderr,"MultiPath may not consist of multiple types of path sections\n");
    return false;
  }
  MultiPathResource* p = new MultiPathResource;
  if(!multiPaths.empty()) {
    //concatenate paths
    for(size_t i=0;i<multiPaths.size();i++) {
      //merge settings
      for(map<string,string>::const_iterator j=multiPaths[i]->path.settings.begin();j!=multiPaths[i]->path.settings.end();j++)
	p->path.settings[j->first] = j->second;
      //can't do global holds yet
      Assert(multiPaths[i]->path.holdSet.empty());
      Assert(multiPaths[i]->path.holdSetNames.empty());
      //append sections
      p->path.sections.insert(p->path.sections.end(),multiPaths[i]->path.sections.begin(),multiPaths[i]->path.sections.end());
    }
  }
  if(!configs.empty()) {
    if(configs.size() != 1 && configs.size() != stances.size()) {
      delete p;
      fprintf(stderr,"Mismatch in number of stances and config sets\n");
      return NULL;
    }
    p->path.sections.resize(configs.size());
    for(size_t i=0;i<configs.size();i++)
      p->path.sections[i].milestones = configs[i]->configs;
    if(!stances.empty()) {
      for(size_t i=0;i<stances.size();i++)      
	p->path.SetStance(stances[i]->stance,i);
    }
  }
  if(!linearPaths.empty()) {
    if(linearPaths.size() != 1 && linearPaths.size() != stances.size()) {
      delete p;
      fprintf(stderr,"Mismatch in number of stances and LinearPaths\n");
      return NULL;
    }
    p->path.sections.resize(linearPaths.size());
    for(size_t i=0;i<linearPaths.size();i++) {
      p->path.sections[i].times = linearPaths[i]->times;
      p->path.sections[i].milestones = linearPaths[i]->milestones;
    }
    if(!stances.empty()) {
      for(size_t i=0;i<stances.size();i++)      
	p->path.SetStance(stances[i]->stance,i);
    }
  }
  return p;
}

bool Pack(ResourceLibrary& lib,const char* outname,const char* outtype="auto")
{
  string type = outtype;
  if(strcmp(outtype,"auto")==0) {
    type = AutoTypePack(lib);
    if(type == "") return false;
    cout<<"Auto-detected type: "<<type<<endl;
  }
  ResourcePtr r;
  if(type=="Configs") {
    r = PackConfigs(lib);
  }
  else if(type=="Stance") {
    r = PackStance(lib);
  }
  else if(type=="Hold") {
    r = PackHold(lib);
  }
  else if(type=="LinearPath") {
    r = PackLinearPath(lib);
  }
  else if(type=="MultiPath") {
    r = PackMultiPath(lib);
  }
  /*else if(type=="World") {
    r = PackWorld(lib);
  }
  */
  else if(type=="xml") {
    return lib.SaveXml(outname);
  }
  else {
    printf("Unable to pack type %s\n",type.c_str());
    return false;
  }

  if(r == NULL) {
    printf("Error packing type %s\n",type.c_str());
    return false;
  }
  r->name = outname;
  r->fileName = lib.DefaultFileName(r);
  cout<<"Saving to "<<r->fileName<<endl;
  return r->Save();
}

int main(int argc,const char** argv)
{
  if(argc <= 1) {
    printf("Usage: Pack [options] {files_and_directories}\n");
    printf("Options: \n");
    printf(" -u: Unpack one or more files\n");
    printf(" -o name: Specify output name (default out)\n");
    printf(" -t type: Specify output type (default auto)\n");
    return 0;
  }
  bool unpack = false;
  const char* outname = NULL;
  const char* type = "auto";
  int i;
  for(i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-u")) {
	unpack = true;
      }
      else if(0==strcmp(argv[i],"-o")) {
	outname = argv[i+1];
	i++;
      }
      else if(0==strcmp(argv[i],"-t")) {
	type = argv[i+1];
	i++;
      }
      else {
	printf("Unknown option %s",argv[i]);
	return 1;
      }
    }
    else {
      break;
    }
  }
  if(argc - i == 0) {
    printf("No files specified on command line\n");
    return 1;
  }

  MakeCompoundTypes();
  if(unpack == true) {
    for(;i<argc;i++)
      if(!Unpack(argv[i],outname)) return 1;
  }
  else {
    ResourceLibrary lib;
    MakeRobotResourceLibrary(lib);
    if(argc - i == 1) {
      //TODO: possible buffer overflow
      static char buf[1024];
      const char* ext = FileExtension(argv[i]);
      if(ext!=NULL && 0==strcmp(ext,"xml")) {
	if(!lib.LoadXml(argv[i])) {
	  printf("Error loading from XML file %s\n",argv[i]);
	  return 1;
	}
	if(outname == NULL) {
	  strcpy(buf,argv[i]);
	  StripExtension(buf);
	  outname = buf;
	}
      }
      else {
	if(!lib.LoadAll(argv[i])) {
	  printf("Error loading from directory %s\n",argv[i]);
	  return 1;
	}
	if(outname == NULL) {
	  strcpy(buf,argv[i]);
	  //strip trailing slash
	  if(buf[strlen(argv[i])-1]=='/')
	    buf[strlen(argv[i])-1] = 0;
	  outname = buf;
	}
      }
    }
    else {
      for(;i<argc;i++) {
	if(!lib.LoadItem(argv[i])) {
	  printf("Error loading file %s\n",argv[i]);
	  return 1;
	}
      }
    }
    if(outname == NULL) outname="out";
    if(!Pack(lib,outname,type)) return 1;
  }
  return 0;
}
