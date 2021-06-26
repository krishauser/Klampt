#include "Modeling/Resources.h"
#include <KrisLibrary/utils/stringutils.h>
#include <fstream>
#include <algorithm>
#include <map>
#include <vector>
#include <string.h>
using namespace std;
using namespace Klampt;

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

bool Pack(ResourceLibrary& lib,const char* outname,const char* outtype="auto")
{
  string type = outtype;
  if(strcmp(outtype,"auto")==0) {
    type = AutoTypePack(lib);
    if(type == "") return false;
    cout<<"Auto-detected type: "<<type<<endl;
  }
  ResourcePtr r;
  if(type=="xml") {
    return lib.SaveXml(outname);
  }
  else {
    string errorMessage;
    r = PackResources(lib,type,&errorMessage);
    if(!r) {
      printf("Unable to pack type %s\n",type.c_str());
      printf("  Error message: %s\n",errorMessage.c_str());
      return false;
    }
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
      static char buf[1024];
      buf[1023] = 0;
      const char* ext = FileExtension(argv[i]);
      if(ext!=NULL && 0==strcmp(ext,"xml")) {
	if(!lib.LoadXml(argv[i])) {
	  printf("Error loading from XML file %s\n",argv[i]);
	  return 1;
	}
	if(outname == NULL) {
	  strncpy(buf,argv[i],1023);
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
    strncpy(buf, argv[i], 1023);
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
