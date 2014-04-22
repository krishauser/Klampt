#include "ResourceGUI.h"
#include "utils/stringutils.h"
#include "utils/ioutils.h"
#include "IO/XmlWorld.h"

ResourceGUIBackend::ResourceGUIBackend(RobotWorld* world,ResourceLibrary* library)
  :WorldGUIBackend(world),resources(library)
{
  cur_resource_type = "";
  cur_resource_name = "";
}


inline bool LoadResources(const char* fn,ResourceLibrary& lib)
{
  size_t origsize = lib.itemsByName.size();
  if(!lib.LoadAll(fn)) {
    fprintf(stderr,"Warning, couldn't load library %s\n",fn);
    return false;
  }
  else {
    printf("Loaded %d items from %s\n",lib.itemsByName.size()-origsize,fn);
    return true;
  }
}

inline bool LoadResources(TiXmlElement* e,ResourceLibrary& lib)
{
  size_t origsize = lib.itemsByName.size();
  if(!lib.Load(e)) {
    fprintf(stderr,"Warning, couldn't load library from XML\n");
    return false;
  }
  else {
    printf("Loaded %d items from XML\n",lib.itemsByName.size()-origsize);
    return true;
  }
}

inline bool LoadItem(const char* fn,ResourceLibrary& lib)
{
  ResourcePtr r=lib.LoadItem(fn);
  if(r) {
    printf("Loaded %s as type %s\n",fn,r->Type());
    lib.Add(r);
    return true;
  }
  else {
    fprintf(stderr,"Couldn't load resource file %s\n",fn);
    return false;
  }
}


bool ResourceGUIBackend::LoadCommandLine(int argc,char** argv)
{
  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-l")) {
	LoadResources(argv[i+1],*resources);
	i++;
      }
      else {
	printf("Unknown option %s",argv[i]);
	return 0;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	TiXmlDocument doc;
	if(!doc.LoadFile(argv[i])) {
	  printf("Error loading XML file %s\n",argv[i]);
	  return false;
	}
	if(0 == strcmp(doc.RootElement()->Value(),"world")) {
	  XmlWorld xmlWorld;
	  if(!xmlWorld.Load(doc.RootElement(),GetFilePath(argv[i]))) {
	    printf("Error loading world file %s\n",argv[i]);
	    return 0;
	  }
	  if(!xmlWorld.GetWorld(*world)) {
	    printf("Error loading world from %s\n",argv[i]);
	    return 0;
	  }
	}
	else if(0 == strcmp(doc.RootElement()->Value(),"resource_library")) {
	  LoadResources(doc.RootElement(),*resources);
	}
	else {
	  if(!LoadItem(argv[i],*resources))
	    return 0;
	}
      }
      else {
	//try loading into the world
	if(world->LoadElement(argv[i])>= 0) {
	  //loaded successfully
	}
	else {
	  //failed, now try resource library load
	  if(!LoadItem(argv[i],*resources))
	    return 0;
	}
      }
    }
  }
  return true;
}



void ResourceGUIBackend::Add(ResourcePtr r)
{
  if(r->name.empty()) {
    stringstream ss;
    ss<<r->Type()<<"["<<resources->CountByType(r->Type())<<"]";
    r->name = ss.str();
  }
  last_added = r;
  resources->Add(r);
}

ResourcePtr ResourceGUIBackend::Add(const string& name,const string& type)
{
  if(resources->knownTypes.count(type)==0) return NULL;
  ResourcePtr r=resources->knownTypes[type][0]->Make();
  r->name = name;
  Add(r);
  return r;
}

void ResourceGUIBackend::SaveCur(const string& file)
{
  ResourcePtr r=CurrentResource();
  if(file.empty()) {
    r->fileName = resources->DefaultFileName(r);
  }
  else
    r->fileName = file;

  if(!r->Save()) 
    fprintf(stderr,"Unable to save %s to %s\n",r->name.c_str(),r->fileName.c_str());
  else {
    printf("Saved %s to %s\n",r->name.c_str(),r->fileName.c_str());
  }
}

bool ResourceGUIBackend::LoadNew(const string& file)
{
  ResourcePtr r=resources->LoadItem(file);
  if(!r) {
    fprintf(stderr,"Unable to load resource from  %s\n",file.c_str());
    return false;
  }
  else {
    printf("Loaded %s from %s\n",r->name.c_str(),file.c_str());
    return true;
  }
}

void ResourceGUIBackend::SaveAll(const string& path)
{
  for(ResourceLibrary::Map::iterator i=resources->itemsByType.begin();i!=resources->itemsByType.end();i++) {
    for(size_t j=0;j<i->second.size();j++)
      if(i->second[j]->fileName.empty())
	i->second[j]->fileName = resources->DefaultFileName(i->second[j]);
  }
  resources->ChangeBaseDirectory(path);
  if(!resources->SaveAll()) 
    fprintf(stderr,"Unable to save all resources to %s\n",path.c_str());
  else
    fprintf(stderr,"Saved all resources to %s\n",path.c_str());
}

bool ResourceGUIBackend::LoadAll(const string& path)
{
  if(!resources->LoadAll(path)) {
    fprintf(stderr,"Error loading resources from %s\n",path.c_str());
    return false;
  }
  else {
    fprintf(stderr,"Loaded all resources from %s\n",path.c_str());
    return true;
  }
}

void ResourceGUIBackend::SetLastActive()
{
  if(last_added == NULL) {
    cur_resource_type = "";
    cur_resource_name = "";
  }
  else {
    SetActive(last_added->Type(),last_added->name);
  }
}


void ResourceGUIBackend::SetActive(const string& type,const string& name)
{
  cur_resource_type = type;
  cur_resource_name = name;
}

bool ResourceGUIBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd == "set_resource") {
    stringstream ss;
    string type,name;
    if(!SafeInputString(ss,type)) {
      cout<<"Error reading resource type from "<<args<<endl;
      return true;
    }
    if(!SafeInputString(ss,name)) {
      cout<<"Error reading resource name from "<<args<<endl;
      return true;
    }
    cur_resource_type = type;
    cur_resource_name = name;
  }
  else if(cmd == "get_resource") {
    stringstream ss;
    ss<<cur_resource_type<<" "<<cur_resource_name;
    SendCommand("current_resource",ss.str());
    return true;
  }
  else if(cmd == "set_resource_name") {
    CurrentResource()->name = args;
    printf("Updating name to %s\n",args.c_str());
  }
  else if(cmd == "delete_resource") {
    string type = CurrentResource()->Type();
    resources->Erase(CurrentResource());
  }
  else if(cmd == "add_resource") {
    string type,name;
    stringstream ss(args);
    if(!SafeInputString(ss,type)) {
      cout<<"Error reading resource type from "<<args<<endl;
      return true;
    }
    if(!SafeInputString(ss,name)) {
      cout<<"Error reading resource name from "<<args<<endl;
      return true;
    }
    last_added = Add(name,type);
  }
  else if(cmd == "load_resource") {
    if(LoadNew(args)) {
      stringstream ss;
      ss<<last_added->Type()<<" "<<last_added->name;
      SendCommand("new_resource",ss.str());
    }
  }
  else if(cmd == "save_resource") {
    SaveCur(args);
  }
  else if(cmd == "load_resource_dir") {
    if(LoadAll(args)) {
      SendCommand("refresh_resources","");
    }
  }
  else if(cmd == "save_resouce_dir") {
    SaveAll(args);
  }
  else if(cmd == "convert") {
    ResourcePtr r=CurrentResource();
    ResourcePtr res = CastResource(r,args.c_str());
    if(!res) {
      fprintf(stderr,"Conversion failed\n");
      return true;
    }
    res->name = r->name;
    Add(res);     
    SetLastActive();
  }
  else if(cmd == "extract") {
    ResourcePtr r=CurrentResource();
    if(!r) return true;
    vector<ResourcePtr> res = ExtractResources(r,args.c_str());
    for(size_t i=0;i<res.size();i++)
      Add(res[i]);     
    if(!res.empty())
      SetLastActive();
  }
  else if(cmd == "set_path_time") {
    stringstream ss(args);
    ss>>viewResource.pathTime;
  }
  else {
    return WorldGUIBackend::OnCommand(cmd,args);
  }
  return true;
}


ResourcePtr ResourceGUIBackend::CurrentResource()
{
  if(resources->itemsByType.count(cur_resource_type) == 0) return 0;
  vector<ResourcePtr >& v=resources->itemsByType[cur_resource_type];
  for(size_t i=0;i<v.size();i++)
    if(v[i]->name == cur_resource_name)
      return v[i];
  return NULL;
}

void ResourceGUIBackend::RenderCurResource()
{
  viewResource.DrawGL(CurrentResource());
}
