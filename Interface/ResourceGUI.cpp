#include "ResourceGUI.h"
#include "utils/stringutils.h"
#include "utils/ioutils.h"
#include "IO/XmlWorld.h"

ResourceGUIBackend::ResourceGUIBackend(RobotWorld* world,ResourceManager* library)
  :WorldGUIBackend(world),resources(library)
{
  cur_resource_type = "";
  cur_resource_name = "";
}


inline bool LoadResources(const char* fn,ResourceManager& lib)
{
  size_t origsize = lib.size();
  //TODO
  /*
  if(!lib.LoadAllItems(fn)) {
    fprintf(stderr,"Warning, couldn't load library %s\n",fn);
    return false;
  }
  else {
    printf("Loaded %d items from %s\n",lib.itemsByName.size()-origsize,fn);
    return true;
  }
  */return false;
}

inline bool LoadResources(TiXmlElement* e,ResourceManager& lib)
{
  //TODO
  /*
  size_t origsize = lib.itemsByName.size();
  if(!lib.Load(e)) {
    fprintf(stderr,"Warning, couldn't load library from XML\n");
    return false;
  }
  else {
    printf("Loaded %d items from XML\n",lib.itemsByName.size()-origsize);
    return true;
    }*/return false;
}

inline bool LoadItem(const char* fn,ResourceManager& lib)
{
  ResourceNode r=lib.LoadResource(fn);
  if(r != NULL) {
    printf("Loaded %s as type %s\n",fn,r->Type());
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
//      if(resources == NULL) return;
      if(resources->selected.isNull()){
          ss<<r->Type()<<"["<<resources->toplevel.size()<<"]";
      }
      else{
          ss<<"["<<resources->selected->children.size()<<"]";
      }
      r->name = ss.str();
  }
  last_added = r;
  resources->AddAsChild(r);
  SendCommand("inform_new_resource", r->name);
}

ResourcePtr ResourceGUIBackend::Add(const string& name,const string& type)
{
  if(resources->library.knownTypes.count(type)==0) return NULL;
  ResourcePtr r=resources->library.knownTypes[type][0]->Make();
  r->name = name;
  Add(r);
  return r;
}

void ResourceGUIBackend::SaveCur(const string& file)
{
  resources->SaveSelected(file);
}

bool ResourceGUIBackend::LoadNew(const string& file)
{
  return resources->LoadResource(file) != NULL;
}

void ResourceGUIBackend::SaveAll(const string& path)
{
  /*
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
  */
}

bool ResourceGUIBackend::LoadAll(const string& path)
{
  //TODO
  /*
  if(!resources->LoadAll(path)) {
    fprintf(stderr,"Error loading resources from %s\n",path.c_str());
    return false;
  }
  else {
    fprintf(stderr,"Loaded all resources from %s\n",path.c_str());
    return true;
  }
  */
  return false;
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

void ResourceGUIBackend::SetPathTime(double time){
  viewResource.pathTime = time;
}

bool ResourceGUIBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd == "set_resource") {
    stringstream ss(args);
    string name;
    if(!SafeInputString(ss,name)) {
      cout<<"Error reading resource name from "<<args<<endl;
      return true;
    }
    resources->ChangeSelected(name);
    if(resources->selected){
      const LinearPathResource* rc=dynamic_cast<const LinearPathResource*>((const ResourceBase*)resources->selected->resource);
      if(rc){
	stringstream ss;
	ss<<rc->times.back()<<" "<<rc->times.front();
	SendCommand("enable_path",ss.str());
      }
      else{
	const MultiPathResource* rc=dynamic_cast<const MultiPathResource*>((const ResourceBase*)resources->selected->resource);
	if(rc){
	  Real minTime = 0, maxTime = 1;
	  if(rc->path.HasTiming()) {
	    minTime = rc->path.sections.front().times.front();
	    maxTime = rc->path.sections.back().times.back();
	  }
	  stringstream ss;
	  ss<<maxTime<<" "<<minTime;
	  SendCommand("enable_path",ss.str());
	}
      }
    }
    return true;      
  }
  else if(cmd == "get_resource") {
    stringstream ss;
    ss<<resources->selected->Type()<<" "<<resources->selected->Name();
    SendCommand("current_resource",ss.str());
    return true;
  }
  else if(cmd == "set_resource_name") {
    resources->ChangeSelectedName(args);
    printf("Updating name to %s\n",args.c_str());
  }
  else if(cmd == "delete_resource") {
    string type = CurrentResource()->Type();
    resources->DeleteSelected();
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
    for(size_t i=0;i<res.size();i++){
      Add(res[i]);     
      stringstream ss;
      ss<<r->name<<" "<<r->Type();
      SendCommand("resource_added",ss.str());
    }
    if(!res.empty())
      SetLastActive();
  }
  else if(cmd == "set_path_time") {
    stringstream ss(args);
    double time;
    ss>>time;
    SetPathTime(time);
  }
  else {
    return WorldGUIBackend::OnCommand(cmd,args);
  }
  return true;
}


ResourcePtr ResourceGUIBackend::CurrentResource()
{
  //if(resources->itemsByName.count(cur_resource_type) == 0)
  //      return 0;
    if(resources && resources->selected)
        return resources->selected->resource;
    else return 0;
}

void ResourceGUIBackend::RenderCurResource()
{
  /*
    ResourcePtr current = CurrentResource();
    if(current)
        viewResource.DrawGL(current);
  */
  //separate open?
  viewResource.SetRobot(world->robots[0].robot);
  if(resources && resources->selected)
    viewResource.DrawGL(resources->selected->resource);
}
