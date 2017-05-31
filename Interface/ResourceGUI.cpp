#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "ResourceGUI.h"
#include "IO/XmlWorld.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/ioutils.h>

ResourceGUIBackend::ResourceGUIBackend(RobotWorld* world,ResourceManager* library)
  :WorldGUIBackend(world),resources(library)
{
}


inline bool LoadResources(const char* fn,ResourceTree& lib)
{
  size_t origsize = lib.topLevel.size();
  if(!lib.LoadFolder(fn)) {
    lib.TreeFromLibrary();
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, couldn't load library "<<fn);
    return false;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Loaded "<<lib.topLevel.size()-origsize<<" items from "<<fn);
    return true;
  }
  return false;
}

inline bool LoadResources(TiXmlElement* e,ResourceTree& lib)
{
  size_t origsize = lib.topLevel.size();
  if(!lib.library.Load(e)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, couldn't load library from XML\n");
    lib.TreeFromLibrary();
    return false;
  }
  else {
    lib.TreeFromLibrary();
    LOG4CXX_INFO(KrisLibrary::logger(),"Loaded "<<lib.topLevel.size()-origsize);
    return true;
  }
  return false;
}

inline bool LoadItem(const char* fn,ResourceTree& lib)
{
  ResourceNodePtr r=lib.LoadFile(fn);
  if(r != NULL) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Loaded "<<fn<<" as type "<<r->Type());
    return true;
  }
  else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load resource file "<<fn);
    return false;
  }
}


bool ResourceGUIBackend::LoadCommandLine(int argc,const char** argv)
{
  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-l")) {
	LoadResources(argv[i+1],*resources);
	i++;
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"Unknown option "<<argv[i]);
	return 0;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	TiXmlDocument doc;
	if(!doc.LoadFile(argv[i])) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading XML file "<<argv[i]);
	  return false;
	}
	if(0 == strcmp(doc.RootElement()->Value(),"world")) {
	  XmlWorld xmlWorld;
	  if(!xmlWorld.Load(doc.RootElement(),GetFilePath(argv[i]))) {
	    LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world file "<<argv[i]);
	    return 0;
	  }
	  if(!xmlWorld.GetWorld(*world)) {
	    LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world from "<<argv[i]);
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
	const char* ext = FileExtension(argv[i]);
	if(world->CanLoadElementExt(ext) && world->LoadElement(argv[i])>= 0) {
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



ResourceNodePtr ResourceGUIBackend::Add(ResourcePtr r)
{
  if(resources == NULL) return NULL;
  ResourceNode* parent = (resources->selected != NULL ? resources->selected->parent : NULL);
  if(r->name.empty()) {
      stringstream ss;
      if(parent == NULL){
	ss<<r->Type()<<"["<<resources->topLevel.size()<<"]";
      }
      else{
	ss<<r->Type()<<"["<<parent->children.size()<<"]";
      }
      r->name = ss.str();
  }
  //need to get the reference counted version of parent;
  ResourceNodePtr parentSafe;
  if(parent != NULL) {
    parentSafe = resources->SafePtr(parent);
  }
  last_added = resources->Add(r,parentSafe);
  //SendCommand("new_resource", r->Type()+string(" ")+r->name);
  SendCommand("new_resource", last_added->Identifier());
  return last_added;
}

ResourceNodePtr ResourceGUIBackend::Add(const string& name,const string& type)
{
  if(resources->library.knownTypes.count(type)==0) return NULL;
  ResourcePtr r=resources->library.knownTypes[type][0]->Make();
  r->name = name;
  return Add(r);
}

void ResourceGUIBackend::SaveCur(const string& file)
{
  resources->SaveSelected(file);
}

bool ResourceGUIBackend::LoadNew(const string& file)
{
  return resources->LoadFile(file) != NULL;
}

void ResourceGUIBackend::SaveAll(const string& path)
{
  resources->SaveFolder(path);
}

bool ResourceGUIBackend::LoadAll(const string& path)
{
  if(!resources->LoadFolder(path)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading resources from "<<path.c_str());
    return false;
    }
    else {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Loaded all resources from "<<path.c_str());
    return true;
  }
  return false;
}


void ResourceGUIBackend::SetLastActive()
{
  if(last_added == NULL) {
    resources->selected = NULL;
  }
  else {
    resources->selected = last_added;
  }
}


void ResourceGUIBackend::SetActive(const string& identifier)
{
  resources->Select(identifier);
}

void ResourceGUIBackend::SetPathTime(double time){
  viewResource.pathTime = time;
}

bool ResourceGUIBackend::OnCommand(const string& cmd,const string& args)
{
  if(cmd == "set_resource") {
    resources->Select(args);
    return true;      
  }
  else if(cmd == "get_resource") {
    stringstream ss;
    ss<<resources->selected->Identifier();
    SendCommand("current_resource",ss.str());
    return true;
  }
  else if(cmd == "set_resource_name") {
    if(resources->selected) resources->selected->resource->name = args;
    LOG4CXX_INFO(KrisLibrary::logger(),"Updating name to "<<args.c_str());
  }
  else if(cmd == "delete_resource") {
    resources->DeleteSelected();
  }
  else if(cmd == "add_resource") {
    string type,name;
    stringstream ss(args);
    if(!SafeInputString(ss,type)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading resource type from args \""<<args<<"\""<<"\n");
      return true;
    }
    if(!SafeInputString(ss,name)) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading resource name from args \""<<args<<"\""<<"\n");
      return true;
    }
    last_added = Add(name,type);
  }
  else if(cmd == "load_resource") {
    if(LoadNew(args)) {
      SendCommand("new_resource",last_added->Identifier());
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
  else if(cmd == "save_resource_dir") {
    SaveAll(args);
  }
  else if(cmd == "convert") {
    ResourcePtr r=CurrentResource();
    ResourcePtr res = CastResource(r,args.c_str());
    if(!res) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Conversion failed\n");
      return true;
    }
    Add(res);     
    SetLastActive();
  }
  else if(cmd == "extract") {
    ResourcePtr r=CurrentResource();
    if(!r) return true;
    vector<ResourcePtr> res = ExtractResources(r,args.c_str());
    for(size_t i=0;i<res.size();i++){
      Add(res[i]);     
    }
    if(!res.empty())
      SetLastActive();
  }
  else if(cmd == "set_path_time") {
    stringstream ss(args);
    double time;
    ss>>time;
    SetPathTime(time);
    SendRefresh();
  }
  else {
    return WorldGUIBackend::OnCommand(cmd,args);
  }
  return true;
}


ResourcePtr ResourceGUIBackend::CurrentResource()
{
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
  viewResource.SetRobot(world->robots[0]);
  if(resources && resources->selected)
    viewResource.DrawGL(resources->selected->resource);
}
