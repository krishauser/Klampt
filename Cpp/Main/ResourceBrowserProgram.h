#ifndef RESOURCE_BROWSER_PROGRAM_H
#define RESOURCE_BROWSER_PROGRAM_H

#include "WorldViewProgram.h"
#include "Modeling/Resources.h"
#include "View/ViewResource.h"
#include "IO/XmlWorld.h"
#include <GL/glui.h>
#include <sstream>
using namespace Math3D;
using namespace GLDraw;
using namespace Klampt;

//GLUI widget ID's

#ifndef RESOURCE_BROWSER_GLUI_ID_START
#define RESOURCE_BROWSER_GLUI_ID_START 0
#endif // RESOURCE_BROWSER_GLUI_ID_START

enum {
  RESOURCE_TYPE_LISTBOX_ID = RESOURCE_BROWSER_GLUI_ID_START,
  RESOURCE_NAME_LISTBOX_ID,
  RESOURCE_NAME_EDITBOX_ID,
  RESOURCE_DELETE_BUTTON_ID,
  RESOURCE_CONVERT_BUTTON_ID,
  RESOURCE_EXTRACT_BUTTON_ID,
  RESOURCE_SAVE_FILE_BUTTON_ID,
  RESOURCE_LOAD_FILE_BUTTON_ID,
  RESOURCE_SAVE_ALL_BUTTON_ID,
  RESOURCE_LOAD_ALL_BUTTON_ID,
};

#define RESOURCE_BROWSER_GLUI_ID_END RESOURCE_BROWSER_GLUI_ID_START+10

///Natively supports configs, paths, transforms, ik goals, holds, stance,
///trimeshes
class ResourceBrowserProgram : public WorldViewProgram
{
public:
  ResourceBrowserProgram(WorldModel* world);
  bool LoadCommandLine(int argc,char** argv);

  //subclass will call this to add controls
  void AddGLUIControls(GLUI* glui,bool rollout=false);
  void Add(ResourcePtr r);
  template <class T>
  void Add(const string& name,const T& val) { Add(MakeResource(name,val)); }
  void Add(const string& name,const vector<Real>& ts,const vector<Config>& qs) { Add(MakeResource(name,ts,qs)); }
  //save current resource to disk
  void SaveCur(const string& file);
  //load a new resource from the given file
  void LoadNew(const string& file);
  //save all resources to a given path / XML file
  void SaveAll(const string& path);
  //load all resources in a given path / XML file
  void LoadAll(const string& path);
  //sets the active item to the last added item
  void SetLastActive(); 
  //sets the active item to the one of the given type/name
  void SetActive(const string& name);
  void SetActive(const string& type,const string& name);

  //subclasses should call this so that the GLUI controls will work
  virtual void Handle_Control(int id);
  //subclasses should call this to register new types
  void AddType(ResourcePtr res,const char* typeDisplay="");

  ResourcePtr CurrentResource();
  void RenderCurResource();

  //helpers
  void RefreshTypes();
  void RefreshNames();
  void RefreshNames(const string& type);
  void RefreshCurrent();

  //stores all resources
  ResourceLibrary resources;
  //choosing resource types and names
  GLUI_Listbox* resource_type_listbox;
  GLUI_Listbox* resource_name_listbox;
  vector<string> resource_types;
  vector<vector<string> > resource_names;
  int cur_resource_type,cur_resource_name;
  //for edit boxes
  string resource_name,resource_file_name;
  map<string,string> typeToDisplay;

  ResourcePtr last_added;

  ViewResource viewResource;
};

ResourceBrowserProgram::ResourceBrowserProgram(WorldModel* world)
  :WorldViewProgram(world)
{
  MakeRobotResourceLibrary(resources);
  cur_resource_type = 0;
  cur_resource_name = 0;
  typeToDisplay["LinearPath"] = "Linear path";
  typeToDisplay["IKGoal"] = "IK goal";
  typeToDisplay["RigidTransform"] = "Rigid transform";
  typeToDisplay["Configs"] = "Config set";
  resource_type_listbox=NULL;
  resource_name_listbox=NULL;
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


bool ResourceBrowserProgram::LoadCommandLine(int argc,char** argv)
{
  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-l")) {
	LoadResources(argv[i+1],resources);
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
	  LoadResources(doc.RootElement(),resources);
	}
	else {
	  if(!LoadItem(argv[i],resources))
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
	  if(!LoadItem(argv[i],resources))
	    return 0;
	}
      }
    }
  }
  return true;
}

void ResourceBrowserProgram::AddGLUIControls(GLUI* glui,bool rollout)
{
  GLUI_Panel* panel = (rollout ? glui->add_rollout("Resources") : glui->add_panel("Resources"));
  resource_type_listbox = glui->add_listbox_to_panel(panel,"Type",&cur_resource_type,RESOURCE_TYPE_LISTBOX_ID,ControlFunc);
  resource_name_listbox = glui->add_listbox_to_panel(panel,"Item",&cur_resource_name,RESOURCE_NAME_LISTBOX_ID,ControlFunc);

  glui->add_edittext_to_panel(panel,"Name",GLUI_EDITTEXT_STRING,&resource_name,RESOURCE_NAME_EDITBOX_ID,ControlFunc)->set_alignment(GLUI_ALIGN_LEFT);
  glui->add_button_to_panel(panel,"Delete",RESOURCE_DELETE_BUTTON_ID,ControlFunc);
  glui->add_button_to_panel(panel,"Convert",RESOURCE_CONVERT_BUTTON_ID,ControlFunc);
  glui->add_button_to_panel(panel,"Extract",RESOURCE_EXTRACT_BUTTON_ID,ControlFunc);

  glui->add_column_to_panel(panel,true);
  glui->add_edittext_to_panel(panel,"File",GLUI_EDITTEXT_STRING,&resource_file_name)->set_alignment(GLUI_ALIGN_LEFT);
  glui->add_button_to_panel(panel,"Load file",RESOURCE_LOAD_FILE_BUTTON_ID,ControlFunc);
  glui->add_button_to_panel(panel,"Save file",RESOURCE_SAVE_FILE_BUTTON_ID,ControlFunc);
  glui->add_button_to_panel(panel,"Load all",RESOURCE_LOAD_ALL_BUTTON_ID,ControlFunc);
  glui->add_button_to_panel(panel,"Save all",RESOURCE_SAVE_ALL_BUTTON_ID,ControlFunc);

  RefreshTypes();
  RefreshNames();
  RefreshCurrent();
}

void delete_all(GLUI_Listbox* listbox)
{
  while(listbox->items_list.first_child()) {
    listbox->delete_item(((GLUI_Listbox_Item*)listbox->items_list.first_child())->id);
  }
}

void ResourceBrowserProgram::RefreshTypes()
{
  resource_types.resize(0);
  delete_all(resource_type_listbox);
  int k=0;
  for(ResourceLibrary::Map::const_iterator i=resources.itemsByType.begin();i!=resources.itemsByType.end();i++,k++) {
    resource_types.push_back(i->first);
    map<string,string>::iterator it=typeToDisplay.find(i->first);
    if(it == typeToDisplay.end()) 
      resource_type_listbox->add_item(k,i->first.c_str());
    else
      resource_type_listbox->add_item(k,it->second.c_str());
  }
}

void ResourceBrowserProgram::RefreshNames()
{
  resource_names.resize(0);
  for(ResourceLibrary::Map::const_iterator i=resources.itemsByType.begin();i!=resources.itemsByType.end();i++) {
    resource_names.resize(resource_names.size()+1);
    resource_names.back().resize(i->second.size());
    for(size_t j=0;j<i->second.size();j++)
      resource_names.back()[j] = i->second[j]->name;
  }
  delete_all(resource_name_listbox);
  if(cur_resource_type < (int)resource_types.size()) {
    for(size_t j=0;j<resource_names[cur_resource_type].size();j++)
      resource_name_listbox->add_item(j,resource_names[cur_resource_type][j].c_str());
  }
}

void ResourceBrowserProgram::RefreshNames(const string& type)
{
  int k=0;
  for(ResourceLibrary::Map::const_iterator i=resources.itemsByType.begin();i!=resources.itemsByType.end();i++,k++) {
    if(i->first == type) {
      resource_names[k].resize(i->second.size());
      for(size_t j=0;j<i->second.size();j++)
	resource_names[k][j] = i->second[j]->name;
      break;
    }
  }
  if(cur_resource_type == k) {
    delete_all(resource_name_listbox);
    for(size_t j=0;j<resource_names[cur_resource_type].size();j++)
      resource_name_listbox->add_item(j,resource_names[cur_resource_type][j].c_str());
  }
}


void ResourceBrowserProgram::Add(ResourcePtr r)
{
  if(r->name.empty()) {
    stringstream ss;
    ss<<r->Type()<<"["<<resources.CountByType(r->Type())<<"]";
    r->name = ss.str();
  }
  last_added = r;

  bool newType = (resources.CountByType(r->Type())==0);
  resources.Add(r);

  //refresh GUI if necessary
  if(!resource_name_listbox) return;

  if(newType) {
    RefreshTypes();
    RefreshNames();
  }
  else {
    cur_resource_type = 0;
    for(size_t i=0;i<resource_types.size();i++)
      if(resource_types[i] == r->Type()) cur_resource_type = (int)i;
    RefreshNames(r->Type());
  }
}

void ResourceBrowserProgram::SaveCur(const string& file)
{
  ResourcePtr r=CurrentResource();
  if(file.empty()) {
    r->fileName = resources.DefaultFileName(r);
  }
  else
    r->fileName = file;

  if(!r->Save()) 
    fprintf(stderr,"Unable to save %s to %s\n",r->name.c_str(),r->fileName.c_str());
  else {
    printf("Saved %s to %s\n",r->name.c_str(),r->fileName.c_str());
  }
}

void ResourceBrowserProgram::LoadNew(const string& file)
{
  size_t oldcount = resources.itemsByType.size();
  ResourcePtr r=resources.LoadItem(file);
  if(!r) {
    fprintf(stderr,"Unable to load resource from  %s\n",file.c_str());
    return;
  }
  else {
    printf("Loaded %s from %s\n",r->name.c_str(),file.c_str());
  }

  if(resources.itemsByType.size() != oldcount) {
    RefreshTypes();
    for(size_t i=0;i<resource_types.size();i++)
      if(resource_types[i] == r->Type()) cur_resource_type = (int)i;
    RefreshNames();
  }
  else {
    RefreshNames(r->Type());
  }
}

void ResourceBrowserProgram::SaveAll(const string& path)
{
  for(ResourceLibrary::Map::iterator i=resources.itemsByType.begin();i!=resources.itemsByType.end();i++) {
    for(size_t j=0;j<i->second.size();j++)
      if(i->second[j]->fileName.empty())
	i->second[j]->fileName = resources.DefaultFileName(i->second[j]);
  }
  resources.ChangeBaseDirectory(path);
  if(!resources.SaveAll()) 
    fprintf(stderr,"Unable to save all resources to %s\n",path.c_str());
  else
    fprintf(stderr,"Saved all resources to %s\n",path.c_str());
}

void ResourceBrowserProgram::LoadAll(const string& path)
{
  if(!resources.LoadAll(path))
    fprintf(stderr,"Error loading resources from %s\n",path.c_str());
  else
    fprintf(stderr,"Loaded all resources from %s\n",path.c_str());
  RefreshTypes();
  RefreshNames();
}

void ResourceBrowserProgram::SetLastActive()
{
  if(last_added == NULL) {
    cur_resource_type = -1;
    cur_resource_name = -1;
    RefreshNames();
    RefreshCurrent();
  }
  else {
    SetActive(last_added->Type(),last_added->name);
  }
}

void ResourceBrowserProgram::SetActive(const string& name)
{
  if(resources.Count(name)==0) {
    cur_resource_name = -1;
    RefreshCurrent();
  }
  else {
    ResourcePtr r=resources.itemsByName[name][0];
    SetActive(r->Type(),r->name);
  }
}


void ResourceBrowserProgram::SetActive(const string& type,const string& name)
{
  if(resources.CountByType(type)==0) {
    cur_resource_type = -1;
    cur_resource_name = -1;
    RefreshNames();
    RefreshCurrent();
  }
  else {
    int old_resource_type = cur_resource_type;
    cur_resource_type = -1;
    cur_resource_name = -1;
    for(size_t i=0;i<resource_types.size();i++) {
      if(resource_types[i] == type) {
	cur_resource_type = (int)i;
	for(size_t j=0;j<resource_names[i].size();j++)
	  if(resource_names[i][j] == name) {
	    cur_resource_name = (int)j;
	    break;
	  }
      }
    }
    if(cur_resource_type != old_resource_type) { //fix the names listbox
      delete_all(resource_name_listbox);
      for(size_t j=0;j<resource_names[cur_resource_type].size();j++) {
	resource_name_listbox->add_item(j,resource_names[cur_resource_type][j].c_str());
      }
    }
    RefreshCurrent();
  }
}

void ResourceBrowserProgram::RefreshCurrent()
{
  //update name box
  ResourcePtr r=CurrentResource();
  if(r) {
    resource_name = r->name;
    resource_file_name = r->fileName;
  }
  else {
    resource_name = "";
  }
  viewResource.pathTime=0;
  GLUI_Master.sync_live_all();
}

void ResourceBrowserProgram::Handle_Control(int id)
{
  switch(id) {
  case RESOURCE_TYPE_LISTBOX_ID:
    {
      cur_resource_name = 0;
      delete_all(resource_name_listbox);
      for(size_t j=0;j<resource_names[cur_resource_type].size();j++) {
	resource_name_listbox->add_item(j,resource_names[cur_resource_type][j].c_str());
      }
      RefreshCurrent();
    } 
    break;
  case RESOURCE_NAME_LISTBOX_ID:
    {
      RefreshCurrent();
    }
    break;
  case RESOURCE_NAME_EDITBOX_ID:
    {
      CurrentResource()->name = resource_name;
      printf("Updating name to %s\n",resource_name.c_str());
      //TODO: update listbox
    }
    break;
  case RESOURCE_DELETE_BUTTON_ID:
    {
      string type = CurrentResource()->Type();
      resources.Erase(CurrentResource());
      if(cur_resource_name >= (int)resource_names[cur_resource_type].size()) {
	cur_resource_name--;
      }
      RefreshNames(type);
      RefreshCurrent();
    }
    break;
  case RESOURCE_CONVERT_BUTTON_ID:
    {
      ResourcePtr r=CurrentResource();
      if(!r) return;
      ResourcePtr res;
      string type;
      cout<<"To what type? > ";
      cin >> type;
      cin.ignore(1024,'\n');
      res = CastResource(r,type.c_str());
      if(!res) {
	fprintf(stderr,"Conversion failed\n");
	return;
      }
      res->name = r->name;
      Add(res);     
      SetLastActive();
    }
    break;
  case RESOURCE_EXTRACT_BUTTON_ID:
    {
      ResourcePtr r=CurrentResource();
      if(!r) return;
      vector<ResourcePtr> res;
      string type;
      cout<<"To what type? > ";
      cin >> type;
      cin.ignore(1024,'\n');
      //string type = resource_types[cur_resource_type];
      res = ExtractResources(r,type.c_str());
      for(size_t i=0;i<res.size();i++)
	Add(res[i]);     
      if(!res.empty())
	SetLastActive();
    }
    break;
  case RESOURCE_SAVE_ALL_BUTTON_ID:
    SaveAll(resource_file_name);
    break;
  case RESOURCE_LOAD_ALL_BUTTON_ID:
    LoadAll(resource_file_name);
    break;
  case RESOURCE_SAVE_FILE_BUTTON_ID:
    SaveCur(resource_file_name);
    break;
  case RESOURCE_LOAD_FILE_BUTTON_ID:
    LoadNew(resource_file_name);
    break;
  }
}

void ResourceBrowserProgram::AddType(ResourcePtr res,const char* typeDisplay)
{
  string curType;
  if(cur_resource_type < (int)resource_types.size())
    curType = resource_types[cur_resource_type];

  resources.itemsByType[res->Type()] = vector<ResourcePtr >();
  if(typeDisplay != NULL)
    typeToDisplay[res->Type()]=typeDisplay;

  RefreshTypes();
  for(size_t i=0;i<resource_types.size();i++)
    if(curType == resource_types[i])
      cur_resource_type = i;
  RefreshNames();
  GLUI_Master.sync_live_all();
}

ResourcePtr ResourceBrowserProgram::CurrentResource()
{
  if(cur_resource_type < 0 || cur_resource_type >= (int)resource_types.size()) return NULL;
  vector<ResourcePtr >& v=resources.itemsByType[resource_types[cur_resource_type]];
  if(cur_resource_name < 0 || cur_resource_name >= (int)v.size()) return NULL;
  return v[cur_resource_name];
}

void ResourceBrowserProgram::RenderCurResource()
{
  viewResource.DrawGL(CurrentResource());
}


#endif
