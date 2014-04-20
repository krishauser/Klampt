#include <resourcemanager.h>
#include <Interface/GenericGUI.h>
#include <Modeling/Resources.h>
#include <string.h>

ResourceTracker::ResourceTracker(ResourcePtr p,ResourceTracker* _parent):
  resource(p),parent(_parent)
{
  //children = new vector<ResourceTracker>();
  dirty= false;
  expanded = false;
}

ResourceManager::ResourceManager(){
  //toplevel = new vector<ResourceTracker>();
  MakeRobotResourceLibrary(library);
  selected=NULL;
  open=NULL;
}

ResourceTracker* ResourceTracker::AddChild(ResourcePtr p){
  ResourceTracker *rt = new ResourceTracker(p);
  children.push_back(rt);
  cout<<"Added Child "<<rt->resource->name<<endl;
  return rt;
}

vector<ResourceTracker*> ResourceTracker::AddChildren(vector<ResourcePtr> ptrs){
  vector<ResourceTracker*> ret;
  for(int i=0;i<ptrs.size();i++){
    ret.push_back(AddChild(ptrs[i]));
  }
  return ret;
}

void ResourceTracker::SetDirty(){
  dirty=true;
  if(parent != NULL) parent->SetDirty();
}


ResourceTracker* ResourceManager::LoadResource(const string& fn){
  ResourcePtr r = library.LoadItem(fn);
  ResourceTracker* rt = new ResourceTracker(r);
  toplevel.push_back(rt);
  return rt;
}

const char* ResourceTracker::Type(){
    return resource->Type();
}

bool ResourceManager::ChangeSelected(ResourceTracker* sel){
  selected = sel;
}

vector<ResourceTracker*> ResourceManager::ExtractSelectedChildren(vector<string> types){
  vector<ResourcePtr> added;
  for(int i=0;i<types.size();i++){
      vector<ResourcePtr> seg = ExtractResources(selected->resource,types[i].c_str());
    added.insert(added.end(),seg.begin(),seg.end());
  }
  return selected->AddChildren(added);
}

vector<ResourceTracker*> ResourceManager::ExpandSelected(){
  vector<string> types;
  if(selected->expanded) return vector<ResourceTracker*>();
    if(strcmp(selected->Type(),"Stance") == 0){
      types.push_back("Hold");
      types.push_back("IKGoal");
    }
    else if(strcmp(selected->Type(),"Hold") == 0){
        //contacts...
        types.push_back("IKGoal");
    }
    else if(strcmp(selected->Type(),"Grasp") == 0){
        types.push_back("Hold");
        types.push_back("Stance");
        types.push_back("IKGoal");
    }
    else if(strcmp(selected->Type(),"LinearPath") == 0){
        types.push_back("Configs");
    }
    else if(strcmp(selected->Type(),"MultiPath") == 0){
        types.push_back("LinearPath");
    }
    else if(strcmp(selected->Type(),"Configs") ==0 ){
        types.push_back("Config");

    }
    else return vector<ResourceTracker*>();
    selected->expanded = true;
    return ExtractSelectedChildren(types);
}

bool ResourceManager::DeleteSelected(){
    if(selected==NULL) return false;
  if(selected->parent != NULL){
    selected->parent->children.erase(std::remove(selected->parent->children.begin(), selected->parent->children.end(),selected),selected->parent->children.end());
    //should do stuff with children
   }
  selected->SetDirty();
  return true;
}

bool ResourceManager::SendSelectedToGUI(){
    open = selected;
}

bool ResourceManager::AddFromPoser(){
}

bool ResourceManager::ReplaceFromPoser(){
}
