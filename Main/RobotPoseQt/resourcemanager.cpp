#include <resourcemanager.h>
#include <Modeling/Resources.h>
#include <string.h>

#include <boost/foreach.hpp>

ResourceTracker::ResourceTracker(ResourcePtr p,ResourceNode _parent):
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

ResourceNode ResourceTracker::AddChild(ResourcePtr p){
  ResourceNode rt = new ResourceTracker(p);
  children.push_back(rt);
  cout<<"Added Child "<<rt->resource->name<<endl;
  return rt;
}

vector<ResourceNode> ResourceTracker::AddChildren(vector<ResourcePtr> ptrs){
  vector<ResourceNode> ret;
  for(int i=0;i<ptrs.size();i++){
    ret.push_back(AddChild(ptrs[i]));
  }
  return ret;
}

void ResourceTracker::SetDirty(){
    if(!dirty){
      dirty=true;
    if(parent != NULL) parent->SetDirty();
    }
}

ResourceNode ResourceManager::LoadResource(const string& fn){
  ResourcePtr r = library.LoadItem(fn);
  if(r==NULL) return NULL;
  ResourceNode rt = new ResourceTracker(r);
  itemsByName.insert(pair<string,ResourceNode>(rt->Name(),rt));
  toplevel.push_back(rt);
  return rt;
}

/*
ResourceNode ResourceManager::LoadResources(const string& fn){
    library.LoadAll(fn);
}
*/

int ResourceManager::size(){
  return itemsByName.size();
}

bool ResourceManager::SaveSelected(const string &file)
{
    ResourcePtr r = selected->resource;
    if(file.empty()){
        r->fileName = library.DefaultFileName(r);
    }
    else
        r->fileName = file;
  if(!r->Save()){
    fprintf(stderr,"Unable to save %s to %s\n",r->name.c_str(),r->fileName.c_str());
    return true;
  }
  else {
    printf("Saved %s to %s\n",r->name.c_str(),r->fileName.c_str());
    return false;
  }
}

bool ResourceManager::ChangeSelected(ResourceNode sel){
  selected = sel;
}

bool ResourceManager::ChangeSelectedName(string name){
  itemsByName.erase(selected->Name());
  selected->resource->name = name;
  itemsByName[name] = selected;  
}

bool ResourceManager::AddAsChild(ResourcePtr r){
    ResourceNode rt = selected->AddChild(r);
    rt->SetDirty();
    itemsByName.insert(pair<string,ResourceNode>(rt->Name(),rt));
}

vector<ResourceNode> ResourceManager::ExtractSelectedChildren(vector<string> types){
  vector<ResourcePtr> added;
  for(int i=0;i<types.size();i++){
      vector<ResourcePtr> seg = ExtractResources(selected->resource,types[i].c_str());
    added.insert(added.end(),seg.begin(),seg.end());
  }
  vector<ResourceNode> addedNodes =  selected->AddChildren(added);
  BOOST_FOREACH(ResourceNode node,addedNodes){
    itemsByName.insert(pair<string,ResourceNode>(node->Name(),node));
  }
  return addedNodes;
}

vector<ResourceNode> ResourceManager::ExpandSelected(){
  vector<string> types;
  if(selected->expanded){
    printf("Selected resource is already expanded");
    return vector<ResourceNode>();
  }
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
    else{
      printf("No expansion known for type %s",selected->Type());
      return vector<ResourceNode>();
    }
    selected->expanded = true;
    return ExtractSelectedChildren(types);
}

bool ResourceManager::DeleteNode(ResourceNode r){
    if(r==NULL) return false;
    itemsByName.erase(r->Name());
    BOOST_FOREACH(ResourceNode child,r->children){
      DeleteNode(child);
    }
    delete &(*r);
    return true;
}

bool ResourceManager::DeleteSelected(){
  DeleteNode(selected);
  if(selected->parent != NULL){
    selected->parent->children.erase(std::remove(selected->parent->children.begin(), selected->parent->children.end(),selected),selected->parent->children.end());
    selected->parent->SetDirty();
    }
  selected = NULL;
}

bool ResourceManager::Print(ResourceNode current,int level){
  if(current == NULL){
    BOOST_FOREACH(ResourceNode child,toplevel){
      Print(child,0);
    }
  }
  else{
    for(int i=0;i<level;i++) cout<<"-";
    cout<<current->Name()<<endl;
    BOOST_FOREACH(ResourceNode child,current->children){
      Print(child,level+1);
    }
  }
}
