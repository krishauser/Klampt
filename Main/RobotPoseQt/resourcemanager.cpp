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
  ResourceNode node(new ResourceTracker(p));
  children.push_back(node);
  cout<<"Added Child "<<node->resource->name<<endl;
  return node;
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

bool ResourceTracker::Print(int level){
    for(int i=0;i<level;i++)
       printf("-");
    BOOST_FOREACH(ResourceNode child,children)
            child->Print();
    printf("%s\n",Name());
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

bool ResourceManager::ChangeSelected(string str){
    if(itemsByName[str] != NULL)
        ChangeSelected(itemsByName[str]);
    else selected = NULL;
}

bool ResourceManager::ChangeSelectedName(string name){
  itemsByName.erase(selected->Name());
  selected->resource->name = name;
  itemsByName[name] = selected;  
}

ResourceNode ResourceManager::AddTopLevel(ResourcePtr r){
    ResourceNode node(new ResourceTracker(r));
    toplevel.push_back(node);
    cout<<"Added Child "<<node->resource->name<<endl;
    return node;
}

ResourceNode ResourceManager::AddAsChild(ResourcePtr r){
    ResourceNode node;
    if(selected)
        node = selected->AddChild(r);
    else{
        node = AddTopLevel(r);
    }
    node->SetDirty();
    itemsByName.insert(pair<string,ResourceNode>(node->Name(),node));
    return node;
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
  if(selected == NULL){
      printf("No resource is currently selected");
      return vector<ResourceNode>();
  }
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
    else if(strcmp(selected->Type(),"Configs") == 0 ){
        types.push_back("Config");
    }
    else{
      printf("No expansion known for type %s",selected->Type());
      return vector<ResourceNode>();
    }
    selected->expanded = true;
    return ExtractSelectedChildren(types);
}

bool ResourceManager::DeleteNode(ResourceNode r,bool delete_reference){
    if(r==NULL) return false;
    itemsByName.erase(r->Name());
    while(!r->children.empty()){
        ResourceNode child = r->children.back();
        r->children.pop_back();
        DeleteNode(child,false);
    }
    if(delete_reference){
        if(r->parent != NULL){
            vector<ResourceNode>::iterator index= std::remove(r->parent->children.begin(), r->parent->children.end(), r);
            r->parent->children.erase(index, r->parent->children.end());
        }
    }
    return true;
}

bool ResourceManager::DeleteSelected(){
  if(selected == NULL){
    printf("No resource is currently selected");
    return false;
  }
  DeleteNode(selected);
  if(selected->parent != NULL){
    selected->parent->children.erase(std::remove(selected->parent->children.begin(), selected->parent->children.end(),selected),selected->parent->children.end());
    selected->parent->SetDirty();
    }
  selected = NULL;
  return true;
}

bool ResourceManager::Print(){
    BOOST_FOREACH(ResourceNode child,toplevel)
      child->Print();
}
