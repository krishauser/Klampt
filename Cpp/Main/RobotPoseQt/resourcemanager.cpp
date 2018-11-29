#include <resourcemanager.h>
#include <Modeling/Resources.h>
#include <string.h>
#include <KrisLibrary/utils/ioutils.h>

//#include <boost/foreach.hpp>

ResourceNode::ResourceNode(const ResourcePtr& p,ResourceNode* _parent)
  :resource(p),parent(_parent)
{
  childrenChanged= false;
  childrenEditable = true;
  expanded = false;
  valid = true;
  saved = false;
}

int ResourceNode::Depth() const
{
  if(!parent) return 0;
  return 1+parent->Depth();
}

ResourceNodePtr ResourceNode::AddChild(const ResourcePtr& p)
{
  printf("Adding new child of resource, existing %d\n",(int)children.size());
  ResourceNodePtr node = make_shared<ResourceNode>(p,this);
  children.push_back(node);
  childrenChanged = true;
  if(!childrenEditable)
    node->childrenEditable = false;
  return node;
}

vector<ResourceNodePtr> ResourceNode::AddChildren(const vector<ResourcePtr>& ptrs)
{
  vector<ResourceNodePtr> ret;
  for(size_t i=0;i<ptrs.size();i++){
    ret.push_back(AddChild(ptrs[i]));
    assert(ret.back()->resource == ptrs[i]);
  }
  return ret;
}

void ResourceNode::SetChanged()
{
  if(parent) parent->SetChildrenChanged();
  saved = false;
}

void ResourceNode::SetChildrenChanged() 
{
  childrenChanged=true;
  ResourceNode* n=this;
  while(n != NULL) {
    n->saved = false;
    n = n->parent;
  }
}

bool ResourceNode::IsDirty() const
{
  if(childrenChanged) return true;
  for(size_t i=0;i<children.size();i++)
    if(children[i]->IsDirty()) return true;
  return false;
}

bool ResourceNode::IsExpandable() const
{
  return (dynamic_cast<const CompoundResourceBase*>(&*resource)!=NULL);
}

void ResourceNode::Expand()
{
  if(expanded){
    return;
  }
  bool successful,incomplete;
  vector<ResourcePtr> seg = UnpackResource(resource,&successful,&incomplete);
  AddChildren(seg);
  childrenChanged = false;
  expanded = true;
  if(incomplete) childrenEditable = false;
}

bool ResourceNode::Editable() const
{
  if(!parent) return true;
  else return parent->childrenEditable;
}

bool ResourceNode::Backup(string* errorMessage,ResourceNode** where)
{
  for(size_t i=0;i<children.size();i++)
    if(!children[i]->Backup(errorMessage,where)) return false;
  if(!childrenChanged) return true;

  printf("Packing resources of type %s into %s\n",Type(),Identifier().c_str());
  //try doing the packing
  vector<ResourcePtr> temp;
  for(size_t i=0;i<children.size();i++)
    temp.push_back(children[i]->resource);
  string packerror;
  CompoundResourceBase* cr = dynamic_cast<CompoundResourceBase*>(&*resource);
  if(!cr) return false;
  if(!cr->Pack(temp,&packerror)) {
    //TODO: better errors
    if(errorMessage)
      *errorMessage = packerror;
    if(where)
      *where = this;
    valid = false;
    return false;
  }

  printf("Successful\n");
  valid = true;
  childrenChanged = false;
  return true;
}

void ResourceNode::ClearExpansion()
{
  children.resize(0);
  childrenChanged = false;
  expanded = false;
  valid = true;
}

const char* ResourceNode::Decorator() const
{
  if(!valid) return "!";
  if(childrenChanged) return "@";
  if(parent==NULL && !saved) return "*";
  return "";
}

void ResourceNode::Print(int level)
{
  for(int i=0;i<level;i++)
    LOG4CXX_INFO(KrisLibrary::logger(),"-");
  LOG4CXX_INFO(KrisLibrary::logger(),""<<Name()<<""<<Decorator());
  //BOOST_FOREACH(ResourceNodePtr child,children)
    //child->Print(level+1);
  //children is a vector of resourceNodePtrs
  for(vector<ResourceNodePtr>::iterator it=children.begin(); it!=children.end(); it++){
    (*it)->Print(level+1);
  }
}

vector<string> ResourceNode::PathTo() const
{
  vector<string> res;
  if(parent) 
    res = parent->PathTo();
  res.push_back(resource->name);
  return res;
}

string ResourceNode::Identifier() const
{
  string root;
  if(parent) 
    root = parent->Identifier() + "/";

  stringstream ss;
  ss<<root;
  SafeOutputString(ss,resource->name);
  return ss.str();
}

ResourceTree::ResourceTree(){
  MakeRobotResourceLibrary(library);
}

bool ResourceTree::AnyUnsaved() const
{
  for(size_t i=0;i<topLevel.size();i++)
    if(!topLevel[i]->IsSaved()) return false;
  return true;
}

ResourceNodePtr ResourceTree::LoadFile(const string& fn)
{
  ResourcePtr r = library.LoadItem(fn);
  if(!r) return NULL;
  topLevel.push_back(make_shared<ResourceNode>(r));
  topLevel.back()->SetSaved();
  return topLevel.back();
}

bool ResourceTree::LoadFolder(const string& fn){
  if(!library.LoadAll(fn)) return false;
  TreeFromLibrary();
  for(size_t i=0;i<topLevel.size();i++)
    topLevel[i]->SetSaved();
  return true;
}

bool ResourceTree::Save(ResourceNode* node,string file)
{
  ResourcePtr r = node->resource;
  if(file.empty()){
    r->fileName = library.DefaultFileName(r);
  }
  else
    r->fileName = file;
  if(!r->Save()){
    fprintf(stderr,"Unable to save %s to %s\n",r->name.c_str(),r->fileName.c_str());
    return false;
  }
  else {
    printf("Saved %s to %s\n",r->name.c_str(),r->fileName.c_str());
    node->SetSaved();
    return true;
  }
}

bool ResourceTree::SaveFolder(const string& path)
{
  TreeToLibrary();
  for(ResourceLibrary::Map::iterator i=library.itemsByType.begin();i!=library.itemsByType.end();i++) {
    for(size_t j=0;j<i->second.size();j++)
      if(i->second[j]->fileName.empty())
        i->second[j]->fileName = library.DefaultFileName(i->second[j]);
  }
  library.ChangeBaseDirectory(path);
  if(!library.SaveAll()) {
    fprintf(stderr,"Unable to save all resources to %s\n",path.c_str());
    return false;
  }
  else {
    for(size_t i=0;i<topLevel.size();i++)
      topLevel[i]->SetSaved();
    fprintf(stderr,"Saved all resources to %s\n",path.c_str());
    return true;
  }
}

ResourceNodePtr ResourceTree::Add(ResourcePtr r,ResourceNodePtr parent)
{
  vector<shared_ptr<ResourceNode> >& siblings = (parent ? parent->children : topLevel);
  int samenamecount = 0;
  string origName = r->name;
  if(r->name[r->name.length()-1] == ']') {
    //look at base name
    size_t pos = r->name.rfind('[');
    if(pos!=string::npos)
      origName = r->name.substr(0,pos);
  }
  for(size_t i=0;i<siblings.size();i++) 
    if(r->name == siblings[i]->resource->name) {
      samenamecount++;
      char buf[64];
      sprintf(buf,"[%d]",samenamecount+1);
      r->name = origName + buf;
    }
  if(parent) {
    ResourceNodePtr c = parent->AddChild(r);
    parent->SetChanged();
    return c;
  }
  else {
    topLevel.push_back(make_shared<ResourceNode>(r));
    library.Add(r);
    return topLevel.back();
  }
}



void ResourceTree::Delete(ResourceNode* r){
  if(r->parent == NULL) { //top level
    library.Erase(r->resource);
    int index = ChildIndex(r);
    if(index < 0)
      fprintf(stderr,"ResourceTree::Delete: inconsistency found, resource %s has no parent but is not in topLevel list\n",r->Identifier().c_str());
    assert(index >= 0);
    assert(index < (int)topLevel.size());
    topLevel.erase(topLevel.begin()+index);
  }
  else {
    int index = ChildIndex(r);
    if(index < 0)
      fprintf(stderr,"ResourceTree::Delete: inconsistency found, resource %s has parent but is not in child list of %s\n",r->Identifier().c_str(),r->parent->Identifier().c_str());
    assert(index >= 0);
    assert(index < (int)r->parent->children.size());
    r->parent->children.erase(r->parent->children.begin()+index);
    r->parent->SetChildrenChanged();
    r->parent = NULL;
  }
}

bool ResourceTree::IsValid() const
{
  for(size_t i=0;i<topLevel.size();i++) {
    if(!topLevel[i]->IsValid()) return false;
  }
  return true;
}

bool ResourceTree::BackupAll(string* errorMessage) 
{
  ResourceNode* nerror;
  for(size_t i=0;i<topLevel.size();i++) {
    if(!topLevel[i]->Backup(errorMessage,&nerror)) {
      //TODO: report where
      return false;
    }
  }
  return true;
}

void ResourceTree::Print(){
    //BOOST_FOREACH(ResourceNodePtr child,topLevel)
      //child->Print();
    //assuming topLevel is a vector of ResourceNodePointers too
    for(vector<ResourceNodePtr>::iterator it=topLevel.begin(); it!=topLevel.end(); it++){
      (*it)->Print();
    }
}

int ResourceTree::ChildIndex(ResourceNode* n) const
{
  if(n->parent == NULL) {
    for(size_t j=0;j<topLevel.size();j++) 
      if(topLevel[j].get() == n) return (int)j;
    return -1;
  }
  else {
    for(size_t j=0;j<n->parent->children.size();j++)
      if(n->parent->children[j].get() == n) return (int)j;
    return -1;
  }
}

void ResourceTree::TreeFromLibrary()
{
  for(ResourceLibrary::Map::iterator i=library.itemsByType.begin();i!=library.itemsByType.end();i++) {
    for(size_t j=0;j<i->second.size();j++) {
      bool exists = false;
      for(size_t k=0;k<topLevel.size();k++)
        if(i->second[j] == topLevel[k]->resource) {
          exists=true;
          break;
        }
      if(!exists) Add(i->second[j]);
    }
  }
}

bool ResourceTree::TreeToLibrary(bool trybackup)
{
  bool backupOk = true;
  library.itemsByType.clear();
  library.itemsByName.clear();
  for(size_t i=0;i<topLevel.size();i++) {
    if(trybackup) {
      if(topLevel[i]->IsDirty())
        if(!topLevel[i]->Backup())
          backupOk = false;
    }
    library.Add(topLevel[i]->resource);
  }
  return backupOk;
}



ResourceManager::ResourceManager()
  :selected(NULL)
{}

void ResourceManager::Select(const string& identifier)
{
  selected = Get(identifier).get();
}

void ResourceManager::Select(const vector<string>& path)
{
  selected = Get(path).get();
}

ResourceNodePtr ResourceManager::Next()
{
  if(selected->parent == NULL) {
    //top level
    for(size_t i=0;i<topLevel.size();i++) {
      if(topLevel[i].get() == selected) 
        if(i+1 < topLevel.size()) return topLevel[i+1];
    }
  }
  else {
    for(size_t i=0;i<selected->parent->children.size();i++) {
      if(selected->parent->children[i].get() == selected) 
        if(i+1 < selected->parent->children.size()) return selected->parent->children[i+1];
    }
  }
  return NULL;
}

ResourceNodePtr ResourceManager::Get(const string& name)
{
  stringstream ss(name);
  vector<string> path;
  string temp;
  while(SafeInputString(ss,temp)) {
    //discard /
    int c = ss.get();
    if(c != EOF && c != '/')
      printf("Warning, strange character %c in identifier\n",(char)c);
    path.push_back(temp);
  }
  return Get(path);
}

ResourceNodePtr ResourceManager::Get(const vector<string>& path)
{
  vector<ResourceNodePtr>* curList = &topLevel;
  for(size_t i=0;i<path.size();i++) {
    bool found = false;
    for(size_t j=0;j<curList->size();j++) {
      if((*curList)[j]->resource->name == path[i]) {
        found = true;
        if(i+1 == path.size()) return (*curList)[j];
        else curList = &(*curList)[j]->children;
        break;
      }
    }
    if(!found) return NULL;
  }
  return NULL;
}


ResourceNodePtr ResourceManager::AddChildOfSelected(ResourcePtr r,bool changeSelection)
{
  ResourceNodePtr n = Add(r,SafePtr(selected));
  if(changeSelection)
    selected = n.get();
  return n;
}

void ResourceManager::DeleteSelected()
{
  if(selected) {
    ResourceNodePtr n = Next();
    Delete(selected);
    selected = n.get();
  }
}

bool ResourceManager::BackupSelected(string* errorString,ResourceNode** which)
{
  if(selected) return selected->Backup(errorString,which);
  return false;
}

vector<ResourceNodePtr> ResourceManager::ExpandSelected()
{
  if(selected) {
    if(!selected->IsExpanded()) {
      selected->Expand();
      return selected->children;
    }
  }
  return vector<ResourceNodePtr>();
}

bool ResourceManager::SaveSelected()
{
  if(selected) return Save(selected);
  return false;
}

bool ResourceManager::SaveSelected(const string& fn)
{
  if(selected) {
    return Save(selected,fn);
  }
  return false;
}

ResourceNodePtr ResourceManager::SafePtr(ResourceNode* ptr)
{
  if(!ptr) return NULL;
  if(ptr->parent == NULL) {
    for(size_t i=0;i<topLevel.size();i++) 
      if(&*topLevel[i] == ptr) return topLevel[i];
    fprintf(stderr,"ResourceManager::SafePtr: could not find element named %s, ptr %p?\n",ptr->resource->name.c_str(),&*ptr->resource);
    fprintf(stderr,"Candidates:");
    for(size_t i=0;i<topLevel.size();i++) 
      fprintf(stderr,"  %s %p\n",topLevel[i]->resource->name.c_str(),&*topLevel[i]->resource);
  }
  else {
    for(size_t i=0;i<ptr->parent->children.size();i++) 
      if(&*ptr->parent->children[i] == ptr) return ptr->parent->children[i];
    fprintf(stderr,"ResourceManager::SafePtr: could not find element named %s, ptr %p?\n",ptr->resource->name.c_str(),&*ptr->resource);
    fprintf(stderr,"Candidates:");
    for(size_t i=0;i<ptr->parent->children.size();i++) 
      fprintf(stderr,"  %s %p\n",ptr->parent->children[i]->resource->name.c_str(),&*ptr->parent->children[i]->resource);
  }
  AssertNotReached();
  return NULL;
}