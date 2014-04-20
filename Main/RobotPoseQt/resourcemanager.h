#ifndef RESOURCE_MANGER_H
#define RESOURCE_MANGER_H


#include <Modeling/Resources.h>
#include <Interface/GenericGUI.h>

using namespace std;

class ResourceTracker
{

 public:
  ResourcePtr resource;
  vector<ResourceTracker*> children;
  bool dirty;
  bool expanded;
  ResourceTracker* parent;

  ResourceTracker(ResourcePtr,ResourceTracker* parent=NULL);
  ResourceTracker *AddChild(ResourcePtr);
  vector<ResourceTracker*> AddChildren(vector<ResourcePtr>);
  void SetDirty();
  const char *Type();
};

class ResourceManager
{
  
 public:
  GenericBackendBase* backend;
  ResourceLibrary library;
  vector<ResourceTracker*> toplevel;
  ResourceTracker* selected;
  ResourceTracker* open;

  ResourceManager();
  ResourceTracker* LoadResource(const string& fn);
  bool DeleteSelected();
  bool ChangeSelected(ResourceTracker*);
  bool SendSelectedToGUI();
  bool AddFromPoser();
  bool ReplaceFromPoser();

  vector<ResourceTracker *> ExtractSelectedChildren(vector<string> types);
  vector<ResourceTracker *> ExpandSelected();
};

#endif
