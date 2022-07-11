#ifndef RESOURCE_MANGER_H
#define RESOURCE_MANGER_H


#include <Klampt/Modeling/Resources.h>

using namespace std;
using namespace Klampt;

/** @brief Allows a resource to be expanded into component resources,
 * edited, and then backed up.
 */
class ResourceNode
{
 public:
  ResourceNode(const ResourcePtr& res,ResourceNode* parent=NULL);
  const char *Type(){return resource->Type();}
  const char *Name(){return resource->name.c_str();}
  int Depth() const;
  void Print(int level=0);
  string Identifier() const;
  vector<string> PathTo() const;
  shared_ptr<ResourceNode> AddChild(const ResourcePtr& res);
  vector<shared_ptr<ResourceNode> > AddChildren(const vector<ResourcePtr>& children);
  bool Editable() const;
  void SetSaved() { saved = true; }
  bool IsSaved() const { return saved; }
  void SetChanged();
  void SetChildrenChanged();
  bool ChildrenChanged() const { return childrenChanged; }
  bool IsDirty() const;
  ///Extracts everything to children
  void Expand();
  bool IsExpandable() const;
  bool IsExpanded() const { return expanded; }
  ///Backs up changes from children, marks as invalid if changes
  ///couldn't be made, and returns false.  The reason is given in 
  ///the error string if is is non-NULL.
  bool Backup(string* errorMessage=NULL,ResourceNode** where=NULL);
  bool IsValid() const { return valid; }
  ///If a backup couldn't be made, either the expansion should be cleared
  ///or the backup needs to be fixed
  void ClearExpansion();
  ///Decorator can be:
  ///- '!' for an invalid resource
  ///- '@' for a non-backed up resource
  ///- '*' for a backed-up, unsaved top-level resource
  ///- empty for an unchanged inner resource, or an unchanged saved top-level
  ///  resource
  const char* Decorator() const;

  ResourcePtr resource;
  vector<shared_ptr<ResourceNode> > children;
  ResourceNode* parent;
 private:
  bool saved;
  bool childrenChanged;
  bool childrenEditable;
  bool expanded;
  bool valid;
};

typedef shared_ptr<ResourceNode> ResourceNodePtr;

class ResourceTree
{  
 public:
  ResourceLibrary library;
  vector<ResourceNodePtr> topLevel;

  ResourceTree();
  ResourceNodePtr LoadFile(const string& fn);
  bool LoadFolder(const string& fn);
  bool Save(ResourceNode* node,string file="");
  bool SaveFolder(const string& fn);
  void Delete(ResourceNode* r);
  bool AnyUnsaved() const;
  bool IsValid() const;
  bool BackupAll(string* errorMessage);
  void TreeFromLibrary();
  bool TreeToLibrary(bool trybackup=true);
  int ChildIndex(ResourceNode* n) const;

  ResourceNodePtr Add(ResourcePtr r,ResourceNodePtr parent=NULL);

  void Print();
};

class ResourceManager : public ResourceTree
{
 public:
  ResourceManager();
  void Select(const string& identifier);
  void Select(const vector<string>& path);
  ResourceNodePtr Next();
  ResourceNodePtr Get(const string& identifier);
  ResourceNodePtr Get(const vector<string>& path);
  ResourceNodePtr AddChildOfSelected(ResourcePtr r,bool changeSelection=true);
  void DeleteSelected();
  bool BackupSelected(string* errorString=NULL,ResourceNode** which=NULL);
  vector<ResourceNodePtr> ExpandSelected();
  bool SaveSelected();
  bool SaveSelected(const string& fn);
  ResourceNodePtr SafePtr(ResourceNode* ptr);

  ResourceNode* selected;
};

#endif
