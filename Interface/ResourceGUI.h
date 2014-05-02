#ifndef RESOURCE_GUI_H
#define RESOURCE_GUI_H

#include "WorldGUI.h"
#include "Modeling/Resources.h"
#include "View/ViewResource.h"
//#include "resourcemanager.h"
#include "Main/RobotPoseQt/resourcemanager.h"
#include <sstream>
using namespace Math3D;


/**@brief A backend for resource browsing.
 * Natively supports configs, paths, transforms, ik goals, holds, stance,
 * trimeshes
 *
 * Handles the following commands:
 * - set_resource name: sets the current resource name
 * - get_resource: requests a return message getting the current resource
 *   type and name
 * - set_resource_name name: sets the name of the current resource
 * - delete_resource: deletes the current resource
 * - add_resource type name: creates a new resource of the given type and name
 * - load_resource fn: loads a new resource from the given file name
 * - save_resource fn: saves the current resource to the given file name
 * - load_resource_dir dirfn: loads the entire resource library from the
 *   directory
 * - save_resource_dir dirfn: saves the entire resource library to the
 *   directory
 * - convert type: converts the resource to the given type
 * - extract type: extracts all sub-resources of the given type
 * - set_path_time t: sets the path time for the current view.
 *
 * Sends the following commands:
 * - current_resource type name: returns the currently selected resource
 * - refresh_resources: major change to the library, refresh display
 * - new_resource type name: a new resource was added
 *
 * Caller must have previously called MakeRobotResourceLibrary(library)
 * on the initialized library
 */
class ResourceGUIBackend : public WorldGUIBackend
{
public:
  ResourceGUIBackend(RobotWorld* world,ResourceManager* library);
  bool LoadCommandLine(int argc,char** argv);

  void Add(ResourcePtr r);
  ResourcePtr Add(const string& name,const string& type);
  template <class T>
  ResourcePtr Add(const string& name,const T& val) {
    ResourcePtr r=MakeResource(name,val);
    Add(r);
    return r;
  }
  ResourcePtr Add(const string& name,const vector<Real>& ts,const vector<Config>& qs) {
    ResourcePtr r=MakeResource(name,ts,qs);
    Add(r); 
    return r;
  }
  //save current resource to disk
  void SaveCur(const string& file);
  //load a new resource from the given file
  bool LoadNew(const string& file);
  //save all resources to a given path / XML file
  void SaveAll(const string& path);
  //load all resources in a given path / XML file
  bool LoadAll(const string& path);
  //sets the active item to the last added item
  void SetLastActive(); 
  //sets the active item to the one of the given type/name
  void SetActive(const string& type,const string& name);

  //subclasses should call this so that the GLUI controls will work
  virtual bool OnCommand(const string& cmd,const string& args);

  ResourcePtr CurrentResource();
  void RenderCurResource();

  //stores all resources
  ResourceManager* resources;
  string cur_resource_type,cur_resource_name;

  //stores the last added resource
  ResourcePtr last_added;

  //view
  ViewResource viewResource;
};



#endif
