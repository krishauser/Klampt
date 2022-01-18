#ifndef RESOURCE_GUI_H
#define RESOURCE_GUI_H

#include "WorldGUI.h"
#include <Klampt/Modeling/Resources.h>
#include <Klampt/View/ViewResource.h>
//#include "resourcemanager.h"
#include "Main/RobotPoseQt/resourcemanager.h"
#include <sstream>

namespace Klampt {
  using namespace Math3D;

/**@brief A backend for resource browsing.
 * Natively supports configs, paths, transforms, ik goals, holds, stance,
 * trimeshes
 *
 * Handles the following commands:
 * - set_resource id: sets the current resource to the given identifier
 * - get_resource: requests a return message getting the current resource
 *   identifier.
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
 * - current_resource identifier: returns the currently selected resource
 * - refresh_resources: major change to the library, refresh display
 * - new_resource identifier: a new resource was added
 *
 * Caller must have previously called MakeRobotResourceLibrary(library)
 * on the initialized library
 */
class ResourceGUIBackend : public WorldGUIBackend
{
public:
  ResourceGUIBackend(WorldModel* world,ResourceManager* library);
  bool LoadCommandLine(int argc,const char** argv);

  ResourceNodePtr Add(ResourcePtr r);
  ResourceNodePtr Add(const string& name,const string& type);
  template <class T>
  ResourceNodePtr Add(const string& name,const T& val) {
    ResourcePtr r=MakeResource(name,val);
    return Add(r);
  }
  ResourceNodePtr Add(const string& name,const vector<Real>& ts,const vector<Config>& qs) {
    ResourcePtr r=MakeResource(name,ts,qs);
    return Add(r); 
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
  //sets the active item to the one of the given identifier
  void SetActive(const string& identifier);
  virtual void SetPathTime(double time);

  //subclasses should call this so that the GLUI controls will work
  virtual bool OnCommand(const string& cmd,const string& args);

  ResourcePtr CurrentResource();
  void RenderCurResource();

  //stores all resources
  ResourceManager* resources;

  //stores the last added resource
  ResourceNodePtr last_added;

  //view
  ViewResource viewResource;
};

} // namespace Klampt

#endif
