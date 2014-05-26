#ifndef INTERFACE_ROBOTPOSEGUI_H
#define INTERFACE_ROBOTPOSEGUI_H

#include "ResourceGUI.h"
#include "View/RobotPoseWidget.h"
#include "View/ObjectPoseWidget.h"
#include <fstream>

using namespace Math3D;
using namespace GLDraw;

#define GLUT_LEFT_BUTTON 0
#define GLUT_MIDDLE_BUTTON 1
#define GLUT_RIGHT_BUTTON 2

class ProgramSettings : public AnyCollection
{
public:
  ProgramSettings() {  }
  bool read(const char* fn) {
    ifstream in(fn,ios::in);
    if(!in) return false;
    AnyCollection newEntries;
    if(!newEntries.read(in)) return false;
    merge(newEntries);
    return true;
  }
  bool write(const char* fn) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    AnyCollection::write(out);
    out.close();
    return true;
  }
};

/** @brief Contains the functionality for the RobotPose program
 *
 * Accepts button toggle messages:
 * - draw_geom
 * - draw_bbs
 * - draw_com
 * - draw_frame
 * 
 * Accepts commands (in addition to ResourceGUIBackend and WorldGUIBackend):
 * - pose_mode: next clicks will pose the robot's joints
 * - constrain_link_mode: next clicks will add constraints to a link
 * - constrain_point_mode: next clicks will add point constraints
 * - delete_constraint_mode: next clicks will delete constraints
 * - set_link(link): sets the selected link
 * - set_link_value(value): sets the value of the selected link
 * - set_driver(link): sets the selected driver
 * - set_driver_value(value): sets the value of the selected driver
 * - poser_to_resource type: Takes the poser and converts it to a resource
 *   of the given type (can be Config, IKGoal, Stance, Grasp)
 * - poser_to_resource_overwrite: Uses the poser to overwrite the current
 *   resource.
 * - resource_to_poser: Takes the current resource and sends it to the poser
 *   (can be Config, IKGoal, Stance, Grasp)
 * - create_path: creates a path given the current resource.  If it's a
 *   Config, creates a path from the config to the poser's config.
 *   If it's a Configs, creates a path between those milestones (possibly
 *   obeying the IKGoals)
 * - discretize_path num: creates a set of Configs by discretizing the
 *   selected LinearPath, MultiPath, or Configs into the given number of
 *   evenly spaced milestones.
 * - optimize_path: optimizes the current LinearPath, MultiPath, Configs
 *   resource.
 * - store_flat_contacts: gets the stance for the robot standing on flat
 *   ground and adds it as a new resource
 * - clean_contacts [xtol] [ntol]: cleans up the current Stance or Hold
 *   resource. Points and normals within xtol and ntol, respectively, will
 *   be merged.
 */
class RobotPoseBackend : public ResourceGUIBackend
{
 public:
  ProgramSettings settings;
  Robot* robot;
  int cur_link,cur_driver;
  vector<bool> self_colliding, env_colliding;

  int pose_objects;
  vector<RobotPoseWidget> robotWidgets;
  vector<RigidObjectPoseWidget> objectWidgets;
  WidgetSet allWidgets;
  int draw_geom,draw_bbs,draw_com,draw_frame;

  RobotPoseBackend(RobotWorld* world,ResourceManager* library);
  virtual void Start();
  void UpdateConfig();
  virtual void RenderWorld();
  virtual bool OnButtonPress(const string& button);
  virtual bool OnButtonToggle(const string& button,int checked);
  virtual bool OnCommand(const string& cmd,const string& args);
  virtual void DoPassiveMouseMove(int x, int y);
  virtual void BeginDrag(int x,int y,int button,int modifiers);
  virtual void EndDrag(int x,int y,int button,int modifiers);
  virtual void DoFreeDrag(int dx,int dy,int button);
  void SetDrawExpanded(int value);

  Stance GetFlatStance();
  void CleanContacts(Hold&,Real xtol=0,Real ntol=0);
  ResourcePtr PoserToResource(const string& type);
};



#endif //INTERFACE_ROBOTPOSEGUI_H
