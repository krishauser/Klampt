#ifndef INTERFACE_ROBOTPOSEGUI_H
#define INTERFACE_ROBOTPOSEGUI_H

#include "ResourceGUI.h"
#include <Klampt/View/RobotPoseWidget.h>
#include <Klampt/View/ObjectPoseWidget.h>
#include <Klampt/Sensing/Sensor.h>
#include <KrisLibrary/utils/apputils.h>
#include <fstream>

#define GLUT_LEFT_BUTTON 0
#define GLUT_MIDDLE_BUTTON 1
#define GLUT_RIGHT_BUTTON 2

namespace Klampt {
  using namespace Math3D;
  using namespace GLDraw;

/** @brief Contains the functionality for the RobotPose program
 *
 * Accepts button toggle messages:
 * - draw_geom
 * - draw_bbs
 * - draw_com
 * - draw_frame
 * - draw_poser
 * - draw_sensors
 * 
 * Accepts commands (in addition to ResourceGUIBackend and WorldGUIBackend):
 * - pose_mode: next clicks will pose the robot's joints
 * - undo_pose: undoes the last robot pose
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
 * - split_path: splits the current LinearPath or MultiPath at the current
 *   time into two paths.
 * - store_flat_contacts [xtol]: gets the stance for the robot standing on
 *   flat ground and adds it as a new resource
 * - get_flat_contacts [xtol]: gets the stance for the robot standing on
 *   flat ground and stores it in the current Stance resource.
 * - get_nearby_contacts [xtol]: gets the stance for the robot touching the
 *   environment with the given position tolerance, and stores it in the
 *   current Stance resource.
 * - clean_contacts [xtol] [ntol]: cleans up the current Stance or Hold
 *   resource. Points and normals within xtol and ntol, respectively, will
 *   be merged.
 * - resample res: resamples geomery with the given resolution.
 */
class RobotPoseBackend : public ResourceGUIBackend
{
 public:
  AppUtils::ProgramSettings settings;
  RobotModel* robot;
  int cur_link,cur_driver;
  vector<bool> self_colliding, env_colliding;

  int pose_objects;
  vector<RobotPoseWidget> robotWidgets;
  vector<RigidObjectPoseWidget> objectWidgets;
  WidgetSet allWidgets;
  GLDraw::Widget* lastActiveWidget;
  int draw_geom,draw_poser,draw_bbs,draw_com,draw_frame,draw_sensors;
  //temp: sensors storage
  RobotSensors robotSensors;

  RobotPoseBackend(WorldModel* world,ResourceManager* library);
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

  Stance GetFlatStance(Real tolerance=0);
  Stance GetNearbyStance(Real tolerance=0);
  void CleanContacts(Hold&,Real xtol=0,Real ntol=0);
  ResourcePtr PoserToResource(const string& type);
};

} // namespace Klampt

#endif //INTERFACE_ROBOTPOSEGUI_H
