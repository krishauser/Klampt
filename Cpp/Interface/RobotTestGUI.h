#ifndef ROBOTTESTGUI
#define ROBOTTESTGUI

#include "WorldGUI.h"
#include "View/RobotPoseWidget.h"
#include "View/ObjectPoseWidget.h"
#include "Sensing/Sensor.h"
#include "GLUIGUI.h"
#include <fstream>

namespace Klampt {
  using namespace Math3D;
  using namespace GLDraw;

/** @brief RobotTest program.
 *
 * Messages are defined as follows.
 * 
 * command:
 * - set_link(link): sets the selected link
 * - set_link_value(value): sets the value of the selected link
 * - set_driver(link): sets the selected driver
 * - set_driver_value(value): sets the value of the selected driver
 * - constrain_current_link(): constrains the currently hovered link
 * - delete_current_constraint(): deletes constraints on the currently hovered link
 * - print_pose()
 * - print_self_collisions()
 * - load_file(file): loads an element into the world (inherited from WorldGUIBackend)
 * - reload_file(file): reloads an element whose file may have changed (inherited from WorldGUIBackend)
 * - load_view(file): loads a previously saved view (inherited from GLNavigationProgram)
 * - save_view(file): saves a view to a file (inherited from GLNavigationProgram)
 * - undo_pose(): undo's last pose action
 *
 * button_press
 * - print_pose
 * - print_self_collisions
 * 
 * button_toggle
 * - pose_ik
 * - draw_geom 
 * - draw_bbs
 * - draw_com 
 * - draw_frame
 * - draw_expanded 
 * - draw_sensors
 * - draw_self_collision_tests
 * - output_ros
 *
 * Signals sent back to GUI are defined as follows:
 * - command update_config: notifies that the configuration of the world has changed.
 */
class RobotTestBackend : public WorldGUIBackend
{
public:
  RobotModel* robot;
  //internal state
  int cur_link,cur_driver;
  vector<bool> self_colliding;
  //temp: sensors storage
  RobotSensors robotSensors;

  int pose_ik,pose_objects;
  vector<RobotPoseWidget> robotWidgets;
  vector<RigidObjectPoseWidget> objectWidgets;
  WidgetSet allWidgets;
  GLDraw::Widget* lastActiveWidget;
  int draw_geom,draw_bbs,draw_com,draw_frame,draw_expanded,draw_sensors;
  int draw_self_collision_tests;
  int output_ros, ros_status;

  vector<GLDisplayList> originalDisplayLists,expandedDisplayLists;

  RobotTestBackend(WorldModel* world);
  virtual void Start();
  void UpdateConfig();
  virtual void RenderWorld();
  virtual bool OnQuit();
  virtual bool OnButtonPress(const string& button);
  virtual bool OnButtonToggle(const string& button,int checked);
  virtual bool OnCommand(const string& cmd,const string& args);
  virtual void DoPassiveMouseMove(int x, int y);
  virtual void BeginDrag(int x,int y,int button,int modifiers);
  virtual void EndDrag(int x,int y,int button,int modifiers);
  virtual void DoFreeDrag(int dx,int dy,int button);
  void RefreshSensors();
  void SetDrawExpanded(int value);
};

} // namespace Klampt

#if HAVE_GLUI

#if defined (__APPLE__) || defined (MACOSX)
#include <GL/glui.h>
#else
#include <GL/glui.h>
#endif //__APPLE__ || MACOSX

namespace Klampt {

class GLUIRobotTestGUI : public GLUIGUI
{
 public:
  WorldModel* world;
  RobotModel* robot;
  //GUI state
  GLUI* glui;
  GLUI_Spinner* link_spinner, *link_value_spinner;
  GLUI_Listbox* link_listbox;
  GLUI_StaticText* link_info;
  GLUI_Spinner* driver_spinner, *driver_value_spinner;
  GLUI_Listbox* driver_listbox;
  GLUI_StaticText* driver_info;
  int cur_link,cur_driver;

  GLUIRobotTestGUI(GenericBackendBase* backend,WorldModel* world,int w=800,int h=600);
  virtual bool Initialize();
  virtual void Handle_Control(int id);
  virtual bool OnCommand(const string& cmd,const string& args);
  void UpdateGUI();
};

} // namespace Klampt

#endif //HAVE_GLUI


#endif //ROBOTTESTGUI
