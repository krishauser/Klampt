#ifndef INTERFACE_SIM_TEST_GUI_H
#define INTERFACE_SIM_TEST_GUI_H

#include "SimulationGUI.h"
#include <Klampt/View/ViewPlot.h>
#include <Klampt/View/WorldDragWidget.h>
#include <Klampt/View/RobotPoseWidget.h>
#include <Klampt/View/ObjectPoseWidget.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/apputils.h>
#include <fstream>

#define GLUT_LEFT_BUTTON 0
#define GLUT_MIDDLE_BUTTON 1
#define GLUT_RIGHT_BUTTON 2

namespace Klampt {
  using namespace Math3D;
  using namespace GLDraw;

struct SensorPlot
{
  ViewPlot view;
  int sensorIndex;
  vector<bool> drawMeasurement;
};

/** @brief SimTest program.
 *
 * Messages are defined as follows.
 * 
 * command:
 * - set_link(link): sets the selected link
 * - set_link_value(value): sets the poser value of the selected link
 * - set_driver(link): sets the selected driver
 * - set_driver_value(value): sets the poser value of the selected driver
 * - advance(): takes one simulation step
 * - reset(): reverts to the initial simulation state
 * - command_pose(): sends the current pose(s) to the robot controller(s)
 * - command_config(q): sends the specified config command to the first robot controller
 * - pose_mode: next clicks will pose the robot's joints
 * - constrain_link_mode: next clicks will add constraints to a link
 * - constrain_point_mode: next clicks will add point constraints
 * - delete_constraint_mode: next clicks will delete constraints
 * - force_application_mode: next clicks will add forces
 * - constrain_current_link(): constrains the currently hovered link
 * - constrain_current_point(): point-constrains the currently hovered link
 * - delete_current_constraint(): deletes constraints on the currently hovered link
 * - simulate(active): turns simulation on or off (inherited from SimGUIBackend)
 * - toggle_simulate(): toggles simulation activity value (inherited from SimGUIBackend)
 * - reset(): resets the simulation (inherited from SimGUIBackend)
 * - load_file(file): loads a file of format simulation state (.state) or a path (.path,.xml,.milestones). (inherited from SimGUIBackend)
 * - load_state(file): loads a simulation state from file (inherited from SimGUIBackend)
 * - save_state(file): saves a simulation state to file (inherited from SimGUIBackend)
 * - load_view(file): loads a previously saved view (inherited from GLNavigationProgram)
 * - save_view(file): saves a view to a file (inherited from GLNavigationProgram)
 * - log_sim(file): saves simulation log to a given file, or "" to set no logging.
 * - log_commanded_path(robot,file): saves commanded path to a given linear path file, or "" to set no logging.
 * - log_sensed_path(robot,file): saves actual path to a given linear path file, or "" to set no logging.
 * - log_torque_path(robot,file): saves applied torques as a linear path file, or "" to set no logging.
 * - log_contact_state(file): saves contact state log to a given file, or "" to set no logging.
 * - log_contact_wrenches(file): saves contact wrench log to a given file, or "" to set no logging.
 *
 * button_toggle
 * - pose_objects
 * - draw_poser
 * - draw_bbs
 * - draw_contacts
 * - draw_wrenches
 * - draw_expanded 
 * - draw_time
 * - output_ros
 *
 * Signals sent back to GUI are defined as follows:
 * - command update_config: notifies that the configuration of the world has changed.
 */
class SimTestBackend : public SimGUIBackend
{
public:
  typedef SimGUIBackend BaseT;
  AppUtils::ProgramSettings settings;

  int cur_link,cur_driver;
  int pose_objects;
  int output_ros;
  enum {ModeNormal, ModeForceApplication};
  int click_mode;
  vector<RobotPoseWidget> robotWidgets;
  vector<RigidObjectPoseWidget> objectWidgets;
  WidgetSet allRobotWidgets,allObjectWidgets;
  WorldDragWidget dragWidget;
  WidgetSet allWidgets;

  int forceSpringActive;
  Vector3 forceSpringAnchor;
  int drawBBs,drawPoser,drawDesired,drawEstimated,drawContacts,drawWrenches,drawExpanded,drawTime,doLogging;
  string simLogFile;
  string contactStateLogFile, contactWrenchLogFile;
  map<int,string> robotCommandLogFiles, robotSensedLogFiles, robotTorqueLogFiles;

  vector<SensorPlot> sensorPlots;
  vector<vector<bool> > drawSensors;
  vector<GeometryAppearance> originalAppearance,expandedAppearance;

  SimTestBackend(WorldModel* world);
  //message handlers
  virtual void Start();
  virtual bool OnCommand(const string& cmd,const string& args);
  virtual bool OnIdle();
  void ToggleSensorPlot(int sensorIndex,int enabled);
  void ToggleSensorMeasurement(int sensorIndex,int measurement,int enabled);
  void ToggleDrawExpandedCheckbox(int checked);
  void SimStep(Real dt);
  void SensorPlotUpdate();

  bool LoadFile(const char* fn);

  //overrides of GLNavigationBackend
  virtual void RenderWorld();
  virtual void RenderScreen(); 
  virtual void DoPassiveMouseMove(int x, int y);
  virtual void BeginDrag(int x,int y,int button,int modifiers);
  virtual void EndDrag(int x,int y,int button,int modifiers);
  virtual void DoFreeDrag(int dx,int dy,int button);
  virtual void DoCtrlDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragPan(dx,dy);
  }
  
  virtual void DoAltDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragZoom(dx,dy);
  }

  virtual void DoShiftDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); SendRefresh(); }
  }
};

} //namespace Klampt


#if HAVE_GLUI

#include <KrisLibrary/GLdraw/GLScreenshotProgram.h>
#include "GLUIGUI.h"
#if defined (__APPLE__) || defined (MACOSX)
#include <GL/glui.h>
#else
#include <GL/glui.h>
#endif //__APPLE__ || MACOSX

namespace Klampt {

class GLUISimTestGUI : public GLScreenshotProgram<GLUIGUI>
{
public:
  typedef GLScreenshotProgram<GLUIGUI> BaseT;

  WorldModel* world;
  Simulator* sim;
  AppUtils::ProgramSettings settings;

  //GUI state
  GLUI* glui;
  GLUI_Button* save_movie_button;
  GLUI_Listbox* driver_listbox;
  GLUI_Spinner* driver_value_spinner;
  GLUI_String file_name;

  vector<string> controllerSettings;
  int controllerSettingIndex;
  GLUI_Listbox* settingsBox;
  GLUI_EditText* settingEdit;
  vector<string> controllerCommands;
  int controllerCommandIndex;
  GLUI_EditText* commandEdit;

  int cur_driver;

  vector<bool> sensorDrawn;
  vector<vector<bool> > sensorMeasurementDrawn;
  GLUI_Listbox* sensorBox;
  int sensorSelectIndex;
  int sensorMeasurementSelectIndex;
  GLUI_Checkbox* toggleSensorDrawCheckbox;
  GLUI_Checkbox* toggleMeasurementDrawCheckbox;
  GLUI_Button* isolateMeasurementButton;
  GLUI_Listbox* measurementListbox;

  GLUISimTestGUI(GenericBackendBase* backend,WorldModel* _world,int w=800,int h=600);
  virtual bool Initialize();
  void UpdateGUI();
  void UpdateControllerSettingGUI();
  void UpdateSensorGUI();
  void UpdateSensorMeasurementGUI();
  virtual void Handle_Control(int id);
  virtual void Handle_Keypress(unsigned char c,int x,int y);
  virtual bool OnCommand(const string& cmd,const string& args);
};

} // namespace Klampt

#endif //HAVE_GLUI

#endif
