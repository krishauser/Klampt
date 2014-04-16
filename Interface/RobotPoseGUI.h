#ifndef INTERFACE_ROBOTPOSEGUI_H
#define INTERFACE_ROBOTPOSEGUI_H

#include "WorldGUI.h"
#include "View/RobotPoseWidget.h"
#include "View/ObjectPoseWidget.h"
#include "Modeling/MultiPath.h"
#include <fstream>

using namespace Math3D;
using namespace GLDraw;

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

class RobotPoseBackend : public WorldGUIBackend
{
public :
  typedef GLNavigationBackend BaseT;
  ProgramSettings settings;
  Robot* robot;
  int cur_link,cur_driver;
  vector<bool> self_colliding, env_colliding;

  RobotPoseWidget poseWidget;

  int pose_ik,pose_objects;
  vector<RobotPoseWidget> robotWidgets;
  vector<RigidObjectPoseWidget> objectWidgets;
  WidgetSet allWidgets;
  int draw_geom,draw_bbs,draw_com,draw_frame,draw_expanded;

  vector<GLDisplayList> originalDisplayLists,expandedDisplayLists;

  RobotPoseBackend(RobotWorld* world);
  virtual void Start();
  void UpdateConfig();
  virtual void RenderWorld();
  virtual bool OnButtonPress(const string& button){}
  virtual bool OnButtonToggle(const string& button,int checked){}
  virtual bool OnCommand(const string& cmd,const string& args){}
  virtual void DoPassiveMouseMove(int x, int y){}
  virtual void BeginDrag(int x,int y,int button,int modifiers){}
  virtual void EndDrag(int x,int y,int button,int modifiers){}
  virtual void DoFreeDrag(int dx,int dy,int button){}
  void SetDrawExpanded(int value);

  Stance GetFlatStance();
  //  void CleanContacts(Hold&);
};
#endif //ROBOTPOSEGUI
