#include "RobotTestGUI.h"
#include <KrisLibrary/GLdraw/drawMesh.h>
#include <KrisLibrary/GLdraw/drawgeometry.h>
#include <KrisLibrary/GLdraw/GLView.h>
#include "IO/ROS.h"
#include <string.h>
#include <sstream>

#ifndef GLUT_LEFT_BUTTON
#define GLUT_LEFT_BUTTON 0
#define GLUT_MIDDLE_BUTTON 1
#define GLUT_RIGHT_BUTTON 2
#endif //GLUT_LEFT_BUTTON

RobotTestBackend::RobotTestBackend(RobotWorld* world)
  :WorldGUIBackend(world)
{
}


bool RobotTestBackend::OnQuit()
{
  if(ros_status == 1)
    ROSShutdown();
  return true;
}

void RobotTestBackend::Start()
{
  WorldGUIBackend::Start();
  world->InitCollisions();
  robot = world->robots[0].get();
  cur_link=0;
  cur_driver=0;
  draw_geom = 1;
  draw_bbs = 0;
  draw_com = 0;
  draw_frame = 0;
  draw_expanded = 0;
  draw_sensors = 0;
  draw_self_collision_tests = 0;
  output_ros = 0;
  ros_status = 0;
  pose_ik = 0;
  self_colliding.resize(robot->links.size(),false);   

  robotWidgets.resize(world->robots.size());
  for(size_t i=0;i<world->robots.size();i++) {
    robotWidgets[i].Set(world->robots[i].get(),&world->robotViews[i]);
    robotWidgets[i].linkPoser.highlightColor.set(0.75,0.75,0);
  }
  objectWidgets.resize(world->rigidObjects.size());
  for(size_t i=0;i<world->rigidObjects.size();i++)
    objectWidgets[i].Set(world->rigidObjects[i].get());
  for(size_t i=0;i<world->robots.size();i++)
    allWidgets.widgets.push_back(&robotWidgets[i]);
  for(size_t i=0;i<world->rigidObjects.size();i++)
    allWidgets.widgets.push_back(&objectWidgets[i]);

  MapButtonToggle("draw_geom",&draw_geom);
  MapButtonToggle("draw_bbs",&draw_bbs);
  MapButtonToggle("draw_com",&draw_com);
  MapButtonToggle("draw_frame",&draw_frame);
  MapButtonToggle("draw_sensors",&draw_sensors);
  MapButtonToggle("draw_self_collision_tests",&draw_self_collision_tests);
  MapButtonToggle("output_ros",&output_ros);
}
  
void RobotTestBackend::UpdateConfig()
{
  for(size_t i=0;i<robotWidgets.size();i++)
    world->robots[i]->UpdateConfig(robotWidgets[i].Pose());

  //update collisions
  for(size_t i=0;i<robot->links.size();i++)
    self_colliding[i]=false;
  robot->UpdateGeometry();
  //test: built-in self collision testing, which may be faster
  vector<pair<int,int> > collpairs;
  robot->SelfCollisions(collpairs);
  for(size_t i=0;i<collpairs.size();i++)
    self_colliding[collpairs[i].first] = self_colliding[collpairs[i].second] = true;
  /*
  for(size_t i=0;i<robot->links.size();i++) {
    for(size_t j=i+1;j<robot->links.size();j++) {
      if(robot->SelfCollision(i,j)) {
        self_colliding[i]=self_colliding[j]=true;
      }
    }
  }
  */
  if(draw_sensors)
    RefreshSensors();
  //tell the front end that the configuration is updated
  SendCommand("update_config","");

  //If ROS is enabled, broadcast the transforms of the updated world
  if(output_ros && ros_status == 1) {
    ROSPublishTransforms(*world);
  }
}

void RobotTestBackend::RenderWorld()
{
  ViewRobot& viewRobot = world->robotViews[0];
  //WorldViewProgram::RenderWorld();
  glDisable(GL_LIGHTING);
  drawCoords(0.1f);
  glEnable(GL_LIGHTING);
  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i]->DrawGL();
  for(size_t i=0;i<world->rigidObjects.size();i++)
    world->rigidObjects[i]->DrawGL();

  if(draw_sensors) {
    if(robotSensors.sensors.empty()) {
      robotSensors.MakeDefault(robot);
    }
    for(size_t i=0;i<robotSensors.sensors.size();i++) {
      vector<double> measurements;
      if(0 == strcmp(robotSensors.sensors[i]->Type(),"CameraSensor") || 0 == strcmp(robotSensors.sensors[i]->Type(),"LaserRangeSensor")) {
        robotSensors.sensors[i]->GetMeasurements(measurements);
        if(measurements.empty()) {
          RefreshSensors(); //this might mess up the view... reset it
          glMatrixMode(GL_MODELVIEW);
          glLoadIdentity();
          GLDraw::GLView view;
          view.setViewport(viewport);
          view.setCurrentGL();
        }
      }
      robotSensors.sensors[i]->DrawGL(*robot,measurements);
    }
  }
   
  if(draw_geom) {
    //set the robot colors
    GLColor highlight(1,1,0);
    GLColor driven(1,0.5,0);
    GLColor colliding(1,0,0);
    GLColor blue(0,0,1);
    viewRobot.RestoreAppearance();
    viewRobot.PushAppearance();
    for(size_t i=0;i<robot->links.size();i++) {
      if(self_colliding[i]) viewRobot.SetColor(i,colliding);
      if((int)i == cur_link)
        viewRobot.SetColor(i,highlight); 
      else if(cur_driver >= 0 && cur_driver < (int)robot->drivers.size() &&
              robot->DoesDriverAffect(cur_driver,i))
        viewRobot.SetColor(i,driven); 
      if(draw_self_collision_tests) {
        //draw a little blue
        if(robot->selfCollisions(i,cur_link) || robot->selfCollisions(cur_link,i) )  {
          GLDraw::GeometryAppearance &app  = viewRobot.Appearance(i);
          app.ModulateColor(blue,0.5);
        }
      }
    }
    //this will set the hover colors
    allWidgets.DrawGL(viewport);
    //viewRobot.Draw();
    viewRobot.PopAppearance();
  }
  else {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].linkPoser.draw = 0;
    allWidgets.DrawGL(viewport);
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].linkPoser.draw = 1;
  }
  if(draw_com) {
    viewRobot.DrawCenterOfMass();
    for(size_t i=0;i<robot->links.size();i++)
      viewRobot.DrawLinkCenterOfMass(i,0.01);
  }
  if(draw_frame) {
    viewRobot.DrawLinkFrames();
    glDisable(GL_DEPTH_TEST);
    glPushMatrix();
    glMultMatrix((Matrix4)robot->links[cur_link].T_World);
    drawCoords(0.2f);
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
  }
  if(draw_self_collision_tests) {
    glDisable(GL_LIGHTING);
    glLineWidth(2.0);
    glColor3f(1,0,0);
    glBegin(GL_LINES);
    for(size_t i=0;i<robot->links.size();i++) {
      Vector3 comi = robot->links[i].T_World*robot->links[i].com;
      for(size_t j=0;j<robot->links.size();j++) {
        if(robot->selfCollisions(i,j)) {
          Vector3 comj = robot->links[j].T_World*robot->links[j].com;
          glVertex3v(comi);
          glVertex3v(comj);
        }
      }
    }
    glEnd();
  }
  if(draw_bbs) {
    for(size_t j=0;j<robot->geometry.size();j++) {
      if(robot->IsGeometryEmpty(j)) continue;
      Box3D bbox = robot->geometry[j]->GetBB();
      Matrix4 basis;
      bbox.getBasis(basis);
      glColor3f(1,0,0);
      drawOrientedWireBox(bbox.dims.x,bbox.dims.y,bbox.dims.z,basis);
    }
  }
}

void RobotTestBackend::RefreshSensors()
{
  if(draw_sensors) {
    if(robotSensors.sensors.empty()) {
      robotSensors.MakeDefault(robot);
    }
    for(size_t i=0;i<robotSensors.sensors.size();i++) {
      vector<double> measurements;
      if(0 == strcmp(robotSensors.sensors[i]->Type(),"CameraSensor") || 0 == strcmp(robotSensors.sensors[i]->Type(),"LaserRangeSensor")) {
        robotSensors.sensors[i]->SimulateKinematic(*robot,*world);
      }
    }
  }
}

bool RobotTestBackend::OnButtonPress(const string& button)
{
  if(button=="print_pose") {
    cout<<robot->q<<endl;
    return true;
  }
  else if(button=="print_self_collisions") {
    printf("Self-colliding:\n");
    for(size_t i=0;i<robot->links.size();i++) {
      if(self_colliding[i]) {
        bool printed=false;
        for(size_t j=i;j<robot->links.size();j++) {
          if(self_colliding[j] && robot->SelfCollision(i,j)) {
            printf("%d %d\t",i,j);
            printed = true;
          }
        }
        if(printed) printf("\n");
      }
    }
    return true;
  }
  else if(button=="request_self_collisions"){
    stringstream ss;
    for(size_t i=0;i<robot->links.size();i++) {
      if(self_colliding[i]) {
        bool printed=false;
        for(size_t j=i;j<robot->links.size();j++) {
          if(self_colliding[j] && robot->SelfCollision(i,j)) {
            ss<<i<<" "<<j<<" ";
            printed = true;
          }
        }
        if(printed) ss<<"\n";
      }
    }
    SendCommand("return_self_collisions",ss.str());
  }
  else if(!GenericBackendBase::OnButtonPress(button)) {
    cout<<"RobotTestBackend: Unknown button: "<<button<<endl;
    return false;
  }
  return true;
}

bool RobotTestBackend::OnButtonToggle(const string& button,int checked)
{
  if(button=="draw_expanded") {
    SetDrawExpanded(checked);
    SendRefresh();
    return true;
  }
  else if(button=="output_ros") {
    if(checked) {
      //initialize ROS if not initialized
      if(ros_status == 0) {
        if(ROSInit()) {
          ros_status = 1;
          bool res = ROSPublishTransforms(*world);
          if(!res) printf("Error publishing transforms?\n");
        }
        else ros_status = -1;
      }
    }
    if(!GenericBackendBase::OnButtonToggle(button,checked)) {
      cout<<"RobotTestBackend: Unknown button: "<<button<<endl;
      return false;
    }
  }
  else if(!GenericBackendBase::OnButtonToggle(button,checked)) {
    cout<<"RobotTestBackend: Unknown button: "<<button<<endl;
    return false;
  }
  return true;
}

bool RobotTestBackend::OnCommand(const string& cmd,const string& args)
{
  //cout<<"Command: "<<cmd<<", args "<<args<<endl;
  stringstream ss(args);
  if(cmd=="set_link") {
    ss >> cur_link;
  }
  else if(cmd=="set_link_value") {
    double value;
    ss>>value;
    Vector q = robotWidgets[0].Pose();
    q(cur_link)=value;
    robotWidgets[0].SetPose(q);
    robot->q(cur_link)=value;
    robot->UpdateFrames();
    SendRefresh();
    UpdateConfig();
  }
  else if(cmd=="set_driver") {
    ss >> cur_driver;
  }
  else if(cmd=="set_driver_value") {
    double driver_value;
    ss>>driver_value;
    robot->UpdateConfig(robotWidgets[0].Pose());
    robot->SetDriverValue(cur_driver,driver_value);
    robotWidgets[0].SetPose(robot->q);
    UpdateConfig();
  }
  else if(cmd=="pose_mode") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].SetFixedPoseIKMode(false);
  }
  else if(cmd=="constrain_link_mode") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].SetFixedPoseIKMode(true);
  }
  else if(cmd=="constrain_point_mode") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].SetPoseIKMode(true);
  }
  else if(cmd=="delete_constraint_mode") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].SetDeleteIKMode(true);
  }
  else if(cmd=="constrain_current_link") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].FixCurrent();
  }
  else if(cmd == "delete_current_constraint") {
    for(size_t i=0;i<robotWidgets.size();i++)
      robotWidgets[i].DeleteConstraint();
  }
  else if(cmd == "print_pose") {
    cout<<robot->q<<endl;
  }
  else if(cmd == "reload_file") {
    if(!WorldGUIBackend::ReloadFile(args.c_str())) return false;
    //now update the widgets
    robotWidgets.resize(world->robots.size());
    for(size_t i=0;i<world->robots.size();i++) 
      robotWidgets[i].Set(world->robots[i].get(),&world->robotViews[i]);
    robotSensors.sensors.resize(0);
    robot = world->robots[0].get();
  }
  else if(cmd == "undo_pose") {
    for(size_t i=0;i<world->robots.size();i++) 
      if(lastActiveWidget == &robotWidgets[i]) {
        printf("Undoing robot poser %d\n",i);
        robotWidgets[i].Undo();
        UpdateConfig();
      }
  }
  else {
    return WorldGUIBackend::OnCommand(cmd,args);
  }
  SendRefresh();
  return true;
}

void RobotTestBackend::DoPassiveMouseMove(int x, int y)
{
  double d;
  if(allWidgets.Hover(x,viewport.h-y,viewport,d))
    allWidgets.SetHighlight(true);
  else
    allWidgets.SetHighlight(false);
  if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
  
}

void RobotTestBackend::BeginDrag(int x,int y,int button,int modifiers)
{  
  if(button == GLUT_RIGHT_BUTTON) {
    double d;
    if(allWidgets.BeginDrag(x,viewport.h-y,viewport,d)) {
      allWidgets.SetFocus(true);
      lastActiveWidget = allWidgets.activeWidget;
    }
    else
      allWidgets.SetFocus(false);
    if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
  }
}

void RobotTestBackend::EndDrag(int x,int y,int button,int modifiers)
{
  if(button == GLUT_RIGHT_BUTTON) {
    if(allWidgets.hasFocus) {
      allWidgets.EndDrag();
      allWidgets.SetFocus(false);
      if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
    }
  }
}

void RobotTestBackend::DoFreeDrag(int dx,int dy,int button)
{
  if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
  else if(button == GLUT_RIGHT_BUTTON) {
    if(allWidgets.hasFocus) {
      allWidgets.Drag(dx,-dy,viewport);
      if(allWidgets.requestRedraw) {
        allWidgets.requestRedraw = false;
        SendRefresh();
        UpdateConfig();
      }
    }
  }
}

void RobotTestBackend::SetDrawExpanded(int value)
{
  draw_expanded = value;
  if(originalDisplayLists.empty()) {
    //first call -- initialize display lists
    originalDisplayLists.resize(robot->links.size());
    expandedDisplayLists.resize(robot->links.size());
    for(size_t j=0;j<robot->links.size();j++) {
      originalDisplayLists[j]=robot->geomManagers[j].Appearance()->faceDisplayList;
      if(!robot->IsGeometryEmpty(j)) {
        Real p=robot->geometry[j]->margin;
        if(p > 0) {
          //draw the expanded mesh
          expandedDisplayLists[j].beginCompile();
          drawExpanded(*robot->geometry[j],p);
          expandedDisplayLists[j].endCompile();
        }
        else
          expandedDisplayLists[j] = originalDisplayLists[j];
      }
    }
  }
  if(draw_expanded) {
    for(size_t j=0;j<robot->links.size();j++) {
      robot->geomManagers[j].Appearance()->faceDisplayList = expandedDisplayLists[j];
    }
  }
  else {
    for(size_t j=0;j<robot->links.size();j++) 
      robot->geomManagers[j].Appearance()->faceDisplayList = originalDisplayLists[j];
  }
}



#if HAVE_GLUI

GLUIRobotTestGUI::GLUIRobotTestGUI(GenericBackendBase* backend,RobotWorld* _world,int w,int h)
  :GLUIGUI(backend,w,h),world(_world)
{}

bool GLUIRobotTestGUI::Initialize()
{
  if(!GLUIGUI::Initialize()) return false;
  robot = world->robots[0].get();
  cur_link = 0;
  cur_driver = 0;

  //setup GUI 
  glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
  glui->set_main_gfx_window(main_window);
  GLUI_Panel* panel = glui->add_panel("Link controls");
  link_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_link);
  link_spinner->set_int_limits(0,robot->links.size(),GLUI_LIMIT_WRAP);
  AddControl(link_spinner,"link");

  link_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_link);
  for(size_t i=0;i<robot->links.size();i++) {
    char buf[256];
    strcpy(buf,robot->linkNames[i].c_str());
    link_listbox->add_item(i,buf);
  }
  AddControl(link_listbox,"link");

  link_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT);
  link_info = glui->add_statictext_to_panel(panel,"Info");
  AddControl(link_value_spinner,"link_value");

  panel = glui->add_panel("Driver controls");
  AddControl(glui->add_checkbox_to_panel(panel,"Pose by IK"),"pose_ik");
  driver_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_driver);
  driver_spinner->set_int_limits(0,(int)robot->drivers.size(),GLUI_LIMIT_WRAP);
  AddControl(driver_spinner,"driver");

  driver_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_driver);
  for(size_t i=0;i<robot->drivers.size();i++) {
    char buf[256];
    strcpy(buf,robot->driverNames[i].c_str());
    driver_listbox->add_item(i,buf);
  }
  AddControl(driver_listbox,"driver");

  driver_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT);
  AddControl(driver_value_spinner,"driver_value");
  driver_info = glui->add_statictext_to_panel(panel,"Info");

  GLUI_Checkbox* checkbox = glui->add_checkbox("Draw geometry");
  checkbox->set_int_val(1);
  AddControl(checkbox,"draw_geom");
  checkbox = glui->add_checkbox("Draw COM");
  AddControl(checkbox,"draw_com");
  checkbox = glui->add_checkbox("Draw frame");
  AddControl(checkbox,"draw_frame");
  checkbox = glui->add_checkbox("Draw sensors");
  AddControl(checkbox,"draw_sensors");
  checkbox = glui->add_checkbox("Draw bboxes");
  AddControl(checkbox,"draw_bbs");
  checkbox = glui->add_checkbox("Draw expanded");
  AddControl(checkbox,"draw_expanded");
  checkbox = glui->add_checkbox("Draw collision tests");
  AddControl(checkbox,"draw_self_collision_tests");
  AddControl(glui->add_button("Print self colliding links"),"print_self_collisions");
  AddControl(glui->add_button("Print config"),"print_pose");
  UpdateGUI();

  const char* rules = "[ \
[{type:key_down,key:c}, {type:command,cmd:constrain_current_link,args:\"\"}],   \
[{type:key_down,key:d}, {type:command,cmd:delete_current_constraint,args:\"\"}], \
[{type:key_down,key:p}, {type:command,cmd:print_pose,args:\"\"}],       \
[{type:key_down,key:z}, {type:command,cmd:undo_pose,args:\"\"}], \
[{type:button_press,button:print_config}, {type:command,cmd:print_pose,args:\"\"}], \
[{type:button_toggle,button:pose_ik,checked:1}, {type:command,cmd:constrain_point_mode,args:\"\"}], \
[{type:button_toggle,button:pose_ik,checked:0}, {type:command,cmd:pose_mode,args:\"\"}], \
[{type:widget_value,widget:link,value:_0}, {type:command,cmd:set_link,args:_0}], \
[{type:widget_value,widget:link_value,value:_0}, {type:command,cmd:set_link_value,args:_0}], \
[{type:widget_value,widget:driver,value:_0}, {type:command,cmd:set_driver,args:_0}], \
[{type:widget_value,widget:driver_value,value:_0}, {type:command,cmd:set_driver_value,args:_0}] \
]";
  stringstream ss(rules);
  bool res=LoadRules(ss);
  assert(res==true);
  return true;
}

void GLUIRobotTestGUI::Handle_Control(int id)
{
  GLUIGUI::Handle_Control(id);
  if(controls[id]==link_spinner || controls[id]==link_listbox)
    UpdateGUI();
  if(controls[id]==driver_spinner || controls[id]==driver_listbox)
    UpdateGUI();
}

bool GLUIRobotTestGUI::OnCommand(const string& cmd,const string& args)
{
  if(cmd=="update_config") {
    UpdateGUI();
    return true;
  }
  else return GLUIGUI::OnCommand(cmd,args);
}

void GLUIRobotTestGUI::UpdateGUI()
{
  link_spinner->set_int_val(cur_link);
  link_listbox->set_int_val(cur_link);
  Vector2 limits(robot->qMin(cur_link),robot->qMax(cur_link));
  link_value_spinner->set_float_limits(limits.x,limits.y);
  double link_value = robot->q(cur_link);
  link_value_spinner->set_float_val(link_value);
  
  char buf[256];
  sprintf(buf,"V [%g %g], T [%g,%g]",robot->velMin(cur_link),robot->velMax(cur_link),-robot->torqueMax(cur_link),robot->torqueMax(cur_link));
  link_info->set_text(buf);
  
  if(cur_driver < 0 || cur_driver >= (int)robot->drivers.size()) return;
  driver_spinner->set_int_val(cur_driver);
  driver_listbox->set_int_val(cur_driver);
  limits = robot->GetDriverLimits(cur_driver);
  driver_value_spinner->set_float_limits(limits.x,limits.y);
  double driver_value = robot->GetDriverValue(cur_driver);
  driver_value_spinner->set_float_val(driver_value);
  sprintf(buf,"V [%g %g], T [%g,%g], PID %g,%g,%g",robot->drivers[cur_driver].vmin,robot->drivers[cur_driver].vmax,robot->drivers[cur_driver].tmin,robot->drivers[cur_driver].tmax,robot->drivers[cur_driver].servoP,robot->drivers[cur_driver].servoI,robot->drivers[cur_driver].servoD);
  driver_info->set_text(buf);
}


#endif


