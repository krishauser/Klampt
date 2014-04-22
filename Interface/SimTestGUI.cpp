#include "SimTestGUI.h"
#include <utils/AnyValue.h>
#include <math/random.h>


SimTestBackend::SimTestBackend(RobotWorld* world)
  :SimGUIBackend(world)
{
  settings["movieWidth"] = 640;
  settings["movieHeight"] = 480;
  settings["updateStep"] = 0.01;
  settings["pathDelay"] = 0.1;
  settings["sensorPlot"]["x"]=20;
  settings["sensorPlot"]["y"]=20;
  settings["sensorPlot"]["width"]=300;
  settings["sensorPlot"]["height"]=200;
  settings["sensorPlot"]["spacing"]=20;
  settings["sensorPlot"]["autotime"]=false;
  settings["sensorPlot"]["duration"]=2.0;
  settings["poser"]["color"][0] = 1;
  settings["poser"]["color"][1] = 1;
  settings["poser"]["color"][2] = 0;
  settings["poser"]["color"][3] = 0.5;
  settings["desired"]["color"][0] = 0;
  settings["desired"]["color"][1] = 1;
  settings["desired"]["color"][2] = 0;
  settings["desired"]["color"][3] = 0.5;
  settings["contact"]["pointSize"] = 5;
  settings["contact"]["normalLength"] = 0.05;
  settings["contact"]["forceScale"] = 0.01;
  settings["dragForceMultiplier"] = 10.0;
}

void SimTestBackend::Start()
{
  if(!settings.read("simtest.settings")) {
    printf("Didn't read settings from simtest.settings\n");
    printf("Writing default settings to simtest_default.settings\n");
    settings.write("simtest_default.settings");
  }

  cur_link=0;
  cur_driver=0;

  dragWidget.Set(world);
  robotWidgets.resize(world->robots.size());
  for(size_t i=0;i<world->robots.size();i++)
    robotWidgets[i].Set(world->robots[i].robot,&world->robots[i].view);
  objectWidgets.resize(world->rigidObjects.size());
  for(size_t i=0;i<world->rigidObjects.size();i++) 
    objectWidgets[i].Set(world->rigidObjects[i].object,&world->rigidObjects[i].view);
  allWidgets.widgets.push_back(&dragWidget);
  for(size_t i=0;i<world->robots.size();i++)
    allWidgets.widgets.push_back(&robotWidgets[i]);
  for(size_t i=0;i<world->rigidObjects.size();i++)
    allObjectWidgets.widgets.push_back(&objectWidgets[i]);
  //don't pose objects first
  //allWidgets.widgets.push_back(&allObjectWidgets);

  drawBBs = 0;
  drawPoser = 1;
  drawDesired = 0;
  drawEstimated = 0;
  drawContacts = 0;
  drawWrenches = 1;
  drawExpanded = 0;
  pose_ik = 0;
  pose_objects = 0;
  forceApplicationMode = false, forceSpringActive = false;

  MapButtonToggle("draw_bbs",&drawBBs);
  MapButtonToggle("draw_poser",&drawPoser);
  MapButtonToggle("draw_desired",&drawDesired);
  MapButtonToggle("draw_estimated",&drawEstimated);
  MapButtonToggle("draw_contacts",&drawContacts);
  MapButtonToggle("draw_wrenches",&drawWrenches);
  MapButtonToggle("draw_expanded",&drawExpanded);
  MapButtonToggle("pose_ik",&pose_ik);
  MapButtonToggle("pose_objects",&pose_objects);
  MapButtonToggle("force_application_mode",&forceApplicationMode);
 
  /*
  //TEMP: testing determinism
  bool nondet = false;
  int n=20;
  Real dt = 0.01;
  vector<string> trace(n);
  for(int i=0;i<n;i++) {
  string start,temp;
  sim.WriteState(start);
  sim.Advance(dt);
  sim.WriteState(trace[i]);

  sim.ReadState(start);
  sim.Advance(dt);
  sim.WriteState(temp);
  if(temp != trace[i]) {
  printf("Warning, sim state nondeterministic @ step %d: ",i);
  nondet = true;
  if(temp.length() != trace[i].length())
  printf("different lengths %d vs %d\n",temp.length(),trace[i].length());
  else {
  for(size_t j=0;j<temp.length();j++)
  if(temp[j] != trace[i][j]) {
  printf("byte %d\n",j);
  break;
  }
  }
  getchar();
  }
  }
  */

  SimGUIBackend::Start();
}

void SimTestBackend::ToggleSensorPlot(int sensorIndex,int enabled) {
  RobotSensors& sensors = sim.controlSimulators[0].sensors;
  if(enabled) {
    for(size_t i=0;i<sensorPlots.size();i++) {
      if(sensorPlots[i].sensorIndex == sensorIndex) return;
    }
    int spacing = int(settings["sensorPlot"]["spacing"]);
    SensorPlot sensorPlot;
    sensorPlot.sensorIndex=sensorIndex;
    sensorPlot.view.x = int(settings["sensorPlot"]["x"]);
    sensorPlot.view.y = int(settings["sensorPlot"]["y"]);
    sensorPlot.view.width = int(settings["sensorPlot"]["width"]);
    sensorPlot.view.height = int(settings["sensorPlot"]["height"]);
    sensorPlot.view.autoXRange = bool(settings["sensorPlot"]["autotime"]);
    sensorPlot.view.xmin = sim.time-Real(settings["sensorPlot"]["duration"]);
    sensorPlot.view.xmax = sim.time;
    if(!sensorPlots.empty()) 
      sensorPlot.view.y += sensorPlots.back().view.height+spacing;
    
    SensorBase* sensor = NULL;
    sensor = sensors.sensors[sensorIndex];
    vector<string> names;
    sensor->MeasurementNames(names);
    sensorPlot.drawMeasurement.resize(names.size());
    fill(sensorPlot.drawMeasurement.begin(),sensorPlot.drawMeasurement.end(),true);
    sensorPlot.view.curves.resize(0);
    const static float colors[12][3] = {{1,0,0},{1,0.5,0},{1,1,0},{0.5,1,0},{0,1,0},{0,1,0.5},{0,1,1},{0,0.5,1},{0,0,1},{0.5,0,1},{1,0,1},{1,0,0.5}};
    sensorPlot.view.curveColors.resize(Min((size_t)12,names.size()));
    for(size_t i=0;i<sensorPlot.view.curveColors.size();i++)
      sensorPlot.view.curveColors[i].set(colors[i][0],colors[i][1],colors[i][2]);
    size_t start=sensorPlot.view.curveColors.size();
    sensorPlot.view.curveColors.resize(names.size());
    for(size_t i=start;i<sensorPlot.view.curveColors.size();i++) 
      sensorPlot.view.curveColors[i].setHSV(Rand()*360.0,1,1);

    sensorPlots.push_back(sensorPlot);
  }
  else {
    int spacing = int(settings["sensorPlot"]["spacing"]);
    for(size_t i=0;i<sensorPlots.size();i++) {
      if(sensorPlots[i].sensorIndex == sensorIndex) {
	int h = sensorPlots[i].view.height+spacing;
	sensorPlots.erase(sensorPlots.begin()+i);
	//shift everything else up one
	for(size_t j=i;j<sensorPlots.size();j++)
	  sensorPlots[j].view.y -= h; 
	break;
      }
    }
  }
  SendRefresh();
}

void SimTestBackend::ToggleSensorMeasurement(int sensorIndex,int measurement,int checked)
{
  SensorPlot* plot=NULL;
  for(size_t i=0;i<sensorPlots.size();i++)
    if(sensorPlots[i].sensorIndex == sensorIndex) {
      plot = &sensorPlots[i];
      break;
    }
  if(!plot) return;
  if(measurement < 0 || measurement >= (int)plot->drawMeasurement.size()) return;
  plot->drawMeasurement[measurement] = checked;
  plot->view.curveColors[measurement].rgba[3] = checked;
  plot->view.AutoYRange();
  SendRefresh();
}

void SimTestBackend::RenderWorld()
{
  BaseT::RenderWorld();

  glEnable(GL_BLEND);
  Robot* robot=world->robots[0].robot;
  RobotController* rc=sim.robotControllers[0];
  if(drawPoser) {
    //draw desired milestone
    AnyCollection poserColorSetting = settings["poser"]["color"];
    GLColor poserColor(poserColorSetting[0],poserColorSetting[1],poserColorSetting[2],poserColorSetting[3]);
    for(size_t i=0;i<robotWidgets.size();i++) { 
      world->robots[i].robot->UpdateConfig(robotWidgets[i].Pose());
      world->robots[i].view.SetColors(poserColor);
    }
    //draw poser and such
    allWidgets.DrawGL(viewport);
    for(size_t i=0;i<robotWidgets.size();i++) { 
      world->robots[i].view.Draw();
    }
  }

  //applying a force
  if(forceSpringActive) {
    sim.UpdateModel();
  }

  //draw commanded setpoint
  if(drawDesired) {
    robot->UpdateConfig(robotWidgets[0].Pose());
    sim.controlSimulators[0].GetCommandedConfig(robot->q);
    robot->UpdateFrames();
    AnyCollection desiredColorSetting = settings["desired"]["color"];
    GLColor desiredColor(desiredColorSetting[0],desiredColorSetting[1],desiredColorSetting[2],desiredColorSetting[3]);
    world->robots[0].view.SetColors(desiredColor);
    world->robots[0].view.Draw();
  }
  if(drawEstimated) {
  }

  //draw collision feedback
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
  if(drawContacts) {
    glDisable(GL_DEPTH_TEST);
    Real pointSize = settings["contact"]["pointSize"];
    Real fscale=settings["contact"]["forceScale"];
    Real nscale=settings["contact"]["normalLength"];
    DrawContacts(pointSize,fscale,nscale);
  }

  if(drawWrenches) {
    Real robotsize = 1.5;
    Real fscale = robotsize/(robot->GetTotalMass()*9.8);
    DrawWrenches(fscale);
  }

  //draw bounding boxes
  if(drawBBs) {
    sim.UpdateModel();
    for(size_t i=0;i<world->robots.size();i++) {
      for(size_t j=0;j<world->robots[i].robot->geometry.size();j++) {
	if(world->robots[i].robot->geometry[j].Empty()) continue;
	Box3D bbox = world->robots[i].robot->geometry[j].GetBB();
	Matrix4 basis;
	bbox.getBasis(basis);
	glColor3f(1,0,0);
	drawOrientedWireBox(bbox.dims.x,bbox.dims.y,bbox.dims.z,basis);
      }
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      Box3D bbox = world->rigidObjects[i].object->geometry.GetBB();
      Matrix4 basis;
      bbox.getBasis(basis);
      glColor3f(1,0,0);
      drawOrientedWireBox(bbox.dims.x,bbox.dims.y,bbox.dims.z,basis);
    }
    for(size_t i=0;i<world->terrains.size();i++) {
      Box3D bbox = world->terrains[i].terrain->geometry.GetBB();
      Matrix4 basis;
      bbox.getBasis(basis);
      glColor3f(1,0.5,0);
      drawOrientedWireBox(bbox.dims.x,bbox.dims.y,bbox.dims.z,basis);
    }
  }
}

void SimTestBackend::RenderScreen()
{
  //SimGUIBackend::DrawClock(20,40);
  for(size_t i=0;i<sensorPlots.size();i++)
    sensorPlots[i].view.DrawGL();
}

void SimTestBackend::ToggleDrawExpandedCheckbox(int checked)
{
  if(originalAppearance.empty()) {
    //first call -- initialize display lists
    originalAppearance.resize(world->NumIDs());
    expandedAppearance.resize(world->NumIDs());
    vector<Real> paddings(world->NumIDs(),0);
    for(size_t i=0;i<originalAppearance.size();i++) {
      //robots don't get a geometry
      if(world->IsRobot(i) >= 0) continue;
      originalAppearance[i] = world->GetAppearance(i);
      expandedAppearance[i].geom = originalAppearance[i].geom;
      expandedAppearance[i].faceColor = originalAppearance[i].faceColor;
      expandedAppearance[i].lightFaces = true;
      expandedAppearance[i].drawFaces = true;
      expandedAppearance[i].drawVertices = false;
      paddings[i] = world->GetGeometry(i).margin;
    }
    for(size_t i=0;i<world->robots.size();i++) {
      for(size_t j=0;j<world->robots[i].robot->links.size();j++) {
	int id=world->RobotLinkID(i,j);
	if(sim.odesim.robot(i)->triMesh(j)) 
	  paddings[id] += sim.odesim.robot(i)->triMesh(j)->GetPadding();
      }
    }
    for(size_t i=0;i<world->terrains.size();i++) {
      int id=world->TerrainID(i);
      if(sim.odesim.envGeom(i)) 
	paddings[id] += sim.odesim.envGeom(i)->GetPadding();
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      int id = world->RigidObjectID(i);
      if(sim.odesim.object(i)->triMesh()) 
	paddings[id] += sim.odesim.object(i)->triMesh()->GetPadding();
    }
    for(size_t i=0;i<originalAppearance.size();i++) {
      //robots don't get a geometry
      if(world->IsRobot(i) >= 0) continue;
      if(paddings[i] > 0) {
	//draw the expanded mesh
	expandedAppearance[i].faceDisplayList.beginCompile();
	GLDraw::drawExpanded(world->GetGeometry(i),paddings[i]);
	expandedAppearance[i].faceDisplayList.endCompile();
      }
      else
	expandedAppearance[i] = originalAppearance[i];
    }
  }
  drawExpanded = checked;
  if(drawExpanded) {
    for(size_t i=0;i<originalAppearance.size();i++) {
      //robots don't get a geometry
      if(world->IsRobot(i) >= 0) continue;
      world->GetAppearance(i) = expandedAppearance[i];
    }
  }
  else {
    for(size_t i=0;i<originalAppearance.size();i++) {
      //robots don't get a geometry
      if(world->IsRobot(i) >= 0) continue;
      world->GetAppearance(i) = originalAppearance[i];
    }
  }
  SendRefresh();
}

bool SimTestBackend::OnCommand(const string& cmd,const string& args)
{
  stringstream ss(args);
  if(cmd=="advance") {
    SimStep(sim.simStep);
  }
  else if(cmd=="reset")  {
    BaseT::OnCommand(cmd,args);
    for(size_t i=0;i<world->robots.size();i++) {
      robotWidgets[i].SetPose(world->robots[i].robot->q);
      robotWidgets[i].ikPoser.poseGoals.clear();
      robotWidgets[i].ikPoser.RefreshWidgets();
    }
  }
  else if(cmd=="command_pose") {
    for(size_t i=0;i<sim.robotControllers.size();i++) {
      RobotController* rc=sim.robotControllers[i];
      stringstream ss;
      ss<<robotWidgets[i].Pose();
      if(!rc->SendCommand("set_q",ss.str())) {
	fprintf(stderr,"set_q command does not work with the robot's controller\n");
      }
    }
  }
  else if(cmd=="command_config") {
    RobotController* rc=sim.robotControllers[0];
    if(!rc->SendCommand("set_q",args)) {
      fprintf(stderr,"set_q command does not work with the robot's controller\n");
    }
  }
  else if(cmd=="load_file") {
    LoadFile(args.c_str());
  }
  else if(cmd=="show_sensor") {
    int index;
    ss>>index;
    ToggleSensorPlot(index,1);
  }
  else if(cmd=="hide_sensor") {
    int index;
    ss>>index;
    ToggleSensorPlot(index,0);
  }
  else if(cmd=="show_sensor_measurement") {      
    int index,measurement;
    ss>>index>>measurement;
    ToggleSensorMeasurement(index,measurement,1);
  }
  else if(cmd=="hide_sensor_measurement") {
    int index,measurement;
    ss>>index>>measurement;
    ToggleSensorMeasurement(index,measurement,0);
  }
  else if(cmd=="constrain_link") {
      robotWidgets[0].FixCurrent();
  }
  else if(cmd=="constrain_link_point"){
    robotWidgets[0].FixCurrentPoint();
  }
  else if(cmd == "delete_constraint") {
      robotWidgets[0].DeleteConstraint();
  }
  else if(cmd=="set_link") {
    ss >> cur_link;
  }
  else if(cmd=="set_link_value") {
    double value;
    ss>>value;
    Vector q = robotWidgets[0].Pose();
    q(cur_link)=value;
    robotWidgets[0].SetPose(q);
  }
  else if(cmd=="set_driver") {
    ss >> cur_driver;
  }
  else if(cmd=="set_driver_value") {
    double driver_value;
    ss>>driver_value;
    Robot* robot = world->robots[0].robot;
    robot->UpdateConfig(robotWidgets[0].Pose());
    robot->SetDriverValue(cur_driver,driver_value);
    robotWidgets[0].SetPose(robot->q);
  }
  else if(cmd=="log_sim") {
    simLogFile = args;
  }
  else if(cmd=="log_contact_state") {
    contactStateLogFile = args;
  }

  else if(cmd=="log_contact_wrenches") {
    contactWrenchLogFile = args;
  }

  else
    return BaseT::OnCommand(cmd,args);
  SendRefresh();
  return true;
}

bool SimTestBackend::LoadFile(const char* fn)
{
  if(SimGUIBackend::LoadFile(fn)) {
    //refresh the posers if there's an added object
    size_t nr = robotWidgets.size();
    size_t no = objectWidgets.size();
    robotWidgets.resize(world->robots.size());
    for(size_t i=nr;i<world->robots.size();i++) {
      robotWidgets[i].Set(world->robots[i].robot,&world->robots[i].view);
      allWidgets.widgets.push_back(&robotWidgets[i]);
    }
    objectWidgets.resize(world->rigidObjects.size());
    for(size_t i=no;i<world->rigidObjects.size();i++) {
      objectWidgets[i].Set(world->rigidObjects[i].object,&world->rigidObjects[i].view);
      allObjectWidgets.widgets.push_back(&objectWidgets[i]);
    }
    return true;
  }
  return false;
}

void SimTestBackend::DoPassiveMouseMove(int x, int y)
{
  dragWidget.active = (forceApplicationMode==1);
  for(size_t i=0;i<robotWidgets.size();i++)
    robotWidgets[i].poseIKMode = (pose_ik != 0);

  if(pose_objects == 0) {
    if(allWidgets.widgets.back() == &allObjectWidgets)
      //disable object widgets
      allWidgets.widgets.resize(allWidgets.widgets.size()-1);
  }
  else {
    if(allWidgets.widgets.back() != &allObjectWidgets)
      //enable object widgets
      allWidgets.widgets.push_back(&allObjectWidgets);
  }

  double d;
  if(allWidgets.Hover(x,viewport.h-y,viewport,d))
    allWidgets.SetHighlight(true);
  else
    allWidgets.SetHighlight(false);
  if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
}

void SimTestBackend::BeginDrag(int x,int y,int button,int modifiers)
{
  dragWidget.active = (forceApplicationMode==1);
  for(size_t i=0;i<robotWidgets.size();i++)
    robotWidgets[i].poseIKMode = (pose_ik != 0);
  if(pose_objects == 0) {
    if(allWidgets.widgets.back() == &allObjectWidgets)
      //disable object widgets
      allWidgets.widgets.resize(allWidgets.widgets.size()-1);
  }
  else {
    if(allWidgets.widgets.back() != &allObjectWidgets)
      //enable object widgets
      allWidgets.widgets.push_back(&allObjectWidgets);
  }

  Robot* robot = world->robots[0].robot;
  if(button == GLUT_RIGHT_BUTTON) {
    double d;
    if(allWidgets.BeginDrag(x,viewport.h-y,viewport,d))
      allWidgets.SetFocus(true);
    else
      allWidgets.SetFocus(false);
    if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
  }
}

void SimTestBackend::EndDrag(int x,int y,int button,int modifiers)
{
  dragWidget.active = (forceApplicationMode==1);
  for(size_t i=0;i<robotWidgets.size();i++)
    robotWidgets[i].poseIKMode = (pose_ik != 0);
  if(pose_objects == 0) {
    if(allWidgets.widgets.back() == &allObjectWidgets)
      //disable object widgets
      allWidgets.widgets.resize(allWidgets.widgets.size()-1);
  }
  else {
    if(allWidgets.widgets.back() != &allObjectWidgets)
      //enable object widgets
      allWidgets.widgets.push_back(&allObjectWidgets);
  }

  if(button == GLUT_RIGHT_BUTTON) {
    if(allWidgets.hasFocus) {
      allWidgets.EndDrag();
      allWidgets.SetHighlight(false);
      allWidgets.SetFocus(false);
      
      double d;
      if(allWidgets.Hover(x,viewport.h-y,viewport,d))
	allWidgets.SetHighlight(true);
      else
	allWidgets.SetHighlight(false);
      if(allWidgets.requestRedraw) { SendRefresh(); allWidgets.requestRedraw=false; }
    }
  }
}

void SimTestBackend::DoFreeDrag(int dx,int dy,int button)
{
  dragWidget.active = (forceApplicationMode==1);
  for(size_t i=0;i<robotWidgets.size();i++)
    robotWidgets[i].poseIKMode = (pose_ik != 0);
  if(pose_objects == 0) {
    if(allWidgets.widgets.back() == &allObjectWidgets)
      //disable object widgets
      allWidgets.widgets.resize(allWidgets.widgets.size()-1);
  }
  else {
    if(allWidgets.widgets.back() != &allObjectWidgets)
      //enable object widgets
      allWidgets.widgets.push_back(&allObjectWidgets);
  }

  Robot* robot = world->robots[0].robot;
  if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
  else if(button == GLUT_RIGHT_BUTTON) {
    if(allWidgets.hasFocus) {
      allWidgets.Drag(dx,-dy,viewport);
      if(allWidgets.requestRedraw) {
	allWidgets.requestRedraw = false;
	SendRefresh();
	SendCommand("update_config","");
      }
      //instead of updating object pose in model, update it in simulation
      if(allObjectWidgets.hasFocus) {
	for(size_t i=0;i<objectWidgets.size();i++)
	  if(objectWidgets[i].hasFocus) {
	    sim.odesim.object(i)->SetTransform(world->rigidObjects[i].object->T);
	    sim.odesim.object(i)->SetVelocity(Vector3(0.0),Vector3(0.0));
	  }
      }
    }
  }
}


void SimTestBackend::SimStep(Real dt) 
{
  if(allWidgets.activeWidget == &dragWidget && dragWidget.dragging) {
    forceSpringActive = true;
    double dragForce = double(settings["dragForceMultiplier"]);
    ODEObjectID obj = sim.WorldToODEID(dragWidget.hoverID);
    dBodyID body;
    RigidTransform T;
    if(obj.IsRigidObject()) {
      body = sim.odesim.object(obj.index)->body();
      sim.odesim.object(obj.index)->GetTransform(T);
    }
    else if(obj.IsRobot()) {
      body = sim.odesim.robot(obj.index)->baseBody(obj.bodyIndex);
      sim.odesim.robot(obj.index)->GetLinkTransform(obj.bodyIndex,T);
    }
    Vector3 wp = T*dragWidget.hoverPt;
    sim.hooks.push_back(new SpringHook(body,wp,dragWidget.dragPt,dragForce));
  }
  else
    forceSpringActive = false;
    
  sim.Advance(dt);

  //logging
  if(!simLogFile.empty()) DoLogging(simLogFile.c_str());
  if(!contactStateLogFile.empty()) DoContactStateLogging(contactStateLogFile.c_str());
  if(!contactWrenchLogFile.empty()) DoContactWrenchLogging(contactWrenchLogFile.c_str());
  //TODO: robot linear path logging

  if(forceSpringActive)
    sim.hooks.resize(sim.hooks.size()-1);
    
  //update root of poseConfig from simulation
  for(size_t r=0;r<world->robots.size();r++) {
    Robot* robot=world->robots[r].robot;
    robot->UpdateConfig(robotWidgets[r].Pose());
    Vector driverVals(robot->drivers.size());
    for(size_t i=0;i<robot->drivers.size();i++)
      driverVals(i) = robot->GetDriverValue(i);
    sim.UpdateRobot(r);
    for(size_t i=0;i<robot->drivers.size();i++)
      robot->SetDriverValue(i,driverVals(i));
    robotWidgets[r].SetPose(robot->q);
  }

  //update object configurations from simulation
  sim.UpdateModel();
  for(size_t i=0;i<world->rigidObjects.size();i++)
    objectWidgets[i].SetPose(world->rigidObjects[i].object->T);

  SendCommand("update_sim_time",LexicalCast(sim.time));
}

void SimTestBackend::SensorPlotUpdate()
{
  RobotSensors& sensors = sim.controlSimulators[0].sensors;
  SensorBase* sensor = NULL;
  for(size_t i=0;i<sensorPlots.size();i++) {
    SensorPlot& sensorPlot = sensorPlots[i];
    sensor = sensors.sensors[sensorPlot.sensorIndex];
      
    sensorPlot.view.xmin = sim.time - Real(settings["sensorPlot"]["duration"]);
    sensorPlot.view.xmax = sim.time;
      
    vector<double> values;
    sensor->GetMeasurements(values);
    for(size_t i=0;i<values.size();i++) {
      sensorPlot.view.AddPoint(sim.time,values[i],i);
    }
  }
}

bool SimTestBackend::OnIdle() {
  bool res=BaseT::OnIdle();
  if(simulate) {
    double dt=settings["updateStep"];
    Timer timer;
    SimStep(dt);
    SendRefresh();

    SensorPlotUpdate();

    SendPauseIdle(Max(0.0,dt-timer.ElapsedTime()));
    return true;
  }
  return res;
}



#ifdef HAVE_GLUI

void delete_all(GLUI_Listbox* listbox)
{
  while(listbox->items_list.first_child()) {
    listbox->delete_item(((GLUI_Listbox_Item*)listbox->items_list.first_child())->id);
  }
}

GLUISimTestGUI::GLUISimTestGUI(GenericBackendBase* _backend,RobotWorld* _world,int w,int h)
  :world(_world)
{
  BaseT::backend = _backend;
  BaseT::width = w;
  BaseT::height = h;
  SimTestBackend* sbackend = dynamic_cast<SimTestBackend*>(_backend);
  Assert(sbackend != NULL);
  sim = &sbackend->sim;
  sbackend->gui = this;
}

bool GLUISimTestGUI::Initialize()
{
  if(!BaseT::Initialize()) return false;

  settings["movieWidth"] = 640;
  settings["movieHeight"] = 480;
  if(!settings.read("simtest.settings")) {
  }

  glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
  glui->set_main_gfx_window(main_window);
  AddControl(glui->add_button("Simulate"),"simulate");
  AddControl(glui->add_button("Reset"),"reset");
  //we're going to manually intercept this one to resize the screen and start movie mode
  save_movie_button = glui->add_button("Save movie");
  AddControl(save_movie_button,"");
  AddControl(glui->add_checkbox("Log simulation state"),"do_logging");
  AddControl(glui->add_checkbox("Log contact state"),"do_contact_state_logging");
  AddControl(glui->add_checkbox("Log contact wrenches"),"do_contact_wrench_logging");
  GLUI_Panel* panel = glui->add_rollout("Input/output");
  AddControl(glui->add_edittext_to_panel(panel,"File"),"load_text");
  AddControl(glui->add_button_to_panel(panel,"Load"),"load_file");
  AddControl(glui->add_button_to_panel(panel,"Save state"),"save_state");

  AddControl(glui->add_checkbox("Force application mode"),"force_application_mode");
  GLUI_Checkbox* checkbox = glui->add_checkbox("Draw poser");
  checkbox->set_int_val(1);
  AddControl(checkbox,"draw_poser");
  checkbox = glui->add_checkbox("Draw desired");
  AddControl(checkbox,"draw_desired");
  checkbox = glui->add_checkbox("Draw estimated");
  AddControl(checkbox,"draw_estimated");
  checkbox = glui->add_checkbox("Draw bboxes");
  AddControl(checkbox,"draw_bbs");
  checkbox = glui->add_checkbox("Draw contacts");
  AddControl(checkbox,"draw_contacts");
  checkbox = glui->add_checkbox("Draw wrenches");
  checkbox->set_int_val(1);
  AddControl(checkbox,"draw_wrenches");
  checkbox = glui->add_checkbox("Draw expanded");
  AddControl(checkbox,"draw_expanded");


  map<string,string> csettings = sim->robotControllers[0]->Settings();
  controllerCommands = sim->robotControllers[0]->Commands();
  panel = glui->add_rollout("Controller settings");
  if(!csettings.empty()) {
    for(map<string,string>::iterator i=csettings.begin();i!=csettings.end();i++) {
      controllerSettings.push_back(i->first);
    }
    controllerSettingIndex = 0;
    settingsBox = glui->add_listbox_to_panel(panel,"Setting",&controllerSettingIndex);
    for(size_t i=0;i<controllerSettings.size();i++) {
      settingsBox->add_item((int)i,controllerSettings[i].c_str());
    }
    settingEdit=glui->add_edittext_to_panel(panel,"Value");
    AddControl(settingsBox,"controller_setting");
    AddControl(settingEdit,"controller_setting_value");
  }
  controllerCommandIndex = 0;
  GLUI_Listbox* commandsBox = glui->add_listbox_to_panel(panel,"Command",&controllerCommandIndex);
  for(size_t i=0;i<controllerCommands.size();i++) {
    commandsBox->add_item((int)i,controllerCommands[i].c_str());
  }
  commandEdit = glui->add_edittext_to_panel(panel,"Arg",GLUI_EDITTEXT_TEXT);
  commandEdit->set_text("");
  AddControl(commandsBox,"controller_command");
  AddControl(commandEdit,"controller_command_arg");
  
  //sensors panel
  RobotSensors& sensors = sim->controlSimulators[0].sensors;
  //record default state
  sensorDrawn.resize(sensors.sensors.size(),false);
  sensorMeasurementDrawn.resize(sensors.sensors.size());
  for(size_t i=0;i<sensors.sensors.size();i++) {
    vector<string> names;
    sensors.sensors[i]->MeasurementNames(names);
    sensorMeasurementDrawn[i].resize(names.size(),true);
  }
  //add sensors
  panel = glui->add_rollout("Sensors");
  sensorSelectIndex = 0;
  sensorMeasurementSelectIndex = 0;
  sensorBox = glui->add_listbox_to_panel(panel,"Sensor",&sensorSelectIndex);
  for(size_t i=0;i<sensors.sensors.size();i++)
    sensorBox->add_item(i,sensors.sensors[i]->name.c_str());
  toggleSensorDrawCheckbox = glui->add_checkbox_to_panel(panel,"Show");
  measurementListbox = glui->add_listbox_to_panel(panel,"Measurement",&sensorMeasurementSelectIndex);
  toggleMeasurementDrawCheckbox = glui->add_checkbox_to_panel(panel,"Plot value");
  isolateMeasurementButton = glui->add_button_to_panel(panel,"Isolate");
  //we'll have to intercept these signals manually and dispatch the right command
  AddControl(sensorBox,"");
  AddControl(toggleSensorDrawCheckbox,"");
  AddControl(measurementListbox,"");
  AddControl(toggleMeasurementDrawCheckbox,"");
  AddControl(isolateMeasurementButton,"");

  //posing panel
  panel = glui->add_rollout("Robot posing");

  AddControl(glui->add_checkbox_to_panel(panel,"Pose objects"),"pose_objects");
  AddControl(glui->add_checkbox_to_panel(panel,"Pose by IK"),"pose_ik");
  driver_listbox = glui->add_listbox_to_panel(panel,"Driver",&cur_driver);
  Robot* robot = world->robots[0].robot;
  for(size_t i=0;i<robot->drivers.size();i++) {
    char buf[256];
    strcpy(buf,robot->driverNames[i].c_str());
    driver_listbox->add_item(i,buf);
  }
  AddControl(driver_listbox,"driver");

  driver_value_spinner = glui->add_spinner_to_panel(panel,"Value",GLUI_SPINNER_FLOAT);
  AddControl(driver_value_spinner,"driver_value");
  AddControl(glui->add_button_to_panel(panel,"Set milestone"),"set_milestone");

  UpdateGUI();

  const static int NR = 20;
  const static char* rules [NR*3]= {"{type:key_down,key:c}","constrain_link","",
				    "{type:key_down,key:d}","delete_constraint","",
				    "{type:key_down,key:p}","print_config","",
				    "{type:key_down,key:a}","advance","",
				    "{type:key_down,key:\" \"}","command_pose","",
				    "{type:key_down,key:v}","save_view","view.txt",
				    "{type:key_down,key:V}","load_view","view.txt",
				    "{type:button_press,button:simulate}","toggle_simulate","",
				    "{type:button_press,button:reset}","reset","",
				    "{type:button_press,button:set_milestone}","command_pose","",
				    "{type:button_toggle,button:do_logging,checked:1}","log_sim","simtest_log.csv",
				    "{type:button_toggle,button:do_logging,checked:0}","log_sim","",
				    "{type:button_toggle,button:do_contact_state_logging,checked:1}","log_contact_state","simtest_contact_log.csv",
				    "{type:button_toggle,button:do_contact_state_logging,checked:0}","log_contact_state","",
				    "{type:button_toggle,button:do_contact_wrench_logging,checked:1}","log_contact_wrenches","simtest_wrench_log.csv",
				    "{type:button_toggle,button:do_contact_wrench_logging,checked:0}","log_contact_wrenches","",
				    "{type:widget_value,widget:link,value:_0}","set_link","_0",
				    "{type:widget_value,widget:link_value,value:_0}","set_link_value","_0",
				    "{type:widget_value,widget:driver,value:_0}","set_driver","_0",
				    "{type:widget_value,widget:driver_value,value:_0}","set_driver_value","_0",
  };
  for(int i=0;i<NR;i++) {
    AnyCollection c;
    bool res=c.read(rules[i*3]);
    Assert(res == true);
    AddCommandRule(c,rules[i*3+1],rules[i*3+2]);
  }
  return true;
}

void GLUISimTestGUI::UpdateGUI()
{
  Robot* robot = world->robots[0].robot;
  if(cur_driver >= 0 && cur_driver < (int)robot->drivers.size()) {
    driver_listbox->set_int_val(cur_driver);
    Vector2 limits = robot->GetDriverLimits(cur_driver);
    driver_value_spinner->set_float_limits(limits.x,limits.y);
    double driver_value = robot->GetDriverValue(cur_driver);
    driver_value_spinner->set_float_val(driver_value);
  }
  UpdateSensorGUI();
  UpdateControllerSettingGUI();
}

void GLUISimTestGUI::UpdateControllerSettingGUI()
{
  if(controllerSettingIndex >= 0 && controllerSettingIndex < (int)controllerSettings.size()) {
    string value;
    bool res=sim->robotControllers[0]->GetSetting(controllerSettings[controllerSettingIndex],value);
    if(!res) printf("Failed to get setting %s\n",controllerSettings[controllerSettingIndex].c_str());
    else
      settingEdit->set_text(value.c_str());
  }
}

void GLUISimTestGUI::UpdateSensorGUI()
{
  if(sensorSelectIndex >= 0 && sensorSelectIndex < (int)sensorDrawn.size()) {
    //remember whether the sensor is shown, restore gui widget state
    if(sensorDrawn[sensorSelectIndex]) 
      toggleSensorDrawCheckbox->set_int_val(1);
    else
      toggleSensorDrawCheckbox->set_int_val(0);
    delete_all(measurementListbox);
    //set up the measurements
    RobotSensors& sensors = sim->controlSimulators[0].sensors;
    vector<string> names;
    sensors.sensors[sensorSelectIndex]->MeasurementNames(names);
    for(size_t i=0;i<names.size();i++)
      measurementListbox->add_item(i,names[i].c_str());
    sensorMeasurementSelectIndex = 0;
    UpdateSensorMeasurementGUI();
  }
}

void GLUISimTestGUI::UpdateSensorMeasurementGUI()
{
  if(sensorSelectIndex >= 0 && sensorSelectIndex < (int)sensorDrawn.size() && sensorMeasurementSelectIndex >= 0 && sensorMeasurementSelectIndex < (int)sensorMeasurementDrawn[sensorSelectIndex].size()) {
    
    //remember whether the measurement is shown, restore gui widget state
    if(sensorMeasurementDrawn[sensorSelectIndex][sensorMeasurementSelectIndex]) 
      toggleMeasurementDrawCheckbox->set_int_val(1);
    else
      toggleMeasurementDrawCheckbox->set_int_val(0);
  }
}

void GLUISimTestGUI::Handle_Control(int id)
{
  if(controls[id]==save_movie_button) {
    //resize for movie
    {
      GLint vp[4];
      glGetIntegerv(GL_VIEWPORT,vp);
      int x=vp[0];
      int y=vp[1];
      int width = vp[2];
      int height = vp[3];

      int totalw = glutGet(GLUT_WINDOW_WIDTH);
      int totalh = glutGet(GLUT_WINDOW_HEIGHT);
      int toolbarw = totalw - width;
      int toolbarh = totalh - height;
      glutReshapeWindow(toolbarw+int(settings["movieWidth"]),toolbarh+int(settings["movieHeight"]));
    }
    ToggleMovie();
    return;
  }
  else if(controls[id]==commandEdit) {
    if(controllerCommandIndex >= 0 && controllerCommandIndex < controllerCommands.size()) {
      bool res=sim->robotControllers[0]->SendCommand(controllerCommands[controllerCommandIndex],commandEdit->get_text());
      if(!res) printf("Failed to send command %s(%s)\n",controllerCommands[controllerCommandIndex].c_str(),commandEdit->get_text());
    }
    return;
  }
  else if(controls[id]==settingsBox) {
    UpdateControllerSettingGUI();
  }
  else if(controls[id]==settingEdit) {
    if(controllerSettingIndex >= 0 && controllerSettingIndex<(int)controllerSettings.size()) {
      bool res=sim->robotControllers[0]->SetSetting(controllerSettings[controllerSettingIndex],settingEdit->get_text());
      if(!res) printf("Failed to set setting %s\n",controllerSettings[controllerSettingIndex].c_str());
    }
    return;
  }
  else if(controls[id]==sensorBox) {
    UpdateSensorGUI();
  }
  else if(controls[id]==toggleSensorDrawCheckbox) {
    if(sensorSelectIndex >= 0 && sensorSelectIndex < (int)sensorDrawn.size()) {
      if(toggleSensorDrawCheckbox->get_int_val()!=0) {
	sensorDrawn[sensorSelectIndex]=1;
	SendCommand("show_sensor",sensorSelectIndex);
      }
      else {
	sensorDrawn[sensorSelectIndex]=0;
	SendCommand("hide_sensor",sensorSelectIndex);
      }
    }
  }
  else if(controls[id]==measurementListbox) {
    UpdateSensorMeasurementGUI();

  }
  else if(controls[id]==toggleMeasurementDrawCheckbox) {
    if(sensorSelectIndex >= 0 && sensorSelectIndex < (int)sensorDrawn.size() && sensorMeasurementSelectIndex >= 0 && sensorMeasurementSelectIndex < (int)sensorMeasurementDrawn[sensorSelectIndex].size()) {
      if(toggleMeasurementDrawCheckbox->get_int_val()!=0) {
	sensorMeasurementDrawn[sensorSelectIndex][sensorMeasurementSelectIndex]=1;
	SendCommand("show_sensor_measurement",sensorSelectIndex,sensorMeasurementSelectIndex);
      }
      else {
	sensorMeasurementDrawn[sensorSelectIndex][sensorMeasurementSelectIndex]=0;
	SendCommand("hide_sensor_measurement",sensorSelectIndex,sensorMeasurementSelectIndex);
      }
    }
  }
  else if(controls[id]==isolateMeasurementButton) {
    if(sensorSelectIndex >= 0 && sensorSelectIndex < (int)sensorDrawn.size() && sensorMeasurementSelectIndex >= 0 && sensorMeasurementSelectIndex < (int)sensorMeasurementDrawn[sensorSelectIndex].size()) {
      for(size_t i=0;i<sensorMeasurementDrawn[sensorSelectIndex].size();i++) {
	if((int)i != sensorMeasurementSelectIndex) {
	  sensorMeasurementDrawn[sensorSelectIndex][i] = 0;
	  SendCommand("hide_sensor_measurement",sensorSelectIndex,i);
	}
	else {
	  sensorMeasurementDrawn[sensorSelectIndex][i] = 1;
	  SendCommand("show_sensor_measurement",sensorSelectIndex,i);
	}
      }
    }
  }

  BaseT::Handle_Control(id);
  if(controls[id]==driver_listbox)
    UpdateGUI();
}

void GLUISimTestGUI::Handle_Keypress(unsigned char c,int x,int y)
{
  switch(c) {
  case 'h':
    printf("Keyboard help:\n");
    printf("[space]: sends the milestone to the controller\n");
    printf("s: toggles simulation\n");
    printf("a: advances the simulation one step\n");
    printf("c: in IK mode, constrains link rotation and position\n");
    printf("d: in IK mode, deletes an ik constraint\n");
    printf("v: save current viewport\n");
    printf("V: load viewport\n");
    break;
  default:
    BaseT::Handle_Keypress(c,x,y);
  }
}

bool GLUISimTestGUI::OnCommand(const string& cmd,const string& args)
{
  if(cmd=="update_config") {
    UpdateGUI();
    return true;
  }
  else if(cmd=="update_sim_time") {
    double time;
    if(LexicalCast<double>(args,time)) {
      MovieUpdate(time);
    }
    return true;
  }
  else return BaseT::OnCommand(cmd,args);
}



#endif

