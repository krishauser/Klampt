#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "SimulationGUI.h"
#include "Control/JointSensors.h"
#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Control/SerialController.h"
#include "Modeling/MultiPath.h"
#include "IO/XmlWorld.h"
#include "IO/XmlODE.h"
#include "IO/ROS.h"
#include "Planning/RobotTimeScaling.h"
#include "View/ViewWrench.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/drawgeometry.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <KrisLibrary/GLdraw/GLLight.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <KrisLibrary/utils/stringutils.h>
#include <fstream>
#include <iomanip>
using namespace GLDraw;

typedef LoggingController MyController;
typedef PolynomialPathController MyMilestoneController;

bool SimGUIBackend::OnCommand(const string& cmd,const string& args)
{
  stringstream ss(args);
  if(cmd=="simulate") {
    ss >> simulate;
    if(simulate==0) {
      SendPauseIdle();
    }
    else {
      SendPauseIdle(0);
    }
  }
  if(cmd=="toggle_simulate") {
    if(simulate==0) {
      simulate = 1;
      SendPauseIdle(0);
    }
    else {
      simulate = 0;
      SendPauseIdle();
    }
  }
  else if(cmd=="reset") {
    simulate = 0;
    ResetSim();
    sim.UpdateModel();
  }
  else if(cmd=="load_file") {
    LoadFile(args.c_str());
  }
  else if(cmd=="load_path") {
    LoadPath(args.c_str());
  }
  else if(cmd=="save_state") {
    File f;
    if(!f.Open((char *)args.c_str(),FILEWRITE)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, couldn't open file "<<args.c_str());
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"Writing simulation state to "<<args.c_str());
      sim.WriteState(f);
      f.Close();
    }
  }
  else if(cmd=="load_state") {
    if(!LoadState(args.c_str())) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load file "<<args.c_str());
    }
  }
  else if(cmd=="connect_serial_controller") {
    stringstream ss(args);
    int robot,port;
    Real rate;
    ss>>robot>>port>>rate;
    if(!ss) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't parse arguments to connect_serial_controller\n");
      return false;
    }
    if(robot < 0 || robot >= (int)world->robots.size()) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid robot argument to connect_serial_controller\n");
      return false;
    }
    if(port < 0 || port > 0xffff) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid port argument to connect_serial_controller\n");
      return false;
    }
    if(rate <= 0 || rate > 1000) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Invalid rate argument to connect_serial_controller\n");
      return false;
    }
    ConnectSerialController(robot,port,rate);
    return true;
  }
  else if(cmd == "output_ros") {
    stringstream ss(args);
    string prefix;
    ss >> prefix;
    if(ss) {
            if(!OutputROS(prefix.c_str())) { LOG4CXX_ERROR(KrisLibrary::logger(),"Error starting ROS output\n"); }
    }
    else {
            if(!OutputROS()) { LOG4CXX_ERROR(KrisLibrary::logger(),"Error starting ROS output\n"); }
    }
    return true;
  }
  else {
    return BaseT::OnCommand(cmd,args);
  }
  SendRefresh();
  return true;
}


void SimGUIBackend::InitController(int i)
{
  Robot* robot=world->robots[i];
  sim.SetController(i,MakeDefaultController(robot)); 
  sim.controlSimulators[i].sensors.MakeDefault(robot);
}

void SimGUIBackend::InitContactFeedbackAll()
{
  //world-object
  for(size_t i=0;i<world->rigidObjects.size();i++) 
    sim.EnableContactFeedback(world->RigidObjectID(i),world->TerrainID(0));
  //robot-object
  for(size_t i=0;i<world->rigidObjects.size();i++) {
    for(size_t r=0;r<world->robots.size();r++) {
      for(size_t j=0;j<world->robots[r]->links.size();j++) {
	sim.EnableContactFeedback(world->RigidObjectID(i),world->RobotLinkID(r,j));
      }
    }
  }
  for(size_t i=0;i<world->rigidObjects.size();i++) {
    for(size_t j=i+1;j<world->rigidObjects.size();j++) {
      sim.EnableContactFeedback(world->RigidObjectID(i),world->RigidObjectID(j));
    }
  }
  for(size_t i=0;i<world->terrains.size();i++) {
    for(size_t r=0;r<world->robots.size();r++) {
      for(size_t j=0;j<world->robots[r]->links.size();j++) {
	sim.EnableContactFeedback(world->TerrainID(i),world->RobotLinkID(r,j));
      }
    }
  }
}

void SimGUIBackend::ConnectSerialController(int i,int port,Real writeRate)
{
  Robot* robot=world->robots[i];
  stringstream ss;
  ss<<"tcp://localhost:"<<port;
  sim.SetController(i,new SerialController(*robot,ss.str(),writeRate));
}

void SimGUIBackend::InitSim()
{
    sim.Init(world);

    //setup controllers and sensing parameters
    sim.robotControllers.resize(world->robots.size());
    for(size_t i=0;i<sim.robotControllers.size();i++) {    
      InitController(i);
    }

    InitContactFeedbackAll();
}

void SimGUIBackend::ResetSim()
{
  if(!sim.ReadState(initialState)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, ReadState doesn't work\n");
  }
  inContact.clear();
}

bool SimGUIBackend::LoadAndInitSim(const char* xmlFile)
{
  const char* argv[2] = {"SimView",xmlFile};
  return LoadAndInitSim(2,argv);
}

bool SimGUIBackend::LoadAndInitSim(int argc,const char** argv)
{
  XmlWorld xmlWorld;
  vector<string> states;
  vector<string> paths;
  vector<string> milestones;
  vector<string> configs;
  world->lights.resize(1);
  world->lights[0].setColor(GLColor(1,1,1));
  world->lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world->lights[0].setColor(GLColor(1,1,1));

  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-path")) {
	paths.push_back(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-state")) {
	states.push_back(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-milestones")) {
	milestones.push_back(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-config")) {
	configs.push_back(argv[i+1]);
	i++;
      }
      else {
	LOG4CXX_INFO(KrisLibrary::logger(),"Unknown option "<<argv[i]);
	return false;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	if(!xmlWorld.Load(argv[i])) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world file "<<argv[i]);
	  return false;
	}
	if(!xmlWorld.GetWorld(*world)) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading world from "<<argv[i]);
	  return false;
	}
      }
      else {
	if(world->LoadElement(argv[i]) < 0) {
	  return false;
	}
      }
    }
  }

  if(configs.size() > world->robots.size()) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, too many configs specified\n");
  }
  for(size_t i=0;i<configs.size();i++) {
    if(i >= world->robots.size()) break;
    ifstream in(configs[i].c_str(),ios::in);
    if(!in) LOG4CXX_INFO(KrisLibrary::logger(),"Could not open config file "<<configs[i].c_str());
    Vector temp;
    in >> temp;
    if(!in) LOG4CXX_ERROR(KrisLibrary::logger(),"Error reading config file "<<configs[i].c_str());
    if(temp.n != (int)world->robots[i]->links.size()) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Incorrect number of DOFs in config "<<i);
      continue;
    }
    world->robots[i]->UpdateConfig(temp);
  }

  //initialize the simulation defaults
  InitSim();

  //setup settings, if any
  if(xmlWorld.elem) {
    TiXmlElement* e=xmlWorld.GetElement("simulation");
    if(e) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Reading simulation settings...\n");
      XmlSimulationSettings s(e);
      if(!s.GetSettings(sim)) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Warning, simulation settings not read correctly\n");
      }
    }
  }

  if(!paths.empty()) {
    const char* ext=FileExtension(paths[0].c_str());
    if(0 == strcmp(ext,"xml")) {
      if(!LoadMultiPath(paths[0].c_str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load MultiPath file "<<paths[0].c_str());
	return false;
      }
    }
    else {
      if(!LoadLinearPath(paths[0].c_str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load linear path file "<<paths[0].c_str());
	return false;
      }
    }
    //TODO: set initial sim state from start of path?
    //this is challenging because the simulation has already been initialized
  }

  if(!milestones.empty()) {
    if(!LoadMilestones(milestones[0].c_str())) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load milestone file "<<milestones[0].c_str());
      return false;
    }
    //TODO: set initial sim state from start of path?
    //this is challenging because the simulation has already been initialized
  }

  if(!states.empty()) {
    if(!LoadState(states[0].c_str())) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"Couldn't load file "<<states[0].c_str());
      return false;
    }
  }

  sim.WriteState(initialState);
  LOG4CXX_INFO(KrisLibrary::logger(),"Simulation initial state: "<<initialState.length());
  return true;
}


void SimGUIBackend::DrawClock(int x,int y)
{
    void* fontface = GLUT_BITMAP_HELVETICA_18;
    //const int fontheight = 18;
    //const int lineSpacing = 36;
    //const int margin = 5;    

    glColor3f(0,0,0);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glRasterPos2i(x,y);
    glutBitmapString(fontface,"Time: ");
    glutBitmapFloat(fontface,sim.time);

    glEnable(GL_DEPTH_TEST);
}

void SimGUIBackend::DrawSensor(int robot,int sensor)
{
  if(robot < 0) {
    for(size_t i=0;i<sim.controlSimulators.size();i++) {
      DrawSensor((int)i);
    }
  }
  else {
    Assert(robot >= 0 && robot < (int)sim.controlSimulators.size());
    int i=robot;
    sim.controlSimulators[i].UpdateRobot();
    if(sensor < 0) {
      for(size_t j=0;j<sim.controlSimulators[i].sensors.sensors.size();j++) {
        vector<double> measurements;
        sim.controlSimulators[i].sensors.sensors[j]->GetMeasurements(measurements);
        sim.controlSimulators[i].sensors.sensors[j]->DrawGL(*sim.controlSimulators[i].robot,measurements);
      }
    }
    else {
      Assert(sensor >= 0 && sensor < (int)sim.controlSimulators[i].sensors.sensors.size());
      int j=sensor;
      vector<double> measurements;
      sim.controlSimulators[i].sensors.sensors[j]->GetMeasurements(measurements);
      sim.controlSimulators[i].sensors.sensors[j]->DrawGL(*sim.controlSimulators[i].robot,measurements);
    }
  }
}

void SimGUIBackend::SetForceColors()
{
  for(size_t i=0;i<world->robots.size();i++) {
    for(size_t j=0;j<world->robots[i]->links.size();j++) {
      float color[4] = {0.5,0.5,0.5,1.0};
      if(i==0) {
	Real kg=sim.ContactForce(world->RobotLinkID(i,j)).norm()/9.8;
	Assert(!(kg < 0.0));
	kg /= world->robots[i]->GetTotalMass();
	Real green = 0.1, yellow = 1.0, red = 1.5; 
	if(kg < green) { //grey->green
	  color[0]=0.5-0.5*kg/green;
	  color[1]=0.5+0.5*kg/green;
	  color[2]=0.5-0.5*kg/green;
	}
	else if(kg < yellow) { //green->yellow
	  Real u=(kg-green)/(yellow-green);
	  color[0]=u;
	  color[1]=1.0;
	  color[2]=0;
	}
	else if(kg < red) { //yellow->red
	  Real u=(kg-yellow)/(red-yellow);
	  color[0]=u;
	  color[1]=1.0-u;
	  color[2]=0;
	}
	else {
	  color[0]=1.0;
	  color[1]=0;
	  color[2]=0;
	}
      }
      world->robotViews[i].SetColor(j,GLColor(color));
    }
  }
}

void SimGUIBackend::SetTorqueColors()
{
  for(size_t i=0;i<world->robots.size();i++) {
    Vector T;
    sim.controlSimulators[i].GetActuatorTorques(T);
    world->robotViews[i].SetTorqueColors(T);
  }
}

void SimGUIBackend::RenderWorld()
{
  //WorldGUIBackend::RenderWorld();
  glDisable(GL_LIGHTING);
  drawCoords(0.1);
  glEnable(GL_LIGHTING);
  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i]->DrawGL();
  for(size_t i=0;i<world->rigidObjects.size();i++)
    world->rigidObjects[i]->DrawGL();

  for(size_t i=0;i<world->robots.size();i++) {
    for(size_t j=0;j<world->robots[i]->links.size();j++) {
      sim.odesim.robot(i)->GetLinkTransform(j,world->robots[i]->links[j].T_World);
      world->robotViews[i].DrawLink_World(j);
    }
  }
}


void SimGUIBackend::DrawContacts(Real pointSize, Real fscale, Real nscale) 
{
  //draw collision feedback
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
  glDisable(GL_DEPTH_TEST);
  glPointSize(pointSize);
  for (WorldSimulation::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
    ODEContactList* c = sim.odesim.GetContactFeedback(i->first.first,
						      i->first.second);
    Assert(c != NULL);
    glColor3f(1,1,0);
    glBegin(GL_POINTS);
    for(size_t j=0;j<c->points.size();j++) 
      glVertex3v(c->points[j].x);
    glEnd();
    glBegin(GL_LINES);
    for(size_t j=0;j<c->points.size();j++) {
      glVertex3v(c->points[j].x);
      glVertex3v(c->points[j].x+nscale*c->points[j].n);
    }
    glEnd();
    glColor3f(1,0.5,0);
    Assert(c->forces.size() <= c->points.size());
    glBegin(GL_LINES);
    for(size_t j=0;j<c->forces.size();j++) {
      glVertex3v(c->points[j].x);
      glVertex3v(c->points[j].x+fscale*c->forces[j]);
    }
    glEnd();
  }
  glEnable(GL_DEPTH_TEST);
}

void SimGUIBackend::DrawWrenches(Real fscale)
{
  glEnable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  for (WorldSimulation::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
    if(!i->second.inContact) continue;
    /*
    ODEContactList* c = sim.odesim.GetContactFeedback(i->first.first,
						      i->first.second);
    Assert(c != NULL);
    Vector3 f(Zero),m(Zero);
    Vector3 center(Zero);
    for(size_t i=0;i<c->points.size();i++) {
      f += c->forces[i];
      center += c->points[i].x;
    }
    center /= c->points.size();
    for(size_t i=0;i<c->points.size();i++) 
      m += cross((c->points[i].x-center),c->forces[i]);
    */
    Vector3 f=i->second.meanForce,m=i->second.meanTorque;
    Vector3 center=i->second.meanPoint;
    ViewWrench w;
    w.fscale = fscale;
    w.mscale = fscale;
    w.DrawGL(center,f,m);
  }
  glEnable(GL_DEPTH_TEST);
}

bool SimGUIBackend::LoadFile(const char* fn)
{
  const char* ext = FileExtension(fn);
  if(0==strcmp(ext,"state"))
    return LoadState(fn);
  else if(0==strcmp(ext,"milestones"))
    return LoadMilestones(fn);
  else if(0==strcmp(ext,"path"))
    return LoadLinearPath(fn);
  else if(0==strcmp(ext,"xml"))
    return LoadMultiPath(fn);
  else if(0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf") || 
	  0==strcmp(ext,"obj") || 0==strcmp(ext,"ext") ||
	  Geometry::AnyGeometry3D::CanLoadExt(ext)) {
    int res = world->LoadElement(fn);
    if(res<0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"SimGUIBackend::LoadFile: Error loading entity "<<fn);
      return false;
    }
    sim.OnAddModel();
    //refresh contact feedback
    InitContactFeedbackAll();
    return true;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"SimGUIBackend::LoadFile: Unknown file extension on "<<fn);
    return false;
  }
}

bool SimGUIBackend::LoadPath(const char* fn)
{
  const char* ext = FileExtension(fn);
  if(0==strcmp(ext,"milestones"))
    return LoadMilestones(fn);
  else if(0==strcmp(ext,"path"))
    return LoadLinearPath(fn);
  else if(0==strcmp(ext,"xml"))
    return LoadMultiPath(fn);
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"SimGUIBackend::LoadPath: Unknown file extension on "<<fn);
    return false;
  }
}

bool SimGUIBackend::LoadMilestones(const char* fn)
{
  vector<Vector> milestones;
  vector<Vector> dmilestones;
  ifstream in(fn,ios::in);
  if(!in) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, couldn't open file "<<fn);
    return false;
  }
  Vector x,dx;
  while(in) {
    in >> x >> dx;
    if(in) {
      milestones.push_back(x);
      dmilestones.push_back(dx);
      Assert(x.n == dx.n);
    }
  }
  if(in.bad()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error during read of file "<<fn);
    return false;
  }
  in.close();
    
  Assert(sim.robotControllers.size()>=1);
  if(sim.robotControllers.size()>1)
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, sending path to robot 0 by default\n");
  for(size_t i=0;i<milestones.size();i++) {
    stringstream ss;
    ss<<milestones[i]<<"\t"<<dmilestones[i];
    if(i==0) {
      if(!sim.robotControllers[0]->SendCommand("set_qv",ss.str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"set_qv command does not work with the robot's controller\n");
	return false;
      }
    }
    else {
      if(!sim.robotControllers[0]->SendCommand("append_qv",ss.str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"append_qv command does not work with the robot's controller\n");
	return false;
      }
    }
  }
  /*
    MyMilestoneController* c=GetMilestoneController();
    ParabolicRamp::DynamicPath path;
    path.Init(world->robots[0]->velMax,world->robots[0]->accMax);
    path.SetMilestones(milestones,dmilestones);
    c->SetPath(path);
  */
  return true;
}

bool SimGUIBackend::LoadState(const char* fn)
{
  string curState;
  sim.WriteState(curState);
  File f;
  if(!f.Open((char *)fn,FILEREAD)) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, couldn't open file "<<fn);
    return false;
  }
  else {
    if(!sim.ReadState(f)) {
      LOG4CXX_WARN(KrisLibrary::logger(),"Warning, couldn't read state from "<<fn);
      sim.ReadState(curState);
      f.Close();
      return false;
    }
    else
      LOG4CXX_INFO(KrisLibrary::logger(),"Loaded simulation state from "<<fn);
    f.Close();
  }
  return true;
}

bool SimGUIBackend::LoadMultiPath(const char* fn,bool constrainedInterpolate,Real interpolateTolerance,Real durationScale)
{
  if(world->robots.size()==0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Cannot match path to a robot, no robots in world\n");
    return false;
  }
  //load and convert MultiPath
  MultiPath path;
  if(!path.Load(fn)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to load file "<<fn);
    return false;
  }
  bool timed = path.HasTiming();
  bool timeOptimizePath = false;
  if(!path.HasConstraints()) constrainedInterpolate = false;
  if(!timed && timeOptimizePath) {
    //this function both discretizes and optimizes at once
    LOG4CXX_INFO(KrisLibrary::logger(),"Discretizing MultiPath by resolution "<<interpolateTolerance<<" and time-optimizing with res "<<0.05);
    if(!GenerateAndTimeOptimizeMultiPath(*world->robots[0],path,interpolateTolerance,0.05)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"   failed!\n");
      return false;
    }
    if(durationScale != 1.0) {
      path.SetDuration(path.Duration()*durationScale);
    }
  }
  else {
    if(constrainedInterpolate) {
      MultiPath dpath;
      LOG4CXX_INFO(KrisLibrary::logger(),"Discretizing MultiPath by resolution "<<interpolateTolerance);
      if(!DiscretizeConstrainedMultiPath(*world->robots[0],path,dpath,interpolateTolerance)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"   failed!\n");
	return false;
      }
      dpath.SetDuration(dpath.Duration());
      path = dpath;
    }
    if(!timed) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Assigning times to MultiPath via smooth timing, duration "<<path.sections.size()*durationScale);
      //path.SetSmoothTiming(path.sections.size()*durationScale,false);
      LOG4CXX_INFO(KrisLibrary::logger(),"Assigning times to MultiPath via linear timing, duration "<<path.Duration()*durationScale);
      path.SetDuration(path.Duration()*durationScale);
    }
    else 
      LOG4CXX_INFO(KrisLibrary::logger(),"Using existing timing in MultiPath, duration "<<path.Duration());
  }

  if(path.HasVelocity()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Path has velocity, doing smooth interpolation\n");
    //hermite interpolators
    LOG4CXX_INFO(KrisLibrary::logger(),"Path time range ["<<path.StartTime()<<","<<path.EndTime());
    bool first=true;
    for(size_t i=0;i<path.sections.size();i++) {
      if(path.HasTiming(i)) {
	for(size_t j=0;j<path.sections[i].milestones.size();j++) {
	  stringstream ss;
	  ss<<path.sections[i].times[j]<<"\t"<<path.sections[i].milestones[j]<<"\t"<<path.sections[i].velocities[j];
	  if(first) {
	    if(!sim.robotControllers[0]->SendCommand("set_tqv",ss.str())) { 
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"set_tqv command failed or does not work with the robot's controller\n");
	      return false;
	    }
	    first = false;
	  }
	  else {
	    if(!sim.robotControllers[0]->SendCommand("append_tqv",ss.str())) { 
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"append_tqv command failed or does not work with the robot's controller\n");
	      return false;
	    }
	  }
	}
      }
      else {
	for(size_t j=0;j<path.sections[i].milestones.size();j++) {
	  stringstream ss;
	  ss<<path.sections[i].milestones[j]<<"\t"<<path.sections[i].velocities[j];
	  if(first) {
	    if(!sim.robotControllers[0]->SendCommand("set_qv",ss.str())) { 
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"set_qv command failed or does not work with the robot's controller\n");
	      return false;
	    }
	    first = false;
	  }
	  else {
	    if(!sim.robotControllers[0]->SendCommand("append_qv",ss.str())) { 
	      	      LOG4CXX_ERROR(KrisLibrary::logger(),"append_qv command failed or does not work with the robot's controller\n");
	      return false;
	    }
	  }
	}
      }
    }
    return true;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"No velocity in path, using piecewise linear evaluation\n");
    vector<Real> times,stimes;
    vector<Vector> milestones,smilestones;  
    for(size_t i=0;i<path.sections.size();i++) {
      path.GetTimedMilestones(stimes,smilestones,i);
      //skip a milestones at each section
      int ofs = (i==0 ? 0 : 1);
      if(i > 0) {
	if(stimes.front() != times.back()) ofs = 0; //some fixed delay
      }
      times.insert(times.end(),stimes.begin()+ofs,stimes.end());
      milestones.insert(milestones.end(),smilestones.begin()+ofs,smilestones.end());
    }
    LOG4CXX_INFO(KrisLibrary::logger(),"Path time range ["<<times.front()<<","<<times.back());
    return SendLinearPath(times,milestones);
  }
}


bool SimGUIBackend::LoadLinearPath(const char* fn)
{
  vector<Real> times;
  vector<Vector> milestones;
  ifstream in(fn,ios::in);
  if(!in) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, couldn't open file "<<fn);
    return false;
  }
  Real t;
  Vector x;
  while(in) {
    in >> t >> x;
    if(in) {
      times.push_back(t);
      milestones.push_back(x);
    }
  }
  if(in.bad()) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error during read of file "<<fn);
    return false;
  }
  in.close();

  return SendLinearPath(times,milestones);
}

bool SimGUIBackend::SendLinearPath(const vector<Real>& times,const vector<Config>& milestones,Real pathDelay) 
{
  Assert(sim.robotControllers.size()>=1);
  if(sim.robotControllers.size()>1)
    LOG4CXX_WARN(KrisLibrary::logger(),"Warning, sending path to robot 0 by default\n");
  //if(sim.time > 0) LOG4CXX_INFO(KrisLibrary::logger(),"SendLinearPath: simulation time offset "<<sim.time);
  for(size_t i=0;i<milestones.size();i++) {
    stringstream ss;
    ss<<fixed<<setprecision(5);
    //TODO: why do you need to add a path delay to the simulation time?
    ss<<sim.time+pathDelay+times[i]<<"\t"<<milestones[i];
    if(i==0) {
      if(!sim.robotControllers[0]->SendCommand("set_tq",ss.str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"set_tq command failed or does not work with the robot's controller\n");
	return false;
      }
    }
    else {
      if(!sim.robotControllers[0]->SendCommand("append_tq",ss.str())) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"append_tq command failed or does not work with the robot's controller\n");
	return false;
      }
    }
  }
  return true;
}

bool SimGUIBackend::OutputROS(const char* prefix)
{
  if(!ROSInit()) return false;
  for(size_t i=0;i<world->robots.size();i++) {
    if(!ROSPublishCommandedJointState(sim.controlSimulators[i],(string(prefix)+"/"+world->robots[i]->name+"/commanded_joint_state").c_str())) return false;
    if(!ROSPublishSensedJointState(sim.controlSimulators[i],(string(prefix)+"/"+world->robots[i]->name+"/sensed_joint_state").c_str())) return false;
    for(size_t j=0;j<sim.controlSimulators[i].sensors.sensors.size();j++) {
      SensorBase* s = sim.controlSimulators[i].sensors.sensors[j];
      if(!ROSPublishSensorMeasurement(s,*world->robots[i],(string(prefix)+"/"+world->robots[i]->name+"/"+s->name).c_str())) {
        LOG4CXX_INFO(KrisLibrary::logger(),"OutputROS: Couldn't publish sensor "<<s->name.c_str());
        return false;
      }
    }
  }
  if(!ROSPublishTransforms(sim,prefix)) return false;
  return true;
}

void SimGUIBackend::DoLogging(const char* fn)
{
  ofstream out(fn,ios::app);
  if(out.tellp()==std::streamoff(0)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving simulation state to "<<fn<<"\n");
    out<<"time,";
    for(size_t i=0;i<world->robots.size();i++) {
      out<<world->robots[i]->name<<"_cmx,";
      out<<world->robots[i]->name<<"_cmy,";
      out<<world->robots[i]->name<<"_cmz,";
      for(size_t j=0;j<world->robots[i]->links.size();j++)
	out<<world->robots[i]->name<<"_q["<<world->robots[i]->linkNames[j]<<"],";
      out<<",";
      for(size_t j=0;j<world->robots[i]->links.size();j++)
	out<<world->robots[i]->name<<"_dq["<<world->robots[i]->linkNames[j]<<"],";
      out<<",";
      for(size_t j=0;j<world->robots[i]->drivers.size();j++)
	out<<world->robots[i]->name<<"_t["<<world->robots[i]->linkNames[world->robots[i]->drivers[j].linkIndices[0]]<<"],";
      out<<",";
      for(size_t j=0;j<sim.controlSimulators[i].sensors.sensors.size();j++) {
	SensorBase* s=sim.controlSimulators[i].sensors.sensors[j];
	vector<string> mnames;
	s->MeasurementNames(mnames);
	for(size_t k=0;k<mnames.size();k++)
	  out<<world->robots[i]->name<<"_"<<s->name<<"["<<mnames[k]<<"],";
	out<<",";
      }
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      out<<world->rigidObjects[i]->name<<"_x,"<<world->rigidObjects[i]->name<<"_y,"<<world->rigidObjects[i]->name<<"_z,"<<world->rigidObjects[i]->name<<"_rz,"<<world->rigidObjects[i]->name<<"_ry,"<<world->rigidObjects[i]->name<<"_rx,"<<world->rigidObjects[i]->name<<"_dx,"<<world->rigidObjects[i]->name<<"_dy,"<<world->rigidObjects[i]->name<<"_dz,"<<world->rigidObjects[i]->name<<"_wz,"<<world->rigidObjects[i]->name<<"_wy,"<<world->rigidObjects[i]->name<<"_wx,";
      out<<",";
    }
    out<<endl;
  }
  out<<sim.time<<",";
  for(size_t i=0;i<world->robots.size();i++) {
    sim.UpdateRobot(i);
    Vector3 com = world->robots[i]->GetCOM();
    out<<com.x<<","<<com.y<<","<<com.z<<",";
    Config q,dq,t;
    sim.controlSimulators[i].GetSimulatedConfig(q);
    sim.controlSimulators[i].GetSimulatedVelocity(dq);
    sim.controlSimulators[i].GetActuatorTorques(t);
    for(size_t j=0;j<world->robots[i]->links.size();j++)
      out<<q[j]<<",";
    out<<",";
    for(size_t j=0;j<world->robots[i]->links.size();j++)
      out<<dq[j]<<",";
    out<<",";
    for(size_t j=0;j<world->robots[i]->drivers.size();j++)
      out<<t[j]<<",";
    out<<",";
    for(size_t j=0;j<sim.controlSimulators[i].sensors.sensors.size();j++) {
      SensorBase* s=sim.controlSimulators[i].sensors.sensors[j];
      vector<double> ms;
      s->GetMeasurements(ms);
      for(size_t k=0;k<ms.size();k++)
	out<<ms[k]<<",";
      out<<",";
    }
  }
  for(size_t i=0;i<world->rigidObjects.size();i++) {
    RigidTransform T;
    Vector3 w,v;
    sim.odesim.object(i)->GetTransform(T);
    sim.odesim.object(i)->GetVelocity(w,v);
    EulerAngleRotation r;
    r.setMatrixZYX(T.R);
    out<<T.t.x<<","<<T.t.y<<","<<T.t.z<<","<<r.x<<","<<r.y<<","<<r.z<<","<<v.x<<","<<v.y<<","<<v.z<<","<<w.x<<","<<w.y<<","<<w.z<<",";
    out<<",";
  }
  out<<endl;
  out.close();
}

void SimGUIBackend::DoCommandLogging_LinearPath(int robot,const char* fn)
{
  Assert(robot >= 0 && robot < (int)world->robots.size());
  ofstream out(fn,ios::app);
  out<<sim.time<<"\t";
  Config q;
  sim.controlSimulators[robot].GetCommandedConfig(q);
  out<<q<<endl;
  out.close();
}

void SimGUIBackend::DoSensorLogging_LinearPath(int robot,const char* fn)
{
  Assert(robot >= 0 && robot < (int)world->robots.size());
  ofstream out(fn,ios::app);
  out<<sim.time<<"\t";
  Config q;
  sim.controlSimulators[robot].GetSensedConfig(q);
  out<<q<<endl;
  out.close();
}

void SimGUIBackend::DoStateLogging_LinearPath(int robot,const char* fn)
{
  Assert(robot >= 0 && robot < (int)world->robots.size());
  ofstream out(fn,ios::app);
  out<<sim.time<<"\t";
  Config q;
  sim.controlSimulators[robot].GetSimulatedConfig(q);
  out<<q<<endl;
  out.close();
}

void SimGUIBackend::DoContactStateLogging(const char* fn)
{
  ofstream out(fn,ios::app);
  if(out.tellp()==std::streamoff(0)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving simulation contact state to "<<fn<<"\n");
    out<<"time,body1,body2,contact"<<endl;
  }
  for(WorldSimulation::ContactFeedbackMap::iterator i=sim.contactFeedback.begin();i!=sim.contactFeedback.end();i++) {
    int aid = sim.ODEToWorldID(i->first.first);
    int bid = sim.ODEToWorldID(i->first.second);
    bool hadContact = sim.HadContact(aid,bid);
    bool hadSeparation = sim.HadSeparation(aid,bid);
    bool nowInContact = sim.InContact(aid,bid);
    bool wasInContact = (inContact.count(pair<int,int>(aid,bid)) != 0);
    if(wasInContact && nowInContact) {
      if(hadSeparation) { //must have separated and contacted within the last time step
	out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<0<<endl;
	out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<1<<endl;
      }
    }
    else if(!wasInContact && !nowInContact) {
      if(hadContact) { //must have contacted and separated witihn the last time step
	out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<1<<endl;
	out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<0<<endl;
      }
    }
    else if(wasInContact && !nowInContact) {
      out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<0<<endl;
    }
    else if(!wasInContact && nowInContact) {
      out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<","<<1<<endl;
    }
    if(nowInContact)
      inContact.insert(pair<int,int>(aid,bid));
    else if(wasInContact)
      inContact.erase(inContact.find(pair<int,int>(aid,bid)));
  }
}

void SimGUIBackend::DoContactWrenchLogging(const char* fn)
{
  ofstream out(fn,ios::app);
  if(out.tellp()==std::streamoff(0)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Saving simulation contact wrenches to "<<fn<<"\n");
    out<<"time,body1,body2,cop x,cop y,cop z,fx,fy,fz,tx,ty,tz"<<endl;
  }
  for(WorldSimulation::ContactFeedbackMap::iterator i=sim.contactFeedback.begin();i!=sim.contactFeedback.end();i++) {
    if(i->second.contactCount==0) continue;
    int aid = sim.ODEToWorldID(i->first.first);
    int bid = sim.ODEToWorldID(i->first.second);
    out<<sim.time<<","<<world->GetName(aid)<<","<<world->GetName(bid)<<",";
    out<<i->second.meanPoint.x<<","<<i->second.meanPoint.y<<","<<i->second.meanPoint.z<<",";
    out<<i->second.meanForce.x<<","<<i->second.meanForce.y<<","<<i->second.meanForce.z<<",";
    out<<i->second.meanTorque.x<<","<<i->second.meanTorque.y<<","<<i->second.meanTorque.z<<endl;
  }
}
