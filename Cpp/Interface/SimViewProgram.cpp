#include "SimViewProgram.h"

#if HAVE_GLUI || HAVE_GLUT

#include <Klampt/Sensing/JointSensors.h>
#include <Klampt/Control/FeedforwardController.h>
#include <Klampt/Control/LoggingController.h>
#include <Klampt/Modeling/MultiPath.h>
#include <Klampt/IO/XmlWorld.h>
#include <Klampt/IO/XmlODE.h>
#include <Klampt/Planning/RobotTimeScaling.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/drawgeometry.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLLight.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <fstream>

using namespace Klampt;

typedef LoggingController MyController;
typedef PolynomialPathController MyMilestoneController;

namespace Klampt {

PolynomialMotionQueue* GetMotionQueue(RobotController* rc)
{
  LoggingController* lc = dynamic_cast<LoggingController*>(rc);
  if(!lc) {
    FatalError("Robot controller isn't a LoggingController");
  }
  FeedforwardController* fc = dynamic_cast<FeedforwardController*>(lc->base.get());
  if(!fc) {
    FatalError("LoggingController base is not a feedforward controller");
  }
  PolynomialPathController* c = dynamic_cast<PolynomialPathController*>(fc->base.get());
  if(!c) {
    FatalError("Feedforward base is not a PolynomialPathController");
  }
  return c;
}

} //namespace Klampt

SimViewProgram::SimViewProgram(WorldModel* world)
  :WorldViewProgram(world),simulate(0)
{}


void SimViewProgram::InitSim()
{
    sim.Init(world);
    //setup controllers and sensing parameters

    sim.robotControllers.resize(world->robots.size());
    for(size_t i=0;i<sim.robotControllers.size();i++) {    
      RobotModel* robot=world->robots[i].get();
      sim.SetController(i,MakeDefaultController(robot)); 
      sim.controlSimulators[i].sensors.MakeDefault(robot);
    }

    //world-object
    for(size_t i=0;i<world->rigidObjects.size();i++) 
      sim.EnableContactFeedback(world->RigidObjectID(i),world->TerrainID(0));
    //robot-object
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      for(size_t j=0;j<world->robots[0]->links.size();j++) {
        sim.EnableContactFeedback(world->RigidObjectID(i),world->RobotLinkID(0,j));
      }
    }
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      for(size_t j=i+1;j<world->rigidObjects.size();j++) {
        sim.EnableContactFeedback(world->RigidObjectID(i),world->RigidObjectID(j));
      }
    }
    for(size_t i=0;i<world->terrains.size();i++) {
      for(size_t j=0;j<world->robots[0]->links.size();j++) {
        sim.EnableContactFeedback(world->TerrainID(i),world->RobotLinkID(0,j));
      }
    }

    sim.WriteState(initialState);
    printf("Simulation initial state: %d bytes\n",initialState.length());
}

void SimViewProgram::ResetSim()
{
  if(!sim.ReadState(initialState)) {
    fprintf(stderr,"Warning, ReadState doesn't work\n");
  }
}

bool SimViewProgram::LoadAndInitSim(const char* xmlFile)
{
  const char* argv[2] = {"SimView",xmlFile};
  return LoadAndInitSim(2,argv);
}

bool SimViewProgram::LoadAndInitSim(int argc,const char** argv)
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
        printf("Unknown option %s",argv[i]);
        return false;
      }
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
        if(!xmlWorld.Load(argv[i])) {
          printf("Error loading world file %s\n",argv[i]);
          return false;
        }
        if(!xmlWorld.GetWorld(*world)) {
          printf("Error loading world from %s\n",argv[i]);
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
    printf("Warning, too many configs specified\n");
  }
  for(size_t i=0;i<configs.size();i++) {
    if(i >= world->robots.size()) break;
    ifstream in(configs[i].c_str(),ios::in);
    if(!in) printf("Could not open config file %s\n",configs[i].c_str());
    Vector temp;
    in >> temp;
    if(!in) printf("Error reading config file %s\n",configs[i].c_str());
    if(temp.n != (int)world->robots[i]->links.size()) {
      printf("Incorrect number of DOFs in config %d\n",i);
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
      printf("Reading simulation settings...\n");
      XmlSimulationSettings s(e);
      if(!s.GetSettings(sim)) {
        fprintf(stderr,"Warning, simulation settings not read correctly\n");
      }
    }
  }

  if(!paths.empty()) {
    const char* ext=FileExtension(paths[0].c_str());
    if(0 == strcmp(ext,"xml")) {
      if(!LoadMultiPath(paths[0].c_str())) {
        fprintf(stderr,"Couldn't load MultiPath file %s\n",paths[0].c_str());
        return false;
      }
    }
    else {
      if(!LoadLinearPath(paths[0].c_str())) {
        fprintf(stderr,"Couldn't load linear path file %s\n",paths[0].c_str());
        return false;
      }
    }
    //TODO: set initial sim state from start of path?
    //this is challenging because the simulation has already been initialized
  }

  if(!milestones.empty()) {
    if(!LoadMilestones(milestones[0].c_str())) {
      fprintf(stderr,"Couldn't load milestone file %s\n",milestones[0].c_str());
      return false;
    }
    //TODO: set initial sim state from start of path?
    //this is challenging because the simulation has already been initialized
  }

  if(!states.empty()) {
    if(!LoadState(states[0].c_str())) {
      fprintf(stderr,"Couldn't load file %s\n",states[0].c_str());
      return false;
    }
  }
  return true;
}



void SimViewProgram::RenderWorld()
{
  //WorldViewProgram::RenderWorld();
  glDisable(GL_LIGHTING);
  drawCoords(0.1);
  glEnable(GL_LIGHTING);
  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i]->DrawGLOpaque(true);
  for(size_t i=0;i<world->rigidObjects.size();i++)
    world->rigidObjects[i]->DrawGLOpaque(true);

  for(size_t i=0;i<world->robots.size();i++) {
    world->robotViews[i].PushAppearance();
    for(size_t j=0;j<world->robots[i]->links.size();j++) {
      sim.odesim.robot(i)->GetLinkTransform(j,world->robots[i]->links[j].T_World);
      if(i==0) {
        float amount = 0;
        float color[4] = {0.5,0.5,0.5,1.0};
        Real kg=sim.ContactForce(world->RobotLinkID(i,j)).norm()/9.8;
        Assert(!(kg < 0.0));
        kg /= world->robots[i]->GetTotalMass();
        Real green = 0.1, yellow = 1.0, red = 1.5; 
        if(kg < green) { //grey->green
          amount = kg/green;
          color[0]=float(0.5-0.5*amount);
          color[1]=float(0.5+0.5*amount);
          color[2]=float(0.5-0.5*amount);
        }
        else if(kg < yellow) { //green->yellow
          amount = 1;
          Real u=(kg-green)/(yellow-green);
          color[0]=u;
          color[1]=1.0;
          color[2]=0;
        }
        else if(kg < red) { //yellow->red
          amount = 1;
          Real u=(kg-yellow)/(red-yellow);
          color[0]=u;
          color[1]=1.0-u;
          color[2]=0;
        }
        else {
          amount = 1;
          color[0]=1.0;
          color[1]=0;
          color[2]=0;
        }
        world->robotViews[i].SetTintColor(j,GLColor(color),amount);
      }
      world->robotViews[i].DrawLink_World(j);
    }
    world->robotViews[i].PopAppearance();
  }

  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i]->DrawGLOpaque(false);
  for(size_t i=0;i<world->rigidObjects.size();i++)
    world->rigidObjects[i]->DrawGLOpaque(false);
}


void SimViewProgram::DrawContacts(Real pointSize, Real fscale, Real nscale) 
{
  //draw collision feedback
  glDisable(GL_LIGHTING);
  glDisable(GL_BLEND);
  glEnable(GL_POINT_SMOOTH);
  glDisable(GL_DEPTH_TEST);
  glPointSize(pointSize);
  for (Simulator::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
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

void SimViewProgram::DrawWrenches(Real fscale)
{
  glEnable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  for (Simulator::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
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
      
    f *= fscale;
    m *= fscale;
      
    GLColor yellow(1,1,0),orange(1,0.5,0),cyan(0,1,1);
      
    glPushMatrix();
    glTranslate(center);
      
    Real r=0.01;
    Real arrowLen = 0.1,arrowWidth=1.7;
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,orange);
    Real len = 0.5*Exp(-f.length()*2.0);
    drawCylinder(f*(1.0-len),r);
    glPushMatrix();
    glTranslate(f*(1.0-len));
    drawCone(f*len,r*arrowWidth,8);
    glPopMatrix();
      
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cyan);
    len = 0.5*Exp(-m.length()*2.0);
    drawCylinder(m*(1.0-len),r);
    glPushMatrix();
    glTranslate(m*(1.0-len));
    drawCone(m*len,r*arrowWidth,8);
    glPopMatrix();
      
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,yellow);
    drawSphere(0.015,16,8);
      
    glPopMatrix();
  }
  glEnable(GL_DEPTH_TEST);
}

bool SimViewProgram::LoadMilestones(const char* fn)
{
  vector<Vector> milestones;
  vector<Vector> dmilestones;
  ifstream in(fn,ios::in);
  if(!in) {
    printf("Warning, couldn't open file %s\n",fn);
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
    printf("Error during read of file %s\n",fn);
    return false;
  }
  in.close();
    
  Assert(sim.robotControllers.size()>=1);
  if(sim.robotControllers.size()>1)
    printf("Warning, sending path to robot 0 by default\n");
  for(size_t i=0;i<milestones.size();i++) {
    stringstream ss;
    ss<<milestones[i]<<"\t"<<dmilestones[i];
    if(i==0) {
      if(!sim.robotControllers[0]->SendCommand("set_qv",ss.str())) {
        fprintf(stderr,"set_qv command does not work with the robot's controller\n");
        return false;
      }
    }
    else {
      if(!sim.robotControllers[0]->SendCommand("append_qv",ss.str())) {
        fprintf(stderr,"append_qv command does not work with the robot's controller\n");
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

bool SimViewProgram::LoadState(const char* fn)
{
  string curState;
  sim.WriteState(curState);
  File f;
  if(!f.Open((char *)fn,FILEREAD)) {
    printf("Warning, couldn't open file %s\n",fn);
    return false;
  }
  else {
    if(!sim.ReadState(f)) {
      printf("Warning, couldn't read state from %s\n",fn);
      sim.ReadState(curState);
      f.Close();
      return false;
    }
    else
      printf("Loaded simulation state from %s\n",fn);
    f.Close();
  }
  return true;
}

bool SimViewProgram::LoadMultiPath(const char* fn,bool constrainedInterpolate,Real interpolateTolerance,Real durationScale)
{
  //load and convert MultiPath
  MultiPath path;
  if(!path.Load(fn)) {
    fprintf(stderr,"Unable to load file %s\n",fn);
    return false;
  }
  bool timed = path.HasTiming();
  bool timeOptimizePath = false;
  if(!timed && timeOptimizePath) {
    //this function both discretizes and optimizes at once
    printf("Discretizing MultiPath by resolution %g and time-optimizing with res %g\n",interpolateTolerance,0.05);
    if(!GenerateAndTimeOptimizeMultiPath(*world->robots[0],path,interpolateTolerance,0.05)) {
      printf("   failed!\n");
      return false;
    }
    if(durationScale != 1.0) {
      path.SetDuration(path.Duration()*durationScale);
    }
  }
  else {
    if(constrainedInterpolate) {
      MultiPath dpath;
      printf("Discretizing MultiPath by resolution %g\n",interpolateTolerance);
      if(!DiscretizeConstrainedMultiPath(*world->robots[0],path,dpath,interpolateTolerance)) {
        printf("   failed!\n");
        return false;
      }
      dpath.SetDuration(dpath.Duration());
      path = dpath;
    }
    if(!timed) {
      //printf("Assigning times to MultiPath via smooth timing, duration %g\n",path.sections.size()*durationScale);
      //path.SetSmoothTiming(path.sections.size()*durationScale,false);
      printf("Assigning times to MultiPath via linear timing, duration %g\n",path.Duration()*durationScale);
      path.SetDuration(path.Duration()*durationScale);
    }
    else 
      printf("Using existing timing in MultiPath, duration %g\n",path.Duration());
  }

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
  printf("Path time range [%g,%g]\n",times.front(),times.back());
  return SendLinearPath(times,milestones);
}


bool SimViewProgram::LoadLinearPath(const char* fn)
{
  vector<Real> times;
  vector<Vector> milestones;
  ifstream in(fn,ios::in);
  if(!in) {
    printf("Warning, couldn't open file %s\n",fn);
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
    printf("Error during read of file %s\n",fn);
    return false;
  }
  in.close();

  return SendLinearPath(times,milestones);
}

bool SimViewProgram::SendLinearPath(const vector<Real>& times,const vector<Config>& milestones,Real pathDelay) 
{
  Assert(sim.robotControllers.size()>=1);
  if(sim.robotControllers.size()>1)
    printf("Warning, sending path to robot 0 by default\n");
  if(sim.time > 0) printf("SendLinearPath: simulation time offset %g\n",sim.time);
  for(size_t i=0;i<milestones.size();i++) {
    stringstream ss;
    //TODO: why do you need to add dt to the simulation time?
    ss<<sim.time+pathDelay+times[i]<<"\t"<<milestones[i];
    if(i==0) {
      if(!sim.robotControllers[0]->SendCommand("set_tq",ss.str())) {
        fprintf(stderr,"set_tq command failed or does not work with the robot's controller\n");
        return false;
      }
    }
    else {
      if(!sim.robotControllers[0]->SendCommand("append_tq",ss.str())) {
        fprintf(stderr,"append_tq command failed or does not work with the robot's controller\n");
        return false;
      }
    }
  }
  return true;
}

void SimViewProgram::DoLogging(const char* fn)
{
  ofstream out(fn,ios::app);
  if(out.tellp()==std::streamoff(0)) {
    cout<<"Saving simulation state to "<<fn<<endl;
    out<<"time,";
    for(size_t i=0;i<world->robots.size();i++) {
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
        SensorBase* s=sim.controlSimulators[i].sensors.sensors[j].get();
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
      SensorBase* s=sim.controlSimulators[i].sensors.sensors[j].get();
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

#endif //HAVE_GLUI || HAVE_GLUT