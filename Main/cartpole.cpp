#include "Simulation/WorldSimulation.h"
#include "Simulation/TabulatedController.h"
#include <utils/indexing.h>
#include <utils/stringutils.h>
#include "Xml/XmlWorld.h"
#include "Xml/XmlODE.h"
#include "WorldViewProgram.h"
#include <fstream>

void OptimizeCartPole(Robot& robot)
{
  TabulatedController controller(robot);
  Assert(robot.q.n==2);
  Vector bmin(4),bmax(4),h(4);
  bmin.copySubVector(0,robot.qMin);
  bmax.copySubVector(0,robot.qMax);
  bmin.copySubVector(robot.q.n,robot.velMin);
  bmax.copySubVector(robot.q.n,robot.velMax);
  h(0) = h(1) = 0.25;
  h(2) = h(3) = 0.5;
  Vector qdes=robot.q,w(2);
  w(0) = 0.1;
  w(1) = 1.0;
  controller.commands.grid.h = h;
  controller.commands.grid.PointToIndex(bmin,controller.commands.imin);
  controller.commands.grid.PointToIndex(bmax,controller.commands.imax);
  controller.commands.Init(controller.commands.imin,controller.commands.imax);
  OptimizeMDP(controller,qdes,w,100,0.999);
  ofstream out("cartpole.policy",ios::out);
  controller.Save(out);
  out.close();
}

void OptimizeSwingUp(Robot& robot)
{
  TabulatedController controller(robot);
  Assert(robot.q.n==1);
  Vector bmin(2),bmax(2),h(2);
  bmin.copySubVector(0,robot.qMin);
  bmax.copySubVector(0,robot.qMax);
  bmin.copySubVector(robot.q.n,robot.velMin);
  bmax.copySubVector(robot.q.n,robot.velMax);
  h(0)=0.1;
  //h(1)=0.2;
  h(1)=0.1;
  Vector qdes=robot.q,w(1);
  w(0) = 1.0;
  controller.commands.grid.h = h;
  controller.commands.grid.PointToIndex(bmin,controller.commands.imin);
  controller.commands.grid.PointToIndex(bmax,controller.commands.imax);
  controller.commands.Init(controller.commands.imin,controller.commands.imax);
  OptimizeMDP(controller,qdes,w,20,0.999);
  ofstream out("swingup.policy",ios::out);
  controller.Save(out);
  out.close();
}



typedef TabulatedController MyController;
inline MyController* GetController(RobotController* rc)
{
  return dynamic_cast<MyController*>(rc);
}
inline RobotController* MakeController(Robot* robot,const char* file)
{
  TabulatedController* c = new TabulatedController(*robot);
  ifstream in(file,ios::in);
  if(!in) {
    fprintf(stderr,"Unable to open policy file %s\n",file);
    exit(-1);
  }
  if(!c->Load(in)) {
    fprintf(stderr,"Error loading policy from file %s\n",file);
    exit(-1);
  }
  return c;
}
inline void MakeSensors(Robot* robot,RobotSensors& sensors)
{
  sensors.hasJointPosition=true;
  sensors.hasJointVelocity=true;
}



enum {
  SIMULATE_BUTTON_ID,
  SAVE_MOVIE_BUTTON_ID,
  RESET_BUTTON_ID,
  SETTINGS_LISTBOX_ID,
  SETTINGS_ID,
};


class SimProgram : public WorldViewProgram
{
public:
  int simulate;
  WorldSimulation* sim;
  string initialState;

  //GUI state
  GLUI* glui;

  RobotInfo* hoverRobot;
  RigidObjectInfo* hoverObject;
  int hoverLink;
  Vector3 hoverPt;
  bool forceApplicationMode,forceSpringActive;
  Vector3 forceSpringAnchor;
  int drawContacts;
  vector<string> controllerSettings;
  int controllerSettingIndex;
  GLUI_EditText* settingEdit;

  SimProgram(RobotWorld* world,WorldSimulation* _sim)
    :WorldViewProgram(world),sim(_sim)
  {}

  virtual bool Initialize()
  {
    drawContacts = 1;
    forceApplicationMode = false, forceSpringActive = false;
    map<string,string> settings = GetController(sim->robotControllers[0])->Settings();
    simulate=0;

    //world-object
    for(size_t i=0;i<world->rigidObjects.size();i++) 
      sim->EnableContactFeedback(world->RigidObjectID(i),world->TerrainID(0));
    //robot-object
    for(size_t i=0;i<world->rigidObjects.size();i++) {
      for(size_t j=0;j<world->robots[0].robot->links.size();j++) {
	sim->EnableContactFeedback(world->RigidObjectID(i),world->RobotLinkID(0,j));
      }
    }
    for(size_t i=0;i<world->terrains.size();i++) {
      for(size_t j=0;j<world->robots[0].robot->links.size();j++) {
	sim->EnableContactFeedback(world->TerrainID(i),world->RobotLinkID(0,j));
      }
    }

    printf("Writing initial state\n");
    sim->WriteState(initialState);
    printf("Wrote %d bytes\n",initialState.length());

    hoverRobot=NULL;
    hoverObject=NULL;
    hoverLink=-1;
    hoverPt.setZero();

    if(!WorldViewProgram::Initialize()) return false;

    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);
    glui->add_button("Simulate",SIMULATE_BUTTON_ID,ControlFunc);
    glui->add_button("Reset",RESET_BUTTON_ID,ControlFunc);
    glui->add_button("Save movie",SAVE_MOVIE_BUTTON_ID,ControlFunc);
    glui->add_checkbox("Draw contacts",&drawContacts);
    if(!settings.empty()) {
      GLUI_Panel* panel = glui->add_panel("Controller settings");
      for(map<string,string>::iterator i=settings.begin();i!=settings.end();i++) {
	controllerSettings.push_back(i->first);
      }
      controllerSettingIndex = 0;
      GLUI_Listbox* settingsBox = glui->add_listbox_to_panel(panel,"Setting",&controllerSettingIndex,SETTINGS_LISTBOX_ID,ControlFunc);
      for(size_t i=0;i<controllerSettings.size();i++) {
	settingsBox->add_item((int)i,controllerSettings[i].c_str());
      }
      settingEdit=glui->add_edittext_to_panel(panel,"Value",GLUI_EDITTEXT_TEXT,NULL,SETTINGS_ID,ControlFunc);
      settingEdit->set_text(settings[controllerSettings[0]].c_str());
    }
    
    printf("Starting glui...\n");
    return true;
  }

  virtual void RenderWorld()
  {
    //WorldViewProgram::RenderWorld();
    glDisable(GL_LIGHTING);
    drawCoords(0.1);
    glEnable(GL_LIGHTING);
    for(size_t i=0;i<world->terrains.size();i++)
      world->terrains[i].view.Draw();
    for(size_t i=0;i<world->rigidObjects.size();i++)
      world->rigidObjects[i].view.Draw();

    glEnable(GL_BLEND);
    glEnable(GL_LIGHTING);
    for(size_t i=0;i<world->robots.size();i++) {
      for(size_t j=0;j<world->robots[i].robot->links.size();j++) {
	RigidTransform T;
	sim->odesim.robot(i)->GetLinkTransform(j,T);
	glPushMatrix();
	glMultMatrix(Matrix4(T));
	float color[4] = {0.5,0.5,0.5,1.0};
	if(i==0) {
	  Real kg=sim->ContactForce(world->RobotLinkID(i,j)).norm()/9.8;
	  Assert(!(kg < 0.0));
	  kg /= 10.0;
	  if(kg < 1.0) { //grey->green
	    color[0]=0.5-kg*0.5;
	    color[1]=0.5+kg*0.5;
	    color[2]=0.5-kg*0.5;
	  }
	  else if(kg < 2.5) { //green->yellow
	    Real u=(kg-1.0)/1.5;
	    color[0]=u;
	    color[1]=1.0;
	    color[2]=0;
	  }
	  else if(kg < 10.0) { //yellow->red
	    Real u=(kg-2.5)/7.5;
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
	glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color); 
	world->robots[i].view.DrawLink_Local(j);
	glPopMatrix();
      }
    }
    Robot* robot=world->robots[0].robot;
    RobotController* rc=sim->robotControllers[0];
    //applying a force
    if(forceSpringActive && hoverRobot && hoverLink >= 0) {
      Robot* robot=hoverRobot->robot;
      RigidTransform T;
      sim->odesim.robot(0)->GetLinkTransform(hoverLink,T);
      Vector3 wp = T*hoverPt;
      glDisable(GL_LIGHTING);
      glColor3f(1,0.5,0);
      glPointSize(5.0);
      glBegin(GL_POINTS);
      glVertex3v(wp);
      glVertex3v(forceSpringAnchor);
      glEnd();
      glLineWidth(3.0);
      glBegin(GL_LINES);
      glVertex3v(wp);
      glVertex3v(forceSpringAnchor);
      glEnd();
      glLineWidth(1.0);
      glEnable(GL_LIGHTING);
    }

    //draw collision feedback
    glDisable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    if(drawContacts) {
      for (WorldSimulation::ContactFeedbackMap::iterator i = sim->contactFeedback.begin(); i != sim->contactFeedback.end(); i++) {
	ODEContactList* c = sim->odesim.GetContactFeedback(i->first.first,
							  i->first.second);
	Assert(c != NULL);
	glColor3f(1,1,0);
	glPointSize(5.0);
	glBegin(GL_POINTS);
	for(size_t j=0;j<c->points.size();j++) 
	  glVertex3v(c->points[j].x);
	glEnd();
	Real scale=0.1;
	glBegin(GL_LINES);
	for(size_t j=0;j<c->points.size();j++) {
	  glVertex3v(c->points[j].x);
	  glVertex3v(c->points[j].x+scale*c->points[j].n);
	}
	glEnd();
	glColor3f(1,0.5,0);
	Assert(c->forces.size() <= c->points.size());
	Real fscale=0.1;
	glBegin(GL_LINES);
	for(size_t j=0;j<c->forces.size();j++) {
	  glVertex3v(c->points[j].x);
	  glVertex3v(c->points[j].x+fscale*c->forces[j]);
	}
	glEnd();
      }
    }
  }

  virtual void Handle_Control(int id)
  {
    switch(id) {
    case SIMULATE_BUTTON_ID:
      if(simulate) {
	simulate=0;
	SleepIdleCallback();
      }
      else {
	simulate=1;
	SleepIdleCallback(0);
      }
      break;
    case RESET_BUTTON_ID:
      simulate = 0;
      if(!sim->ReadState(initialState)) {
	fprintf(stderr,"Warning, ReadState doesn't work\n");
      }
      Refresh();
      break;
    case SAVE_MOVIE_BUTTON_ID:
      //resize for movie
      {
	int totalw = glutGet(GLUT_WINDOW_WIDTH);
	int totalh = glutGet(GLUT_WINDOW_HEIGHT);
	int toolbarw = totalw - viewport.w;
	int toolbarh = totalh - viewport.h;
	glutReshapeWindow(toolbarw+MOVIE_W,toolbarh+MOVIE_H);
      }
      ToggleMovie();
      break;
    case SETTINGS_LISTBOX_ID:
      {
	string value;
	bool res=GetController(sim->robotControllers[0])->GetSetting(controllerSettings[controllerSettingIndex],value);
	if(!res) printf("Failed to get setting %s\n",controllerSettings[controllerSettingIndex].c_str());
	else
	  settingEdit->set_text(value.c_str());
      }
    case SETTINGS_ID:
      {
	bool res=GetController(sim->robotControllers[0])->SetSetting(controllerSettings[controllerSettingIndex],settingEdit->get_text());
	if(!res) printf("Failed to set setting %s\n",controllerSettings[controllerSettingIndex].c_str());
      }
      break;
    }
  }
  
  virtual void BeginDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      if(hoverObject) {
	//TODO: apply force to the object
      }
    }
    else {
      WorldViewProgram::BeginDrag(x,y,button,modifiers);
    }
  }

  virtual void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      if(hoverRobot && hoverLink != -1) {
	forceSpringActive = false;
      }
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      if(hoverRobot && hoverLink != -1) {
	if(forceApplicationMode) {
	  Robot* robot=hoverRobot->robot;
	  Vector3 wp = robot->links[hoverLink].T_World*hoverPt;
	  Vector3 ofs;
	  Vector3 vv;
	  viewport.getViewVector(vv);
	  Vector3 cp,cv;
	  viewport.getClickSource(oldmousex+dx,viewport.h-(oldmousey+dy),cp);
	  viewport.getClickVector(oldmousex+dx,viewport.h-(oldmousey+dy),cv);
	  //vv^T (wp-cp) = vv^T cv*t
	  Real t = vv.dot(wp-cp)/vv.dot(cv);
	  forceSpringAnchor = cp + cv*t;
	  forceSpringActive = true;
	}
	Refresh();
      }
    }
  }

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
    if(button == GLUT_LEFT_BUTTON) { camera.dist *= (1 + 0.01*Real(dy)); }
  }

  void Handle_Motion(int x, int y)
  {
    Ray3D r;
    ClickRay(x,y,r);
    int body;
    Vector3 rlocalpt,olocalpt;
    Real robDist = Inf;
    Real objDist = Inf;

    //check click on the the desired configurations
    for(size_t k=0;k<world->robots.size();k++) {
      sim->UpdateRobot(k);
    }
    RobotInfo* rob = ClickRobot(r,body,rlocalpt);
    RigidObjectInfo* obj = ClickObject(r,olocalpt);
    if(rob) 
      robDist = r.closestPointParameter(rob->robot->links[body].T_World*rlocalpt);
    if(obj) 
      objDist = r.closestPointParameter(obj->object->T*olocalpt);
    if(objDist < robDist) rob=NULL;
    else obj=NULL;
    if(rob) {
      if(hoverRobot != rob || hoverLink != body) Refresh();
      hoverRobot = rob;
      hoverLink = body;
      hoverPt = rlocalpt;
    }
    else {
      if(hoverRobot != NULL)  Refresh();
      hoverRobot = NULL;
      hoverLink = -1;
    }
    if(obj) {
      if(hoverObject != obj) Refresh();
      hoverObject = obj;
      hoverPt = olocalpt;
    }
    else {
      if(hoverObject != NULL)  Refresh();
      hoverObject = NULL;
    }
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    switch(key) {
    case 'h':
      printf("Keyboard help:\n");
      printf("[space]: sends the milestone to the controller\n");
      printf("s: toggles simulation\n");
      printf("a: advances the simulation one step\n");
      printf("f: toggles force application mode (right click and drag)\n");
      break;
    case 'a':
      sim->Advance(sim->simStep);
      break;
    case 'f':
      forceApplicationMode = !forceApplicationMode;
      if(forceApplicationMode) {
	printf("Force application mode\n");
      }
      else {
	printf("Joint control mode\n");
      }
      break;
    case 's':
      if(simulate) {
	simulate=0;
	SleepIdleCallback();
      }
      else {
	simulate=1;
	SleepIdleCallback(0);
      }
      break;
    }
    Refresh();
  }

  void SimStep(Real dt) {
    if(forceSpringActive) {
      assert(hoverRobot && hoverLink != -1);
      int robotIndex = 0;
      //TODO: more than one robot
      Real mass = hoverRobot->robot->GetTotalMass();
      dBodyID body = sim->odesim.robot(robotIndex)->baseBody(hoverLink);
      Vector3 wp = hoverRobot->robot->links[hoverLink].T_World*hoverPt;
      sim->hooks.push_back(new SpringHook(body,wp,forceSpringAnchor,mass));
    }
    sim->Advance(dt);
    if(forceSpringActive)
      sim->hooks.resize(sim->hooks.size()-1);
  }

  virtual void Handle_Idle() {
    if(simulate) {
      double dt=0.01;
      Timer timer;
      SimStep(dt);
      Refresh();

      MovieUpdate(sim->time);
      if(!saveMovie) SleepIdleCallback(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
    }
    WorldViewProgram::Handle_Idle();
  }
};


int main(int argc, char** argv)
{  
  if(argc < 2) {
    printf("USAGE: CartPole [options] [robot, terrain, object, world files]\n");
    return 0;
  }
  XmlWorld xmlWorld;
  RobotWorld world;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1,1,1));
  world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world.lights[0].setColor(GLColor(1,1,1));

  for(int i=1;i<argc;i++) {
    if(argv[i][0] == '-') {
      printf("Unknown option %s",argv[i]);
      return 1;
    }
    else {
      const char* ext=FileExtension(argv[i]);
      if(0==strcmp(ext,"xml")) {
	if(!xmlWorld.Load(argv[i])) {
	  printf("Error loading world file %s\n",argv[i]);
	  return 1;
	}
	if(!xmlWorld.GetWorld(world)) {
	  printf("Error loading world from %s\n",argv[i]);
	  return 1;
	}
      }
      else {
	if(world.LoadElement(argv[i]) < 0) {
	  return 1;
	}
      }
    }
  }

  Config qorig = world.robots[0].robot->q;

  //OptimizeSwingUp(*world.robots[0].robot);
  //OptimizeCartPole(*world.robots[0].robot);

  world.robots[0].robot->q(0) = 3.0*Pi/2.0;
  world.robots[0].robot->UpdateFrames();
  //world.robots[0].robot->UpdateConfig(qorig);
  world.robots[0].robot->dq.setZero();

  WorldSimulation sim;
  sim.Init(&world);
  //setup controllers and sensing parameters
  sim.robotControllers.resize(world.robots.size());
  for(size_t i=0;i<sim.robotControllers.size();i++) {
    Robot* robot=world.robots[i].robot;
    sim.SetController(i,MakeController(robot,"swingup.policy")); 
    //sim.SetController(i,MakeController(robot,"cartpole.policy")); 
    MakeSensors(robot,sim.controlSimulators[i].sensors);
  }

  //setup settings, if any
  TiXmlElement* e=xmlWorld.GetElement("simulation");
  if(e) {
    printf("Reading simulation settings...\n");
    XmlSimulationSettings s(e);
    if(!s.GetSettings(sim)) {
      fprintf(stderr,"Warning, simulation settings not read correctly\n");
    }
  }

  SimProgram program(&world,&sim);
  return program.Run("Cart-pole simulation");
}
