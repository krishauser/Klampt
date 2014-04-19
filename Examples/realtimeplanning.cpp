#include "Interface/UserInterface.h"
#include "Interface/SimRobotInterface.h"
#include "Interface/SimulationGUI.h"
#include "Interface/GLUIGUI.h"
#include "Interface/SimTestGUI.h"
#include <GLdraw/GLScreenshotProgram.h>
#include <utils/StatCollector.h>
#include <GLdraw/drawextra.h>
#include <GL/glui.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

#ifdef CYGWIN
#undef WIN32
#endif // CYGWIN

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;

enum { NEW_TARGET_BUTTON_ID };

///This input processor doesn't do anything except for generate random targets
///for the robot's end effector when Randomize is called
class MyInputProcessor : public InputProcessorBase
{
public:
  MyInputProcessor(int _link=-1)
    :link(_link),target(0.0),newTarget(false)
  {}
  void Randomize(Robot* robot)
  {
    target.x = Rand(-1.0,1.0);
    target.y = Rand(-1.0,1.0);
    target.z = Rand(0,1.0);
    newTarget = true;
  }
  virtual bool HasUpdate() { return newTarget; }
  virtual PlannerObjectiveBase* MakeObjective(Robot* robot)
  { 
    newTarget = false;
    CartesianObjective* pgoal = new CartesianObjective(robot);
    pgoal->ikGoal.link = link;
    if(link < 0) pgoal->ikGoal.link = (int)robot->links.size()-1;
    pgoal->ikGoal.localPosition.setZero();
    pgoal->ikGoal.SetFixedPosition(target);
    return pgoal;
  }
  virtual void DrawGL()
  {
    //draw a sphere at the target
    GLColor col(1,0,0);
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col.rgba);
    glPushMatrix();
    glTranslate(target);
    GLDraw::drawSphere(0.05,16,8);
    glPopMatrix();
  }
  int link;
  Vector3 target;
  bool newTarget;
};

class RealTimePlannerGUIBackend : public SimGUIBackend
{
public:
  WorldPlannerSettings settings;

  SimRobotInterface robotInterface;
  SmartPointer<InputProcessingInterface> ui;
  SmartPointer<InputProcessorBase> inputProcessor;

  Real collisionMargin;
  int drawCommanded,drawDesired,drawPath,drawUI,drawContacts;

  RealTimePlannerGUIBackend(RobotWorld* world)
    :SimGUIBackend(world),robotInterface(&sim)
  {
  }


  void Start()
  {
    settings.InitializeDefault(*world);
    //choose and set the collision avoidance margin
    collisionMargin = 0.01;
    Robot* robot = world->robots[0].robot;
    for(size_t i=0;i<robot->geometry.size();i++)
      robot->geometry[i].margin += collisionMargin;
    drawCommanded = 0;
    drawDesired = 1;
    drawPath = 0;
    drawUI = 1;
    drawContacts = 1;

    MapButtonToggle("draw_commanded",&drawCommanded);
    MapButtonToggle("draw_desired",&drawDesired);
    MapButtonToggle("draw_path",&drawPath);
    MapButtonToggle("draw_ui",&drawUI);
    MapButtonToggle("draw_contacts",&drawContacts);

    //set up user interface
#ifndef WIN32
    ui = new MTRRTCommandInterface;
#else
    ui = new RRTCommandInterface;
#endif //WIN32
    ui->world = world;
    ui->robotInterface = &robotInterface;
    ui->viewport = &viewport;
    ui->settings = &settings;
    inputProcessor = new MyInputProcessor;
    ui->SetProcessor(inputProcessor);

    //activate current UI
    string res=ui->ActivateEvent(true);

    //enable simulation (this flag is in SimulationGUI)
    simulate=1;

    SimGUIBackend::Start();
  }

  virtual void RenderWorld()
  {
    Robot* robot=world->robots[0].robot;
    RobotController* rc=sim.robotControllers[0];

    SimGUIBackend::RenderWorld();

    //draw current commanded configuration -- transparent
    if(drawCommanded) {
      GLColor newColor(0,1,0,0.5);
      world->robots[0].view.SetColors(newColor);
      Config q;
      sim.controlSimulators[0].GetCommandedConfig(q);
      robot->UpdateConfig(q);
      world->robots[0].view.Draw();
    }

    if(drawDesired) {
      Config curBest;
      robotInterface.GetEndConfig(curBest);
      robot->UpdateConfig(curBest); 
      world->robots[0].view.SetColors(GLColor(1,1,0,0.5));
      world->robots[0].view.Draw();
      /*
      if(curGoal) {
	glPointSize(5.0);
	glDisable(GL_LIGHTING);
	glColor3f(1,1,0);
	glBegin(GL_POINTS);
	glVertex3v(curGoal->ikGoal.endPosition);
	glEnd();

	//draw end effector path
	glColor3f(1,0.5,0);
	glBegin(GL_LINE_STRIP);
	for(Real t=c->pathParameter;t<c->ramp.endTime;t+=0.05) {
	  c->ramp.Evaluate(t,robot->q);
	  robot->UpdateFrames();
	  glVertex3v(robot->links[curGoal->ikGoal.link].T_World*curGoal->ikGoal.localPosition);
	}
	for(size_t i=0;i<c->path.ramps.size();i++) {
	  for(Real t=0;t<c->path.ramps[i].endTime;t+=0.05) {
	    c->path.ramps[i].Evaluate(t,robot->q);
	    robot->UpdateFrames();
	    glVertex3v(robot->links[curGoal->ikGoal.link].T_World*curGoal->ikGoal.localPosition);
	  }
	}
	glEnd();
	glEnable(GL_LIGHTING);
      }
      */
    }
    if(drawUI) {
      ui->DrawGL();
    }

    //draw desired path
    if(drawPath) {
      Real tstart = robotInterface.GetCurTime();
      Real tend = robotInterface.GetEndTime();
      Real dt = 0.05;
      //draw end effector path
      glColor3f(1,1,0);
      glLineWidth(2.0);
      glBegin(GL_LINES);
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      for(int i=istart;i<iend;i++) {
	Real t1=i*dt;
	Real t2=t1+0.5*dt;
	robotInterface.GetConfig(t1,robot->q);
	robot->UpdateFrames();
	glVertex3v(robot->links.back().T_World.t);
	robotInterface.GetConfig(t2,robot->q);
	robot->UpdateFrames();
	glVertex3v(robot->links.back().T_World.t);
      }
      glEnd();
    }

    //draw collision feedback
    if(drawContacts) {
      DrawContacts();
    }
  }

  virtual bool OnCommand(const string& cmd,const string& args)
  {
    if(cmd=="new_target") {
      dynamic_cast<MyInputProcessor*>(&*inputProcessor)->Randomize(world->robots[0].robot);
    }
    SendRefresh();
    return true;
  }
  
  void BeginDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
    }
    else {
      SimGUIBackend::BeginDrag(x,y,button,modifiers);
    }
  }

  void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      string res=ui->MouseInputEvent(0,0,true);
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      string res=ui->MouseInputEvent(dx,dy,true);
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

  virtual void Handle_Motion(int x,int y)
  {
    string res=ui->MouseInputEvent(x,y,false);
    SendRefresh();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    string res=ui->KeypressEvent(key,x,y);
    SendRefresh();
  }

  virtual bool OnIdle() {
    bool res=SimGUIBackend::OnIdle();
    if(simulate) {
      Timer timer;
      string res=ui->UpdateEvent();

      Robot* robot = world->robots[0].robot;
      for(size_t i=0;i<robot->geometry.size();i++)
	robot->geometry[i].margin -= collisionMargin;
      sim.Advance(dt);

      for(size_t i=0;i<robot->geometry.size();i++)
	robot->geometry[i].margin += collisionMargin;
      SendRefresh();

      SendPauseIdle(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
      return true;
    }
    return res;
  }
};


class GLUIRealTimePlannerGUI : public GLScreenshotProgram<GLUIGUI>
{
public:
  typedef GLScreenshotProgram<GLUIGUI> BaseT;

  RobotWorld* world;
  WorldSimulation* sim;
  ProgramSettings settings;

  //GUI state
  GLUI* glui;

  GLUIRealTimePlannerGUI(GenericBackendBase* backend,RobotWorld* _world,int w=800,int h=600);
  bool Initialize();
  void UpdateGUI();
  virtual bool OnCommand(const string& cmd,const string& args);
};

GLUIRealTimePlannerGUI::GLUIRealTimePlannerGUI(GenericBackendBase* _backend,RobotWorld* _world,int w,int h)
  :world(_world)
{
  BaseT::backend = _backend;
  BaseT::width = w;
  BaseT::height = h;
  RealTimePlannerGUIBackend* sbackend = dynamic_cast<RealTimePlannerGUIBackend*>(_backend);
  Assert(sbackend != NULL);
  sim = &sbackend->sim;
  sbackend->gui = this;
}

bool GLUIRealTimePlannerGUI::Initialize()
{
  if(!BaseT::Initialize()) return false;

  glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
  glui->set_main_gfx_window(main_window);
  AddControl(glui->add_button("New target"),"new_target");
  AddControl(glui->add_checkbox("Draw desired"),"draw_desired");
  AddControl(glui->add_checkbox("Draw commanded"),"draw_desired");
  AddControl(glui->add_checkbox("Draw UI"),"draw_ui");
  AddControl(glui->add_checkbox("Draw path"),"draw_path");
  AddControl(glui->add_checkbox("Draw contacts"),"draw_contacts");


  AnyCollection c;
  bool res=c.read("{type:button_press,button:new_target}");
  Assert(res == true);
  AddCommandRule(c,"new_target","");
  
  printf("Done initializing...\n");
  return true;
}

bool GLUIRealTimePlannerGUI::OnCommand(const string& cmd,const string& args)
{
  if(cmd=="update_sim_time") {
    double time;
    if(LexicalCast<double>(args,time)) {
      MovieUpdate(time);
    }
    return true;
  }
  else return BaseT::OnCommand(cmd,args);
}

int main(int argc, const char** argv)
{
  if(argc < 2) {
    printf("USAGE: RealTimePlanning [world XML file or element files]\n");
    return 0;
  }

  RobotWorld world;
  RealTimePlannerGUIBackend backend(&world);
  if(!backend.LoadAndInitSim(argc,argv)) {
    return 1;
  }
  GLUIRealTimePlannerGUI gui(&backend,&world);
  gui.SetWindowTitle("Real-time planner");
  gui.Run();
  return 0;
}
