//#if !HAVE_ZMQ
//#error "Must have ZeroMQ installed and enabled in Makefile.config"
//#endif

#include "Interface/UserInterface.h"
#include "Interface/SimRobotInterface.h"
#include "SimViewProgram.h"
#include <utils/AnyCollection.h>
#include <GL/glui.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;
#ifdef CYGWIN
#undef WIN32
#endif // CYGWIN

enum {
  SIMULATE_BUTTON_ID,
  RESET_BUTTON_ID,
  UI_LISTBOX_ID
};

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;





class SafeSerialProgram : public SimViewProgram
{
public:
  //zmq::context_t context;
  WorldPlannerSettings plannerSettings;
  string initialState;

  SimRobotInterface robotInterface;
  vector<SmartPointer<RobotUserInterface> > uis;
  SmartPointer<InputProcessorBase> zmqInputProcessor;
  int currentUI,oldUI;

  //GUI state
  GLUI* glui;
  GLUI_Listbox* ui_listbox;

  int drawDesired,drawPath,drawUI,drawContacts;

  SafeSerialProgram(RobotWorld* world)
    :SimViewProgram(world),/*context(1),*/robotInterface(&sim)
  {
    //setup the settings
    AnyCollection settings;
    ifstream in("safeserialclient.settings",ios::in);
    bool readsettings = false;
    if(in) {
      in>>settings;
      if(in) 
	readsettings = true;
      else
	cerr<<"Error reading settings from safeserialclient.settings"<<endl;
    }
    if(!readsettings) {
      fprintf(stderr,"Need safeserialclient.settings file, copy and paste the following lines into the file.\n");
      settings = AnyCollection();
      settings["objective_address"]=string("tcp://localhost:3456");
      settings["objective_filter"]=string("objective");
      settings["time_publish_address"]=string("tcp://*:3457");
      cerr<<settings<<endl;
      exit(-1);
    }
    string objsubaddr = settings["objective_address"];
    string objfilter = settings["objective_filter"];
    string timepubaddr = settings["time_publish_address"];
    //ZMQObjectiveProcessor* processor = new ZMQObjectiveProcessor(context,objsubaddr.c_str(),(objfilter.empty()?NULL:objfilter.c_str()));
    //if(!timepubaddr.empty())
    //processor->InitTimePublisher(timepubaddr.c_str());
    SocketObjectiveProcessor* processor = new SocketObjectiveProcessor(objsubaddr.c_str());
    zmqInputProcessor = processor;
  }


  virtual bool Initialize()
  {
    drawDesired = 1;
    drawPath = 0;
    drawUI = 1;
    drawContacts = 1;

    plannerSettings.InitializeDefault(*world);
    uis.resize(0);
    uis.push_back(new IKPlannerCommandInterface);
    uis.push_back(new RRTCommandInterface);
#ifndef WIN32
    uis.push_back(new MTIKPlannerCommandInterface);
    uis.push_back(new MTRRTCommandInterface);
#endif //WIN32
    for(size_t i=0;i<uis.size();i++) {
      uis[i]->world = world;
      uis[i]->robotInterface = &robotInterface;
      uis[i]->viewport = &viewport;
      uis[i]->settings = &plannerSettings;
      dynamic_cast<InputProcessingInterface*>((RobotUserInterface*)uis[i])->SetProcessor(zmqInputProcessor);
    }
    currentUI = oldUI = 0;

    //activate current UI
    string res=uis[currentUI]->ActivateEvent(true);

    if(!WorldViewProgram::Initialize()) return false;

    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);
    glui->add_button("Simulate",SIMULATE_BUTTON_ID,ControlFunc);
    glui->add_button("Reset",RESET_BUTTON_ID,ControlFunc);
    ui_listbox = glui->add_listbox("UI",&currentUI,UI_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<uis.size();i++) {
      char buf[256];
      strcpy(buf,uis[i]->Description().c_str());
      ui_listbox->add_item(i,buf);
    }

    glui->add_checkbox("Draw desired",&drawDesired);
    glui->add_checkbox("Draw UI",&drawUI);
    glui->add_checkbox("Draw path",&drawPath);
    glui->add_checkbox("Draw contacts",&drawContacts);

    printf("Done initializing...\n");
    return true;
  }

  virtual void RenderWorld()
  {
    Robot* robot=world->robots[0].robot;
    RobotController* rc=sim.robotControllers[0];

    SimViewProgram::RenderWorld();

    //draw current commanded configuration -- transparent
    GLColor newColor(0,1,0,0.5);
    world->robots[0].view.SetColors(newColor);
    Config q;
    sim.controlSimulators[0].GetCommandedConfig(q);
    robot->UpdateConfig(q);
    world->robots[0].view.Draw();

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
      uis[currentUI]->DrawGL();
    }

    //draw desired path
    if(drawPath) {
      RobotController* rc=sim.robotControllers[0];
      for(size_t i=0;i<robot->drivers.size();i++) {
	robot->SetDriverValue(i,rc->command->actuators[i].qdes);
      }
      robot->UpdateFrames();
      world->robots[0].view.SetColors(GLColor(0,1,0,0.5));
      world->robots[0].view.Draw();
    }

    //draw collision feedback
    if(drawContacts) {
      DrawContacts();
    }
  }

  virtual void RenderScreen()
  {
    void* fontface = GLUT_BITMAP_TIMES_ROMAN_24;
    const int fontheight = 24;
    const int lineSpacing = 36;
    const int margin = 5;
    int x,y;
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    x = 20;
    y = 40;

    if(!simulate) {
      glColor3f(1,1,0);
      glRasterPos2i(x,y);
      glutBitmapString(fontface,"Select a desired UI mode");
      y += lineSpacing;
      glRasterPos2i(x,y);
      glutBitmapString(fontface,"Press Simulate when you are ready");
    }
    else {
      glColor3f(1,1,1);
      glRasterPos2i(x,y);
      glutBitmapString(fontface,uis[currentUI]->Instructions().c_str());
      y += lineSpacing;
    }

    glEnable(GL_DEPTH_TEST);
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
      ResetSim();
      break;
    case UI_LISTBOX_ID:
      {
	string res=uis[oldUI]->ActivateEvent(false);
	res=uis[currentUI]->ActivateEvent(true);
	oldUI=currentUI;
      }
      break;
    }
  }
  
  void BeginDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
    }
    else {
      WorldViewProgram::BeginDrag(x,y,button,modifiers);
    }
  }

  void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      string res=uis[currentUI]->MouseInputEvent(0,0,true);
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      string res=uis[currentUI]->MouseInputEvent(dx,dy,true);
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
    string res=uis[currentUI]->MouseInputEvent(x,y,false);
    Refresh();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    string res=uis[currentUI]->KeypressEvent(key,x,y);
    Refresh();
  }

  virtual void Handle_Idle() {
    if(simulate) {
      Timer timer;
      string res=uis[currentUI]->UpdateEvent();

      sim.Advance(dt);
      Refresh();

      SleepIdleCallback(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
    }
    WorldViewProgram::Handle_Idle();
  }
};


int main(int argc, const char** argv)
{
  if(argc < 2) {
    printf("USAGE: SafeSerialClient XML_file\n");
    return 0;
  }
  RobotWorld world;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1,1,1));
  world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world.lights[0].setColor(GLColor(1,1,1));

  SafeSerialProgram program(&world);
  program.LoadAndInitSim(argc,argv);
  return program.Run();
}
