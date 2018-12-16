#include "Interface/UserInterface.h"
#include "Interface/RobotInterface.h"
#include "Main/SimViewProgram.h"
#include <KrisLibrary/utils/StatCollector.h>
#if defined (__APPLE__) || defined (MACOSX)
#include <glui.h>
#else
#include <GL/glui.h>
#endif //__APPLE__ || MACOSX
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

enum {
  SIMULATE_BUTTON_ID,
  RESET_BUTTON_ID,
  UI_LISTBOX_ID
};

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;


class UserTrialProgram : public SimViewProgram
{
public:
  RobotWorld planningWorld;
  WorldPlannerSettings settings;
  string initialState;

  string logFile;
  shared_ptr<DefaultMotionQueueInterface> robotInterface;
  vector<shared_ptr<RobotUserInterface> > uis;
  int currentUI,oldUI;

  //GUI state
  GLUI* glui;
  GLUI_Listbox* ui_listbox;

  int drawDesired,drawPath,drawUI,drawContacts;

  UserTrialProgram(RobotWorld* world)
    :SimViewProgram(world)
  {
    settings.InitializeDefault(*world);
    logFile = "trial.log";
  }

  void LogBegin(string parameters="")
  {
    ofstream out(logFile.c_str(),ios::out | ios::app);
    out<<"Begin "<<parameters<<endl;
    out.close();
  }

  void LogActivate(string result)
  {
    ofstream out(logFile.c_str(),ios::out | ios::app);
    out<<"Activate "<<uis[currentUI]->Name()<<" "<<sim.time<<" "<<result<<endl;
    out.close();
  }

  void LogDeactivate(string result)
  {
    ofstream out(logFile.c_str(),ios::out | ios::app);
    out<<"Deactivate "<<uis[currentUI]->Name()<<" "<<sim.time<<" "<<result<<endl;
    out.close();
  }

  void LogUpdate(string result)
  {
    if(!result.empty()) {
      ofstream out(logFile.c_str(),ios::out | ios::app);
      out<<"Update "<<uis[currentUI]->Name()<<" "<<sim.time<<" "<<result<<endl;
      out.close();
    }
  }

  void LogMouseInput(string result)
  {
    if(!result.empty()) {
      ofstream out(logFile.c_str(),ios::out | ios::app);
      out<<"MouseInput "<<uis[currentUI]->Name()<<" "<<sim.time<<" "<<result<<endl;
      out.close();
    }
  }

  void LogKeypress(string result)
  {
    if(!result.empty()) {
      ofstream out(logFile.c_str(),ios::out | ios::app);
      out<<"Keypress "<<uis[currentUI]->Name()<<" "<<sim.time<<" "<<result<<endl;
      out.close();
    }
  }


  virtual bool Initialize()
  {
    drawDesired = 1;
    drawPath = 0;
    drawUI = 1;
    drawContacts = 1;

    robotInterface.reset(new DefaultMotionQueueInterface(GetMotionQueue(sim.robotControllers[0].get())));
    CopyWorld(*world,planningWorld);
    planningWorld.InitCollisions();

    ///Hack to initialize motion queue before the planner tries to get a hold of it
    sim.robotControllers[0]->Update(0); 

    uis.resize(0);
    uis.push_back(make_shared<JointCommandInterface>());
    uis.push_back(make_shared<IKCommandInterface>());
    uis.push_back(make_shared<IKPlannerCommandInterface>());
    uis.push_back(make_shared<RRTCommandInterface>());
#ifndef WIN32
    uis.push_back(make_shared<MTIKPlannerCommandInterface>());
    uis.push_back(make_shared<MTRRTCommandInterface>());
#endif //WIN32
    for(size_t i=0;i<uis.size();i++) {
      uis[i]->world = world;
      uis[i]->robotInterface = &*robotInterface;
      uis[i]->planningWorld = &planningWorld;
      uis[i]->viewport = &viewport;
      uis[i]->settings = &settings;
    }
    currentUI = oldUI = 0;

    //activate current UI
    string res=uis[currentUI]->ActivateEvent(true);
    LogActivate(res);

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
    Robot* robot=world->robots[0].get();
    RobotController* rc=sim.robotControllers[0].get();

    SimViewProgram::RenderWorld();

    //draw current commanded configuration -- transparent
    GLColor newColor(0,1,0,0.5);
    world->robotViews[0].PushAppearance();
    world->robotViews[0].SetColors(newColor);
    Config q;
    sim.controlSimulators[0].GetCommandedConfig(q);
    robot->UpdateConfig(q);
    world->robotViews[0].Draw();
    world->robotViews[0].PopAppearance();

    if(drawDesired) {
      Config curBest;
      robotInterface->GetEndConfig(curBest);
      if(!curBest.empty()) {
        robot->UpdateConfig(curBest); 
        world->robotViews[0].PushAppearance();
        world->robotViews[0].SetColors(GLColor(1,1,0,0.5));
        world->robotViews[0].Draw();
        world->robotViews[0].PopAppearance();
      }
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
      Real tstart = robotInterface->GetCurTime();
      Real tend = robotInterface->GetEndTime();
      Real dt = 0.05;
      //draw end effector path
      glDisable(GL_LIGHTING);
      glColor3f(1,1,0);
      glLineWidth(2.0);
      glBegin(GL_LINES);
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      for(int i=istart;i<iend;i++) {
        Real t1=i*dt;
        Real t2=t1+0.5*dt;
        robotInterface->GetConfig(t1,robot->q);
        robot->UpdateFrames();
        glVertex3v(robot->links.back().T_World.t);
        robotInterface->GetConfig(t2,robot->q);
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
        LogDeactivate(res);
        res=uis[currentUI]->ActivateEvent(true);
        LogActivate(res);
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
      LogMouseInput(res);
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      string res=uis[currentUI]->MouseInputEvent(dx,dy,true);
      LogMouseInput(res);
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
    LogMouseInput(res);
    Refresh();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    string res=uis[currentUI]->KeypressEvent(key,x,y);
    LogKeypress(res);
    Refresh();
  }

  virtual void Handle_Idle() {
    if(simulate) {
      Timer timer;
      string res=uis[currentUI]->UpdateEvent();
      LogUpdate(res);

      sim.Advance(dt);
      Refresh();

      SleepIdleCallback(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
    }
    WorldViewProgram::Handle_Idle();
  }
};


int main(int argc, char** argv)
{
  if(argc < 2) {
    printf("USAGE: UserTrials XML_file [log file]\n");
    return 0;
  }
  RobotWorld world;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1,1,1));
  world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world.lights[0].setColor(GLColor(1,1,1));

  XmlWorld xmlWorld;
  char* logFile = NULL;
  for(int i=1;i<argc;i++) {
    const char* ext=FileExtension(argv[i]);
    if(0==strcmp(ext,"rob")) {
      if(world.LoadRobot(argv[i])<0) {
        printf("Error loading robot file %s\n",argv[i]);
        return 1;
      }
    }
    else if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri")) {
      if(world.LoadTerrain(argv[i])<0) {
        printf("Error loading terrain file %s\n",argv[i]);
        return 1;
      }
    }
    else if(0==strcmp(ext,"obj")) {
      if(world.LoadRigidObject(argv[i])<0) {
        printf("Error loading rigid object file %s\n",argv[i]);
        return 1;
      }
    }
    else if(0==strcmp(ext,"xml")) {
      if(!xmlWorld.Load(argv[i])) {
        printf("Error loading world file %s\n",argv[i]);
        return 1;
      }
      if(!xmlWorld.GetWorld(world)) {
        printf("Error loading world from %s\n",argv[i]);
        return 1;
      }
    }
    else if(0==strcmp(ext,"log")) {
      logFile = argv[i];
    }
    else {
      printf("Unknown file extension %s on file %s\n",ext,argv[i]);
      return 1;
    }
  }

  UserTrialProgram program(&world);
  if(logFile) program.logFile = logFile;
  program.InitSim();

  stringstream params;
  for(int i=1;i<argc;i++)
    params<<argv[i]<<" ";
  program.LogBegin(params.str());
  return program.Run();
}
