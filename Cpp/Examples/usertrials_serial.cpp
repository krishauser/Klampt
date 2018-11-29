/** This program provides a user interface for controlling a (real) robot 
 * via the Klamp't socket communication protocol.
 */
#include "Interface/UserInterface.h"
#include "Interface/RobotInterface.h"
#include "Control/SerialControlledRobot.h"
#include "Main/WorldViewProgram.h"
#if defined (__APPLE__) || defined (MACOSX)
#include <glui.h>
#else
#include <GL/glui.h>
#endif //__APPLE__ || MACOSX
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

enum {
  UI_LISTBOX_ID,
  CONNECT_BUTTON_ID,
  DISCONNECT_BUTTON_ID,
  COLLISION_MARGIN_SPINNER_ID,
};

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;

//serial communication port
int commsPort = 3456;

struct CommunicationThreadData
{
  RobotController* controller;  //(in)
  bool ready;  //(out)
  SerialControlledRobot* comms; //(out): can be used to stop robot
  Mutex mutex;
};

void* communicationThreadFunc(void* vdata)
{
  CommunicationThreadData* data = (CommunicationThreadData*)vdata;
  data->ready = false;
  char buf[64];
  sprintf(buf,"tcp://localhost:%d",commsPort);
  SerialControlledRobot comms(buf);
  comms.SetMutex(&data->mutex);
  data->comms = &comms;
  comms.Init(&data->controller->robot,data->controller);
  if(comms.Process(Inf)) {
    //run loop
    data->ready = true;
    comms.Run();
    //stopped
    data->ready = false;
  }
  else {
    printf("Communication thread: error processing first message\n");
  }
  return data;
}


class UserTrialProgram : public WorldViewProgram
{
public:
  RobotWorld planningWorld;
  WorldPlannerSettings settings;
  string initialState;

  bool connected;
  CommunicationThreadData communicationData;
  Thread communicationThread;

  shared_ptr<PolynomialPathController> controller;
  shared_ptr<DefaultMotionQueueInterface> robotInterface;
  vector<shared_ptr<RobotUserInterface> > uis;
  int currentUI,oldUI;

  //GUI state
  GLUI* glui;
  GLUI_Listbox* ui_listbox;

  float collisionMargin,oldCollisionMargin;
  int drawDesired,drawPath,drawUI;

  UserTrialProgram(RobotWorld* world)
    :WorldViewProgram(world)
  {
    settings.InitializeDefault(*world);
    collisionMargin = oldCollisionMargin = 0;
  }


  virtual bool Initialize()
  {
    drawDesired = 1;
    drawPath = 0;
    drawUI = 1;

    controller = make_shared<PolynomialPathController>(*world->robots[0]);
    robotInterface = make_shared<DefaultMotionQueueInterface>(controller.get());
    connected = false;
    CopyWorld(*world,planningWorld);
    planningWorld.InitCollisions();
    Robot* robot = planningWorld.robots[0].get();
    for(size_t i=0;i<robot->geometry.size();i++) {
      robot->geometry[i]->margin += collisionMargin;
    }

    uis.resize(0);
    uis.push_back(make_shared<JointCommandInterface>());
    uis.push_back(make_shared<IKCommandInterface>());

    //uis.push_back(make_shared<IKPlannerCommandInterface>());
    //uis.push_back(make_shared<RRTCommandInterface>());

    uis.push_back(make_shared<MTIKPlannerCommandInterface>());
    uis.push_back(make_shared<MTRRTCommandInterface>());

    for(size_t i=0;i<uis.size();i++) {
      uis[i]->world = world;
      uis[i]->robotInterface = robotInterface.get();
      uis[i]->planningWorld = &planningWorld;
      uis[i]->viewport = &viewport;
      uis[i]->settings = &settings;
    }
    currentUI = oldUI = 0;

    if(!WorldViewProgram::Initialize()) return false;

    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);

    glui->add_button("Connect",CONNECT_BUTTON_ID,ControlFunc);
    glui->add_button("Disconnect",DISCONNECT_BUTTON_ID,ControlFunc);

    ui_listbox = glui->add_listbox("UI",&currentUI,UI_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<uis.size();i++) {
      char buf[256];
      strcpy(buf,uis[i]->Description().c_str());
      ui_listbox->add_item(i,buf);
    }

    GLUI_Spinner* spinner = glui->add_spinner("Collision margin",GLUI_SPINNER_FLOAT,&collisionMargin,COLLISION_MARGIN_SPINNER_ID,ControlFunc);
    spinner->set_float_limits(0.0,1.0);

    glui->add_checkbox("Draw desired",&drawDesired);
    glui->add_checkbox("Draw UI",&drawUI);
    glui->add_checkbox("Draw path",&drawPath);

    printf("Done initializing...\n");
    return true;
  }

  virtual void RenderWorld()
  {
    Robot* robot=world->robots[0].get();

    //update actual configuration from sensors
    if(connected && controller->sensors != NULL) {
      communicationData.mutex.lock();
      Config q;
      if(controller->GetSensedConfig(q))
        robot->UpdateConfig(q);
      communicationData.mutex.unlock();
      world->robotViews[0].RestoreAppearance();
    }
    WorldViewProgram::RenderWorld();

    //draw current commanded configuration -- transparent
    if(connected && controller->command != NULL) {
      GLColor newColor(0,1,0,0.5);
      world->robotViews[0].PushAppearance();
      world->robotViews[0].SetColors(newColor);
      Config q;
      communicationData.mutex.lock();
      if(controller->GetCommandedConfig(q)) {
        communicationData.mutex.unlock();
        robot->UpdateConfig(q);
        world->robotViews[0].Draw();
      }
      else
        communicationData.mutex.unlock();
      world->robotViews[0].PopAppearance();
    }

    if(drawDesired && connected) {
      communicationData.mutex.lock();
      Config curBest;
      robotInterface->GetEndConfig(curBest);
      robot->UpdateConfig(curBest); 
      world->robotViews[0].PushAppearance();
      world->robotViews[0].SetColors(GLColor(1,1,0,0.5));
      world->robotViews[0].Draw();
      world->robotViews[0].PopAppearance();
      communicationData.mutex.unlock();
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
    if(drawUI && connected) {
      communicationData.mutex.lock();
      uis[currentUI]->DrawGL();
      communicationData.mutex.unlock();
    }

    //draw desired path
    if(drawPath && connected) {
      communicationData.mutex.lock();
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
      communicationData.mutex.unlock();
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

    glColor3f(1,1,1);
    glRasterPos2i(x,y);
    if(!connected) {
      glutBitmapString(fontface,"Disconnected from robot. Click connect when ready");
      y += lineSpacing;
    } 
    else {
      glutBitmapString(fontface,uis[currentUI]->Instructions().c_str());
      y += lineSpacing;
    }

    glEnable(GL_DEPTH_TEST);
  }

  virtual void Handle_Control(int id)
  {
    switch(id) {
    case CONNECT_BUTTON_ID:
      if(!connected) {
        communicationData.ready = false;
        communicationData.controller = controller.get();
        communicationThread = ThreadStart(communicationThreadFunc,&communicationData);
        while(!communicationData.ready) {
          ThreadSleep(0.1);
        }
        connected = true;

        //activate current UI
        string res=uis[currentUI]->ActivateEvent(true);

        SleepIdleCallback(0);
      }
      break;
    case DISCONNECT_BUTTON_ID:
      if(connected) {
        //activate current UI
        string res=uis[currentUI]->ActivateEvent(false);

        communicationData.comms->Stop();
        while(communicationData.ready) {
          ThreadSleep(0.1);
        }
        connected = false;
        SleepIdleCallback();
      }
      break;
    case UI_LISTBOX_ID:
      {
        if(connected) {
          string res=uis[oldUI]->ActivateEvent(false);
          res=uis[currentUI]->ActivateEvent(true);
          oldUI=currentUI;
        }
      }
      break;
    case COLLISION_MARGIN_SPINNER_ID:
      {
        Robot* robot = planningWorld.robots[0].get();
        for(size_t i=0;i<robot->geometry.size();i++)
          robot->geometry[i]->margin -= oldCollisionMargin;
        for(size_t i=0;i<robot->geometry.size();i++)
          robot->geometry[i]->margin += collisionMargin;
        oldCollisionMargin = collisionMargin;
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
      if(connected) {
        ScopedLock lock(communicationData.mutex);
        string res=uis[currentUI]->MouseInputEvent(0,0,true);
      }
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      if(connected) {
        ScopedLock lock(communicationData.mutex);
        string res=uis[currentUI]->MouseInputEvent(dx,dy,true);
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

  virtual void Handle_Motion(int x,int y)
  {
    if(connected) {
      ScopedLock lock(communicationData.mutex);
      string res=uis[currentUI]->MouseInputEvent(x,y,false);
      Refresh();
    }
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(connected) {
      ScopedLock lock(communicationData.mutex);
      string res=uis[currentUI]->KeypressEvent(key,x,y);
      Refresh();
    }
  }

  virtual void Handle_Idle() {
    Timer timer;
    if(connected) {
      ScopedLock lock(communicationData.mutex);
      string res=uis[currentUI]->UpdateEvent();
    }
    
    //the communication thread is running and happily communicating with the 
    //planner thread
    Refresh();

    SleepIdleCallback(int(Max(0.0,dt-timer.ElapsedTime())*1000.0));
    WorldViewProgram::Handle_Idle();
  }
};


int main(int argc, char** argv)
{
  if(argc < 2) {
    printf("USAGE: UserTrials XML_file\n");
    return 0;
  }
  RobotWorld world;
  world.lights.resize(1);
  world.lights[0].setColor(GLColor(1,1,1));
  world.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));
  world.lights[0].setColor(GLColor(1,1,1));

  XmlWorld xmlWorld;
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
    else {
      printf("Unknown file extension %s on file %s\n",ext,argv[i]);
      return 1;
    }
  }

  UserTrialProgram program(&world);
  return program.Run();
}
