#include "qtguibase.h"

QtGUIBase::QtGUIBase(GenericBackendBase *_backend, RobotWorld *_world) :
    world(_world),GenericGUIBase(_backend)
{
  //BaseT::backend = _backend;
  //  BaseT::width = w;
  //  BaseT::height = h;

  SimTestBackend* sbackend = dynamic_cast<SimTestBackend*>(_backend);
  Assert(sbackend != NULL);
  sim = &sbackend->sim;
  sbackend->gui = this;
  idle_timer=new QTimer();
  connect(idle_timer,SIGNAL(timeout()),this,SLOT(SendIdle()));
  idle_timer->start(100);

  driver_tool=new DriverEdit(world);
  connect(driver_tool,SIGNAL(SetDriverValue(int,float)),this,SLOT(SendDriverValue(int,float)));
  //driver_tool->show();

  log_options=new LogOptions();
  connect(log_options,SIGNAL(ShowSensor(int)),this,SLOT(ShowSensor(int)));
  connect(log_options,SIGNAL(HideSensor(int)),this,SLOT(HideSensor(int)));
  connect(log_options,SIGNAL(toggle_measurement(int,int,bool)),this,SLOT(SendMeasurement(int,int,bool)));
  connect(log_options,SIGNAL(toggle_measurement(int,int,bool)),this,SLOT(SendMeasurement(int,int,bool)));
  RobotSensors sensors=sim->controlSimulators[0].sensors;
  log_options->robotsensors=sensors;
  log_options->GetSensors();
  //log_options->show();
  //UpdateMeasurements();

  controller_settings=new ControllerSettings();
  controller_settings->settings=sim->robotControllers[0]->Settings();
  connect(controller_settings,SIGNAL(SendControllerSetting(string,string)),this,SLOT(SendControllerSetting(string,string)));
  controller_settings->Refresh();
  //controller_settings->show();

  command_dialog=new ControllerCommandDialog();
  command_dialog->commands=sim->robotControllers[0]->Commands();
  connect(command_dialog,SIGNAL(ControllerCommand(string,string)),this,SLOT(SendControllerCommand(string,string)));
  command_dialog->Refresh();

  UpdateGUI();

  const static int NR = 14;
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

  constrain_mode = delete_mode = constrain_point_mode= 0;
}

QtGUIBase::~QtGUIBase(){
    delete log_options;
    delete driver_tool;
}

bool QtGUIBase::OnCommand(const string &cmd, const string &args){
    if(cmd=="update_config"){
        UpdateGUI();
        return true;
    }
    else if(cmd=="update_sim_time"){
        double time;
        return true;
    }
    else return GenericGUIBase::OnCommand(cmd,args);
}

void QtGUIBase::UpdateGUI(){
    //Robot* robot=world->robots[0].robot;
    if(driver_tool->isVisible())
        driver_tool->RequestDriverValue();
}

void QtGUIBase::SendMouseMove(QMouseEvent *e){
    GenericGUIBase::SendMouseMove(e->x(),e->y());
}

void QtGUIBase::SendMousePress(QMouseEvent *e){
    if(constrain_mode){
        SendCommand("constrain_link");
        emit EndConstrain();
        constrain_mode=0;
        return;
    }
    if(constrain_point_mode){
        SendCommand("constrain_link_point");
        emit EndConstrain();
        constrain_point_mode=0;
        return;
    }
    if(delete_mode){
        SendCommand("delete_constraint");
        emit EndDelete();
        delete_mode=0;
        return;
    }
    //    if(e->modifiers()&Qt::CTRL)
//        SendKeyUp("control");
    int button=e->button();
    if(button==1) button=0;
    GenericGUIBase::SendMouseClick(button,1,e->x(),e->y());
    if(e->modifiers()&&Qt::CTRL)
      GenericGUIBase::SendKeyDown("control");
    else
      GenericGUIBase::SendKeyUp("control");
    if(e->modifiers()&&Qt::SHIFT)
      GenericGUIBase::SendKeyDown("shift");
    else
      GenericGUIBase::SendKeyUp("shift");
    if(e->modifiers()&&Qt::ALT)
      GenericGUIBase::SendKeyDown("alt");
    else
      GenericGUIBase::SendKeyUp("alt");
}

void QtGUIBase::SendMouseRelease(QMouseEvent *e){
    int button=e->button();
    if(button==1) button=0;
    GenericGUIBase::SendMouseClick(button,0,e->x(),e->y());
}

void QtGUIBase::SendMouseWheel(QWheelEvent *e){
    GenericGUIBase::SendMouseWheel(e->delta());
}

void QtGUIBase::SendKeyDown(QKeyEvent *e){
    int key = e->key();
    if(key==Qt::Key_Shift || key==Qt::Key_Control){
        return;}
    if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
    GenericGUIBase::SendKeyDown(string(1,key));
}

void QtGUIBase::SendKeyUp(QKeyEvent *e){
    int key = e->key();
    if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
    GenericGUIBase::SendKeyUp(string(1,key));
}

void QtGUIBase::SendIdle(){
    GenericGUIBase::SendIdle();
}

void QtGUIBase::SendDriverValue(int dr, float value){
    SendCommand("set_driver",dr);
    SendCommand("set_driver_value",value);
}

void QtGUIBase::ShowSensor(int sensor){
    SendCommand("show_sensor",sensor);
    log_options->sensorDrawn[sensor]=true;
}

void QtGUIBase::HideSensor(int sensor){
    SendCommand("hide_sensor",sensor);
    log_options->sensorDrawn[sensor]=false;
}

void QtGUIBase::SendMeasurement(int sensor,int measurement,bool status){
    if(status)
        SendCommand("show_sensor_measurement",sensor,measurement);
    else
        SendCommand("hide_sensor_measurement",sensor,measurement);
}


void QtGUIBase::LoadFile(QString filename){
  if(filename.isEmpty()){
    QFileDialog f;
    QString filename = f.getOpenFileName(0,"Open File",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull())
    SendCommand("load_file",filename.toStdString());
}

void QtGUIBase::SaveScenario(QString filename){
  if(filename.isEmpty()){
    QFileDialog f;
    filename = f.getSaveFileName(0,"Save Scenario",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
      SendCommand("save_file",filename.toStdString());
      old_filename=filename;
  }
}

void QtGUIBase::SaveLastScenario(){
    if(old_filename.isNull())
        SaveScenario();
    else
        SaveScenario(old_filename);
}

void QtGUIBase::SendControllerSetting(string setting, string value){
    bool res = sim->robotControllers[0]->SetSetting(setting,value);
    if(!res) printf("Failed to set setting %s\n",setting.c_str());
}

void QtGUIBase::SendControllerCommand(string setting,string value){
    bool res = sim->robotControllers[0]->SendCommand(setting,value);
    if(!res) printf("Failed to send command %s\n",setting.c_str());
}

void QtGUIBase::ShowHelp(){
    QMessageBox *help = new QMessageBox();
    QString text=      "Keyboard help:\n"
      "[space]: sends the milestone to the controller\n"
      "s: toggles simulation\n"
      "a: advances the simulation one step\n"
      "f: toggles force application mode (right click and drag)\n"
      "c: in IK mode, constrains link rotation and position\n"
      "d: in IK mode, deletes an ik constraint\n"
      "v: save current viewport\n"
      "V: load viewport\n";
    help->setText(text);
    help->show();
}

//should be made more informative
void QtGUIBase::ShowAbout(){
    QMessageBox *about = new QMessageBox();
    QString text = "Klamp't\n"
            "A robot simulation, planning, and control package from Indiana University";
    about->setText(text);
    about->show();
}
