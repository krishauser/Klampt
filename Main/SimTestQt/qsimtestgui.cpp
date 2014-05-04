#include "qsimtestgui.h"

QSimTestGUI::QSimTestGUI(GenericBackendBase *_backend, RobotWorld *_world) :
  QtGUIBase(_backend,_world)
{
  //BaseT::backend = _backend;
  //  BaseT::width = w;
  //  BaseT::height = h;

  SimTestBackend* sbackend = dynamic_cast<SimTestBackend*>(_backend);
  Assert(sbackend != NULL);
  sim = &sbackend->sim;
  sbackend->gui = this;

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

  connect_serial = new ConnectSerial(world->robots.size());
  connect(connect_serial,SIGNAL(MakeConnect(int,QString,int,int)),this,SLOT(SendConnection(int,QString,int,int)));

  UpdateGUI();

  const static int NR = 15;
  const static char* rules [NR*3]= {"{type:key_down,key:c}","constrain_link","",
                    "{type:key_down,key:C}","constrain_link_point","",
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

QSimTestGUI::~QSimTestGUI(){
    delete log_options;
    delete driver_tool;
}

bool QSimTestGUI::OnCommand(const string &cmd, const string &args){
    if(cmd=="update_sim_time"){
        double time;
        return true;
    }
    else return QtGUIBase::OnCommand(cmd,args);
}

void QSimTestGUI::UpdateGUI(){
    //Robot* robot=world->robots[0].robot;
    if(driver_tool->isVisible())
        driver_tool->RequestDriverValue();
}

void QSimTestGUI::SendMousePress(QMouseEvent *e){
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
    QtGUIBase::SendMousePress(e);
}

void QSimTestGUI::SendDriverValue(int dr, float value){
    SendCommand("set_driver",dr);
    SendCommand("set_driver_value",value);
}

void QSimTestGUI::ShowSensor(int sensor){
    SendCommand("show_sensor",sensor);
    log_options->sensorDrawn[sensor]=true;
}

void QSimTestGUI::HideSensor(int sensor){
    SendCommand("hide_sensor",sensor);
    log_options->sensorDrawn[sensor]=false;
}

void QSimTestGUI::SendMeasurement(int sensor,int measurement,bool status){
    if(status)
        SendCommand("show_sensor_measurement",sensor,measurement);
    else
        SendCommand("hide_sensor_measurement",sensor,measurement);
}


void QSimTestGUI::LoadFile(QString filename){
  if(!filename.isEmpty()) {
    string str = filename.toStdString();
    SendCommand("load_file",str);
  }
}

void QSimTestGUI::LoadFilePrompt(QString directory,QString filter){
  QFileDialog f;
  QString openDir = ini->value(directory,".").toString();
  QString filename = f.getOpenFileName(0,"Open File",openDir,filter);
  if(!filename.isEmpty()){
    ini->setValue(directory,QFileInfo(filename).absolutePath());
    LoadFile(filename);
  }
}

void QSimTestGUI::SaveScenario(QString filename){
  if(filename.isEmpty()){
    QFileDialog f;
    filename = f.getSaveFileName(0,"Save State",QDir::home().absolutePath(),"");
  }
  if(!filename.isNull()){
    string str = filename.toStdString();
    SendCommand("save_state",str);
    old_filename=filename;
  }
}

void QSimTestGUI::SaveLastScenario(){
    if(old_filename.isNull())
        SaveScenario();
    else
        SaveScenario(old_filename);
}

void QSimTestGUI::SendControllerSetting(string setting, string value){
    bool res = sim->robotControllers[0]->SetSetting(setting,value);
    if(!res) printf("Failed to set setting %s\n",setting.c_str());
}

void QSimTestGUI::SendControllerCommand(string setting,string value){
    bool res = sim->robotControllers[0]->SendCommand(setting,value);
    if(!res) printf("Failed to send command %s\n",setting.c_str());
}

void QSimTestGUI::SendConnection(int robot, QString host, int port, int rate)
{
    stringstream ss;
    ss<<robot<<" "<<host.toStdString()<<" "<<port<<" "<<rate;
    SendCommand("connect_serial_controller",ss.str());
}

void QSimTestGUI::ShowHelp(){
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
void QSimTestGUI::ShowAbout(){
    QMessageBox *about = new QMessageBox();
    QString text = "Klamp't\n"
            "A robot simulation, planning, and control package from Indiana University";
    about->setText(text);
    about->show();
}
