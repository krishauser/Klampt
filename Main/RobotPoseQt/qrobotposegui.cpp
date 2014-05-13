#include "qrobotposegui.h"

#include <QSettings>
#include <QCoreApplication>

QRobotPoseGUI::QRobotPoseGUI(GenericBackendBase *_backend, RobotWorld *_world) :
    QtGUIBase(_backend,_world)

{
  const char* rules = "[ \
[{type:key_down,key:c}, {type:command,cmd:constrain_current_link,args:\"\"}],	\
[{type:key_down,key:d}, {type:command,cmd:delete_current_constraint,args:\"\"}], \
[{type:key_down,key:p}, {type:command,cmd:print_config,args:\"\"}],	\
[{type:button_press,button:print_config}, {type:command,cmd:print_pose,args:_0}], \
[{type:widget_value,widget:link,value:_0}, {type:command,cmd:set_link,args:_0}], \
[{type:widget_value,widget:link_value,value:_0}, {type:command,cmd:set_link_value,args:_0}], \
[{type:widget_value,widget:driver,value:_0}, {type:command,cmd:set_driver,args:_0}], \
[{type:widget_value,widget:driver_value,value:_0}, {type:command,cmd:set_driver_value,args:_0}] \
]";

  stringstream ss(rules);
  bool res=LoadRules(ss);
  assert(res==true);
  driver_index=0;
  link_index=0;

  RobotPoseBackend* rbackend = dynamic_cast<RobotPoseBackend*>(_backend);
  Assert(rbackend != NULL);

}

void QRobotPoseGUI::SetDriver(int index){
    driver_index=index;
    SendCommand("set_driver",index);
    emit UpdateDriverValue();
    emit UpdateDriverParameters();
}

void QRobotPoseGUI::SetDriverValue(double val){
    SendCommand("set_driver",driver_index);
    SendCommand("set_driver_value",val);
}

void QRobotPoseGUI::SetLink(int index){
    link_index=index;
    SendCommand("set_link",index);
    emit UpdateLinkValue();
    emit UpdateLinkParameters();
}

void QRobotPoseGUI::SetLinkValue(double val){
    SendCommand("set_link",link_index);
    SendCommand("set_link_value",val);
}

void QRobotPoseGUI::SetRecord(bool status)
{
    SendCommand("record",status);
}

void QRobotPoseGUI::RecordFrame(){
    SendCommand("record_take_frame");
}

bool QRobotPoseGUI::OnCommand(const string &cmd, const string &args){
    if(cmd=="update_config"){
        UpdateGUI();
        return true;
    }
    else if (cmd=="inform_new_resource"){
        //from poser to gui
        resource_frame->updateNewResource(args);
    }
    else if (cmd=="enable_path"){
        play_frame->EnablePath(args);
    }
    else return QtGUIBase::OnCommand(cmd,args);
}

void QRobotPoseGUI::UpdateGUI(){

    emit UpdateLinkValue();
    emit UpdateLinkParameters();
    emit UpdateDriverValue();
    emit UpdateDriverParameters();
}

void QRobotPoseGUI::LoadFile(QString filename){
  if(!filename.isEmpty()) {
    string str = filename.toStdString();
    SendCommand("load_file",str);
  }
}

void QRobotPoseGUI::LoadFilePrompt(QString directory_key,QString filter){
  QFileDialog f;
  QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		QCoreApplication::organizationName(),
		QCoreApplication::applicationName());
  QString openDir = ini.value(directory_key,".").toString();
  QString filename = f.getOpenFileName(0,"Open File",openDir,filter);
  if(!filename.isEmpty()){
      ini.setValue(directory_key,QFileInfo(filename).absolutePath());
    LoadFile(filename);
  }
}
