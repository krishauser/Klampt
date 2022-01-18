#include "qrobottestgui.h"

#include <QSettings>
#include <QApplication>
#include <sstream>

QRobotTestGUI::QRobotTestGUI(QKlamptDisplay* _display,GenericBackendBase *_backend, RobotModel* _robot) :
  QKlamptGUIBase(_display,_backend),
  robot(_robot),
  col_out(new CollisionOutput)

{
  const char* rules = "[ \
[{type:key_down,key:c}, {type:command,cmd:constrain_current_link,args:\"\"}],	\
[{type:key_down,key:d}, {type:command,cmd:delete_current_constraint,args:\"\"}], \
[{type:key_down,key:p}, {type:command,cmd:print_pose,args:\"\"}],	\
[{type:key_down,key:z}, {type:command,cmd:undo_pose,args:\"\"}], \
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
}

QRobotTestGUI::~QRobotTestGUI()
{
  delete col_out;
}

void QRobotTestGUI::SetDriver(int index){
    driver_index=index;
    SendCommand("set_driver",index);
    emit UpdateDriverValue();
    emit UpdateDriverParameters();
}

void QRobotTestGUI::SetDriverValue(double val){
    SendCommand("set_driver",driver_index);
    SendCommand("set_driver_value",val);
}


void QRobotTestGUI::SetLink(int index){
    link_index=index;
    SendCommand("set_link",index);
    emit UpdateLinkValue();
    emit UpdateLinkParameters();
}

void QRobotTestGUI::SetLinkValue(double val){
    SendCommand("set_link",link_index);
    SendCommand("set_link_value",val);
}


bool QRobotTestGUI::OnCommand(const string &cmd, const string &args){
  if(cmd=="update_config"){
        UpdateGUI();
        return true;
    }
    else if(cmd=="return_self_collisions"){
        string str=&(args[0]);
        stringstream ss(str);
        stringstream ss_out;
        int temp,temp2;
        while(ss) {
          ss >> temp >> temp2;
          if(!ss) break;
          if(temp < 0 || temp >= (int)robot->links.size() )
            ss_out << temp;
          else
            ss_out << robot->linkNames[temp];
          ss_out << " ";
          if(temp2 < 0 || temp2 >= (int)robot->links.size() )
            ss_out << temp2;
          else
            ss_out << robot->linkNames[temp2];
          ss_out << "\t";
        }
        col_out->SetValue(str,ss_out.str());
        col_out->show();
		return true;
    }
    else return QtGUIBase::OnCommand(cmd,args);
}

void QRobotTestGUI::UpdateGUI(){

    emit UpdateLinkValue();
    emit UpdateLinkParameters();
    emit UpdateDriverValue();
    emit UpdateDriverParameters();
}

void QRobotTestGUI::LoadFile(QString filename){
  QSettings ini(QSettings::IniFormat, QSettings::UserScope,
		QCoreApplication::organizationName(),
		QCoreApplication::applicationName());
  if(filename.isEmpty()){
    QString openDir = ini.value("last_open_resource_directory",".").toString();
    QFileDialog f;
    filename = f.getOpenFileName(0,"Open File",openDir,"");
    if(!filename.isEmpty())
      ini.setValue("last_open_resource_directory",f.directory().absolutePath());
    }
  if(!filename.isEmpty()) {
    opened_file = filename;
    SendCommand("load_file",filename.toStdString());
  }
}

void QRobotTestGUI::ReloadFile()
{
  if(opened_file.isEmpty()) return;
  SendCommand("reload_file",opened_file.toStdString());
}
