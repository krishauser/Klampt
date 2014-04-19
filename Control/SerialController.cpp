#include "SerialController.h"
#include <utils/AnyCollection.h>

SerialController::SerialController(Robot& robot,const string& _servAddr,Real _writeRate)
  :RobotController(robot),servAddr(_servAddr),writeRate(_writeRate),lastWriteTime(0)
{
  if(!servAddr.empty())
    OpenConnection(servAddr);
}

void SerialController::PackSensorData(AnyCollection& data) const
{
  data["t"] = time;
  data["dt"] = 1.0/writeRate;
  for(size_t i=0;i<sensors->sensors.size();i++) {
    vector<double> values;
    sensors->sensors[i]->GetMeasurements(values);
    data[sensors->sensors[i]->name] = AnyCollection(values);
  }
}

void SerialController::Update(Real dt)
{
  RobotController::Update(dt);
#ifndef WIN32
  if(time >= lastWriteTime + 1.0/writeRate) {
    lastWriteTime += 1.0/writeRate;
    if(time >= lastWriteTime + 1.0/writeRate) {
      printf("Warning, next write time %g is less than controller update time %g\n",lastWriteTime+1.0/writeRate,time);
      lastWriteTime = time;
    }
    AnyCollection sensorData;
    PackSensorData(sensorData);
    stringstream ss;
    ss << sensorData;
    if(controllerPipe && controllerPipe->transport->WriteReady()) {
      controllerPipe->SendMessage(ss.str());
    }
  }
  if(controllerPipe && controllerPipe->NewMessageCount() > 0) {
    string scmd = controllerPipe->NewestMessage();
    AnyCollection cmd;
    if(!cmd.read(scmd.c_str())) {
      fprintf(stderr,"SerialController: Unable to parse incoming message %s\n",scmd.c_str());
      return;
    }
    SmartPointer<AnyCollection> qcmdptr = cmd.find("qcmd");
    SmartPointer<AnyCollection> dqcmdptr = cmd.find("dqcmd");
    SmartPointer<AnyCollection> torquecmdptr = cmd.find("torquecmd");
    SmartPointer<AnyCollection> tcmdptr = cmd.find("tcmd");
    if(qcmdptr) {
      vector<Real> qcmd,dqcmd,torquecmd;
      if(!qcmdptr->asvector(qcmd)) {
	fprintf(stderr,"SerialController: qcmd not of proper type\n");
	return;
      }
      if(dqcmdptr) {
	if(!dqcmdptr->asvector(dqcmd)) {
	  fprintf(stderr,"SerialController: dqcmd not of proper type\n");
	  return;
	}
      }
      else
	dqcmd.resize(qcmd.size(),0);
      if(torquecmdptr) {
	if(!torquecmdptr->asvector(torquecmd)) {
	  fprintf(stderr,"SerialController: torquecmd not of proper type\n");
	  return;
	}
      }
      if(torquecmd.empty()) {
	SetPIDCommand(qcmd,dqcmd);
      }
      else
	SetFeedforwardPIDCommand(qcmd,dqcmd,torquecmd);
    }
    else if(dqcmdptr) {
      if(tcmdptr == NULL) {
	fprintf(stderr,"SerialController: dqcmd not given with tcmd\n");
	return;
      }
      FatalError("Velocity commands not implemented yet");
    }
    else if(torquecmdptr) {
      vector<Real> torquecmd;
      if(!torquecmdptr->asvector(torquecmd)) {
	fprintf(stderr,"SerialController: torquecmd not of proper type\n");
	return;
      }
      SetTorqueCommand(torquecmd);
    }
    else {
      fprintf(stderr,"SerialController: message doesn't contain proper command type\n");
      return;
    }
  }
#endif //WIN32
}

void SerialController::Reset()
{
  RobotController::Reset();
  lastWriteTime = 0;
}

map<string,string> SerialController::Settings() const
{
  map<string,string> settings;
  FILL_CONTROLLER_SETTING(settings,servAddr);
  FILL_CONTROLLER_SETTING(settings,writeRate);
  if(controllerPipe) 
    settings["connected"]="1";
  else
    settings["connected"]="0";
  return settings;
}

bool SerialController::GetSetting(const string& name,string& str) const
{
  READ_CONTROLLER_SETTING(servAddr)
  READ_CONTROLLER_SETTING(writeRate)
  if(name=="connected") {
    if(controllerPipe)
      str = "1";
    else
      str = "0";
  }
  return false;
}

bool SerialController::SetSetting(const string& name,const string& str)
{
  if(name == "servAddr") {
    OpenConnection(str);
    return true;
  }
  WRITE_CONTROLLER_SETTING(writeRate)  
  return false;
}

bool SerialController::OpenConnection(const string& addr)
{
  servAddr = addr;
  if(addr.empty()) {
    CloseConnection();
    return true;
  }
  controllerPipe = new SocketPipeWorker(addr.c_str(),true);
  if(!controllerPipe->Start()) {
    cout<<"Controller could not be opened on address "<<addr<<endl;
    return false;
  }
  return true;
}

bool SerialController::CloseConnection()
{
  if(controllerPipe) {
    controllerPipe->Stop();
    controllerPipe = NULL;
    return true;
  }
  return false;
}
