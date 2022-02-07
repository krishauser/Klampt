#include "SerialController.h"
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <signal.h>
using namespace Klampt;

SerialController::SerialController(RobotModel& robot,const string& _servAddr,Real _writeRate)
  :RobotController(robot),servAddr(_servAddr),writeRate(_writeRate),lastWriteTime(0),endVCmdTime(-1)
{
  //HACK: is this where the sigpipe ignore should be?
#ifndef WIN32
  signal(SIGPIPE, SIG_IGN);
#endif

  if(!servAddr.empty()) {
    while(!OpenConnection(servAddr)) {
      printf("\n...Trying to connect again in 5 seconds...\n");
      ThreadSleep(5);
    }
  }
}

void SerialController::PackSensorData(AnyCollection& data)
{
  data["t"] = time;
  data["dt"] = 1.0/writeRate;

  bool isPID = true;
  for(size_t i=0;i<command->actuators.size();i++) {
    if(command->actuators[i].mode != ActuatorCommand::PID)
      isPID = false;
  }
  if(isPID) {
    Config qcmd,dqcmd;
    GetCommandedConfig(qcmd);
    GetCommandedVelocity(dqcmd);
    data["qcmd"] = vector<double>(qcmd);
    data["dqcmd"] = vector<double>(dqcmd);
  }

  for(size_t i=0;i<sensors->sensors.size();i++) {
    vector<double> values;
    sensors->sensors[i]->GetMeasurements(values);
    data[sensors->sensors[i]->name] = AnyCollection(values);
  }
}

void SerialController::Update(Real dt)
{
  RobotController::Update(dt);
  if(time < endVCmdTime) {
    //do velocity commands
    Assert(!vcmd.empty());
    Config qcmd;
    GetCommandedConfig(qcmd);
    qcmd.madd(vcmd,dt);
    SetPIDCommand(qcmd,vcmd);
  }
  else if(!vcmd.empty()) {
    //stop doing velocity commands
    vcmd.setZero();
    Config qcmd;
    GetCommandedConfig(qcmd);
    SetPIDCommand(qcmd,vcmd);
    vcmd.clear();
  }

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
      controllerPipe->Send(ss.str());
    }
  }
  if(controllerPipe && controllerPipe->UnreadCount() > 0) {
    string scmd = controllerPipe->Newest();
    if(scmd.empty()) return;
    AnyCollection cmd;
    if(!cmd.read(scmd.c_str())) {
      fprintf(stderr,"SerialController: Unable to parse incoming message \"%s\"\n",scmd.c_str());
      return;
    }
    if(cmd.size()==0) {
      return;
    }
    //parse and do error checking
    shared_ptr<AnyCollection> qcmdptr = cmd.find("qcmd");
    shared_ptr<AnyCollection> dqcmdptr = cmd.find("dqcmd");
    shared_ptr<AnyCollection> torquecmdptr = cmd.find("torquecmd");
    shared_ptr<AnyCollection> tcmdptr = cmd.find("tcmd");
    if(qcmdptr) {
      endVCmdTime = -1;
      vcmd.clear();
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
      if(qcmd.size() != robot.q.n) {
	fprintf(stderr,"SerialController: position command of wrong size: %d vs %d \n",(int)qcmd.size(),robot.q.n);
	return;
      }
      if(!dqcmd.empty() && (dqcmd.size() != robot.dq.n)) {
	fprintf(stderr,"SerialController: velocity command of wrong size: %d vs %d \n",(int)dqcmd.size(),robot.q.n);
	return;
      }
      if(!torquecmd.empty() && (torquecmd.size() != robot.drivers.size())) {
	fprintf(stderr,"SerialController: torque command of wrong size: %d vs %d \n",(int)torquecmd.size(),(int)robot.drivers.size());
	return;
      }

      //everything checks out -- now send the command
      if(torquecmd.empty()) {
	SetPIDCommand(Vector(qcmd),Vector(dqcmd));
      }
      else
	SetFeedforwardPIDCommand(Vector(qcmd),Vector(dqcmd),Vector(torquecmd));
    }
    else if(dqcmdptr) {
      if(tcmdptr == NULL) {
	fprintf(stderr,"SerialController: dqcmd not given with tcmd\n");
	return;
      }
      vector<Real> dqcmd;
      if(!dqcmdptr->asvector(dqcmd)) {
	fprintf(stderr,"SerialController: dqcmd not of proper type\n");
	return;
      }
      Real tcmd;
      if(!tcmdptr->as(tcmd)) {
	fprintf(stderr,"SerialController: tcmd not of proper type\n");
	return;
      }
      if(dqcmd.size() != robot.dq.n) {
	fprintf(stderr,"SerialController: velocity command of wrong size: %d vs %d \n",(int)dqcmd.size(),robot.q.n);
	return;
      }
      endVCmdTime = time + tcmd;
      vcmd = dqcmd;
    }
    else if(torquecmdptr) {
      endVCmdTime = -1;
      vcmd.clear();
      vector<Real> torquecmd;
      if(!torquecmdptr->asvector(torquecmd)) {
	fprintf(stderr,"SerialController: torquecmd not of proper type\n");
	return;
      }
      if(!torquecmd.empty() && (torquecmd.size() != robot.drivers.size())) {
	fprintf(stderr,"SerialController: torque command of wrong size: %d vs %d \n",(int)torquecmd.size(),(int)robot.drivers.size());
	return;
      }

      SetTorqueCommand(Vector(torquecmd));
    }
    else {
      fprintf(stderr,"SerialController: message doesn't contain proper command type (qcmd, dqcmd, or torquecmd)\n");
      cout<<"   Message: "<<scmd<<endl;
      return;
    }
  }
}

void SerialController::Reset()
{
  RobotController::Reset();
  lastWriteTime = 0;
  endVCmdTime = -1;
}

map<string,string> SerialController::Settings() const
{
  map<string,string> settings;
  FILL_CONTROLLER_SETTING(settings,servAddr);
  FILL_CONTROLLER_SETTING(settings,writeRate);
  if(controllerPipe) {
    settings["listening"]="1";
  }
  else {
    settings["listening"]="0";
  }
  return settings;
}

bool SerialController::GetSetting(const string& name,string& str) const
{
  READ_CONTROLLER_SETTING(servAddr)
  READ_CONTROLLER_SETTING(writeRate)
  if(name=="listening") {
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
    while(!OpenConnection(str)) {
      printf("\n...Trying to connect again in 5 seconds...\n");
      ThreadSleep(5);
    }
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
  controllerPipe.reset(new SocketPipeWorker(addr.c_str(),true));
  if(!controllerPipe->Start()) {
    cout<<"Controller could not be opened on address "<<addr<<endl;
    return false;
  }
  cout<<"Opened controller on address "<<addr<<endl;
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

