#include "SerialControlledRobot.h"
#include <utils/AnyCollection.h>

SerialControlledRobot::SerialControlledRobot(const char* host)
{
  if(!socketfile.Open(host,FILECLIENT)) {
    fprintf(stderr,"SerialControlledRobot: Error opening socket to %s\n",host);
    Abort();
  }
  timeStep = 0;
}

void SerialControlledRobot::Run()
{
  while(true) {
    ReadSensorData(sensors);
    if(timeStep == 0) {
      //first time -- just read next sensor data again to get timing info
    }
    else {
      if(klamptController) {
	klamptController->sensors = &sensors;
	klamptController->command = &command;
	klamptController->Update(timeStep);
      }
      WriteCommandData(command);
    }
  }
}

void SerialControlledRobot::ReadSensorData(RobotSensors& sensors)
{
  char buf[4096];
  if(!socketfile.ReadString(buf,4096)) {
    fprintf(stderr,"SerialControlledRobot: Unable to read sensor data from robot client\n");
    Abort();
  }
  AnyCollection c;
  if(!c.read(buf)) {
    fprintf(stderr,"SerialControlledRobot: Unable to read parse data from robot client\n");
    Abort();
  }
  //read off timing information
  Real readTime = c["t"];
  timeStep = c["dt"];
  //cout<<"Read time "<<readTime<<", time step "<<timeStep<<endl;
  for(size_t i=0;i<sensors.sensors.size();i++) {
    string sensorName = sensors.sensors[i]->name;
    AnyCollection svalues = c[sensorName];
    vector<double> values;
    if(!svalues.asvector(values)) {
      fprintf(stderr,"SerialControlledRobot: Unable to parse sensors %s from robot client\n",sensorName.c_str());
      Abort();
    }
    sensors.sensors[i]->SetMeasurements(values);
    //cout<<"Reading measurements "<<sensorName<<": "<<Vector(values)<<endl;
  }
}

void SerialControlledRobot::WriteCommandData(const RobotMotorCommand& command)
{
  AnyCollection c;
  AnyCollection qcmd,dqcmd,torquecmd;
  qcmd.resize(command.actuators.size());
  dqcmd.resize(command.actuators.size());
  torquecmd.resize(command.actuators.size());
  bool anyNonzeroV=false,anyNonzeroTorque = false;
  int mode = ActuatorCommand::OFF;
  for(size_t i=0;i<command.actuators.size();i++) {
    if(mode == ActuatorCommand::OFF)
      mode = command.actuators[i].mode;
    else {
      if(mode != command.actuators[i].mode) {
	fprintf(stderr,"SerialControlledRobot: do not support mixed torque / velocity / PID mode\n");
	Abort();
      }
    }
    qcmd[(int)i] = command.actuators[i].qdes;
    dqcmd[(int)i] = command.actuators[i].dqdes;
    torquecmd[(int)i] = command.actuators[i].torque;
    if(command.actuators[i].dqdes!=0) anyNonzeroV=true;
    if(command.actuators[i].torque!=0) anyNonzeroTorque=true;
    if(mode == ActuatorCommand::LOCKED_VELOCITY)
      dqcmd[(int)i] = command.actuators[i].desiredVelocity;
  }
  
  if(mode == ActuatorCommand::LOCKED_VELOCITY) {
    //cout<<"Sending locked velocity command"<<endl;
    c["dqcmd"] = dqcmd;
    c["tcmd"] = timeStep;
  }
  else if(mode == ActuatorCommand::PID) {
    //cout<<"Sending PID command"<<endl;
    c["qcmd"] = qcmd;
    if(anyNonzeroV) c["dqcmd"] = dqcmd;
    if(anyNonzeroTorque) c["torquecmd"] = torquecmd;
  }
  else if(mode == ActuatorCommand::TORQUE) {
    //cout<<"Sending torque command"<<endl;
    c["torquecmd"] = torquecmd;
  }
  else {
    cout<<"SerialControlledRobot: Motors are off??"<<endl;
  }
  //write JSON message to socket file
  stringstream ss;
  c.write(ss);
  socketfile.WriteString(ss.str().c_str());
}
