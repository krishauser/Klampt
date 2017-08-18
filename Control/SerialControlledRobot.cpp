#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "SerialControlledRobot.h"
#include "JointSensors.h"
#include <KrisLibrary/utils/AnyCollection.h>

SerialControlledRobot::SerialControlledRobot(const char* _host,double timeout)
  :host(_host),robotTime(0),timeStep(0),numOverruns(0),stopFlag(false),controllerMutex(NULL)
{
  controllerPipe = new SocketPipeWorker(_host,false,timeout);
}

SerialControlledRobot::~SerialControlledRobot()
{
  controllerPipe = NULL;
}

bool SerialControlledRobot::Init(Robot* _robot,RobotController* _controller)
{
  if(!ControlledRobot::Init(_robot,_controller)) return false;
  if(!controllerPipe->Start()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot: Error opening socket to "<<host.c_str());
    return false;
  }
  return true;
}

bool SerialControlledRobot::Process(double timeout)
{
  if(!controllerPipe->initialized) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot::Process(): did you forget to call Init?\n");
    return false;
  }

  Timer timer;
  int iteration = 0;
  while(!stopFlag) {
    Real lastReadTime = timer.ElapsedTime();
    if(lastReadTime > timeout) return false;
    timeStep = 0;
    if(controllerMutex) controllerMutex->lock();
    ReadSensorData(sensors);
    if(timeStep == 0) {
      //first time, or failed to read -- 
      //read next sensor data again to get timing info
      if(controllerMutex) controllerMutex->unlock();
      if(iteration % 100 == 0)
	LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot(): Error getting timestep? Waiting.\n");
      ThreadSleep(0.01);
    }
    else {
      if(klamptController) {
	klamptController->sensors = &sensors;
	klamptController->command = &command;
	klamptController->Update(timeStep);
      }
      if(controllerMutex) controllerMutex->unlock();
      WriteCommandData(command);

      Real time = timer.ElapsedTime();
      if(time > lastReadTime + timeStep) {
	numOverruns ++;
      }
      return true;
    }
    iteration ++;
  }
  return false;
}

bool SerialControlledRobot::Run()
{
  if(!controllerPipe->initialized) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot::Run(): did you forget to call Init?\n");
    return false;
  }
  Timer timer;
  stopFlag = false;
  while(!stopFlag) {
    Real lastReadTime = timer.ElapsedTime();
    timeStep = 0;
    if(controllerMutex) controllerMutex->lock();
    ReadSensorData(sensors);
    if(timeStep == 0) {
      //first time, or failed to read -- 
      //read next sensor data again to get timing info
      if(controllerMutex) controllerMutex->unlock();
      ThreadSleep(0.01);
    }
    else {
      if(klamptController) {
	klamptController->sensors = &sensors;
	klamptController->command = &command;
	Real dt = robotTime-klamptController->time;
	if(klamptController->time == 0) //first update
	  dt = timeStep;
	klamptController->Update(robotTime-klamptController->time);
      }
      if(controllerMutex) controllerMutex->unlock();

      if(!controllerPipe->initialized) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot::Run(): killed by socket disconnect?\n");
	return false;
      }
      WriteCommandData(command);

      Real time = timer.ElapsedTime();
      if(time > lastReadTime + timeStep) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Klamp't controller overrun, took time "<<time-lastReadTime<<" which exceeds time step "<<timeStep);
	numOverruns ++;
      }
      else {
	ThreadSleep(Max(lastReadTime + timeStep - time,0.0));
      }
    }
  }
  return true;
}

void SerialControlledRobot::Stop()
{
  stopFlag = true;
}

void SerialControlledRobot::SetMutex(Mutex* mutex)
{
  controllerMutex = mutex;
}

void SerialControlledRobot::ReadSensorData(RobotSensors& sensors)
{
  if(controllerPipe && controllerPipe->UnreadCount() > 0) {
    if(controllerPipe->UnreadCount() > 1) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot: Warning, skipping "<<controllerPipe->UnreadCount()-1);
            LOG4CXX_ERROR(KrisLibrary::logger(),"  TODO: debug the controller pipe?\n");
    }
    string msg = controllerPipe->Newest();

    AnyCollection c;
    if(!c.read(msg.c_str())) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot: Unable to read parse data from robot client\n");
      return;
    }
    
    if(sensors.sensors.empty()) {
      //no sensors defined by the user -- initialize default sensors based on
      //what's in the sensor message
      if(c.find("q") != NULL) {
	JointPositionSensor* jp = new JointPositionSensor;
	jp->name = "q";
	jp->q.resize(klamptRobotModel->q.n,Zero);
	sensors.sensors.push_back(jp);
      }
      if(c.find("dq") != NULL) {
	JointVelocitySensor* jv = new JointVelocitySensor;
	jv->name = "dq";
	jv->dq.resize(klamptRobotModel->q.n,Zero);
	sensors.sensors.push_back(jv);
      }
      if(c.find("torque") != NULL) {
	DriverTorqueSensor* ts = new DriverTorqueSensor;
	ts->name = "torque";
	ts->t.resize(klamptRobotModel->drivers.size());
	sensors.sensors.push_back(ts);
      }
    }

    //read off timing information
    vector<AnyKeyable> keys;
    c.enumerate_keys(keys);
    for(size_t i=0;i<keys.size();i++) {
      string key;
      if(LexicalCast(keys[i].value,key)) {
	if(key == "dt") 
	  timeStep = c["dt"];
	else if(key == "t") 
	  robotTime = c["t"];
	else if(key == "qcmd" || key=="dqcmd" || key=="torquecmd")  //echo
	  continue;
	else {
	  SmartPointer<SensorBase> s = sensors.GetNamedSensor(key);
	  if(!s) {
	    	    LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot::ReadSensorData: warning, sensor "<<key.c_str());
	  }
	  else {
	    vector<double> values;
	    bool converted = c[keys[i]].asvector<double>(values);
	    if(!converted){
        LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot::ReadSensorData: key "<<key.c_str());
      } 
	    else 
	      s->SetMeasurements(values);
	  }
	}
      }
      else {
	FatalError("SerialControlledRobot::ReadSensorData: Invalid key, element %d...\n",i);
      }
    }
  }
}

void SerialControlledRobot::WriteCommandData(const RobotMotorCommand& command)
{
  if(controllerPipe && controllerPipe->transport->WriteReady()) {
    AnyCollection c;
    AnyCollection qcmd,dqcmd,torquecmd;
    qcmd.resize(klamptRobotModel->links.size());
    dqcmd.resize(klamptRobotModel->links.size());
    torquecmd.resize(command.actuators.size());
    bool anyNonzeroV=false,anyNonzeroTorque = false;
    int mode = ActuatorCommand::OFF;
    for(size_t i=0;i<command.actuators.size();i++) {
      if(mode == ActuatorCommand::OFF)
	mode = command.actuators[i].mode;
      else {
	if(mode != command.actuators[i].mode) {
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"SerialControlledRobot: do not support mixed torque / velocity / PID mode\n");
	  Abort();
	}
      }
      klamptRobotModel->SetDriverValue(i,command.actuators[i].qdes);
      klamptRobotModel->SetDriverVelocity(i,command.actuators[i].dqdes);
      torquecmd[(int)i] = command.actuators[i].torque;
      if(command.actuators[i].dqdes!=0) anyNonzeroV=true;
      if(command.actuators[i].torque!=0) anyNonzeroTorque=true;
      if(mode == ActuatorCommand::LOCKED_VELOCITY) 
	klamptRobotModel->SetDriverVelocity(i,command.actuators[i].desiredVelocity);
    }
    if(mode == ActuatorCommand::PID) {
      for(size_t i=0;i<klamptRobotModel->links.size();i++)
	qcmd[(int)i] = klamptRobotModel->q[i];
    }
    if(anyNonzeroV || mode == ActuatorCommand::LOCKED_VELOCITY) {
      for(size_t i=0;i<klamptRobotModel->links.size();i++)
	dqcmd[(int)i] = klamptRobotModel->dq[i];
    }

    if(mode == ActuatorCommand::OFF) {
      //nothing to send
      return;
    }    
    else if(mode == ActuatorCommand::LOCKED_VELOCITY) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Sending locked velocity command"<<"\n");
      c["dqcmd"] = dqcmd;
      c["tcmd"] = timeStep;
    }
    else if(mode == ActuatorCommand::PID) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Sending PID command"<<"\n");
      c["qcmd"] = qcmd;
      if(anyNonzeroV) c["dqcmd"] = dqcmd;
      if(anyNonzeroTorque) c["torquecmd"] = torquecmd;
    }
    else if(mode == ActuatorCommand::TORQUE) {
      //LOG4CXX_INFO(KrisLibrary::logger(),"Sending torque command"<<"\n");
      c["torquecmd"] = torquecmd;
    }
    else {
      LOG4CXX_INFO(KrisLibrary::logger(),"SerialControlledRobot: Invalid mode?? "<<mode<<"\n");
    }
    //write JSON message to socket file
    stringstream ss;
    c.write(ss);
    LOG4CXX_INFO(KrisLibrary::logger(),"Writing message: "<<ss.str()<<"\n");
    controllerPipe->Send(ss.str());
  }
}
