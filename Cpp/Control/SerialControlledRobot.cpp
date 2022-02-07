#include "SerialControlledRobot.h"
#include "Sensing/JointSensors.h"
#include <KrisLibrary/utils/AnyCollection.h>

using namespace Klampt;

SerialControlledRobot::SerialControlledRobot(const char* _host,double timeout)
  :host(_host),robotTime(0),timeStep(0),numOverruns(0),stopFlag(false),controllerMutex(NULL)
{
  controllerPipe = make_shared<SocketPipeWorker>(_host,false,timeout);
}

SerialControlledRobot::~SerialControlledRobot()
{
  controllerPipe = NULL;
}

bool SerialControlledRobot::Init(RobotModel* _robot,RobotController* _controller)
{
  if(!ControlledRobot::Init(_robot,_controller)) return false;
  if(!controllerPipe->Start()) {
    fprintf(stderr,"SerialControlledRobot: Error opening socket to %s\n",host.c_str());
    return false;
  }
  return true;
}

bool SerialControlledRobot::Process(double timeout)
{
  if(!controllerPipe->initialized) {
    fprintf(stderr,"SerialControlledRobot::Process(): did you forget to call Init?\n");
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
        printf("SerialControlledRobot(): Error getting timestep? Waiting.\n");
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
    fprintf(stderr,"SerialControlledRobot::Run(): did you forget to call Init?\n");
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
        fprintf(stderr,"SerialControlledRobot::Run(): killed by socket disconnect?\n");
        return false;
      }
      WriteCommandData(command);

      Real time = timer.ElapsedTime();
      if(time > lastReadTime + timeStep) {
        printf("Klamp't controller overrun, took time %g which exceeds time step %g\n",time-lastReadTime,timeStep);
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
      fprintf(stderr,"SerialControlledRobot: Warning, skipping %d sensor messages\n",controllerPipe->UnreadCount()-1);
      fprintf(stderr,"  TODO: debug the controller pipe?\n");
    }
    string msg = controllerPipe->Newest();

    AnyCollection c;
    if(!c.read(msg.c_str())) {
      fprintf(stderr,"SerialControlledRobot: Unable to read parse data from robot client\n");
      return;
    }
    
    if(sensors.sensors.empty()) {
      //no sensors defined by the user -- initialize default sensors based on
      //what's in the sensor message
      if(c.find("q") != NULL) {
        JointPositionSensor* jp = new JointPositionSensor;
        jp->name = "q";
        jp->q.resize(klamptRobotModel->q.n,Zero);
        sensors.sensors.push_back(shared_ptr<SensorBase>(jp));
      }
      if(c.find("dq") != NULL) {
        JointVelocitySensor* jv = new JointVelocitySensor;
        jv->name = "dq";
        jv->dq.resize(klamptRobotModel->q.n,Zero);
        sensors.sensors.push_back(shared_ptr<SensorBase>(jv));
      }
      if(c.find("torque") != NULL) {
        DriverTorqueSensor* ts = new DriverTorqueSensor;
        ts->name = "torque";
        ts->t.resize(klamptRobotModel->drivers.size());
        sensors.sensors.push_back(shared_ptr<SensorBase>(ts));
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
          shared_ptr<SensorBase> s = sensors.GetNamedSensor(key);
          if(!s) {
            fprintf(stderr,"SerialControlledRobot::ReadSensorData: warning, sensor %s not given in model\n",key.c_str());
          }
          else {
            vector<double> values;
            bool converted = c[keys[i]].asvector<double>(values);
            if(!converted) 
              fprintf(stderr,"SerialControlledRobot::ReadSensorData: key %s does not yield a vector\n",key.c_str());
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
          fprintf(stderr,"SerialControlledRobot: do not support mixed torque / velocity / PID mode\n");
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
      cout<<"SerialControlledRobot: Invalid mode?? "<<mode<<endl;
    }
    //write JSON message to socket file
    stringstream ss;
    c.write(ss);
    cout<<"Writing message: "<<ss.str()<<endl;
    controllerPipe->Send(ss.str());
  }
}
