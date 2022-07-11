#include "LoggingController.h"
#include <sstream>
using namespace Klampt;

LoggingController::LoggingController(RobotModel& robot,const shared_ptr<RobotController>& _base)
  : RobotController(robot),base(_base),save(false),replay(false),onlyJointCommands(false),replayIndex(0)
{}


bool LoggingController::SaveLog(const char* fn) const
{
  File f;
  if(!f.Open(fn,FILEWRITE)) return false;
  int size = trajectory.size();
  if(!WriteFile(f,size)) return false;
  for(size_t i=0;i<trajectory.size();i++) {
    if(!WriteFile(f,trajectory[i].first)) return false;
    if(!WriteFile(f,trajectory[i].second)) return false;
  }
  f.Close();
  return true;
}


bool LoggingController::LoadLog(const char* fn)
{
  File f;
  if(!f.Open(fn,FILEREAD)) return false;
  int size;
  if(!ReadFile(f,size)) return false;
  if(size < 0) return false;
  trajectory.resize(size);
  for(size_t i=0;i<trajectory.size();i++) {
    if(!ReadFile(f,trajectory[i].first)) return false;
    if(!ReadFile(f,trajectory[i].second)) return false;
  }
  f.Close();
  return true;
}


void LoggingController::Update(Real dt)
{
  base->command = command;
  base->sensors = sensors;
  if(replay) {   //replay mode
    base->time += dt;
    if(!trajectory.empty()) {
      //look up the right trajectory
      Assert(replayIndex < (int)trajectory.size());
      //go backwards
      while(trajectory[replayIndex].first > RobotController::time && replayIndex > 0) {
	replayIndex--;
      }
      //go forwards
      while(replayIndex+1 < (int) trajectory.size() &&
	    base->time >= trajectory[replayIndex+1].first) {
	replayIndex++;
      }
      //printf("Replay time %g, index %d\n",RobotController::time,replayIndex);
      //read it out
      const RobotMotorCommand& logCmd = trajectory[replayIndex].second;
      RobotMotorCommand* actualCmd = RobotController::command;
      if(onlyJointCommands) {
	for(size_t i=0;i<actualCmd->actuators.size();i++) {
	  actualCmd->actuators[i].qdes = logCmd.actuators[i].qdes;
	  actualCmd->actuators[i].dqdes = logCmd.actuators[i].dqdes;
	  actualCmd->actuators[i].torque = logCmd.actuators[i].torque;
	  actualCmd->actuators[i].desiredVelocity = logCmd.actuators[i].desiredVelocity;
	}
      }
      else {
	*actualCmd = logCmd;
      }
    }
  }
  else {  //normal mode
    RobotController::Update(dt);
    base->Update(dt);

    if(save) {
      if(trajectory.empty() || !EqualCommand(trajectory.back().second,*RobotController::command))
	trajectory.push_back(pair<Real,RobotMotorCommand>(base->time,*RobotController::command));
    }
  }
}

void LoggingController::Reset()
{
  base->Reset();
}

bool LoggingController::ReadState(File& f)
{
  if(!base->ReadState(f)) return false;
  trajectory.resize(0);
  return true;
}

bool LoggingController::WriteState(File& f) const
{
  if(!base->WriteState(f)) return false;
  return true;
}

map<string,string> LoggingController::Settings() const
{
  map<string,string> res=base->Settings();
  FILL_CONTROLLER_SETTING(res,save)
  FILL_CONTROLLER_SETTING(res,replay)
  FILL_CONTROLLER_SETTING(res,onlyJointCommands)
  return res;
}

bool LoggingController::GetSetting(const string& name,string& str) const
{
  if(base->GetSetting(name,str)) return true;
  READ_CONTROLLER_SETTING(save)
  READ_CONTROLLER_SETTING(replay)
  READ_CONTROLLER_SETTING(onlyJointCommands)
  return false;
}

bool LoggingController::SetSetting(const string& name,const string& str)
{
  if(base->SetSetting(name,str)) return true;
  WRITE_CONTROLLER_SETTING(save)
  WRITE_CONTROLLER_SETTING(replay)
  WRITE_CONTROLLER_SETTING(onlyJointCommands)
  return false;
}

vector<string> LoggingController::Commands() const
{
  vector<string> res = base->Commands();
  res.push_back("log");
  res.push_back("replay");
  return res;
}

bool LoggingController::SendCommand(const string& name,const string& str)
{
  if(name=="log") {
    return SaveLog(str.c_str()); 
  }
  else if(name=="replay") {
    if(LoadLog(str.c_str())) {
      replay = true;
      replayIndex = 0;
      onlyJointCommands = true;
      //hack
      printf("HACK: removing delays from recorded commands\n");
      RemoveDelays(0.2);
      printf("Read %d commands\n",trajectory.size());
      //check if it's for the right robot
      if(!trajectory.empty()) {
	if(trajectory[0].second.actuators.size() != command->actuators.size()) {
	  fprintf(stderr,"Command file %s doesn't have the right number of actuators\n",str.c_str());
	  replay = false;
	}
      }
      return true;
    }
    return false;
  }
  else
    return base->SendCommand(name,str);
}



bool LoggingController::EqualCommand(const ActuatorCommand& a,const ActuatorCommand& b) const
{
  if(onlyJointCommands) {
    if(a.qdes != b.qdes) return false;
    if(a.dqdes != b.dqdes) return false;
    //TODO: does feedforward torque matter?
    if(a.torque != b.torque) return false;
    if(a.desiredVelocity != b.desiredVelocity) return false;
  }
  else {
    if(a.mode!=b.mode) return false;
    if(a.measureAngleAbsolute!=b.measureAngleAbsolute) return false;
    if(a.kP!=b.kP) return false;
    if(a.kI!=b.kI) return false;
    if(a.kD!=b.kD) return false;
    if(a.iterm!=b.iterm) return false;
    if(a.qdes!=b.qdes) return false;
    if(a.dqdes!=b.dqdes) return false;
    if(a.mode!=b.mode) return false;
    if(a.torque!=b.torque) return false;
    if(a.desiredVelocity!=b.desiredVelocity) return false;
  }
  return true;
}


bool LoggingController::EqualCommand(const RobotMotorCommand& a,const RobotMotorCommand& b) const
{
  if(a.actuators.size() != b.actuators.size()) return false;
  for(size_t i=0;i<a.actuators.size();i++)
    if(!EqualCommand(a.actuators[i],b.actuators[i])) return false;
  return true;
}


void LoggingController::RemoveDelays(Real maxDelayTime)
{
  Assert(replayIndex == 0);
  int lastEraseIndex = -1;
  for(size_t i=1;i<trajectory.size();i++) {
    if(EqualCommand(trajectory[i].second,trajectory[i-1].second)) {
      if(lastEraseIndex < 0) lastEraseIndex = i;
    }
    else {
      if(lastEraseIndex >= 0) {
	//printf("Erasing trajectory commands %d-%d\n",lastEraseIndex,i-1);
	trajectory.erase(trajectory.begin()+lastEraseIndex,trajectory.begin()+i);
	i = lastEraseIndex-1;
	lastEraseIndex = -1;
      }
    }
  }
  Real shift=0;
  for(size_t i=0;i<trajectory.size();i++) {
    trajectory[i].first -= shift;
    if(i > 0) {
      if(trajectory[i].first-trajectory[i-1].first > maxDelayTime) {
	//printf("Found delay of %g at %d\n",trajectory[i].first-trajectory[i-1].first,i);
	shift += trajectory[i].first-trajectory[i-1].first-maxDelayTime;
	//printf("New shift %g\n",shift);
	trajectory[i].first = trajectory[i-1].first+maxDelayTime;
      }
    }
  }
  //printf("Trimmed %g seconds from trajectory\n",shift);
}
