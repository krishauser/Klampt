#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Command.h"
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/myfile.h>
#include <KrisLibrary/errors.h>

ActuatorCommand::ActuatorCommand()
  :mode(OFF),measureAngleAbsolute(true),qmin(-Inf),qmax(Inf),
   kP(0),kI(0),kD(0),qdes(0),dqdes(0),iterm(0),
  torque(0),desiredVelocity(0)
{}

void ActuatorCommand::SetOff()
{
  mode=OFF;
}

void ActuatorCommand::SetPID(Real _qdes,Real _dqdes,Real _iterm)
{
  if(_qdes < qmin || _qdes > qmax) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Command.cpp: Warning, PID desired is out of joint limits: "<<qmin<<" <= "<<_qdes<<" <= "<<qmax);
  }
  mode=PID;
  qdes=_qdes;
  dqdes=_dqdes;
  iterm=_iterm;
  torque=0;
}

void ActuatorCommand::SetTorque(Real t)
{
  mode=TORQUE;
  torque=t;
}

void ActuatorCommand::SetLockedVelocity(Real vel,Real torqueMax)
{
  mode=LOCKED_VELOCITY;
  desiredVelocity=vel;
  torque=torqueMax;
}

Real ActuatorCommand::GetPIDTorque(Real q,Real dq) const
{
  Real deltaq,deltadq;
  if(measureAngleAbsolute) {
    deltaq=qdes-q;
    if(q < qmin || q > qmax) {
      if(Abs(AngleDiff(qdes,q)) < Abs(deltaq*0.5)) {
        LOG4CXX_WARN(KrisLibrary::logger(),"Command.cpp: Warning, PID loop has a possible angle encoder error, using AngleDiff\n");
        LOG4CXX_INFO(KrisLibrary::logger(),"  qdes = "<<qdes<<", q = "<<q);
        LOG4CXX_INFO(KrisLibrary::logger(),"  AngleDiff "<<AngleDiff(qdes,q)<<", sub "<<deltaq<<"\n");
        //KrisLibrary::loggerWait();
        deltaq = AngleDiff(qdes,q);
      }
    }
  }
  else
    deltaq=AngleDiff(qdes,q);
  deltadq=dqdes-dq;
  //LOG4CXX_INFO(KrisLibrary::logger(),"P torque: "<<kP*deltaq<<", D torque: "<<kD*deltadq<<", I torque "<<kI*iterm<<", FF torque "<<torque);
  return kP*deltaq+kD*deltadq+kI*iterm+torque;
}

void ActuatorCommand::IntegratePID(Real q,Real dt)
{
  if(measureAngleAbsolute) {
    if(Abs(AngleDiff(qdes,q)) < Abs((qdes-q)*0.5)) 
      iterm += AngleDiff(qdes,q)*dt;      
    else
      iterm += (qdes-q)*dt;
  }
  else
    iterm += AngleDiff(qdes,q)*dt;

  //integrate qdes
  if(mode == LOCKED_VELOCITY)
    qdes += dqdes*dt;
}

void RobotMotorCommand::SetTorque(const Vector& torques)
{
  Assert(torques.n == (int)actuators.size());
  for(size_t i=0;i<actuators.size();i++)
    actuators[i].SetTorque(torques(i));
}

void RobotMotorCommand::Clear()
{
  for(size_t i=0;i<actuators.size();i++)
    actuators[i].SetOff();
}

void RobotMotorCommand::ResetPIDIntegrals()
{
  for(size_t i=0;i<actuators.size();i++)
    actuators[i].iterm=0;
}

Real RobotMotorCommand::GetTorque(int i,double q,double dq)
{
  switch(actuators[i].mode) {
  case ActuatorCommand::OFF: return 0;
  case ActuatorCommand::TORQUE: return actuators[i].torque;
  case ActuatorCommand::PID: return actuators[i].GetPIDTorque(q,dq);
  case ActuatorCommand::LOCKED_VELOCITY: return actuators[i].torque;
  }
  return 0;
}

bool RobotMotorCommand::Read(File& f)
{
  int n;
  if(!ReadFile(f,n)) return false;
  if(n < 0) return false;
  actuators.resize(n);
  for(int i=0;i<n;i++) 
    if(!f.ReadData(&actuators[i],sizeof(actuators[i]))) return false;
  return true;
}

bool RobotMotorCommand::Write(File& f) const
{
  int n=(int)actuators.size();
  if(!WriteFile(f,n)) return false;
  for(int i=0;i<n;i++) 
    if(!f.WriteData(&actuators[i],sizeof(actuators[i]))) return false;
  return true;
}
