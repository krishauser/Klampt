#include "ContactController.h"
#include "Sensing/ForceSensors.h"
#include "Modeling/Interpolate.h"
#include "Contact/Utils.h"
#include <KrisLibrary/math/angle.h>

#define TORQUE_CONTROL_ONLY 0
#define NO_VELOCITY_FEEDBACK 0

using namespace Klampt;

ContactJointTrackingController::ContactJointTrackingController(RobotModel& robot,const SmartPointer<JointTrackingController>& _base)
  :RobotController(robot),base(_base),opSpaceController(robot)
{
  opSpaceController.jointTasks.resize(1);
  opSpaceController.jointTasks[0].indices.resize(robot.links.size());
  for(size_t i=0;i<robot.links.size();i++)
    opSpaceController.jointTasks[0].indices[i]=(int)i;
  opSpaceController.jointTasks[0].ddqdes.resize(robot.links.size(),0.0);
  opSpaceController.jointTasks[0].weight = 1.0;

  /*
  //cm control
  opSpaceController.comTasks.resize(1);
  opSpaceController.comTasks[0].R.setIdentity();
  opSpaceController.comTasks[0].numAxes = 1;
  opSpaceController.comTasks[0].ddxdes.resize(1,0.0);
  opSpaceController.comTasks[0].weight = 1.0;
  */
  /*
  opSpaceController.workspaceTasks.resize(1);
  opSpaceController.workspaceTasks[0].workspace.link = 1;
  opSpaceController.workspaceTasks[0].workspace.localPosition.set(1,0,0);
  opSpaceController.workspaceTasks[0].workspace.SetFixedPosition(Vector3(0,0,1.05));
  opSpaceController.workspaceTasks[0].ddxdes.resize(3,0.0);
  opSpaceController.workspaceTasks[0].weight = 10.0;
  */

  //torque regularization
  opSpaceController.torqueTasks.resize(1);
  opSpaceController.torqueTasks[0].indices = opSpaceController.jointTasks[0].indices;
  opSpaceController.torqueTasks[0].Tdes.resize(robot.links.size(),0.0);
  //opSpaceController.torqueTasks[0].indices.resize(robot.drivers.size());
  //for(size_t i=0;i<robot.drivers.size();i++)
  //  opSpaceController.torqueTasks[0].indices[i]=(int)i;
  //opSpaceController.torqueTasks[0].Tdes.resize(robot.drivers.size(),0.0);
  opSpaceController.torqueTasks[0].weight = 0.01;
}


void ContactJointTrackingController::SetGravity(const Vector3& gravity) { opSpaceController.gravity = gravity; }


void ContactJointTrackingController::SetTorqueRegularization(Real regularizationFactor)
{
  opSpaceController.torqueTasks[0].weight = regularizationFactor;
}


void ContactJointTrackingController::SenseContactEstimate(Real kFriction,Real penetrationWeight)
{
  opSpaceController.contactForceTasks.resize(1);
  //flatten contacts
  vector<int>& links = opSpaceController.contactForceTasks[0].links;
  vector<ContactPoint>& cps = opSpaceController.contactForceTasks[0].contacts;
  links.resize(0);
  cps.resize(0);
  vector<ContactSensor*> contactSensors;
  sensors->GetTypedSensors(contactSensors);
  for(size_t i=0;i<contactSensors.size();i++) {
    if(contactSensors[i]->contact) {
      links.push_back(contactSensors[i]->link);
      cps.resize(cps.size()+1);
      cps.back().x = contactSensors[i]->Tsensor.t;
      cps.back().n = Vector3(contactSensors[i]->Tsensor.R.col3());
      cps.back().kFriction = kFriction;
    }
  }
  opSpaceController.contactForceTasks[0].weight = 0;
  opSpaceController.contactForceTasks[0].penetrationWeight = penetrationWeight;
}

void ContactJointTrackingController::SetFlatContactEstimate(Real kFriction,Real penetrationWeight,Real tol)
{
  /*
  base->command = command;
  base->sensors = sensors;
  Config qdes(robot.links.size()),dqdes(robot.links.size());
  base->GetDesiredState(qdes,dqdes);
  robot.UpdateConfig(qdes);
  cout<<"Desired: "<<qdes<<endl;
  */
  ContactFormation contacts;
  GetFlatContacts(robot,tol,contacts);
  for(size_t i=0;i<contacts.contacts.size();i++)
    for(size_t j=0;j<contacts.contacts[i].size();j++)
      contacts.contacts[i][j].kFriction = kFriction;
  SetContactEstimate(contacts,penetrationWeight);
}

void ContactJointTrackingController::SetContactEstimate(const ContactFormation& contacts,Real penetrationWeight)
{
  opSpaceController.contactForceTasks.resize(1);
  //flatten contacts
  vector<int>& links = opSpaceController.contactForceTasks[0].links;
  vector<ContactPoint>& cps = opSpaceController.contactForceTasks[0].contacts;
  contacts.flatten(links,cps);
  opSpaceController.contactForceTasks[0].weight = 0;
  opSpaceController.contactForceTasks[0].penetrationWeight = penetrationWeight;
}


void ContactJointTrackingController::Update(Real dt)
{
  RobotController::Update(dt);

  base->command = command;
  base->sensors = sensors;
  Config qdes(robot.links.size()),dqdes(robot.links.size());
  base->GetDesiredState(qdes,dqdes);
  base->Update(dt);
  
  /*
    COM tasks
  opSpaceController.stateEstimator->UpdateModel();
  Vector3 comcur=robot.GetCOM();
  robot.UpdateConfig(qdes);
  Vector3 comdes=robot.GetCOM();
  opSpaceController.comTasks[0].ddxdes(0) = 1.0*(comdes.x-comcur.x);
  */
  /*
  //Workspace tasks
  opSpaceController.stateEstimator->UpdateModel();
  Vector3 xcur=robot.links[1].T_World*Vector3(1,0,0);
  robot.UpdateConfig(qdes);
  Vector3 xdes=robot.links[1].T_World*Vector3(1,0,0);
  opSpaceController.workspaceTasks[0].ddxdes.copy((Real*)(1.0*(xdes-xcur)));
  */

  //change the (q,dq) command to a ddq command
  DesiredToAccel(dt,qdes,dqdes,opSpaceController.jointTasks[0].ddqdes);

  opSpaceController.stateEstimator->UpdateModel();
  cout<<"Current q: "<<robot.q<<endl;
  cout<<"Current dq: "<<robot.dq<<endl;
  cout<<"Desired q: "<<qdes<<endl;
  cout<<"Desired q': "<<dqdes<<endl;
  cout<<"Desired q'': "<<opSpaceController.jointTasks[0].ddqdes<<endl;
  /*
  //HACK: set estimated translational velocity to zero
  robot.dq(0) = robot.dq(1) = robot.dq(2) = 0;
  //HACK: set estimated rotational velocity to zero
  robot.dq(3) = robot.dq(4) = robot.dq(5) = 0;
  */

  opSpaceController.sensors = sensors;
  opSpaceController.command = command;
  assert(opSpaceController.IsValid());
  opSpaceController.Update(dt);

#if !TORQUE_CONTROL_ONLY
  //change torque commands to feedforward torques
  for(size_t i=0;i<robot.drivers.size();i++) {
    Real feedforwardTorque = command->actuators[i].torque;
    if(robot.drivers[i].type == RobotModelDriver::Normal) {
      command->actuators[i].SetPID(qdes(robot.drivers[i].linkIndices[0]),dqdes(robot.drivers[i].linkIndices[0]),command->actuators[i].iterm);
    }
    else {
      robot.q = qdes;
      robot.dq = dqdes;
      //printf("Desired affine driver value %g, vel %g\n",robot.GetDriverValue(i),robot.GetDriverVelocity(i));
      command->actuators[i].SetPID(robot.GetDriverValue(i),robot.GetDriverVelocity(i),command->actuators[i].iterm);
    }
    command->actuators[i].torque = feedforwardTorque;
  }
#endif
}


void ContactJointTrackingController::Reset()
{
  RobotController::Reset();
  base->Reset();
  opSpaceController.Reset();
}


bool ContactJointTrackingController::ReadState(File& f)
{
  if(!RobotController::ReadState(f)) return false;
  if(!base->ReadState(f)) return false;
  if(!opSpaceController.ReadState(f)) return false;
  return true;
}

bool ContactJointTrackingController::WriteState(File& f) const {
  if(!RobotController::WriteState(f)) return false;
  if(!base->WriteState(f)) return false;
  if(!opSpaceController.WriteState(f)) return false;
  return true;
}

void ContactJointTrackingController::DesiredToAccel(Real dt,const Config& qdes,const Vector& dqdes,Vector& ddqdes)
{
  opSpaceController.stateEstimator->UpdateModel();
#if TORQUE_CONTROL_ONLY
  //PID control
  Real kP = 10;
  Real kD = 1.0/dt*0.25;
  InterpolateDerivative(robot,robot->q,qdes,ddqdes);
  ddqdes *= kP;
  ddqdes += kD*(dqdes-robot.dq);
#else
  //just do a differencing scheme
  if(!dqdes_last.empty() && NO_VELOCITY_FEEDBACK)
    ddqdes.sub(dqdes,dqdes_last);
  else
    ddqdes.sub(dqdes,robot.dq);
  ddqdes /= dt;

  qdes_last = qdes;
  dqdes_last = dqdes;
#endif

  /*
  //HACK: set desired translation to zero
  ddqdes(0) = ddqdes(1) = ddqdes(2) = 0;
  //HACK: set desired rotation to zero
  ddqdes(3) = ddqdes(4) = ddqdes(5) = 0;
  */
}

