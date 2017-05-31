#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "StateEstimator.h"
#include "JointSensors.h"
#include "InertialSensors.h"
#include "Simulation/ODERobot.h"
#include <KrisLibrary/math/angle.h>

void OmniscientStateEstimator::UpdateModel()
{
  oderobot.GetConfig(robot.q);
  oderobot.GetVelocities(robot.dq);
  robot.UpdateFrames();
}

IntegratedStateEstimator::IntegratedStateEstimator(Robot& _robot)
  :RobotStateEstimator(_robot), last_dt(0.0)
{}

void IntegratedStateEstimator::ReadSensors(RobotSensors& sensors)
{
  JointPositionSensor* jp = sensors.GetTypedSensor<JointPositionSensor>();
  JointVelocitySensor* jv = sensors.GetTypedSensor<JointVelocitySensor>();
  if(jv) {
    for(size_t i=0;i<robot.joints.size();i++) {
      if(robot.joints[i].type == RobotJoint::Normal || robot.joints[i].type == RobotJoint::Spin) {
	int link = robot.joints[i].linkIndex;
	dq_predicted[link] = jv->dq[link];
      }
    }
  }
  else if(jp) {
    if(last_dt == 0)
      dq_predicted.setZero();
    else {
      //do finite differencing
      for(size_t i=0;i<robot.joints.size();i++) {
	if(robot.joints[i].type == RobotJoint::Normal) {
	  int link = robot.joints[i].linkIndex;
	  dq_predicted[link] = (jp->q[link] - q_predicted[link])/last_dt;
	}
	else if(robot.joints[i].type == RobotJoint::Spin) {
	  int link = robot.joints[i].linkIndex;
	  dq_predicted[link] = AngleDiff(jp->q[link],q_predicted[link])/last_dt;
	}
      }
    }
  }

  //update joint positions for normal joints
  if(jp) {
    for(size_t i=0;i<robot.joints.size();i++) {
      if(robot.joints[i].type == RobotJoint::Normal || robot.joints[i].type == RobotJoint::Spin) {
	int link = robot.joints[i].linkIndex;
	q_predicted[link] = jp->q[link];
      }
    }
  }

  //use gyro sensors to update q_predicted
  vector<GyroSensor*> gyroSensors;
  sensors.GetTypedSensors(gyroSensors);
  if(!gyroSensors.empty()) {
    robot.UpdateConfig(q_predicted);
    for(size_t j=0;j<gyroSensors.size();j++) {
      int glink = gyroSensors[j]->link;
      int match = -1;
      for(size_t i=0;i<robot.joints.size();i++) {
	if(robot.joints[i].type == RobotJoint::Floating) {
	  if(robot.joints[i].linkIndex == glink) {
	    match = (int)i;
	    break;
	  }
	}
      }
      if(match >= 0) {
	robot.SetJointByOrientation(match,glink,gyroSensors[j]->rotation);
	robot.SetJointVelocityByMoment(match,glink,gyroSensors[j]->angVel,Vector3(Zero));
	vector<int> indices;
	robot.GetJointIndices(match,indices);
	assert(indices.size()==6);
	q_predicted(indices[3]) = robot.q(indices[3]);
	q_predicted(indices[4]) = robot.q(indices[4]);
	q_predicted(indices[5]) = robot.q(indices[5]);
	dq_predicted(indices[3]) = robot.dq(indices[3]);
	dq_predicted(indices[4]) = robot.dq(indices[4]);
	dq_predicted(indices[5]) = robot.dq(indices[5]);
      }
      else {
		LOG4CXX_ERROR(KrisLibrary::logger(),"Warning: can't use gyro sensor on a non-floating link "<<glink);
      }
    }
  }

  vector<Accelerometer*> accelerometers;
  sensors.GetTypedSensors(accelerometers);
  if(accelerometerFrames.empty()) {
    robot.UpdateConfig(q_predicted);
    //initialize
    accelerometerFrames.resize(accelerometers.size());
    accelerometerVels.resize(accelerometers.size());
    for(size_t i=0;i<accelerometers.size();i++) {
      int alink = accelerometers[i]->link;
      accelerometerFrames[i] = robot.links[alink].T_World*accelerometers[i]->Tsensor;
      robot.GetWorldVelocity(accelerometers[i]->Tsensor.t,alink,robot.dq,accelerometerVels[i].v);
      robot.GetWorldAngularVelocity(alink,robot.dq,accelerometerVels[i].w);
    }
  }

  //advance accelerometers
  if(last_dt > 0.0 && !accelerometers.empty()) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"TODO: integrate accelerometers\n");
  }
}

void IntegratedStateEstimator::ReadCommand(const RobotMotorCommand& command)
{
}

void IntegratedStateEstimator::Advance(Real dt)
{
  last_dt = dt;
  q_predicted.madd(dq_predicted,dt);
  if(!ddq_predicted.empty()) {
    dq_predicted.madd(ddq_predicted,dt);
    q_predicted.madd(ddq_predicted,0.5*dt*dt);
  }
  ddq_predicted.resize(0);
}

void IntegratedStateEstimator::Reset()
{
  q_predicted = robot.q;
  dq_predicted.resize(robot.links.size(),Zero);
  ddq_predicted.resize(0);
  last_dt = 0;

  accelerometerFrames.resize(0);
  accelerometerVels.resize(0);
}
