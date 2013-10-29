#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "Sensor.h"
#include "Command.h"
#include "Modeling/Robot.h"
#include <robotics/Wrench.h>

class ODERobot;

/** @ingroup Control
 * @brief A generic state estimator base class.  Base class does nothing.
 *
 * Typical state estimators will update the robot state (q,dq, frames) 
 * for the given sensor input.  They may also do more sophisticated things,
 * like calibrate the robot limb lengths, dynamic parameters, or even a
 * world model.
 *
 * The typical sensor loop is ReadSensors, UpdateModel, ReadCommand,
 * Advance.  A RobotController will call these manually.
 */
struct RobotStateEstimator
{
  RobotStateEstimator(Robot& _robot) :robot(_robot) {}
  virtual ~RobotStateEstimator() {}
  virtual void ReadSensors(RobotSensors& sensors) {}
  virtual void UpdateModel() {}
  virtual void ReadCommand(const RobotMotorCommand& command) {}
  virtual void Advance(Real dt) {}
  virtual void Reset() {}

  Robot& robot;
};

/** @ingroup Control
 * @brief A state estimator will full knowledge of the robot's simulated
 * state.  An ODERobot must be provided at initialization.
 */ 
struct OmniscientStateEstimator : public RobotStateEstimator
{
  OmniscientStateEstimator(Robot& _robot,ODERobot& _oderobot)
    :RobotStateEstimator(_robot),oderobot(_oderobot) {}
  virtual ~OmniscientStateEstimator();
  virtual void UpdateModel();

  ODERobot& oderobot;
};

/** @ingroup Control
 * @brief A state estimator that integrates information from accelerometers
 * (i.e., an Inertial Measurement Unit) and gravity sensors.
 * 
 * It may also integrate a predicted acceleration ddq_predicted through
 * time, although this is not as accurate as a Kalman filter.
 */ 
struct IntegratedStateEstimator : public RobotStateEstimator
{
  IntegratedStateEstimator(Robot& _robot);
  virtual ~IntegratedStateEstimator() {}
  virtual void ReadSensors(RobotSensors& sensors);
  virtual void UpdateModel() {
    robot.UpdateConfig(q_predicted);
    robot.dq = dq_predicted;
  }
  virtual void ReadCommand(const RobotMotorCommand& command);
  virtual void Advance(Real dt);
  virtual void Reset();

  //instead of ReadCommand, this estimator uses SetDDQ
  void SetDDQ(const Vector& ddq) { ddq_predicted = ddq; }

  Real last_dt;
  Vector q_predicted,dq_predicted,ddq_predicted;
  Vector q_sensed,dq_sensed;
  //these correspond to the world positions of the accelerometers in the sensors
  vector<RigidTransform> accelerometerFrames;
  vector<RigidBodyVelocity> accelerometerVels;
};

#endif
