#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "Sensor.h"
#include <Klampt/Control/Command.h>
#include <Klampt/Modeling/Robot.h>
#include <KrisLibrary/robotics/Wrench.h>

namespace Klampt {

class ODERobot;

/** @ingroup Control
 * @brief A generic state estimator base class.  Base class does nothing.
 *
 * Typical state estimators will update the robot state (q,dq, frames) 
 * for the given sensor input.  They may also do more sophisticated things,
 * like calibrate the robot limb lengths, dynamic parameters, or even a
 * world model.
 *
 * The typical state estimation loop is ReadSensors, UpdateModel, ReadCommand,
 * Advance.  To use these in a RobotController, call this sequence manually.
 */
struct RobotStateEstimator
{
  RobotStateEstimator(RobotModel& _robot) :robot(_robot) {}
  virtual ~RobotStateEstimator() {}
  virtual void ReadSensors(RobotSensors& sensors) {}
  virtual void UpdateModel() {}
  virtual void ReadCommand(const RobotMotorCommand& command) {}
  virtual void Advance(Real dt) {}
  virtual void Reset() {}

  RobotModel& robot;
};

/** @ingroup Control
 * @brief A state estimator will full knowledge of the robot's simulated
 * state.  An ODERobot must be provided at initialization.
 */ 
struct OmniscientStateEstimator : public RobotStateEstimator
{
  OmniscientStateEstimator(RobotModel& _robot,ODERobot& _oderobot)
    :RobotStateEstimator(_robot),oderobot(_oderobot) {}
  virtual ~OmniscientStateEstimator();
  virtual void UpdateModel() override;

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
  IntegratedStateEstimator(RobotModel& _robot);
  virtual ~IntegratedStateEstimator() {}
  virtual void ReadSensors(RobotSensors& sensors) override;
  virtual void UpdateModel() override {
    robot.UpdateConfig(q_predicted);
    robot.dq = dq_predicted;
  }
  virtual void ReadCommand(const RobotMotorCommand& command) override;
  virtual void Advance(Real dt) override;
  virtual void Reset() override;

  //instead of ReadCommand, this estimator uses SetDDQ
  void SetDDQ(const Vector& ddq) { ddq_predicted = ddq; }

  Real last_dt;
  Vector q_predicted,dq_predicted,ddq_predicted;
  Vector q_sensed,dq_sensed;
  //these correspond to the world positions of the accelerometers in the sensors
  vector<RigidTransform> accelerometerFrames;
  vector<RigidBodyVelocity> accelerometerVels;
};

} //namespace Klampt

#endif
