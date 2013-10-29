#ifndef CONTROL_MOTOR_COMMAND_H
#define CONTROL_MOTOR_COMMAND_H

#include <vector>
#include <math/vector.h>
#include <math/math.h>
using namespace std;
using namespace Math;

/** @defgroup Control */

/** @ingroup Control
 * @brief A basic motor type.  Handles PID, torque, and locked velocity modes
 */
struct ActuatorCommand
{
  enum { OFF, TORQUE, PID, LOCKED_VELOCITY };

  ActuatorCommand();
  void SetOff();
  void SetPID(Real qdes,Real dqdes=Zero,Real iterm=Zero);
  void SetTorque(Real t);
  void SetLockedVelocity(Real vel,Real torqueMax);
  Real GetPIDTorque(Real q,Real dq) const;
  void IntegratePID(Real q,Real dt);

  int mode;
  bool measureAngleAbsolute;
  Real kP,kI,kD;
  Real qdes,dqdes;
  Real iterm;
  Real torque;   //torque command, feedforward PID torque, or torque limit
  Real desiredVelocity;  //for locked velocity mode
};

/** @ingroup Control
 * @brief A collection of basic motor types.
 */
struct RobotMotorCommand
{
  void SetTorque(const Vector& torques);
  Real GetTorque(int i,double q,double dq);
  void Clear();
  void ResetPIDIntegrals();
  bool Read(File& f);
  bool Write(File& f) const;

  vector<ActuatorCommand> actuators;
};

#endif
