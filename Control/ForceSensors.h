#ifndef CONTROL_FORCE_SENSORS_H
#define CONTROL_FORCE_SENSORS_H

#include "Sensor.h"

/** @ingroup Control
 * @brief Simulates a pressure sensor.  Not currently functional.
 */
class ContactSensor : public SensorBase
{
 public:
  ContactSensor();
  virtual const char* Type() const { return "ContactSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;                ///< The link on which the sensor is located
  RigidTransform Tsensor;  ///< Local frame of the sensor (by convention, origin is at contact patch, z is normal to surface, into robot)
  Vector2 patchMin,patchMax;///< The 2D contact patch in the local frame of the sensor 
  Real patchTolerance;     ///< The deformation tolerance of the contact patch
  bool hasForce[3];        ///< If an element is true, that component of force is measured
  Vector3 fVariance;       ///< Estimated variance of the sensor

  bool contact;            ///< Measurement: true if contact has been made
  Vector3 force;           ///< Measurement: the force magnitude
};

/** @ingroup Control
 * @brief Simulates a force-torque sensor mounted between a link and its
 * parent.
 * Can be configured to be up to 6DOF.
 */
class ForceTorqueSensor : public SensorBase
{
 public:
  ForceTorqueSensor();
  virtual const char* Type() const { return "ForceTorqueSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;                ///< The link on which the sensor is located (between link and parent)
  Vector3 localPos;        ///< The position of the sensor, in the local frame
  bool hasForce[3];        ///< true if force is measured along the given axis
  bool hasMoment[3];       ///< true if moment is measured along the given axis
  Vector3 fVariance, mVariance; ///< Estimated variance of the sensor

  Vector3 f,m;             ///< Measurement: the force/moment at the given position, on the link (negative on the parent link)
};

#endif 
