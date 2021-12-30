#ifndef CONTROL_FORCE_SENSORS_H
#define CONTROL_FORCE_SENSORS_H

#include "Sensor.h"
#include <KrisLibrary/math3d/primitives.h>


namespace Klampt {

using namespace Math3D;

/** @ingroup Sensing
 * @brief Simulates a contact sensor / tactile sensor.
 * 
 * The sensor comprises a block of space which needs to be placed on the
 * surface of a robot.  The x-y coordinates of this block form a 
 * "contact patch" over which the sensor obtains readings.  The sensor origin
 * is the center of this block.
 *
 * If the simulator generates any contacts inside the contact patch, where
 * the contact z-coordinate lies within patchTolerance of 0, then the sensor
 * reports contact.  The sensor can also have up to 3 axes of force sensing.
 * 
 * If falloffCoefficient=0, then the entire patch basically acts as a 
 * rigid strain gauge.  Otherwise, the contact force at local point
 * (x,y) attenuates according to the weight
 * (1-4*|x||y|/(xpatchSize*ypatchSize))^falloffCoefficient so that forces near
 * the edge of the patch are reduced in their influence on the measurement.
 * 
 * Configurable settings:
 * - link (int)
 * - Tsensor (RigidTransform)
 * - patchMin (Vector2)
 * - patchMax (Vector2)
 * - patchTolerance (float)
 * - hasForce (bool [3])
 * - fResolution (Vector3)
 * - fVariance (Vector3)
 * - fSensitivity (float)
 * - fSaturation (Vector3)
 * - falloffCoefficient (float)
 */
class ContactSensor : public SensorBase
{
 public:
  ContactSensor();
  virtual ~ContactSensor() {}
  virtual const char* Type() const override { return "ContactSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;

  int link;                ///< The link on which the sensor is located
  RigidTransform Tsensor;  ///< Local frame of the sensor relative to the link (by convention, origin is at contact patch center, z is normal to surface, out of robot)
  Vector2 patchMin,patchMax;///< The 2D contact patch in the local frame of the sensor 
  Real patchTolerance;     ///< The deformation tolerance of the contact patch (default 0.001)
  bool hasForce[3];        ///< If an element is true, that component of force is measured (default false)
  Vector3 fResolution;     ///< Resolution of the sensor (default 0)
  Vector3 fVariance;       ///< Estimated variance of the sensor (default 0)
  Real fSensitivity;       ///< The minimum z force needed to register contact (default 0)
  Vector3 fSaturation;     ///< The maximum force registered on each axis (default inf)
  Real falloffCoefficient; ///< The falloff coefficient (default 0)

  bool contact;            ///< Measurement: true if contact has been made
  Vector3 force;           ///< Measurement: the force magnitude
};

/** @ingroup Sensing
 * @brief Simulates a force-torque sensor mounted between a link and its
 * parent. Can be configured to be up to 6DOF.
 *
 * To set up an accurate model of a force-torque sensor, you need to 
 * set up a link centered at the sensor frame welded to its parent link.
 * The parent link must contain all the mass of the mount on the mounting side,
 * and the sensor link must contain all the mass on the other (end effector) side.
 * Any descendant links of the end effector (like fingers of a hand) should be
 * attached to the *sensor* link.  NOTE: This conflicts with the way that Klampt's
 * ODE implementation handles welded links! TODO: fix that.
 *
 * Configurable settings:
 * - link (int)
 * - hasForce (bool [3])
 * - hasTorque (bool [3])
 * - fVariance, tVariance (Vector3)
 */
class ForceTorqueSensor : public SensorBase
{
 public:
  ForceTorqueSensor();
  virtual ~ForceTorqueSensor() {}
  virtual const char* Type() const override { return "ForceTorqueSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;

  int link;                ///< The link on which the sensor is located (between link and parent)
  bool hasForce[3];        ///< true if force is measured along the given axis (default false)
  bool hasTorque[3];       ///< true if torque is measured along the given axis (default false)
  Vector3 fVariance, tVariance; ///< Estimated variance of the sensor (default 0)

  Vector3 f,t;             ///< Measurement: the force/torque at the given position, on the link (negative on the parent link)
};

} //namespace Klampt

#endif 
