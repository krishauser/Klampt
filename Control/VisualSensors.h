#ifndef CONTROL_VISUAL_SENSORS_H
#define CONTROL_VISUAL_SENSORS_H

#include "Sensor.h"

/** @ingroup Control
 * @brief Simulates a laser range sensor, either sweeping or stationary.  Can
 * both simulate both 1D sweeping and 2D sweeping.
 * 
 * NOT IMPLEMENTED YET
 */
class LaserRangeSensor : public SensorBase
{
 public:
  LaserRangeSensor();
  virtual const char* Type() const { return "LaserRangeSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;
  RigidTransform Tsensor; ///< z is forward
  double depthResolution;  ///<resolution of the depth measurement
  double depthVarianceLinear,depthVarianceConstant;
  ///enum defining a pattern of the sweep
  enum { SweepSinusoid, SweepTriangular, SweepSawtooth};
  Real xSweepMagnitude,xSweepPeriod,xSweepPhase;
  int xSweepType;
  Real ySweepMagnitude,ySweepPeriod,ySweepPhase;
  int ySweepType;  
};


/** @ingroup Control
 * @brief Simulates a depth camera sensor.  Provides a 2D grid of depth
 * values, capped and quantized.
 * 
 * NOT IMPLEMENTED YET
 */
class DepthCameraSensor : public SensorBase
{
 public:
  DepthCameraSensor();
  virtual const char* Type() const { return "DepthCameraSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);

  int link;
  RigidTransform Tsensor; ///< z is forward
  double zmin,zmax;  ///< range limits
  int zresolution;  ///< resolution in z direction
  double zvarianceLinear;  ///< variance in z estimates, linear term
  double zvarianceConstant;  ///< variance in z estimates, constant term
  double xfov,yfov; ///< field of view in x and y directions
  int xres,yres;  ///< resolution of camera in x and y directions
};

#endif 
