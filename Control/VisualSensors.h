#ifndef CONTROL_VISUAL_SENSORS_H
#define CONTROL_VISUAL_SENSORS_H

#include "Sensor.h"
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/math3d/primitives.h>
using namespace Math3D;

/** @ingroup Control
 * @brief Simulates a laser range sensor, either sweeping or stationary.  Can
 * both simulate both 1D sweeping and 2D sweeping.
 *
 * Default configuration sets up a 1D triangular scan along x direction with 180 degree
 * field of view and instantaneous measurement.  
 *
 * Note: if you place this inside a robot's geometry, the minimum range should extend
 * past the robot's geometry.
 *
 * Currently intensity information is not provided.
 */
class LaserRangeSensor : public SensorBase
{
 public:
  LaserRangeSensor();
  virtual const char* Type() const { return "LaserRangeSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Advance(double dt);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);

  int link;
  RigidTransform Tsensor; ///< z is forward
  int measurementCount;   ///< number of readings per cycle
  double depthResolution; ///<resolution of the depth measurement
  double depthMinimum,depthMaximum;    ///<minimum / maximum depth
  double depthVarianceLinear,depthVarianceConstant;
  ///enum defining a pattern of the sweep.
  ///-Sinusoid is sin(x*2pi),
  ///-Triangular is 2*(1+abs(x mod 2 - 1))-1 (range is [-1,1])
  ///-Sawtooth is 2*(x mod 1)-1 (range is [-1,1])
  enum { SweepSinusoid, SweepTriangular, SweepSawtooth};
  ///left-right angle (rotation about z axis) at time t is magnitude*f((t+phase)/period) where f is the sweep type.
  ///A measurement is produced every 2 * magnitude / measurementCount radians
  ///If period = 0, measurement sweeps over range of [-magnitude,magnitude]
  Real xSweepMagnitude,xSweepPeriod,xSweepPhase;
  int xSweepType;
  Real ySweepMagnitude,ySweepPeriod,ySweepPhase;
  int ySweepType; 

  //simulated depth readings
  vector<double> depthReadings;
  //internal state
  Real last_dt,last_t;
};


/** @ingroup Control
 * @brief Simulates an RGB, D, or RGB+D camera sensor.  Provides a 2D grid of
 * color and/or depth values, capped and quantized.
 *
 * Camera is assumed to be centered at middle of image.  The image
 * is also rectified and depth values are converted to meters (or whatever unit
 * you are generally using).
 *
 * The format of the measurements list is a list of rgb[i,j] pixels if rgb=true, 
 * and followed by a list of d[i,j] pixels giving depth in meters (or whatever unit
 * you are generally using) if depth=true.  The rgb pixels are given as casts from
 * unsigned integers in the pixel format ABGR to doubles.  The depth pixels are given
 * as floats.
 */
class CameraSensor : public SensorBase
{
 public:
  CameraSensor();
  virtual ~CameraSensor();
  virtual const char* Type() const { return "CameraSensor"; }
  virtual void Simulate(ControlledRobotSimulator* robot,WorldSimulation* sim);
  virtual void Reset();
  virtual void MeasurementNames(vector<string>& names) const;
  virtual void GetMeasurements(vector<double>& values) const;
  virtual void SetMeasurements(const vector<double>& values);
  virtual map<string,string> Settings() const;
  virtual bool GetSetting(const string& name,string& str) const;
  virtual bool SetSetting(const string& name,const string& str);
  virtual void DrawGL(const Robot& robot,const vector<double>& measurements);
  void GetViewport(Camera::Viewport& view) const;
  void SetViewport(const Camera::Viewport& view);

  int link;
  RigidTransform Tsensor; ///< z is forward
  bool rgb,depth;  ///< If rgb is true, gives color measurements. If depth is true, gives depth measurements.
  int xres,yres;  ///< resolution of camera in x and y directions (# of pixels)
  double xfov,yfov; ///< field of view in x and y directions (radians)
  double zmin,zmax;  ///< range limits
  int zresolution;  ///< resolution in z direction
  double zvarianceLinear;  ///< variance in z estimates, linear term
  double zvarianceConstant;  ///< variance in z estimates, constant term

  //internal: used for OpenGL rendering / buffers
  bool useGLFramebuffers; 
  unsigned int color_tex,fb,depth_rb;
  vector<unsigned char> pixels;
  vector<float> floats;
  vector<double> measurements;
};

#endif 
