#ifndef CONTROL_VISUAL_SENSORS_H
#define CONTROL_VISUAL_SENSORS_H

#include "Sensor.h"
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/GLdraw/GLRenderToImage.h>
#include <KrisLibrary/GLdraw/GLDisplayList.h>

namespace Klampt {
  using namespace Math3D;

class WorldModel;
class RobotModel;

/** @ingroup Sensing
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
 *
 * Configurable settings:
 * - link
 * - Tsensor
 * - measurementCount
 * - depthResolution
 * - depthMinimum,depthMaximum
 * - depthVarianceLinear,depthVarianceConstant
 * - xSweepMagnitude, xSweepPeriod, xSweepPhase
 * - xSweepType
 * - ySweepMagnitude, ySweepPeriod, ySweepPhase
 * - ySweepType
 */
class LaserRangeSensor : public SensorBase
{
 public:
  LaserRangeSensor();
  virtual ~LaserRangeSensor() {}
  virtual const char* Type() const override { return "LaserRangeSensor"; }
  virtual void Simulate(SimRobotController* robot,Simulator* sim) override;
  virtual void SimulateKinematic(RobotModel& robot,WorldModel& world) override;
  virtual void Advance(double dt) override;
  virtual void Reset() override;
  virtual void MeasurementNames(vector<string>& names) const override;
  virtual void GetMeasurements(vector<double>& values) const override;
  virtual void SetMeasurements(const vector<double>& values) override;
  virtual map<string,string> Settings() const override;
  virtual bool GetSetting(const string& name,string& str) const override;
  virtual bool SetSetting(const string& name,const string& str) override;
  virtual void DrawGL(const RobotModel& robot,const vector<double>& measurements) override;

  int link;
  RigidTransform Tsensor; ///< z is forward, x points left, y points up
  int measurementCount;   ///< number of readings per cycle
  double depthResolution; ///<resolution of the depth measurement
  double depthMinimum,depthMaximum;    ///<minimum / maximum depth
  double depthVarianceLinear,depthVarianceConstant;
  ///enum defining a pattern of the sweep.
  ///
  ///- Sinusoid is sin(u*2pi),
  ///- Triangular is 2*(1+abs(u mod 2 - 1))-1 (range is [-1,1])
  ///- Sawtooth is 2*(u mod 1)-1 (range is [-1,1])
  ///
  ///To do a continuously rotating sensor, do a sawtooth with magnitude 2pi
  enum { SweepSinusoid, SweepTriangular, SweepSawtooth};
  ///if period != 0:
  ///   yaw angle (rotation about y axis) at time t is magnitude*f((t+phase)/period) where f is the sweep type.
  ///   A measurement is produced every 2 * magnitude / measurementCount radians.
  ///If period = 0:
  ///   measurement sweeps over range of [-magnitude,magnitude]
  Real xSweepMagnitude,xSweepPeriod,xSweepPhase;
  int xSweepType;
  ///If period != 0:
  ///    pitch angle (rotation about x axis) at time t is magnitude*f((t+phase)/period) where f is the sweep type.
  ///    A measurement is produced every 2 * magnitude / measurementCount radians.
  ///If period = 0:
  ///    measurement sweeps over range of [-magnitude,magnitude]
  Real ySweepMagnitude,ySweepPeriod,ySweepPhase;
  int ySweepType; 

  //simulated depth readings
  vector<double> depthReadings;
  //internal state
  Real last_dt,last_t;
};


/** @ingroup Sensing
 * @brief Simulates an RGB, D, or RGB+D camera sensor.  Provides a 2D grid of
 * color and/or depth values, capped and quantized.
 *
 * Camera is assumed to be centered at middle of image.  The image
 * is also rectified and depth values are converted to meters (or whatever unit
 * you are generally using).  The coordinate convention is Z forward, X right, Y *down*
 * following OpenCV, ROS and other camera standards.
 *
 * The format of the measurements list is a list of rgb[i,j] pixels if rgb=true, 
 * then followed by a list of d[i,j] pixels giving depth in meters (or whatever unit
 * you are generally using) if depth=true.  The rgb pixels are given as casts from
 * unsigned integers in the pixel format 0xrrggbb to doubles.  The depth pixels are given
 * as floats.
 *
 * The list of measurements proceeds in scan-line order from the upper-left pixel.
 *
 * For optimal performance using the graphics card, you must install the GLEW package
 * on your system.  You must also initialize OpenGL before running the simulator, 
 * which typically requires popping up a visualization window.
 *
 * Configurable settings:
 * - link: int
 * - Tsensor: RigidTransform
 * - rgb,depth: bool
 * - xres,yres: int
 * - xfov,yfov: float
 * - zmin,zmax: float
 * - zresolution: int
 * - zvarianceLinear,zvarianceConstant: float
 */
class CameraSensor : public SensorBase
{
 public:
  CameraSensor();
  virtual ~CameraSensor();
  virtual const char* Type() const override { return "CameraSensor"; }
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
  ///Gets the OpenGL view associated with the camera.  The result is in the link's local frame.
  ///Note that in OpenGL views, Z is backward, and Y is up.
  void GetViewport(Camera::Viewport& view) const;
  ///Sets the camera to match the OpenGL view.  The view is assumed to be in the link's local frame.
  ///Note that in OpenGL views, Z is backward, and Y is up.
  void SetViewport(const Camera::Viewport& view);

  int link;
  RigidTransform Tsensor; ///< z is forward, x is to the right of image, and y is *down*
  bool rgb,depth;  ///< If rgb is true, gives color measurements. If depth is true, gives depth measurements.
  int xres,yres;  ///< resolution of camera in x and y directions (# of pixels)
  double xfov,yfov; ///< field of view in x and y directions (radians)
  double zmin,zmax;  ///< range limits, > 0
  int zresolution;  ///< resolution in z direction
  double zvarianceLinear;  ///< variance in z estimates, linear term
  double zvarianceConstant;  ///< variance in z estimates, constant term

  //internal: used for OpenGL rendering / buffers
  bool useGLFramebuffers; 
  GLDraw::GLRenderToImage renderer;
  //last measurements
  vector<unsigned char> pixels;
  vector<float> floats;
  //visualization state
  GLDraw::GLDisplayList depthDisplayList;
  unsigned int depthDisplayHash;
};

} //namespace Klampt

#endif 
