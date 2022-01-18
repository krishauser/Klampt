#include "VisualSensors.h"
#include "Common_Internal.h"
#if HAVE_GLEW
#include <GL/glew.h>
#endif //HAVE_GLEW
//TEST: fallback with no opengl
//#undef HAVE_GLEW
//#define HAVE_GLEW 0

#include "Simulation/SimRobotController.h"
#include "Simulation/ODESimulator.h"
#include "Simulation/Simulator.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLView.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include "View/ViewCamera.h"
#include <KrisLibrary/Timer.h>

#ifndef GL_BGRA
#ifndef GL_BGRA_EXT
#error "GL_BGRA is not defined on your system?"
#endif //GL_BGRA_EXT
#define GL_BGRA GL_BGRA_EXT
#endif //GL_BGRA

DECLARE_LOGGER(Sensing)
#define DEBUG_GL_RENDER_TIMING 0

using namespace Klampt;
using namespace GLDraw;

LaserRangeSensor::LaserRangeSensor()
:link(-1),measurementCount(180),depthResolution(0),depthMinimum(0.1),depthMaximum(Inf),
 depthVarianceLinear(0),depthVarianceConstant(0),
 xSweepMagnitude(DtoR(90.0)),xSweepPeriod(0),xSweepPhase(0),xSweepType(SweepSawtooth),
 ySweepMagnitude(0),ySweepPeriod(0),ySweepPhase(0),ySweepType(SweepSinusoid),
 last_dt(0),last_t(0)
{
  Tsensor.setIdentity();
}

void LaserRangeSensor::Advance(Real dt)
{
  last_dt = dt;
  last_t += dt;
}

double EvalPattern(int type,double x,double correction=1.0)
{
  if(type == LaserRangeSensor::SweepSinusoid)
    return Sin(x*TwoPi);
  else if(type == LaserRangeSensor::SweepTriangular)
    return 2.0*(1.0+Abs(Mod(x,2.0) - 1.0))-1.0;
  return 2.0*(Mod(x/correction,1.0)*correction)-1.0;
}

void LaserRangeSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  if(link >= 0) 
    robot->oderobot->GetLinkTransform(link,robot->robot->links[link].T_World);
  SimulateKinematic(*robot->robot,*sim->world);
}

void LaserRangeSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  depthReadings.resize(measurementCount);
  //need to make sure that the sawtooth pattern hits the last measurement: scale the time domain so last measurement before
  //loop gets 1
  Real xscale = 1, yscale = 1;
  if(xSweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  if(ySweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
    xscale =  1.0 + 1.0/Real(measurementCount-1);
  Real ux0 = (xSweepPeriod == 0 ? 0 : (last_t - last_dt + xSweepPhase)/xSweepPeriod);
  Real ux1 = (xSweepPeriod == 0 ? 1 : (last_t + xSweepPhase)/xSweepPeriod);
  Real uy0 = (ySweepPeriod == 0 ? 0 : (last_t - last_dt + ySweepPhase)/ySweepPeriod);
  Real uy1 = (ySweepPeriod == 0 ? 1 : (last_t + ySweepPhase)/ySweepPeriod);

  //skip previous measurement
  if(xSweepPeriod != 0 && measurementCount > 1) ux0 += (ux1-ux0)/(measurementCount-1);
  if(ySweepPeriod != 0 && measurementCount > 1) uy0 += (uy1-uy0)/(measurementCount-1);
  Ray3D ray;
  RigidTransform T;
  if(link >= 0) {
    T = robot.links[link].T_World;
    T = T*Tsensor;
  }
  else
    T = Tsensor;
  Real xmin=0,xmax=0;
  Real ymin=0,ymax=0;
  Real step = 1.0/Real(measurementCount-1);
  //need to ignore the robot's link geometry
  vector<int> ignoreLinkIDs;
  int robotIndex = -1;
  for(size_t i=0;i<world.robots.size();i++)
    if(world.robots[i]->name == robot.name) {
      robotIndex = (int)i;
      break;
    }
  if(robotIndex >= 0) {
    ignoreLinkIDs.push_back(world.RobotLinkID(robotIndex,link));
  }
  else {
    /*
    printf("LaserRangeSensor::SimulateKinematic: Hmm... couldn't find world robot? %s?\n",robot.name.c_str());
    printf("World has %d robots\n",world.robots.size());
    for(size_t i=0;i<world.robots.size();i++)
      printf("  Name %d: %s\n",i,world.robots[i]->name.c_str());
    */
  }
  for(int i=0;i<measurementCount;i++) {
    Real xtheta,ytheta;
    if(i+1 < measurementCount) {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,(ux0+Real(i)*step*(ux1-ux0)),xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,(uy0+Real(i)*step*(uy1-uy0)),yscale);
    }
    else {
      xtheta = xSweepMagnitude*EvalPattern(xSweepType,ux1,xscale);
      ytheta = ySweepMagnitude*EvalPattern(ySweepType,uy1,yscale);
    }

    xmin = Min(xtheta,xmin);
    xmax = Max(xtheta,xmax);
    ymin = Min(ytheta,ymin);
    ymax = Max(ytheta,ymax);
    Real x = Sin(xtheta);
    Real y = Cos(xtheta)*Sin(ytheta);
    Real z = Cos(xtheta)*Cos(ytheta);
    ray.source = T*(Vector3(x,y,z)*depthMinimum);
    ray.direction = T.R*Vector3(x,y,z);
    Vector3 pt;
    int obj = world.RayCastIgnore(ray,ignoreLinkIDs,pt);
    if (obj >= 0) 
      depthReadings[i] = pt.distance(ray.source) + depthMinimum;
    else 
      depthReadings[i] = Inf;
  }
  //process all depth readings
  for(size_t i=0;i<depthReadings.size();i++) {
    if(!IsInf(depthReadings[i])) 
      depthReadings[i] = Discretize(depthReadings[i],depthResolution,depthReadings[i]*depthVarianceLinear + depthVarianceConstant);
    if(depthReadings[i] <= depthMinimum || depthReadings[i] >= depthMaximum) depthReadings[i] = depthMaximum;
  }
}

void LaserRangeSensor::Reset()
{
  depthReadings.resize(0);
  last_t = 0;
}

void LaserRangeSensor::MeasurementNames(vector<string>& names) const
{
  names.resize(measurementCount);
  for(int i=0;i<measurementCount;i++) {
    stringstream ss;
    ss<<"d["<<i<<"]";
    names[i] = ss.str();
  }
}

void LaserRangeSensor::GetMeasurements(vector<double>& values) const
{
  values = depthReadings;
}

void LaserRangeSensor::SetMeasurements(const vector<double>& values)
{
  depthReadings = values;
}

map<string,string> LaserRangeSensor::Settings() const
{
  map<string,string> res = SensorBase::Settings();
  FILL_SENSOR_SETTING(res,link);
  FILL_SENSOR_SETTING(res,Tsensor);
  FILL_SENSOR_SETTING(res,measurementCount);
  FILL_SENSOR_SETTING(res,depthResolution);
  FILL_SENSOR_SETTING(res,depthMinimum);
  FILL_SENSOR_SETTING(res,depthMaximum);
  FILL_SENSOR_SETTING(res,depthVarianceLinear);
  FILL_SENSOR_SETTING(res,depthVarianceConstant);
  FILL_SENSOR_SETTING(res,xSweepMagnitude);
  FILL_SENSOR_SETTING(res,xSweepPeriod);
  FILL_SENSOR_SETTING(res,xSweepPhase);
  FILL_SENSOR_SETTING(res,xSweepType);
  FILL_SENSOR_SETTING(res,ySweepMagnitude);
  FILL_SENSOR_SETTING(res,ySweepPeriod);
  FILL_SENSOR_SETTING(res,ySweepPhase);
  FILL_SENSOR_SETTING(res,ySweepType);
  return res;
}
bool LaserRangeSensor::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_SENSOR_SETTING(measurementCount);
  GET_SENSOR_SETTING(depthResolution);
  GET_SENSOR_SETTING(depthMinimum);
  GET_SENSOR_SETTING(depthMaximum);
  GET_SENSOR_SETTING(depthVarianceLinear);
  GET_SENSOR_SETTING(depthVarianceConstant);
  GET_SENSOR_SETTING(xSweepMagnitude);
  GET_SENSOR_SETTING(xSweepPeriod);
  GET_SENSOR_SETTING(xSweepPhase);
  GET_SENSOR_SETTING(xSweepType);
  GET_SENSOR_SETTING(ySweepMagnitude);
  GET_SENSOR_SETTING(ySweepPeriod);
  GET_SENSOR_SETTING(ySweepPhase);
  GET_SENSOR_SETTING(ySweepType);
  if(SensorBase::GetSetting(name,str)) return true;
  return false;
}
bool LaserRangeSensor::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_SENSOR_SETTING(measurementCount);
  SET_SENSOR_SETTING(depthResolution);
  SET_SENSOR_SETTING(depthMinimum);
  SET_SENSOR_SETTING(depthMaximum);
  SET_SENSOR_SETTING(depthVarianceLinear);
  SET_SENSOR_SETTING(depthVarianceConstant);
  SET_SENSOR_SETTING(xSweepMagnitude);
  SET_SENSOR_SETTING(xSweepPeriod);
  SET_SENSOR_SETTING(xSweepPhase);
  SET_SENSOR_SETTING(xSweepType);
  SET_SENSOR_SETTING(ySweepMagnitude);
  SET_SENSOR_SETTING(ySweepPeriod);
  SET_SENSOR_SETTING(ySweepPhase);
  SET_SENSOR_SETTING(ySweepType);
  if(SensorBase::SetSetting(name,str)) return true;
  return false;
}

void LaserRangeSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements) 
{
  RigidTransform T = (link >= 0 ? robot.links[link].T_World*Tsensor : Tsensor);
  glPushMatrix();
  glMultMatrix(Matrix4(T));
    glDisable(GL_LIGHTING);
    GLDraw::drawCoords(0.1);
    
    glEnable(GL_BLEND);
    glDepthMask(GL_FALSE);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    glBegin(GL_LINES);
    //glBegin(GL_TRIANGLE_STRIP);
    //need to make sure that the sawtooth pattern hits the last measurement: scale the time domain so last measurement before
    //loop gets 1
    Real xscale = 1, yscale = 1;
    if(xSweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
      xscale =  1.0 + 1.0/Real(measurementCount-1);
    if(ySweepType == SweepSawtooth && last_dt > 0 && measurementCount > 1) 
      xscale =  1.0 + 1.0/Real(measurementCount-1);
    Real ux0 = (xSweepPeriod == 0 ? 0 : (last_t - last_dt + xSweepPhase)/(xSweepPeriod));
    Real ux1 = (xSweepPeriod == 0 ? 1 : (last_t + xSweepPhase)/(xSweepPeriod));
    Real uy0 = (ySweepPeriod == 0 ? 0 : (last_t - last_dt + ySweepPhase)/(ySweepPeriod));
    Real uy1 = (ySweepPeriod == 0 ? 1.0 : (last_t + ySweepPhase)/(ySweepPeriod));
    if(xSweepPeriod != 0) ux0 += (ux1-ux0)/(measurementCount);
    if(ySweepPeriod != 0) uy0 += (uy1-uy0)/(measurementCount);
    int skip = 1;
    if(measurementCount > 250)
      skip = measurementCount/250;
    Real step = 1.0/Real(measurementCount-1);
    for(int i=0;i<measurementCount;i+=skip) {
      if(!measurements.empty())
        if(IsInf(depthReadings[i])) continue;
      Real u=Real(i)*step;
      Real xtheta,ytheta;
      if(i+1 < measurementCount) {
        xtheta = xSweepMagnitude*EvalPattern(xSweepType,(ux0+u*(ux1-ux0)),xscale);
        ytheta = ySweepMagnitude*EvalPattern(ySweepType,(uy0+u*(uy1-uy0)),yscale);
      }
      else {
        xtheta = xSweepMagnitude*EvalPattern(xSweepType,ux1,xscale);
        ytheta = ySweepMagnitude*EvalPattern(ySweepType,uy1,yscale);
      }
      Real x = Sin(xtheta);
      Real y = Cos(xtheta)*Sin(ytheta);
      Real z = Cos(xtheta)*Cos(ytheta);
      Vector3 dir = Vector3(x,y,z);
      glColor4f(1,0,0,0);
      glVertex3v(depthMinimum*dir);
      glColor4f(1,0,0,1);
      if(measurements.empty()) 
        glVertex3v(depthMaximum*dir);
      else
        glVertex3v(depthReadings[i]*dir);
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  glPopMatrix();
}



CameraSensor::CameraSensor()
:link(-1),rgb(true),depth(true),xres(640),yres(480),
 xfov(DtoR(56.0)),yfov(DtoR(43.0)),
 zmin(0.4),zmax(4.0),zresolution(0),
 zvarianceLinear(0),zvarianceConstant(0),
 useGLFramebuffers(true)
{
  Tsensor.setIdentity();
}

CameraSensor::~CameraSensor()
{
}

void CameraSensor::SimulateKinematic(RobotModel& robot,WorldModel& world)
{
  #if DEBUG_GL_RENDER_TIMING
  Timer timer;
  #endif //DEBUG_GL_RENDER_TIMING
  RigidTransform Tlink;
  if(link >= 0) Tlink = robot.links[link].T_World;
  else Tlink.setIdentity();

  if(useGLFramebuffers) {
    if(!renderer.Setup(xres,yres,true,false)) {
      LOG4CXX_WARN(GET_LOGGER(Sensing),"CameraSensor: Couldn't initialize GLEW, falling back to slow mode");
      const GLubyte* glVersion = glGetString(GL_VERSION);
      if(glVersion) {
        LOG4CXX_WARN(GET_LOGGER(Sensing),"  GL version is: "<<glVersion);
      }
      else {
        LOG4CXX_WARN(GET_LOGGER(Sensing),"  GL version could not be queried");
      }
      useGLFramebuffers = false;
    }
    #if DEBUG_GL_RENDER_TIMING
    printf("CameraSensor: Setup renderer %f\n",timer.ElapsedTime());
    timer.Reset();
    #endif //DEBUG_GL_RENDER_TIMING
  }
  if(useGLFramebuffers) {
    //set up the POV of the camera
    Camera::Viewport vp;
    GetViewport(vp);
    vp.xform = Tlink*vp.xform;
    vp.n = Min(0.1f,vp.n);
    vp.f = Max(100.f,vp.f);
    renderer.Begin(vp);
    //-------------------------
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    //-------------------------
    //now render the scene from the POV of the camera
    world.DrawGL();
    //DONE: now captured on graphics card in framebuffer
    //----------------
    renderer.End();
    #if DEBUG_GL_RENDER_TIMING
    printf("CameraSensor: Draw GL world %f\n",timer.ElapsedTime());
    timer.Reset();
    #endif //DEBUG_GL_RENDER_TIMING
    DEBUG_GL_ERRORS()

    //extract measurements
    if(rgb) {
      //renderer.GetRGBA(pixels);
      renderer.GetRGB(pixels);
      #if DEBUG_GL_RENDER_TIMING
      printf("CameraSensor: Download RGBA %f\n",timer.ElapsedTime());
      timer.Reset();
      #endif //DEBUG_GL_RENDER_TIMING
    }
    if(depth) {
      renderer.GetDepth(vp,floats);
      #if DEBUG_GL_RENDER_TIMING
      printf("CameraSensor: Download depth %f\n",timer.ElapsedTime());
      timer.Reset();
      #endif //DEBUG_GL_RENDER_TIMING
      
      double invzresolution = 1.0/zresolution;
      float fzlim = float(zmax)-1e-4f;
      float fzmax = float(zmax);
      if(zvarianceLinear == 0) {
        Real zstdev = Sqrt(zvarianceConstant);
        int k=0;
        for(int j=0;j<yres;j++) {
          for(int i=0;i<xres;i++,k++) {
            if(floats[k] < fzlim) {
              floats[k] = (float)Discretize2(floats[k],zresolution,invzresolution,zstdev);
            }
            else {
              floats[k] = fzmax;  //assume within numerical error
            }
          }
        }
      }
      else {
        int k=0;
        for(int j=0;j<yres;j++) {
          for(int i=0;i<xres;i++,k++) {
            if(floats[k] < fzlim) {
              floats[k] = (float)Discretize2(floats[k],zresolution,invzresolution,Sqrt(zvarianceLinear*floats[k] + zvarianceConstant));
              //TEMP: testing simpler discretization
              //floats[k] = (float)Discretize2(floats[k],zresolution,invzresolution,zstdev);
            }
            else {
              floats[k] = fzmax;  //assume within numerical error
            }
          }
        }
      }
      #if DEBUG_GL_RENDER_TIMING
      printf("CameraSensor: Discretize depths %f\n",timer.ElapsedTime());
      timer.Reset();
      #endif //DEBUG_GL_RENDER_TIMING
    }
    DEBUG_GL_ERRORS()
  }
  else {
    //fallback will use ray casting: (slow!)
    //set up the POV of the camera
    Camera::Viewport vp;
    GetViewport(vp);
    vp.xform = Tlink*vp.xform;
    Ray3D ray;
    Vector3 vsrc;
    Vector3 vfwd,dx,dy;
    vp.getClickSource(0,0,vsrc);
    vp.getViewVector(vfwd);
    dx = Vector3(vp.xDir());
    dx *= 1.0/(vp.w*vp.scale);
    dy = Vector3(vp.yDir());
    dy *= 1.0/(vp.w*vp.scale);
    if(rgb)
      pixels.resize(xres*yres*3);
    if(depth)
      floats.resize(xres*yres);
    int k=0;
    unsigned char bg_r=0x96, bg_g=0xaa, bg_b = 0xff;
    float fzmax = (float)zmax;
    Vector3 pt;
    for(int j=0;j<yres;j++) {
      Real v = 0.5*yres - Real(j);
      for(int i=0;i<xres;i++,k++) {
        Real u = Real(i) - 0.5*xres;    
        ray.direction = vfwd + u*dx + v*dy;
        ray.direction.inplaceNormalize();
        ray.source = vsrc + ray.direction * zmin / (vfwd.dot(ray.direction));
        int obj = world.RayCast(ray,pt);
        if (obj >= 0) {
          if(rgb) {
            //get color of object
            //TODO: lighting
            WorldModel::AppearancePtr app = world.GetAppearance(obj);
            const float* rgba = app->faceColor.rgba;
            pixels[k*3] = (unsigned char)(rgba[0]*255.0);
            pixels[k*3+1] = (unsigned char)(rgba[1]*255.0);
            pixels[k*3+2] = (unsigned char)(rgba[2]*255.0);
          }
          Real d = vfwd.dot(pt - vsrc);
          d = Min(d,zmax);
          if(depth && d < zmax) floats[k] = (float)Discretize(d,zresolution,zvarianceLinear*d + zvarianceConstant);
        }
        else {
          //no reading
          if(rgb) {
            pixels[k*3] = bg_r;
            pixels[k*3+1] = bg_g;
            pixels[k*3+2] = bg_b;
          }
          if(depth) floats[k] = fzmax;
        }
      }
    }
    static bool warned = false;
    if(!warned) {
      LOG4CXX_WARN(GET_LOGGER(Sensing),"CameraSensor: doing fallback from GLEW... "<<k<<" rays cast, may be slow");
      warned = true;
    }
    #if DEBUG_GL_RENDER_TIMING
    printf("CameraSensor: Software ray-casting and extraction %f\n",timer.ElapsedTime());
    timer.Reset();
    #endif //DEBUG_GL_RENDER_TIMING

    //need to upload the texture for sensor visualization
    if(renderer.color_tex == 0) { 
      //RGBA8 2D texture, 24 bit depth texture, 256x256
      glGenTextures(1, &renderer.color_tex);
      if(renderer.color_tex != 0) {
        glBindTexture(GL_TEXTURE_2D, renderer.color_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      }
    }
    if(renderer.color_tex != 0) {
      glBindTexture(GL_TEXTURE_2D, renderer.color_tex);
      //copy measurements into buffer -- don't forget y flip
      vector<unsigned char> image(xres*yres*3);
      int k=0;
      for(int j=0;j<yres;j++)
        memcpy(&image[(yres-j-1)*xres*3],&pixels[j*xres*3],xres*3);
      //NULL means reserve texture memory, but texels are undefined
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, xres, yres, 0, GL_BGRA, GL_UNSIGNED_BYTE, &image[0]);
    }
    #if DEBUG_GL_RENDER_TIMING
    printf("CameraSensor: Software texture setup %f\n",timer.ElapsedTime());
    #endif //DEBUG_GL_RENDER_TIMING
  }
  depthDisplayList.erase();
}

void CameraSensor::Simulate(SimRobotController* robot,Simulator* sim)
{
  sim->UpdateModel();
  if (link >= 0)  //make sure we get the true simulated link transform
    robot->oderobot->GetLinkTransform(link,robot->robot->links[link].T_World);
  SimulateKinematic(*robot->robot,*sim->world);
}

void CameraSensor::Reset()
{
}

void CameraSensor::MeasurementNames(vector<string>& names) const
{
  char buf[64];
  names.resize(0);
  if(rgb) {
    for(int i=0;i<xres;i++) {
      for(int j=0;j<yres;j++) {
        snprintf(buf,64,"rgb[%d,%d]",i,j);
        names.push_back(buf);
      }
    }
  }
  if(depth) {
    for(int i=0;i<xres;i++) {
      for(int j=0;j<yres;j++) {
        snprintf(buf,64,"d[%d,%d]",i,j);
        names.push_back(buf);
      }
    }
  }
}

void CameraSensor::GetMeasurements(vector<double>& measurements) const
{
  if(pixels.empty() && floats.empty()) {
    measurements.resize(0);
    return;
  }
  #if DEBUG_GL_RENDER_TIMING
  Timer timer;
  #endif //DEBUG_GL_RENDER_TIMING
  measurements.resize((rgb ? xres*yres : 0) + (depth? xres*yres : 0));
  size_t dstart = (rgb ? xres*yres : 0);
  if(rgb) {
    int l=0;
    int k=0;
    for(int j=0;j<yres;j++) {
      //for(int i=0;i<xres;i++,k+=4) {
        //unsigned int pix = (pixels[k] << 24 ) | (pixels[k+1] << 16 ) | (pixels[k+2] << 8 ) | (pixels[k+3]);
      for(int i=0;i<xres;i++,k+=3) {
        unsigned int pix = (pixels[k] << 16 | pixels[k+1] << 8 | pixels[k+2]);
        measurements[l++] = double(pix);
      }
    }
  }
  if(depth) {
    int k=0;
    for(int j=0;j<yres;j++) {
      for(int i=0;i<xres;i++,k++) {
        measurements[dstart+k] = floats[k];
      }
    }
  }
  #if DEBUG_GL_RENDER_TIMING
  printf("CameraSensor: Extract measurements %f\n",timer.ElapsedTime());
  timer.Reset();
  #endif //DEBUG_GL_RENDER_TIMING
}

void CameraSensor::SetMeasurements(const vector<double>& values)
{
  fprintf(stderr,"CameraSensor::SetMeasurements: not implemented yet\n");
  //measurements = values;
  //TODO: copy back into pixel buffers?
}

map<string,string> CameraSensor::Settings() const
{
  map<string,string> res = SensorBase::Settings();
  FILL_SENSOR_SETTING(res,link);
  FILL_SENSOR_SETTING(res,Tsensor);
  FILL_SENSOR_SETTING(res,rgb);
  FILL_SENSOR_SETTING(res,depth);
  FILL_SENSOR_SETTING(res,xres);
  FILL_SENSOR_SETTING(res,xfov);
  FILL_SENSOR_SETTING(res,yres);
  FILL_SENSOR_SETTING(res,yfov);
  FILL_SENSOR_SETTING(res,zresolution);
  FILL_SENSOR_SETTING(res,zmin);
  FILL_SENSOR_SETTING(res,zmax);
  FILL_SENSOR_SETTING(res,zvarianceLinear);
  FILL_SENSOR_SETTING(res,zvarianceConstant);
  return res;
}
bool CameraSensor::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(link);
  GET_SENSOR_SETTING(Tsensor);
  GET_SENSOR_SETTING(rgb);
  GET_SENSOR_SETTING(depth);
  GET_SENSOR_SETTING(xres);
  GET_SENSOR_SETTING(xfov);
  GET_SENSOR_SETTING(yres);
  GET_SENSOR_SETTING(yfov);
  GET_SENSOR_SETTING(zresolution);
  GET_SENSOR_SETTING(zmin);
  GET_SENSOR_SETTING(zmax);
  GET_SENSOR_SETTING(zvarianceLinear);
  GET_SENSOR_SETTING(zvarianceConstant);
  if(SensorBase::GetSetting(name,str)) return true;
  return false;
}
bool CameraSensor::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(link);
  SET_SENSOR_SETTING(Tsensor);
  SET_SENSOR_SETTING(rgb);
  SET_SENSOR_SETTING(depth);
  SET_SENSOR_SETTING(xres);
  SET_SENSOR_SETTING(xfov);
  SET_SENSOR_SETTING(yres);
  SET_SENSOR_SETTING(yfov);
  SET_SENSOR_SETTING(zresolution);
  SET_SENSOR_SETTING(zmin);
  SET_SENSOR_SETTING(zmax);
  SET_SENSOR_SETTING(zvarianceLinear);
  SET_SENSOR_SETTING(zvarianceConstant);
  if(SensorBase::SetSetting(name,str)) return true;
  return false;
}

void doTriangle(const Vector3& a,const Vector3& b,const Vector3& c)
{
  Vector3 n;
  n.setCross(b-a,c-a);
  n.inplaceNormalize();
  glNormal3v(n);
  glVertex3v(a);
  glVertex3v(b);
  glVertex3v(c);
}

void CameraSensor::DrawGL(const RobotModel& robot,const vector<double>& measurements) 
{
  Camera::Viewport v;
  GetViewport(v);
  if(link >= 0) 
    v.xform = robot.links[link].T_World*v.xform;

  size_t vstart = 0;
  if(rgb) vstart = xres*yres;
  if(depth && !measurements.empty()) {
    glPushMatrix();
    glMultMatrix((Matrix4)v.xform);

    //check whether this is the same measurements as before
    unsigned int hash = 0;
    for(size_t i=0;i<measurements.size();i+=measurements.size()/100) {
      auto mptr = (const unsigned int*)&measurements[i];
      hash ^= mptr[0];
      hash ^= mptr[1];
    }
    if(depthDisplayHash != hash) {
      depthDisplayList.erase();
      depthDisplayHash = hash;
    }

    if(!depthDisplayList) {
      depthDisplayList.beginCompileAndExecute();
      glEnable(GL_LIGHTING);
      float white[4]={1,1,1,1};
      glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,white);
      Real vscale = 0.5/Tan(xfov*0.5);
      Real xscale = (0.5/vscale)/(xres/2);
      //Real aspectRatio = Real(xres)/Real(yres);
      Real yscale = xscale;
      vector<Vector3> pts(xres*yres);
      int k=0;
      for(int i=0;i<yres;i++)
        for(int j=0;j<xres;j++,k++) {
          double d = measurements[vstart+k];
          double u = Real(j-xres/2);
          double v = -Real(i-yres/2);
          double x = xscale*d*u;
          double y = yscale*d*v;
          pts[k].set(x,y,-d);
        }
      glBegin(GL_TRIANGLES);
      k=0;
      for(int i=0;i<yres;i++) {
        for(int j=0;j<xres;j++,k++) {
          if(i+1 >= yres || j+1 >= xres) continue;
          //decide on discontinuities in this cell
          int v11 = k;
          int v12 = k+1;
          int v21 = k+xres;
          int v22 = k+xres+1;
          double z11 = -pts[v11].z;
          double z12 = -pts[v12].z;
          double z21 = -pts[v21].z;
          double z22 = -pts[v22].z;
          bool d1x = (z11 >= zmax || z12 >= zmax || Abs(z11 - z12) > 0.02*(z11+z12));
          bool d1y = (z11 >= zmax || z21 >= zmax || Abs(z11 - z21) > 0.02*(z11+z21));
          bool d2x = (z22 >= zmax || z21 >= zmax || Abs(z22 - z21) > 0.02*(z22+z21));
          bool d2y = (z22 >= zmax || z12 >= zmax || Abs(z22 - z12) > 0.02*(z22+z12));
          bool dupperleft = (d1x || d1y);
          bool dupperright = (d1x || d2y);
          bool dlowerleft = (d2x || d1y);
          bool dlowerright = (d2x || d2y);


          if(dupperleft && !dlowerright) 
            //only draw lower right corner
            doTriangle(pts[v12],pts[v21],pts[v22]);
          else if(!dupperleft && dlowerright) 
            //only draw upper left corner
            doTriangle(pts[v11],pts[v21],pts[v12]);
          else if(!dupperright && dlowerleft) 
            //only draw upper right corner
            doTriangle(pts[v11],pts[v22],pts[v12]);
          else if(dupperright && !dlowerleft) 
            //only draw lower left corner
            doTriangle(pts[v11],pts[v21],pts[v22]);
          else if (!dupperleft && !dlowerright) {
            //fully connected -- should draw better conditioned edge, but whatever
            doTriangle(pts[v12],pts[v21],pts[v22]);
            doTriangle(pts[v11],pts[v21],pts[v12]);
          }
        }
      }
      glEnd();
      depthDisplayList.endCompile();
    }
    else {
      depthDisplayList.call();
    }
    glPopMatrix();
  }

  
  if(rgb && !measurements.empty() && renderer.color_tex != 0) {
    //debugging: draw image in frustum
    glPushMatrix();
    glMultMatrix((Matrix4)v.xform);
    Real d = Max(v.n,0.25f);
    Real aspectRatio = Real(xres)/Real(yres);
    Real xmin = Real(v.x - v.w*0.5)/(Real(v.w)*0.5);
    Real xmax = Real(v.x + v.w*0.5)/(Real(v.w)*0.5);
    Real ymax = -Real(v.y - v.h*0.5)/(Real(v.h)*0.5);
    Real ymin = -Real(v.y + v.h*0.5)/(Real(v.h)*0.5);
    Real xscale = 0.5*d/v.scale;
    Real yscale = xscale/aspectRatio;
    xmin *= xscale;
    xmax *= xscale;
    ymin *= yscale;
    ymax *= yscale;
    glBindTexture(GL_TEXTURE_2D,renderer.color_tex);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1,1,1,0.5);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glBegin(GL_TRIANGLE_FAN);
    glTexCoord2f(0,0);
    glVertex3f(xmin,ymin,-d);
    glTexCoord2f(1,0);
    glVertex3f(xmax,ymin,-d);
    glTexCoord2f(1,1);
    glVertex3f(xmax,ymax,-d);
    glTexCoord2f(0,1);
    glVertex3f(xmin,ymax,-d);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
  }


  ViewCamera view;
  view.DrawGL(v);

  DEBUG_GL_ERRORS()
}


void CameraSensor::GetViewport(Camera::Viewport& vp) const
{
  vp.perspective = true;
  vp.x = vp.y = 0;
  vp.w = xres;
  vp.h = yres;
  vp.n = zmin;
  vp.f = zmax;
  vp.setFOV(xfov);
  vp.xform = Tsensor;
  Matrix3 flipYZ;
  flipYZ.setZero();
  flipYZ(0,0) = 1;
  flipYZ(1,1) = -1;
  flipYZ(2,2) = -1;
  vp.xform.R = vp.xform.R*flipYZ;
}

