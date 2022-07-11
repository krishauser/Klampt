#ifndef _WIDGET_H
#define _WIDGET_H

#include <string>

/** @file widget.h 
 * API to interact with with C++ Widgets
 */

class RigidObjectModel;
class RobotModel;
class IKObjective;

class Viewport
{
 public:
  bool fromJson(const std::string& str);
  std::string toJson() const;
  void setModelviewMatrix(const double M[16]);
  void setRigidTransform(const double R[9],const double t[3]);
  void getRigidTransform(double out[9],double out2[3]);

  bool perspective;
  float scale;
  
  int x,y,w,h;
  double n, f;

  //column major model view transform matrix
  std::vector<double> xform;
};

class Widget
{
 public:
  Widget();
  ~Widget();
  bool hover(int x,int y,const Viewport& viewport);
  bool beginDrag(int x,int y,const Viewport& viewport);
  void drag(int dx,int dy,const Viewport& viewport);
  void endDrag();
  void keypress(char c);
  void drawGL(const Viewport& viewport);
  void idle();
  bool wantsRedraw();
  bool hasHighlight();
  bool hasFocus();

  int index;
};

class WidgetSet : public Widget
{
 public:
  WidgetSet();
  void add(const Widget& subwidget);
  void remove(const Widget& subwidget);
  void enable(const Widget& subwidget,bool enabled);
};

class PointPoser : public Widget
{
 public:
  PointPoser();
  void set(const double t[3]);
  void get(double out[3]);
  ///Sets the reference axes (by default aligned to x,y,z)
  void setAxes(const double R[9]);
  void enableAxes(bool x,bool y,bool z);
};

class TransformPoser : public Widget
{
 public:
  TransformPoser();
  void set(const double R[9],const double t[3]);
  void get(double out[9],double out2[3]);
  void enableTranslation(bool);
  void enableRotation(bool);
  void enableTranslationAxes(bool x,bool y,bool z);
  void enableRotationAxes(bool x,bool y,bool z);
};

class ObjectPoser : public Widget
{
 public:
  ObjectPoser(RigidObjectModel& object);
  void set(const double R[9],const double t[3]);
  void get(double out[9],double out2[3]);
};

class RobotPoser : public Widget
{
 public:
  RobotPoser(RobotModel& robot);
  void setActiveDofs(const std::vector<int>& dofs);
  void set(const std::vector<double>& q);
  void get(std::vector<double>& out);
  void getConditioned(const std::vector<double>& qref,std::vector<double>& out);
  void addIKConstraint(const IKObjective& obj);
  void clearIKConstraints();
};

class AABBPoser : public Widget
{
 public:
  AABBPoser();
  void set(const double bmin[3],const double bmax[3]);
  void setFrame(const double R[9],const double t[3]);
  void get(double out[3],double out2[3]);
};

class BoxPoser : public Widget
{
 public:
  BoxPoser();
  void set(const double R[9],const double t[3],const double dims[3]);
  void setTransform(const double R[9],const double t[3]);
  void setDims(const double dims[3]);
  void getTransform(double out[9],double out2[3]);
  void getDims(double out[3]);
};

class SpherePoser : public Widget
{
 public:
  SpherePoser();
  void set(const double cr[4]);
  void get(double out[4]);
};



#endif
