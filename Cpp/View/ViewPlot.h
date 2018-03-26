#ifndef VIEW_PLOT_H
#define VIEW_PLOT_H

#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <deque>
#include <vector>
#include <string>
using namespace std;

/** @brief An OpenGL x-y auto-scrolling plot.
 * Used in SimTest (Interface/SimTestGUI.h) to draw sensor data.
 */
class ViewPlot
{
 public:
  ViewPlot();
  void AddCurve(vector<double>& ys);
  void AddPoint(double x,double y,int curve=0);
  void AutoXRange();
  void AutoYRange();
  void AutoErase();
  void DrawGL();
  
  bool xMonotone,autoXRange,autoYRange,autoErase;
  bool drawBound,drawPlotArea,drawXRangeBottom,drawXRangeTop,drawYRangeLeft,drawYRangeRight;
  string xlabel,ylabel;
  int x,y,width,height;
  GLDraw::GLColor boundColor,plotAreaColor,axisColor;
  vector<deque<pair<double,double> > > curves;
  vector<GLDraw::GLColor> curveColors;
  double xmin,xmax;
  double ymin,ymax;
};

#endif

