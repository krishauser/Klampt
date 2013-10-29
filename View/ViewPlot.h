#ifndef VIEW_PLOT_H
#define VIEW_PLOT_H

#include <GLdraw/drawextra.h>
#include <GLdraw/GLColor.h>
#include <deque>
#include <vector>
using namespace GLDraw;
using namespace std;

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
  GLColor boundColor,plotAreaColor,axisColor;
  vector<deque<pair<double,double> > > curves;
  vector<GLColor> curveColors;
  double xmin,xmax;
  double ymin,ymax;
};

#endif

