#include "ViewPlot.h"
#include <KrisLibrary/utils.h>
#include <KrisLibrary/math/random.h>
#include <stdio.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <iostream>
using namespace GLDraw;

ViewPlot::ViewPlot()
  :xMonotone(true),autoXRange(true),autoYRange(true),autoErase(true),
   drawBound(true),drawPlotArea(true),drawXRangeBottom(true),drawXRangeTop(false),drawYRangeLeft(false),drawYRangeRight(true),
   x(0),y(0),width(100),height(80),xmin(0),xmax(1),ymin(0),ymax(1)
{
  axisColor.set(0.3f,0.3f,0.3f);
  boundColor.set(0.5f,0.5f,0.5f);
  plotAreaColor.set(0,0,0,0.5);
}

void ViewPlot::AddCurve(vector<double>& ys)
{
  if(curves.empty() && autoXRange) xmin = xmax = 0;
  if(curves.empty() && autoYRange) ymin = ymax = ys[0];
  curves.resize(curves.size()+1);
  curveColors.resize(curveColors.size()+1);
  curveColors.back().setHSV((float)Math::Rand()*360,1,1);
  for(size_t i=0;i<ys.size();i++)
    curves.back().push_back(pair<double,double>((int)i,ys[i]));
  if(autoXRange) {
    xmin = Min(xmin,0.0);
    xmax = Max(xmax,double(ys.size()-1));
  }
  if(autoYRange) {
    for(size_t i=0;i<ys.size();i++) {
      ymin = Min(ymin,ys[i]);
      ymax = Max(ymax,ys[i]);
    }
  }
  if(autoErase) {
    AutoErase();
  }
}

void ViewPlot::AutoXRange()
{
  xmin = Inf;
  xmax = -Inf;
  for(size_t i=0;i<curves.size();i++) {
    if(curveColors[i].rgba[3] == 0) continue; //ignore hidden curves
    for(size_t j=0;j<curves[i].size();j++) {
      xmin = Min(xmin,curves[i][j].first);
      xmax = Max(xmax,curves[i][j].first);
    }
  }
  if(IsInf(xmin) || IsInf(xmax)) {
    xmin = 0;
    xmax = 1;
  }
}

void ViewPlot::AutoYRange()
{
  ymin = Inf;
  ymax = -Inf;
  for(size_t i=0;i<curves.size();i++) {
    if(curveColors[i].rgba[3] == 0) continue; //ignore hidden curves
    for(size_t j=0;j<curves[i].size();j++) {
      ymin = Min(ymin,curves[i][j].second);
      ymax = Max(ymax,curves[i][j].second);
    }
  }
  if(IsInf(ymin) || IsInf(ymax)) {
    ymin = 0;
    ymax = 1;
  }
}

void ViewPlot::AutoErase()
{
  //erase empty curves
  for(size_t i=0;i<curves.size();i++) {
    if(curves[i].empty()) {
      curves.erase(curves.begin()+i);
      i--;
      continue;
    }
  }

  //delete points any outside of the x range
  if(xMonotone) {
    for(size_t i=0;i<curves.size();i++) {
      if(curves[i].size() < 2) continue;
      size_t oldsize = curves[i].size();
      while((++curves[i].begin())->first < xmin) {
	curves[i].pop_front();
      }
      while((--(--curves[i].end()))->first > xmax) {
	curves[i].pop_back();
      }
      //if(curves[i].size() != oldsize)
	//printf("Auto-erased %d points\n",oldsize-curves[i].size());
    }
  }
  else {
    //delete curves out of bounds
    for(size_t i=0;i<curves.size();i++) {
      Real cxmin = curves[i].front().first;
      Real cxmax = curves[i].front().first;
      Real cymin = curves[i].front().second;
      Real cymax = curves[i].front().second;
      for(deque<pair<double,double> >::iterator j=curves[i].begin();j!=curves[i].end();j++) {
	if(j->first < cxmin) cxmin = j->first;
	else if(j->first > cxmax) cxmax = j->first;
	if(j->second < cymin) cymin = j->second;
	else if(j->second > cymax) cymax = j->second;
      }
      if(cxmax < xmin || cxmin > xmax || cymin > ymax || cymax < ymin) {
	curves.erase(curves.begin()+i);
	i--;
	continue;
      }
    }
  }
}

void ViewPlot::AddPoint(double x,double y,int curve)
{
  if(curves.empty() && autoXRange) xmin = xmax = x;
  if(curves.empty() && autoYRange) ymin = ymax = y;

  if(curve >= (int)curves.size()) {
    curves.resize(curve+1);
    GLColor randColor; randColor.setHSV((float)Math::Rand()*360,1,1);
    curveColors.resize(curve+1,randColor);
  }
  curves[curve].push_back(pair<double,double>(x,y));
  if(curveColors[curve].rgba[3] != 0) {
    if(autoXRange) {
      xmin = Min(xmin,x);
      xmax = Max(xmax,x);
    }
    if(autoYRange) {
      ymin = Min(ymin,y);
      ymax = Max(ymax,y);
    }
  }
  if(autoErase) {
    AutoErase();
  }
}

void ViewPlot::DrawGL()
{
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  if(drawBound) {
    boundColor.setCurrentGL();
    glBegin(GL_LINE_LOOP);
    glVertex2i(x,y);
    glVertex2i(x+width,y);
    glVertex2i(x+width,y+height);
    glVertex2i(x,y+height);
    glEnd();
  }
  if(drawPlotArea) {
    plotAreaColor.setCurrentGL();
    glBegin(GL_QUADS);
    glVertex2i(x,y);
    glVertex2i(x,y+height);
    glVertex2i(x+width,y+height);
    glVertex2i(x+width,y);
    glEnd();
  }

  if(xmin <= 0 && xmax >= 0) {
    axisColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2d(x+(0-xmin)/(xmax-xmin)*width,y);
    glVertex2d(x+(0-xmin)/(xmax-xmin)*width,y+height);
    glEnd();
  }
  if(ymin <= 0 && ymax >= 0) {
    axisColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2d(x,y+(0-ymin)/(ymax-ymin)*height);
    glVertex2d(x+width,y+(0-ymin)/(ymax-ymin)*height);
    glEnd();
  }

  for(size_t i=0;i<curves.size();i++) {
    if(curveColors[i].rgba[3] == 0) continue;
    curveColors[i].setCurrentGL();
    glBegin(GL_LINE_STRIP);
    for(deque<pair<double,double> >::const_iterator j=curves[i].begin();j!=curves[i].end();j++) {
      double xj=(x + (j->first-xmin)/(xmax-xmin)*width);
      double yj=(y + (j->second-ymin)/(ymax-ymin)*height);
      glVertex2d(xj,yj);
    }
    glEnd();
  }

  //draw labels
#if HAVE_GLUT
  char buf[64];
  void* font=GLUT_BITMAP_HELVETICA_10;
  double h = 10;
  double spacing = 2;
  glColor3f(1,1,1);
  if(drawXRangeBottom || drawXRangeTop) {
    sprintf(buf,"%.3f\n",xmin);
    if(drawXRangeTop) {
      glRasterPos2d(x,y-spacing);
      glutBitmapString(font,buf);
    }
    if(drawXRangeBottom) {
      glRasterPos2d(x,y+h+height+spacing);
      glutBitmapString(font,buf);
    }
    sprintf(buf,"%.3f\n",xmax);
    if(drawXRangeTop) {
      glRasterPos2d(x+width,y-spacing);
      glutBitmapString(font,buf);
    }
    if(drawXRangeBottom) {
      glRasterPos2d(x+width,y+h+height+spacing);
      glutBitmapString(font,buf);
    }
  }
  if(drawYRangeLeft || drawYRangeRight) {
    sprintf(buf,"%.3f\n",ymin);
    if(drawYRangeLeft) {
      glRasterPos2d(x-spacing-glutBitmapStringWidth(font,buf),y);
      glutBitmapString(font,buf);
    }
    if(drawYRangeRight) {
      glRasterPos2d(x+width+spacing,y);
      glutBitmapString(font,buf);
    }
    sprintf(buf,"%.3f\n",ymax);
    if(drawYRangeLeft) {
      glRasterPos2d(x-spacing-glutBitmapStringWidth(font,buf),y+height);
      glutBitmapString(font,buf);
    }
    if(drawYRangeRight) {
      glRasterPos2d(x+width+spacing,y+height);
      glutBitmapString(font,buf);
    }
  }
#endif //HAVE_GLUT
  glEnable(GL_DEPTH_TEST);
}
