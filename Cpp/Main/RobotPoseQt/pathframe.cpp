#include "pathframe.h"
#include "ui_pathframe.h"
#include "qrobotposegui.h"
#include <iostream>
using namespace std;

PathFrame::PathFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   frame(_frame),
   multipath(NULL),
   linearpath(NULL),
   ui(new Ui::PathFrame)
{
  ui->setupUi(this);

  QRobotPoseGUI* gui = dynamic_cast<QRobotPoseGUI*>(frame->gui);
  assert(gui!=NULL);
  QObject::connect(ui->playFrame,SIGNAL(ToggleRecording(bool)),gui,SLOT(SetRecord(bool)));
  QObject::connect(ui->playFrame,SIGNAL(TimeChanged(double)),frame,SLOT(SendPathTime(double)));
}

void PathFrame::set(MultiPathResource* r)
{
  assert(r != NULL);
  multipath = r;
  linearpath = NULL;

  blockSignals(true);
  Real minTime = r->path.StartTime(), maxTime = r->path.EndTime();
  ui->spin_duration->setEnabled(true);
  ui->spin_duration->setValue(maxTime-minTime);
  ui->playFrame->UpdatePlayerTimeRange(minTime,maxTime);
  ui->playFrame->Reset();
  blockSignals(false);
}

void PathFrame::set(LinearPathResource* r)
{
  assert(r != NULL);
  multipath = NULL;
  linearpath = r;

  blockSignals(true);
  double minTime=0,maxTime=1;
  if(r->times.size() >= 2) {
    minTime = r->times.front();
    maxTime = r->times.back();
    ui->spin_duration->setEnabled(true);
  }
  else
    ui->spin_duration->setEnabled(false);
  ui->spin_duration->setValue(maxTime-minTime);
  ui->playFrame->UpdatePlayerTimeRange(minTime,maxTime);
  ui->playFrame->Reset();
  blockSignals(false);
}


PathFrame::~PathFrame()
{
    delete ui;
}

void PathFrame::Optimize()
{
  int numdivs = ui->spin_numDivs->value();
  frame->gui->SendCommand("optimize_path",numdivs);
}

void PathFrame::Discretize()
{
  int numdivs = ui->spin_numDivs->value();
  frame->gui->SendCommand("discretize_path",numdivs);
}

void PathFrame::Split()
{
  frame->gui->SendCommand("split_path");
}

void PathFrame::SetDuration()
{
  double value = ui->spin_duration->value();
  if(value == 0) return;
  if(multipath) {
    if(multipath->path.sections.empty()) return;
    if(!multipath->path.HasTiming()) {
      multipath->path.SetSmoothTiming(value);
    }
    else {
      multipath->path.SetDuration(value);
    }
    double minTime = multipath->path.sections.front().times.front();
    double maxTime = multipath->path.sections.back().times.back();
    ui->playFrame->UpdatePlayerTimeRange(minTime,maxTime);
    ui->playFrame->Reset();
  }
  else if(linearpath) {
    if(linearpath->times.empty() || linearpath->times.size()==1)
      return;
    double oldduration = linearpath->times.back()-linearpath->times.front();
    double t0=linearpath->times.front();
    double scale = value / oldduration;
    for(size_t i=0;i<linearpath->times.size();i++) {
      double t=linearpath->times[i] - t0;
      linearpath->times[i] = t0 + t*scale;
    }
    double minTime = linearpath->times.front();
    double maxTime = linearpath->times.back();
    ui->playFrame->UpdatePlayerTimeRange(minTime,maxTime);
    ui->playFrame->Reset();
  }
  frame->onResourceEdit();
}
