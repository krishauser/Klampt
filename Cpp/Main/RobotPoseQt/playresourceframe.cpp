#include "playresourceframe.h"
#include "ui_playresourceframe.h"
#include <iostream>
using namespace std;

PlayResourceFrame::PlayResourceFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::PlayResourceFrame)
{
    ui->setupUi(this);
    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(Tick()));
    recording = 0;
}

PlayResourceFrame::~PlayResourceFrame()
{
  if(recording)
    emit ToggleRecording(false);
  delete ui;
}

void PlayResourceFrame::Reset()
{
  if(recording)
    emit ToggleRecording(false);
  recording = 0;
  time=start;
  timer->stop();
  ui->slider->setValue(0);
  emit TimeChanged(time);
}

void PlayResourceFrame::Play(){
    time=start;
    timer->start(1000/RESOURCE_RECORD_FPS);
    realTimer.Reset();
    realTimerStartTime=start;
}

void PlayResourceFrame::Tick(){
  time = realTimerStartTime + realTimer.ElapsedTime();
    if(time >= start + duration){
        time= start + duration;
        timer->stop();
        ui->slider->setValue(0);
        if(recording){
            emit ToggleRecording(false);
            recording = 0;
        }
    }
    else{
        ui->slider->setValue((time - start) / duration * 1000 - 1);
        //emit TimeChanged((time - start) / duration);
    }
    emit TimeChanged(time);
}

void PlayResourceFrame::Pause(){
    timer->stop();
}

void PlayResourceFrame::Record(){
    recording = 1;
    emit ToggleRecording(true);
    Play();
}

void PlayResourceFrame::NewTime(int t){
  time = start+(t/1000.0)*duration;
  realTimer.Reset();
  realTimerStartTime=time;
  emit TimeChanged(time);
}

void PlayResourceFrame::UpdatePlayerTimeRange(double minTime,double maxTime)
{
    duration = maxTime-minTime;
    time = start = minTime;
}

void PlayResourceFrame::EnablePlayer(bool enabled)
{
  //show/hide widgets
  ui->pushButton_2->setEnabled(enabled);
  ui->pushButton_3->setEnabled(enabled);
  ui->pushButton_5->setEnabled(enabled);
  ui->slider->setEnabled(enabled);
}
