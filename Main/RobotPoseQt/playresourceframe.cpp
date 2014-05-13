#include "playresourceframe.h"
#include "ui_playresourceframe.h"


PlayResourceFrame::PlayResourceFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::PlayResourceFrame)
{
    ui->setupUi(this);
    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(Tick()));
}

PlayResourceFrame::~PlayResourceFrame()
{
    delete ui;
}

void PlayResourceFrame::Play(){
    time=start;
    timer->start(100);// 10fps
}

void PlayResourceFrame::Tick(){
    time += .1;
    if(time >= start + duration){
        time= start + duration;
        timer->stop();
        ui->slider->setValue((time - start) / duration);
    }
    else{
        ui->slider->setValue((time - start) / duration * 1000 - 1);
        emit TimeChanged((time - start) / duration);
    }
    emit TimeChanged(time);
}

void PlayResourceFrame::Pause(){
    timer->stop();
}

void PlayResourceFrame::Record(){

}

void PlayResourceFrame::NewTime(int t){
    double scaled = t/1000.0;
    emit TimeChanged(scaled);
}

void PlayResourceFrame::EnablePath(string args){
    stringstream ss(args);
    double front,back;
    ss>>back>>front;
    duration = back-front;
    time=start=front;
}

/*
void PlayResourceFrame::setDisabled(bool status){
    ui->pushButton_2->setDisabled(status);
    ui->pushButton_3->setDisabled(status);
    ui->pushButton_5->setDisabled(status);
    ui->slider->setDisabled(status);
}*/
