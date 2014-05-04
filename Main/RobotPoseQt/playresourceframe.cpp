#include "playresourceframe.h"
#include "ui_playresourceframe.h"

PlayResourceFrame::PlayResourceFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::PlayResourceFrame)
{
    ui->setupUi(this);
}

PlayResourceFrame::~PlayResourceFrame()
{
    delete ui;
}

void PlayResourceFrame::Play(){

}

void PlayResourceFrame::Pause(){

}

void PlayResourceFrame::Record(){

}

void PlayResourceFrame::NewTime(int t){
    double scaled = t/1000.0;
    emit TimeChanged(scaled);
}

/*
void PlayResourceFrame::setDisabled(bool status){
    ui->pushButton_2->setDisabled(status);
    ui->pushButton_3->setDisabled(status);
    ui->pushButton_5->setDisabled(status);
    ui->slider->setDisabled(status);
}*/
