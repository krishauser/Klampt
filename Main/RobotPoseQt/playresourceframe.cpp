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
