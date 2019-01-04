#include "collisionoutput.h"
#include "ui_collisionoutput.h"

CollisionOutput::CollisionOutput(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CollisionOutput)
{
    ui->setupUi(this);
}

CollisionOutput::~CollisionOutput()
{
    delete ui;
}

void CollisionOutput::SetText(QString text){
    ui->textEdit->setText(text);
}
