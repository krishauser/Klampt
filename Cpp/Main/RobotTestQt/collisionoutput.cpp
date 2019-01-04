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

void CollisionOutput::SetValue(const std::string& value)
{
    rob_text = QString::fromStdString(value);
    urdf_text = QString::fromStdString(std::string("<noselfcollision pairs=\"") + value + std::string("\" />"));
    ui->textEdit->setText(rob_text);
}

void CollisionOutput::SetFormat(int fmt)
{
    if(fmt == 0)
        ui->textEdit->setText(rob_text);
    else
        ui->textEdit->setText(urdf_text);
}