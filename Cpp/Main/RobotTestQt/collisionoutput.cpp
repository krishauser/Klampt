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

void CollisionOutput::SetValue(const std::string& value,const std::string& value_with_names)
{
    rob_text = QString::fromStdString(value);
    urdf_text = QString::fromStdString(std::string("<noselfcollision pairs=\"") + value + std::string("\" />"));
    rob_names_text = QString::fromStdString(value_with_names);
    urdf_names_text = QString::fromStdString(std::string("<noselfcollision pairs=\"") + value_with_names + std::string("\" />"));
    UpdateText();
}

void CollisionOutput::UpdateText()
{
    if(ui->namesCheck->isChecked()) {
        if(ui->formatCombo->currentIndex() == 0)
           ui->textEdit->setText(rob_names_text);
        else
            ui->textEdit->setText(urdf_names_text);
    }
    else {
        if(ui->formatCombo->currentIndex() == 0)
            ui->textEdit->setText(rob_text);
        else
            ui->textEdit->setText(urdf_text);
    }
}

void CollisionOutput::SetFormat(int fmt)
{
    UpdateText();
}

void CollisionOutput::OnNamesClicked(int state)
{
    UpdateText();
}